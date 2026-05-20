// PMW3901 optical-flow and VL53LXX rangefinder bench test support.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

extern float t;
extern bool armed;
extern void print(const char* format, ...);
void setFlightControlPaused(bool paused);
void stepFixed(float fixedDt);
#if WIFI_ENABLED
void processMavlink();
#endif

// ESP32 default VSPI pins are shared with the IMU. Keep CS separate.
constexpr int FLOW_SPI_SCK = 18;
constexpr int FLOW_SPI_MISO = 19;
constexpr int FLOW_SPI_MOSI = 23;
constexpr int FLOW_PMW_CS = 25;
constexpr int FLOW_POWER_EN = 27; // ATK E_CS1/PB0, used by MiniFly as optical-flow power control.
constexpr int FLOW_I2C_SDA = 21;
constexpr int FLOW_I2C_SCL = 22;
constexpr uint8_t VL53_DEFAULT_ADDR = 0x29;

enum RangeSensorType {
	RANGE_NONE,
	RANGE_VL53L0X,
	RANGE_VL53L1X,
};

Bitcraze_PMW3901 pmw3901(FLOW_PMW_CS);
VL53L0X vl53l0x;
VL53L1X vl53l1x;

bool flowInitTried = false;
bool pmwReady = false;
RangeSensorType rangeType = RANGE_NONE;
uint32_t flowSamples = 0;
int16_t flowDx = 0;
int16_t flowDy = 0;
int16_t flowForwardPx = 0;
int16_t flowLeftPx = 0;
uint16_t flowRangeMm = 0;
bool flowRangeTimeout = false;
const char *flowRangeStatus = "none";
bool flowRangeDataReady = false;
float flowRangeSignal = 0.0f;
float flowRangeAmbient = 0.0f;
bool flowPowerEnabled = true;

const char *rangeTypeName();
bool i2cAddressPresent(uint8_t addr);
void setFlowPowerPin(bool enabled);
bool setupFlowHardware();
bool initRangeSensor();
bool setupFlow();
void readFlowOnce();
void printFlowSample();
void printFlowInfo();
void testFlow(float duration);
void serviceFlowTest();

const char *rangeTypeName() {
	switch (rangeType) {
		case RANGE_VL53L0X: return "VL53L0X";
		case RANGE_VL53L1X: return "VL53L1X";
		default: return "none";
	}
}

bool i2cAddressPresent(uint8_t addr) {
	Wire.beginTransmission(addr);
	return Wire.endTransmission() == 0;
}

void setFlowPowerPin(bool enabled) {
	flowPowerEnabled = enabled;
	pinMode(FLOW_POWER_EN, OUTPUT);
	digitalWrite(FLOW_POWER_EN, enabled ? HIGH : LOW);
	flowInitTried = false;
	print("Flow PWR_EN GPIO%d = %d. Run flowinit again.\n", FLOW_POWER_EN, enabled ? 1 : 0);
}

bool initRangeSensor() {
	rangeType = RANGE_NONE;
	flowRangeMm = 0;
	flowRangeTimeout = false;
	flowRangeStatus = "none";
	flowRangeDataReady = false;
	flowRangeSignal = 0.0f;
	flowRangeAmbient = 0.0f;

	if (!i2cAddressPresent(VL53_DEFAULT_ADDR)) return false;

	vl53l1x.setBus(&Wire);
	vl53l1x.setTimeout(100);
	if (vl53l1x.init()) {
		vl53l1x.setDistanceMode(VL53L1X::Long);
		vl53l1x.setMeasurementTimingBudget(50000);
		vl53l1x.startContinuous(60);
		delay(80);
		rangeType = RANGE_VL53L1X;
		return true;
	}

	vl53l0x.setBus(&Wire);
	vl53l0x.setTimeout(100);
	if (vl53l0x.init()) {
		vl53l0x.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		vl53l0x.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
		vl53l0x.setMeasurementTimingBudget(50000);
		vl53l0x.startContinuous(50);
		rangeType = RANGE_VL53L0X;
		return true;
	}

	return false;
}

bool setupFlowHardware() {
	print("Setup flow module\n");
	flowInitTried = true;

	pinMode(FLOW_POWER_EN, OUTPUT);
	digitalWrite(FLOW_POWER_EN, flowPowerEnabled ? HIGH : LOW);
	pinMode(FLOW_PMW_CS, OUTPUT);
	digitalWrite(FLOW_PMW_CS, HIGH);
	delay(120);

	SPI.begin(FLOW_SPI_SCK, FLOW_SPI_MISO, FLOW_SPI_MOSI);
	Wire.begin(FLOW_I2C_SDA, FLOW_I2C_SCL);
	Wire.setClock(400000);

	pmwReady = pmw3901.begin();
	const bool rangeReady = initRangeSensor();

	print("Flow pins: PMW CS=%d SCK=%d MISO=%d MOSI=%d, I2C SDA=%d SCL=%d PWR_EN=%d level=%d\n",
		FLOW_PMW_CS, FLOW_SPI_SCK, FLOW_SPI_MISO, FLOW_SPI_MOSI, FLOW_I2C_SDA, FLOW_I2C_SCL,
		FLOW_POWER_EN, flowPowerEnabled ? 1 : 0);
	print("PMW3901: %s, range: %s%s\n",
		pmwReady ? "OK" : "FAIL",
		rangeTypeName(),
		rangeReady ? " OK" : " FAIL");
	return pmwReady || rangeReady;
}

bool setupFlow() {
	if (armed) {
		print("Flow setup refused: disarm first\n");
		return false;
	}

	setFlightControlPaused(true);
	delay(20);
	const bool ok = setupFlowHardware();
	setFlightControlPaused(false);
	return ok;
}

void readFlowOnce() {
	if (!flowInitTried) setupFlowHardware();

	flowSamples++;
	if (pmwReady) {
		pmw3901.readMotionCount(&flowDx, &flowDy);
		flowForwardPx = flowDy;
		flowLeftPx = -flowDx;
	}

	if (rangeType == RANGE_VL53L1X) {
		flowRangeDataReady = vl53l1x.dataReady();
		flowRangeMm = vl53l1x.read(true);
		flowRangeTimeout = vl53l1x.timeoutOccurred();
		flowRangeStatus = VL53L1X::rangeStatusToString(vl53l1x.ranging_data.range_status);
		flowRangeSignal = vl53l1x.ranging_data.peak_signal_count_rate_MCPS;
		flowRangeAmbient = vl53l1x.ranging_data.ambient_count_rate_MCPS;
	} else if (rangeType == RANGE_VL53L0X) {
		flowRangeMm = vl53l0x.readRangeContinuousMillimeters();
		flowRangeTimeout = vl53l0x.timeoutOccurred();
		flowRangeStatus = flowRangeTimeout ? "timeout" : "ok";
		flowRangeDataReady = !flowRangeTimeout;
		flowRangeSignal = 0.0f;
		flowRangeAmbient = 0.0f;
	}
}

void printFlowInfo() {
	if (armed) {
		print("Flow read refused: disarm first\n");
		return;
	}

	setFlightControlPaused(true);
	delay(20);
	printFlowSample();
	setFlightControlPaused(false);
}

void printFlowSample() {
	readFlowOnce();
	print("flow: samples=%lu pmw=%d raw_dx=%d raw_dy=%d forward_px=%d left_px=%d range=%s %u mm timeout=%d ready=%d status=%s signal=%.2f ambient=%.2f\n",
		flowSamples, pmwReady, flowDx, flowDy, flowForwardPx, flowLeftPx,
		rangeTypeName(), flowRangeMm, flowRangeTimeout, flowRangeDataReady,
		flowRangeStatus, flowRangeSignal, flowRangeAmbient);
}

void testFlow(float duration) {
	if (armed) {
		print("Flow test refused: disarm first\n");
		return;
	}

	duration = constrain(duration, 1.0f, 30.0f);
	print("Flow test %.1f s. Move the module over a textured surface 8-50 cm above it.\n", duration);

	setFlightControlPaused(true);
	delay(20);
	if (!flowInitTried) setupFlowHardware();
	const uint32_t start = millis();
	const uint32_t durationMs = (uint32_t)(duration * 1000.0f);
	while ((uint32_t)(millis() - start) < durationMs) {
		printFlowSample();
		for (int i = 0; i < 10; i++) serviceFlowTest();
	}
	setFlightControlPaused(false);
	print("Flow test done\n");
}

void serviceFlowTest() {
	stepFixed(0.01f);
#if WIFI_ENABLED
	processMavlink();
#endif
	delay(10);
}
