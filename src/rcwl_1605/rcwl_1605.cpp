#include <Arduino.h>
#include "util.h"
#include "rcwl_1605.h"

extern float t;
extern void print(const char* format, ...);

// Default mapping for one RCWL_1605 module.
// D26/D27 on ESP32 Dev board means GPIO26/GPIO27.
static constexpr int RCWL1605_SENSOR_COUNT = 1;
static const int8_t kTrigPins[RCWL1605_SENSOR_COUNT] = {26};
static const int8_t kEchoPins[RCWL1605_SENSOR_COUNT] = {27};

// Runtime parameters (persisted via parameters.cpp)
float rcwl1605Enabled = 1.0f;
float rcwl1605FailsafeEnabled = 0.0f;
float rcwl1605ObstacleThresholdM = 0.60f;
float rcwl1605MinRangeM = 0.03f;
float rcwl1605MaxRangeM = 4.50f;
float rcwl1605FilterAlpha = 0.35f;
float rcwl1605ReadIntervalMs = 15.0f;

static float distancesM[RCWL1605_SENSOR_COUNT];
static bool validFlags[RCWL1605_SENSOR_COUNT];
static uint32_t lastReadUs = 0;
static int sensorIndex = 0;

static inline bool rcwl1605IsEnabled() {
	return rcwl1605Enabled >= 0.5f;
}

static unsigned long rcwl1605TimeoutUs() {
	float timeout = rcwl1605MaxRangeM * 5800.0f; // echo timeout for distance (m)
	timeout = constrain(timeout, 1500.0f, 30000.0f);
	return (unsigned long)timeout;
}

static float measureDistanceM(int index) {
	if (index < 0 || index >= RCWL1605_SENSOR_COUNT) return NAN;
	if (kTrigPins[index] < 0 || kEchoPins[index] < 0) return NAN;

	digitalWrite(kTrigPins[index], LOW);
	delayMicroseconds(2);
	digitalWrite(kTrigPins[index], HIGH);
	delayMicroseconds(10);
	digitalWrite(kTrigPins[index], LOW);

	unsigned long duration = pulseIn(kEchoPins[index], HIGH, rcwl1605TimeoutUs());
	if (duration == 0) return NAN;

	float distance = duration * 0.0001715f; // meters
	if (distance < rcwl1605MinRangeM || distance > rcwl1605MaxRangeM) return NAN;
	return distance;
}

void setupRCWL1605() {
	for (int i = 0; i < RCWL1605_SENSOR_COUNT; i++) {
		distancesM[i] = NAN;
		validFlags[i] = false;
		if (kTrigPins[i] < 0 || kEchoPins[i] < 0) continue;
		pinMode(kTrigPins[i], OUTPUT);
		pinMode(kEchoPins[i], INPUT);
		digitalWrite(kTrigPins[i], LOW);
	}
	lastReadUs = micros();
	sensorIndex = 0;
	print("RCWL_1605 ready, sensors=%d\n", RCWL1605_SENSOR_COUNT);
}

void readRCWL1605() {
	if (!rcwl1605IsEnabled()) return;

	uint32_t nowUs = micros();
	uint32_t intervalUs = (uint32_t)constrain(rcwl1605ReadIntervalMs * 1000.0f, 4000.0f, 200000.0f);
	if (nowUs - lastReadUs < intervalUs) return;
	lastReadUs = nowUs;

	float measured = measureDistanceM(sensorIndex);
	if (isnan(measured)) {
		validFlags[sensorIndex] = false;
		distancesM[sensorIndex] = NAN;
	} else {
		float alpha = constrain(rcwl1605FilterAlpha, 0.0f, 1.0f);
		if (isnan(distancesM[sensorIndex])) {
			distancesM[sensorIndex] = measured;
		} else {
			distancesM[sensorIndex] = distancesM[sensorIndex] * (1.0f - alpha) + measured * alpha;
		}
		validFlags[sensorIndex] = true;
	}

	sensorIndex++;
	if (sensorIndex >= RCWL1605_SENSOR_COUNT) sensorIndex = 0;
}

int rcwl1605SensorCount() {
	return RCWL1605_SENSOR_COUNT;
}

bool rcwl1605SensorValid(int index) {
	if (index < 0 || index >= RCWL1605_SENSOR_COUNT) return false;
	return validFlags[index];
}

float rcwl1605DistanceM(int index) {
	if (index < 0 || index >= RCWL1605_SENSOR_COUNT) return NAN;
	return distancesM[index];
}

float rcwl1605MinDistanceM() {
	float minDistance = NAN;
	for (int i = 0; i < RCWL1605_SENSOR_COUNT; i++) {
		if (!validFlags[i]) continue;
		if (isnan(minDistance) || distancesM[i] < minDistance) {
			minDistance = distancesM[i];
		}
	}
	return minDistance;
}

bool rcwl1605ObstacleDetected() {
	if (!rcwl1605IsEnabled()) return false;
	float minDistance = rcwl1605MinDistanceM();
	if (isnan(minDistance)) return false;
	return minDistance <= rcwl1605ObstacleThresholdM;
}

void printRCWL1605Status() {
	print("RCWL_1605 enabled=%d failsafe=%d threshold=%.2fm\n",
		rcwl1605Enabled >= 0.5f, rcwl1605FailsafeEnabled >= 0.5f, rcwl1605ObstacleThresholdM);
	for (int i = 0; i < RCWL1605_SENSOR_COUNT; i++) {
		print("  sonar[%d] trig=%d echo=%d valid=%d dist=%.3fm\n", i, kTrigPins[i], kEchoPins[i], validFlags[i], distancesM[i]);
	}
	float minDistance = rcwl1605MinDistanceM();
	print("  min=%.3fm obstacle=%d\n", minDistance, rcwl1605ObstacleDetected());
}
