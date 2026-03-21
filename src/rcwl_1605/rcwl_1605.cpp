#include <Arduino.h>
#include "util.h"
#include "rcwl_1605.h"

extern float t;
extern void print(const char* format, ...);

// 使用硬件串口 2 进行超声波 UART 通信
// ESP32 Dev board: UART2 映射到 GPIO4 (RX) 和 GPIO5 (TX)
HardwareSerial SonarSerial(2);
static const int8_t kSonarRxPin = 4;
static const int8_t kSonarTxPin = 5;

static constexpr int RCWL1605_SENSOR_COUNT = 1;

// Runtime parameters (persisted via parameters.cpp)
float rcwl1605Enabled = 1.0f;
float rcwl1605FailsafeEnabled = 0.0f;
float rcwl1605ObstacleThresholdM = 0.60f;
float rcwl1605MinRangeM = 0.03f;
float rcwl1605MaxRangeM = 4.50f;
float rcwl1605FilterAlpha = 0.35f;
float rcwl1605ReadIntervalMs = 50.0f; // UART 模式下稍微放宽查询间隔，50ms 相当于 20Hz 更新率，足够了

static float distancesM[RCWL1605_SENSOR_COUNT];
static bool validFlags[RCWL1605_SENSOR_COUNT];
static uint32_t lastRequestMs = 0;

static inline bool rcwl1605IsEnabled() {
	return rcwl1605Enabled >= 0.5f;
}

void setupRCWL1605() {
	for (int i = 0; i < RCWL1605_SENSOR_COUNT; i++) {
		distancesM[i] = NAN;
		validFlags[i] = false;
	}
	
	// 初始化串口2：波特率 9600, 8个数据位无校验1个停止位, 接收引脚 4, 发送引脚 5
	SonarSerial.begin(9600, SERIAL_8N1, kSonarRxPin, kSonarTxPin);
	
	// 清空串口接收缓存里的杂音
	while(SonarSerial.available()) SonarSerial.read();

	lastRequestMs = millis();
	print("RCWL_1605 Non-Blocking UART Mode ready. TX=5, RX=4\n");
}

void readRCWL1605() {
	if (!rcwl1605IsEnabled()) return;

	uint32_t nowMs = millis();
	uint32_t intervalMs = (uint32_t)constrain(rcwl1605ReadIntervalMs, 20.0f, 200.0f);

	// 1. 如果时间到了，非阻塞地向超声波发送请求指令：0xA0
	if (nowMs - lastRequestMs >= intervalMs) {
		// 发送前清一下接收缓冲，防止上一次没读完的残余数据错位
		while(SonarSerial.available() > 0) SonarSerial.read();
		
		SonarSerial.write(0xA0); 
		lastRequestMs = nowMs;
	}

	// 2. 检查有没有收到 3 个字节的数据回传
	if (SonarSerial.available() >= 3) {
		uint8_t byteH = SonarSerial.read();
		uint8_t byteM = SonarSerial.read();
		uint8_t byteL = SonarSerial.read();

		// 根据文档组合出距离数据 (毫米)，并除以 1000 转换为我们飞控统一的米 (m)
		uint32_t distanceMm = (byteH << 16) + (byteM << 8) + byteL;
		float measured = (float)distanceMm / 1000.0f;

		// 剔除异常值或盲区值
		if (measured < rcwl1605MinRangeM || measured > rcwl1605MaxRangeM) {
			validFlags[0] = false;
			distancesM[0] = NAN;
		} else {
			// 如果读到了合法数据，走一下一阶低通滤波
			float alpha = constrain(rcwl1605FilterAlpha, 0.0f, 1.0f);
			if (isnan(distancesM[0])) {
				distancesM[0] = measured;
			} else {
				distancesM[0] = distancesM[0] * (1.0f - alpha) + measured * alpha;
			}
			validFlags[0] = true;
		}
	}
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
		print("  sonar[%d] rx=%d tx=%d valid=%d dist=%.3fm\n", i, kSonarRxPin, kSonarTxPin, validFlags[i], distancesM[i]);
	}
	float minDistance = rcwl1605MinDistanceM();
	print("  min=%.3fm obstacle=%d\n", minDistance, rcwl1605ObstacleDetected());
}
