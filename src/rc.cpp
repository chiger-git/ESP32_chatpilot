// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 使用RC接收器，注释修改：B站微辣火龙果 https://space.bilibili.com/544479100
// Work with the RC receiver

#include <Arduino.h>
#include <SBUS.h>
#include "util.h"

// 外部变量与函数声明
extern float t; // main.cpp
extern void print(const char* format, ...); // cli.cpp
extern void pause(float duration); // cli.cpp
extern bool armed;
void setFlightControlPaused(bool paused);
void stopMotors();
void sendMotors();
void stepFixed(float fixedDt);
float getMappedMotorOutput(int motorId, float value);
extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;
#if WIFI_ENABLED
void processMavlink();
#endif

// 输出到全局的控制量
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float motors[4];

// 内部函数前向声明
void normalizeRC();
float normalizeRCChannel(int channel);
void printRCCalibration();
void calibrateRCChannel(float *channel, uint16_t in[16], uint16_t out[16], const char *str);
float getRCRealAge();
void resetRCDiagnostics();
void printRCDiagnostics();
void testRCMotorNoise(float motorValue, float duration);
void testRCMotorNoiseOne(int motorId, float motorValue);
void serviceDuringMotorNoiseTest();

// 全局变量定义
// 定义 SBUS 对象
// Bolder Flight SBUS 构造函数：SBUS(Bus, RX, TX, Invert)
SBUS rc(Serial2, 16, 17, true); 
uint16_t channels[16]; // raw rc channels

float controlTime = 0; // time of the last controls update
uint32_t rcFrameCount = 0;
uint32_t rcLostFrameCount = 0;
uint32_t rcFailsafeFrameCount = 0;
uint32_t rcNoFramePollCount = 0;
uint32_t rcLastFrameMicros = 0;
uint32_t rcMaxFrameGapMicros = 0;
uint32_t rcLastFrameGapMicros = 0;
uint16_t rcLastUartAvailable = 0;
uint16_t rcMaxUartAvailable = 0;
bool rcLastLostFrame = false;
bool rcLastFailsafe = false;
// 校准数据
float channelZero[16]; // calibration zero values
float channelMax[16]; // calibration max values
// Channels mapping (using float to store in parameters):
// 初始化为 NAN 表示未校准/未识别
float rollChannel = NAN, pitchChannel = NAN, throttleChannel = NAN, yawChannel = NAN, modeChannel = NAN;

void setupRC() {
	print("Setup RC\n");
	rc.begin(); 
}

bool readRC() {
	const uint16_t pending = Serial2.available();
	rcLastUartAvailable = pending;
	if (pending > rcMaxUartAvailable) rcMaxUartAvailable = pending;

	if (rc.read()) {
		SBUSData data = rc.data();
		const uint32_t now = micros();
		if (rcLastFrameMicros != 0) {
			rcLastFrameGapMicros = now - rcLastFrameMicros;
			if (rcLastFrameGapMicros > rcMaxFrameGapMicros) rcMaxFrameGapMicros = rcLastFrameGapMicros;
		}
		rcLastFrameMicros = now;
		rcFrameCount++;
		rcLastLostFrame = data.lost_frame;
		rcLastFailsafe = data.failsafe;
		if (data.lost_frame) rcLostFrameCount++;
		if (data.failsafe) rcFailsafeFrameCount++;
		for (int i = 0; i < 16; i++) channels[i] = data.ch[i]; // copy channels data
		normalizeRC();
		controlTime = t;
		return true;
	}
	rcNoFramePollCount++;
	return false;
}
// bool readRC() {
//     // --- 调试区块1：打印串口缓冲区是否有原始字面量 ---
//     static uint32_t lastRxCheck = 0;
//     if (millis() - lastRxCheck > 1000) { // 每1秒打印一次
//         print("Serial2 RX buffer target: %d\n", Serial2.available());
//         lastRxCheck = millis();
//     }
//     // ----------------------------------------------

//     if (rc.read()) {
//         SBUSData data = rc.data();
//         for (int i = 0; i < 16; i++) channels[i] = data.ch[i]; // copy channels data
//         normalizeRC();
//         controlTime = t;

//         // --- 调试区块2：打印成功解析到的通道数值 ---
//         static uint32_t lastChCheck = 0;
//         if (millis() - lastChCheck > 500) { // 每0.5秒打印一次
//             print("SBUS Decoded -> CH1:%d CH2:%d CH3:%d CH4:%d\n", 
//                 channels[0], channels[1], channels[2], channels[3]);
//             lastChCheck = millis();
//         }
//         // -----------------------------------------

//         return true;
//     }
//     return false;
// }

void normalizeRC() {
	float controls[16];
	for (int i = 0; i < 16; i++) {
		controls[i] = normalizeRCChannel(i);
	}
	// Update control values
	controlRoll = rollChannel >= 0 ? controls[(int)rollChannel] : NAN;
	controlPitch = pitchChannel >= 0 ? controls[(int)pitchChannel] : NAN;
	controlYaw = yawChannel >= 0 ? controls[(int)yawChannel] : NAN;
	controlThrottle = throttleChannel >= 0 ? controls[(int)throttleChannel] : NAN;
	controlMode = modeChannel >= 0 ? controls[(int)modeChannel] : NAN;
}

float normalizeRCChannel(int channel) {
	if (channel < 0 || channel >= 16) return NAN;
	if (channelZero[channel] == channelMax[channel]) return NAN;
	return mapf(channels[channel], channelZero[channel], channelMax[channel], 0, 1);
}

void calibrateRC() {
	armed = false;
	setFlightControlPaused(true);
	delay(20);
	stopMotors();
	uint16_t zero[16];
	uint16_t center[16];
	uint16_t max[16];
	print("1/8 校准遥控,所有摇杆归中位置[3秒]\n");
	pause(8);
	calibrateRCChannel(NULL, zero, zero, "2/8 左摇杆:向下,右摇杆:居中[3秒]\n...     ...\n...     .o.\n.o.     ...\n");
	calibrateRCChannel(NULL, center, center, "3/8 左摇杆:居中,右摇杆:居中[3秒]\n...     ...\n.o.     .o.\n...     ...\n");
	calibrateRCChannel(&throttleChannel, zero, max, "4/8 油门通道识别,左摇杆:向上推到底(油门最大),右摇杆：居中[3秒]\n.o.     ...\n...     .o.\n...     ...\n");
	calibrateRCChannel(&yawChannel, center, max, "5/8 偏航通道识别,左摇杆:向右推到底(偏航右转)右摇杆:居中[3秒]\n...     ...\n..o     .o.\n...     ...\n");
	calibrateRCChannel(&pitchChannel, zero, max, "6/8 俯仰通道识别,左摇杆:向下推到底,右摇杆:向上推到底(俯仰前进)[3秒]\n...     .o.\n...     ...\n.o.     ...\n");
	calibrateRCChannel(&rollChannel, zero, max, "7/8 横滚通道识别,左摇杆:向下推到底,右摇杆:向右推到底(横滚右转)[3秒]\n...     ...\n...     ..o\n.o.     ...\n");
	calibrateRCChannel(&modeChannel, zero, max, "8/8 模式通道识别,先将解锁开关拨回锁定位置,然后将模式开关拨到最高档位(如手动模式)[3秒]\n");
	printRCCalibration();
	stopMotors();
	armed = false;
	setFlightControlPaused(false);
}

//第1步：初始化位置,操作：
// ✓ 所有摇杆归中位置
// ✓ 所有开关拨到默认位置（通常是锁定/中间位置）
// ✓ 保持3秒不动
//第2步：基础摇杆移动,操作：
// ✓ 左摇杆：向下
// ✓ 右摇杆：居中
// ✓ 保持3秒
//第3步：摇杆居中,操作：
// ✓ 左摇杆：居中
// ✓ 右摇杆：居中  
// ✓ 保持3秒
//第4步：油门通道识别,操作：
// ✓ 左摇杆：向上推到底（油门最大）
// ✓ 右摇杆：居中
// ✓ 保持3秒
//→ 系统自动识别油门通道
//第5步：偏航通道识别,操作：
// ✓ 左摇杆：向右推到底（偏航右转）
// ✓ 右摇杆：居中
// ✓ 保持3秒
// → 系统自动识别偏航通道
//第6步：俯仰通道识别,操作：
// ✓ 左摇杆：向下推到底
// ✓ 右摇杆：向上推到底（俯仰前进）
// ✓ 保持3秒
// → 系统自动识别俯仰通道
//第7步：横滚通道识别,操作：
// ✓ 左摇杆：向下推到底
// ✓ 右摇杆：向右推到底（横滚右转）
// ✓ 保持3秒
// → 系统自动识别横滚通道
//第8步：模式通道识别,操作：
// ✓ 先将解锁开关拨回锁定位置
// ✓ 然后将模式开关拨到最高档位（如手动模式）
// ✓ 保持3秒
// → 系统自动识别模式通道

void calibrateRCChannel(float *channel, uint16_t in[16], uint16_t out[16], const char *str) {
	print("%s", str);
	pause(8);
	for (int i = 0; i < 30; i++) readRC(); // try update 30 times max
	memcpy(out, channels, sizeof(channels));

	if (channel == NULL) return; // no channel to calibrate

	// Find channel that changed the most between in and out
	int ch = -1, diff = 0;
	for (int i = 0; i < 16; i++) {
		if (abs(out[i] - in[i]) > diff) {
			ch = i;
			diff = abs(out[i] - in[i]);
		}
	}
	if (ch >= 0 && diff > 10) { // difference threshold is 10
		*channel = ch;
		channelZero[ch] = in[ch];
		channelMax[ch] = out[ch];
	} else {
		*channel = NAN;
	}
}

void printRCCalibration() {
	print("Control   Ch     Zero   Max\n");
	print("Roll      %-7g%-7g%-7g\n", rollChannel, rollChannel >= 0 ? channelZero[(int)rollChannel] : NAN, rollChannel >= 0 ? channelMax[(int)rollChannel] : NAN);
	print("Pitch     %-7g%-7g%-7g\n", pitchChannel, pitchChannel >= 0 ? channelZero[(int)pitchChannel] : NAN, pitchChannel >= 0 ? channelMax[(int)pitchChannel] : NAN);
	print("Yaw       %-7g%-7g%-7g\n", yawChannel, yawChannel >= 0 ? channelZero[(int)yawChannel] : NAN, yawChannel >= 0 ? channelMax[(int)yawChannel] : NAN);
	print("Throttle  %-7g%-7g%-7g\n", throttleChannel, throttleChannel >= 0 ? channelZero[(int)throttleChannel] : NAN, throttleChannel >= 0 ? channelMax[(int)throttleChannel] : NAN);
	print("Mode      %-7g%-7g%-7g\n", modeChannel, modeChannel >= 0 ? channelZero[(int)modeChannel] : NAN, modeChannel >= 0 ? channelMax[(int)modeChannel] : NAN);
}

float getRCRealAge() {
	if (rcLastFrameMicros == 0) return NAN;
	return (micros() - rcLastFrameMicros) * 0.000001f;
}

void resetRCDiagnostics() {
	rcFrameCount = 0;
	rcLostFrameCount = 0;
	rcFailsafeFrameCount = 0;
	rcNoFramePollCount = 0;
	rcLastFrameMicros = 0;
	rcMaxFrameGapMicros = 0;
	rcLastFrameGapMicros = 0;
	rcLastUartAvailable = 0;
	rcMaxUartAvailable = 0;
	rcLastLostFrame = false;
	rcLastFailsafe = false;
}

void printRCDiagnostics() {
	print("RC frames: %lu no-frame polls: %lu\n", rcFrameCount, rcNoFramePollCount);
	print("SBUS flags: lost=%lu failsafe=%lu last_lost=%d last_failsafe=%d\n",
		rcLostFrameCount, rcFailsafeFrameCount, rcLastLostFrame, rcLastFailsafe);
	print("RC real age: %.3f s, last gap: %.1f ms, max gap: %.1f ms\n",
		getRCRealAge(), rcLastFrameGapMicros * 0.001f, rcMaxFrameGapMicros * 0.001f);
	print("Serial2 pending bytes: last=%u max=%u\n", rcLastUartAvailable, rcMaxUartAvailable);
}

void testRCMotorNoise(float motorValue, float duration) {
	motorValue = constrain(motorValue, 0.0f, 1.0f);
	duration = constrain(duration, 1.0f, 10.0f);
	print("RC motor-noise test: motors=%.2f mapped RL/RR/FR/FL=%.2f %.2f %.2f %.2f duration=%.1f s. REMOVE PROPS.\n",
		motorValue,
		getMappedMotorOutput(MOTOR_REAR_LEFT, motorValue),
		getMappedMotorOutput(MOTOR_REAR_RIGHT, motorValue),
		getMappedMotorOutput(MOTOR_FRONT_RIGHT, motorValue),
		getMappedMotorOutput(MOTOR_FRONT_LEFT, motorValue),
		duration);

	armed = false;
	setFlightControlPaused(true);
	delay(20);
	stopMotors();
	resetRCDiagnostics();

	for (int i = 0; i < 4; i++) motors[i] = motorValue;
	sendMotors();

	const uint32_t start = millis();
	const uint32_t durationMs = (uint32_t)(duration * 1000.0f);
	while ((uint32_t)(millis() - start) < durationMs) {
		serviceDuringMotorNoiseTest();
	}

	stopMotors();
	armed = false;
	setFlightControlPaused(false);
	printRCDiagnostics();
	print("RC motor-noise test done\n");
}

void testRCMotorNoiseOne(int motorId, float motorValue) {
	if (motorId < 0 || motorId > 3) return;
	motorValue = constrain(motorValue, 0.0f, 1.0f);
	print("RC single-motor noise test: motor=%d value=%.2f mapped=%.2f duration=5.0 s. REMOVE PROPS.\n",
		motorId, motorValue, getMappedMotorOutput(motorId, motorValue));

	armed = false;
	setFlightControlPaused(true);
	delay(20);
	stopMotors();
	resetRCDiagnostics();

	motors[motorId] = motorValue;
	sendMotors();

	const uint32_t start = millis();
	while ((uint32_t)(millis() - start) < 5000) {
		serviceDuringMotorNoiseTest();
	}

	stopMotors();
	armed = false;
	setFlightControlPaused(false);
	printRCDiagnostics();
	print("RC single-motor noise test done\n");
}

void serviceDuringMotorNoiseTest() {
	stepFixed(0.002f);
	readRC();
#if WIFI_ENABLED
	processMavlink();
#endif
	sendMotors();
	delay(2);
}
