// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 故障安全保护功能
// Fail-safe functions

#include <Arduino.h>
#include "quaternion.h"
#include "vector.h"

float rcLossTimeout = 1.0f;

// 外部变量声明 (引用其他文件定义的变量)
// 来自 rc.cpp
extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;
extern uint32_t rcFrameCount;
extern uint32_t rcLostFrameCount;
extern uint32_t rcFailsafeFrameCount;
extern uint32_t rcMaxFrameGapMicros;
extern uint16_t rcLastUartAvailable;
extern uint16_t rcMaxUartAvailable;
float getRCRealAge();

// 来自 control.cpp
extern bool armed;
extern const char *lastStopReason;
extern int mode;
extern Quaternion attitudeTarget;
extern float thrustTarget;
extern const int STAB;
extern float batteryVoltage;
extern float motors[4];

float lastRcLossAge = NAN;
float lastRcLossRealAge = NAN;
float lastRcLossMaxGapMs = NAN;
float lastRcLossBattery = NAN;
float lastRcLossMotorMax = NAN;
uint32_t lastRcLossFrames = 0;
uint32_t lastRcLossLostFrames = 0;
uint32_t lastRcLossFailsafeFrames = 0;
uint16_t lastRcLossUartAvailable = 0;
uint16_t lastRcLossUartMaxAvailable = 0;

// 来自 main.cpp / time.cpp
extern float t;
extern float dt;

// 内部函数前向声明
void rcLossFailsafe();
void autoFailsafe();

void failsafe() {
	rcLossFailsafe();
	autoFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
	if (t - controlTime > rcLossTimeout) {
		if (armed) lastStopReason = "RC loss";
		lastRcLossAge = t - controlTime;
		lastRcLossRealAge = getRCRealAge();
		lastRcLossMaxGapMs = rcMaxFrameGapMicros * 0.001f;
		lastRcLossBattery = batteryVoltage;
		lastRcLossMotorMax = max(max(motors[0], motors[1]), max(motors[2], motors[3]));
		lastRcLossFrames = rcFrameCount;
		lastRcLossLostFrames = rcLostFrameCount;
		lastRcLossFailsafeFrames = rcFailsafeFrameCount;
		lastRcLossUartAvailable = rcLastUartAvailable;
		lastRcLossUartMaxAvailable = rcMaxUartAvailable;
		armed = false;
		mode = STAB;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		// controls changed
		mode = STAB;
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}
