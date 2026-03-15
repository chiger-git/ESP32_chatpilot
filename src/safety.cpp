// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 故障安全保护功能
// Fail-safe functions

#include <Arduino.h>
#include "quaternion.h"
#include "vector.h"

#define RC_LOSS_TIMEOUT 1.0f // 遥控丢失超时 (秒)
#define DESCEND_TIME 10.0f   // 自动降落耗时 (秒)

// 外部变量声明 (引用其他文件定义的变量)
// 来自 rc.cpp
extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

// 来自 control.cpp
extern bool armed;
extern int mode;
extern Quaternion attitudeTarget;
extern float thrustTarget;
extern const int AUTO; // 引用模式常量
extern const int STAB;

// 来自 main.cpp / time.cpp
extern float t;
extern float dt;

// 内部函数前向声明
void rcLossFailsafe();
void autoFailsafe();
void descend();

void failsafe() {
	rcLossFailsafe();
	autoFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
	if (t - controlTime > RC_LOSS_TIMEOUT) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {
	mode = AUTO;
	attitudeTarget = Quaternion();
	thrustTarget -= dt / DESCEND_TIME;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		// controls changed
		if (mode == AUTO) mode = STAB; // regain control by the pilot
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}
