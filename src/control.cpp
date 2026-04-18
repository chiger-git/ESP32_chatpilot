// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 飞行控制Flight control，注释修改：B站微辣火龙果 https://space.bilibili.com/544479100
// 嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane

#include <Arduino.h>
#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

// 高度闭环数据与 PID 初始化
extern float alt_est; 
extern float vz_est;
bool alt_hold_enabled = false;  // 定高开关
float alt_target = 0.0f;        // 目标高度 (m)
float hoverThrottle = 0.35f;    // 基础悬停油门

PID altPid(1.0f, 0.0f, 0.0f);   // 外部位置环 (高度误差 -> 目标爬升速度)
PID velZPid(0.1f, 0.01f, 0.0f); // 内部速度环 (速度误差 -> 增补油门)

extern Quaternion attitude;
extern Vector rates;
extern float motors[4];
extern float t; // 时间

// 外部变量与函数声明
// 来自 main.cpp
extern Quaternion attitude;
extern Vector rates;
extern float motors[4];
extern float t; // 时间
extern float dt; // 时间步长，给PID积分和微分用
// 来自 rc.cpp
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
// 来自 motors.cpp (电机索引常量)
extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;

// 内部函数前向声明
void interpretControls();
void failsafe(); // 原代码漏了这个函数的实现？如果没实现需要删掉或者补上
void controlAttitude();
void controlRates();
void controlTorque();

// ============== 角速率环（内环）参数 ==============
#define PITCHRATE_P 0.05 // 增大P值提高响应速度
#define PITCHRATE_I 0.2 // 中等I值补偿电机差异
#define PITCHRATE_D 0.001 // 小D值抑制震荡
#define PITCHRATE_I_LIM 0.35 // 限制积分积累
#define ROLLRATE_P 0.06 // 横滚和俯仰使用相同参数
#define ROLLRATE_I PITCHRATE_I 
#define ROLLRATE_D 0.02 
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.4 // 偏航需要更高的P值（惯性较小）
#define YAWRATE_I 0.01 // 中等I值补偿
#define YAWRATE_D 0.01 // 小D值
#define YAWRATE_I_LIM 0.3
// ============== 角度环（外环）参数 ==============
#define ROLL_P 7 // 较高的P值快速响应
#define ROLL_I 0 // 角度环通常不需要I项
#define ROLL_D 0 // 角度环通常不需要D项
#define PITCH_P ROLL_P // 横滚和俯仰相同
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3 // 偏航响应稍慢

// ============== 限制值 ==============
#define PITCHRATE_MAX radians(360) // 高转速限制（1000°/s）
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300) // 偏航转速稍低
#define TILT_MAX radians(30) // 最大倾斜角30°
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

// 飞行模式常量
extern const int RAW = 0;
extern const int ACRO = 1;
extern const int STAB = 2;
extern const int AUTO = 3;

int mode = STAB;
bool armed = false;

// PID 对象实例化
PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

// 目标状态量
Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;

void control() {
	interpretControls();
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = STAB;
	if (controlMode < 0.75) mode = STAB;
	if (controlMode > 0.75) mode = STAB;

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

	if (controlThrottle < 0.05 && controlYaw > 0.95) armed = true; // arm gesture
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

// 先把高度环注释掉了，后面有需要再加
	// ====================== Altitude Hold Core ======================
	if (false && alt_hold_enabled) {
		// Panic switch: Exit altitude hold if throttle is pulled down or drone is disarmed
		if (controlThrottle < 0.05f || !armed) {
			alt_hold_enabled = false;
		} 
		else {
			// Position outer loop: calculate target vertical velocity
			float pos_err = alt_target - alt_est;                  
			float target_speed = altPid.update(pos_err);       
			target_speed = constrain(target_speed, -0.6f, 0.6f);   // Limit climb/descent rate

			// Velocity inner loop: calculate throttle adjustment
			float speed_err = target_speed - vz_est;               
			float hover_adj = velZPid.update(speed_err);       

			// Feedforward base + PID compensation
			thrustTarget = hoverThrottle + hover_adj;
			thrustTarget = constrain(thrustTarget, 0.1f, 0.8f);    // Keep motors spinning but prevent aggressive saturation
		}
	} 

	// Revert to manual if disabled
	if (!alt_hold_enabled) {
		thrustTarget = controlThrottle;
		
		// Reset PIDs to prevent integral windup on re-engagement
		altPid.reset();
		velZPid.reset();
	}
	// ==============================================================

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < 0.1) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < 0.1) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < 0.1) {
		motors[0] = 0.1; // idle thrust
		motors[1] = 0.1;
		motors[2] = 0.1;
		motors[3] = 0.1;
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

const char* getModeName() {
	switch (mode) {
		case RAW: return "RAW";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		case AUTO: return "AUTO";
		default: return "UNKNOWN";
	}
}
