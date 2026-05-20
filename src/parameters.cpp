// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 参数存储在闪存
// Parameters storage in flash memory

#include <Arduino.h>
#include <Preferences.h>
#include "util.h"
#include "pid.h"      
#include "vector.h"   
#include "lpf.h"

// 外部变量声明
// 定义在 rc.cpp 中
extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, armedChannel, modeChannel;

// 定义在 control.cpp 中
extern PID rollRatePID;
extern PID pitchRatePID;
extern PID yawRatePID;
extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;
extern Vector maxRate;
extern float tiltMax;

// 定义在 imu.cpp 中
extern Vector accBias;
extern Vector accScale;

// 定义在 estimate.cpp 中
extern float accWeight;
extern LowPassFilter<Vector> ratesFilter; 

// 外部函数声明
extern bool motorsActive();               // 定义在 motors.cpp
extern void print(const char* format, ...); // 定义在 cli.cpp

Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	float *variable;
	float value; // cache
};

// 外部声明，从 control.cpp 引入
extern float idleThrust;
extern float idleKickThrust;
extern float idleKickTime;
extern float throttleStart;
extern float idleStickThreshold;
extern float motorTrimRL;
extern float motorTrimRR;
extern float motorTrimFR;
extern float motorTrimFL;
extern float motorMinOutput;
extern float motorTorqueRamp;
extern float motorTorqueLimit;
extern float motorYawTorqueLimit;
extern float idleYawP;
extern float idleYawD;
extern float idleYawTorqueLimit;
extern float idleYawRampTime;
extern float batteryCompEnable;
extern float batteryRefVoltage;
extern float batteryDividerScale;
extern float rcLossTimeout;
extern float pwmMidRL;
extern float pwmMidRR;
extern float pwmMidFR;
extern float pwmMidFL;
extern float pwmMaxRL;
extern float pwmMaxRR;
extern float pwmMaxFR;
extern float pwmMaxFL;
extern float pwmMinRL;
extern float pwmMinRR;
extern float pwmMinFR;
extern float pwmMinFL;

Parameter parameters[] = {
	// altitude (定高相关的参数注册，最大名字15个字符)
	{"CTL_IDLE_THR", &idleThrust},
	{"CTL_IDLE_KICK", &idleKickThrust},
	{"CTL_KICK_TIME", &idleKickTime},
	{"CTL_START_THR", &throttleStart},
	{"CTL_IDLE_STK", &idleStickThreshold},
	{"CTL_TRIM_RL", &motorTrimRL},
	{"CTL_TRIM_RR", &motorTrimRR},
	{"CTL_TRIM_FR", &motorTrimFR},
	{"CTL_TRIM_FL", &motorTrimFL},
	{"CTL_MOT_MIN", &motorMinOutput},
	{"CTL_TQ_RAMP", &motorTorqueRamp},
	{"CTL_TQ_LIMIT", &motorTorqueLimit},
	{"CTL_Y_LIM", &motorYawTorqueLimit},
	{"CTL_IDLE_Y_P", &idleYawP},
	{"CTL_IDLE_Y_D", &idleYawD},
	{"CTL_IDLE_Y_LIM", &idleYawTorqueLimit},
	{"CTL_IDLE_Y_RMP", &idleYawRampTime},
	{"CTL_BAT_COMP", &batteryCompEnable},
	{"CTL_BAT_REF", &batteryRefVoltage},
	{"CTL_BAT_SCALE", &batteryDividerScale},
	{"RC_LOSS_T", &rcLossTimeout},
	{"PWM_MID_RL", &pwmMidRL},
	{"PWM_MID_RR", &pwmMidRR},
	{"PWM_MID_FR", &pwmMidFR},
	{"PWM_MID_FL", &pwmMidFL},
	{"PWM_MIN_RL", &pwmMinRL},
	{"PWM_MIN_RR", &pwmMinRR},
	{"PWM_MIN_FR", &pwmMinFR},
	{"PWM_MIN_FL", &pwmMinFL},
	{"PWM_MAX_RL", &pwmMaxRL},
	{"PWM_MAX_RR", &pwmMaxRR},
	{"PWM_MAX_FR", &pwmMaxFR},
	{"PWM_MAX_FL", &pwmMaxFL},
	// control
	{"CTL_R_RATE_P", &rollRatePID.p},
	{"CTL_R_RATE_I", &rollRatePID.i},
	{"CTL_R_RATE_D", &rollRatePID.d},
	{"CTL_R_RATE_WU", &rollRatePID.windup},
	{"CTL_P_RATE_P", &pitchRatePID.p},
	{"CTL_P_RATE_I", &pitchRatePID.i},
	{"CTL_P_RATE_D", &pitchRatePID.d},
	{"CTL_P_RATE_WU", &pitchRatePID.windup},
	{"CTL_Y_RATE_P", &yawRatePID.p},
	{"CTL_Y_RATE_I", &yawRatePID.i},
	{"CTL_Y_RATE_D", &yawRatePID.d},
	{"CTL_R_P", &rollPID.p},
	{"CTL_R_I", &rollPID.i},
	{"CTL_R_D", &rollPID.d},
	{"CTL_P_P", &pitchPID.p},
	{"CTL_P_I", &pitchPID.i},
	{"CTL_P_D", &pitchPID.d},
	{"CTL_Y_P", &yawPID.p},
	{"CTL_P_RATE_MAX", &maxRate.y},
	{"CTL_R_RATE_MAX", &maxRate.x},
	{"CTL_Y_RATE_MAX", &maxRate.z},
	{"CTL_TILT_MAX", &tiltMax},
	// imu
	{"IMU_ACC_BIAS_X", &accBias.x},
	{"IMU_ACC_BIAS_Y", &accBias.y},
	{"IMU_ACC_BIAS_Z", &accBias.z},
	{"IMU_ACC_SCALE_X", &accScale.x},
	{"IMU_ACC_SCALE_Y", &accScale.y},
	{"IMU_ACC_SCALE_Z", &accScale.z},
	// estimate
	{"EST_ACC_WEIGHT", &accWeight},
	{"EST_RATES_LPF_A", &ratesFilter.alpha},
	// rc
	{"RC_ZERO_0", &channelZero[0]},
	{"RC_ZERO_1", &channelZero[1]},
	{"RC_ZERO_2", &channelZero[2]},
	{"RC_ZERO_3", &channelZero[3]},
	{"RC_ZERO_4", &channelZero[4]},
	{"RC_ZERO_5", &channelZero[5]},
	{"RC_ZERO_6", &channelZero[6]},
	{"RC_ZERO_7", &channelZero[7]},
	{"RC_MAX_0", &channelMax[0]},
	{"RC_MAX_1", &channelMax[1]},
	{"RC_MAX_2", &channelMax[2]},
	{"RC_MAX_3", &channelMax[3]},
	{"RC_MAX_4", &channelMax[4]},
	{"RC_MAX_5", &channelMax[5]},
	{"RC_MAX_6", &channelMax[6]},
	{"RC_MAX_7", &channelMax[7]},
	{"RC_ROLL", &rollChannel},
	{"RC_PITCH", &pitchChannel},
	{"RC_THROTTLE", &throttleChannel},
	{"RC_YAW", &yawChannel},
	{"RC_MODE", &modeChannel},
};

void setupParameters() {
	storage.begin("flix", false);
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, *parameter.variable);
		}
		*parameter.variable = storage.getFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

int getParameterIndex(const char *name) {
	for (int i = 0; i < parametersCount(); i++) {
		if (strcmp(parameters[i].name, name) == 0) return i;
	}
	return -1;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return *parameters[index].variable;
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			return *parameter.variable;
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			*parameter.variable = value;
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.value == *parameter.variable) continue;
		if (isnan(parameter.value) && isnan(*parameter.variable)) continue; // handle NAN != NAN
		storage.putFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, *parameter.variable);
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}
