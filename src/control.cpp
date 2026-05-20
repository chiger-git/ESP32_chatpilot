// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Flight attitude/rate control and motor mixing.

#include <Arduino.h>
#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

float idleThrust = 0.1f;
float idleKickThrust = 0.04f;
float idleKickTime = 0.20f;
float throttleStart = 0.35f;
float idleStickThreshold = 0.05f;
float motorTrimRL = 0.0f;
float motorTrimRR = 0.0f;
float motorTrimFR = 0.0f;
float motorTrimFL = 0.0f;
float motorMinOutput = 0.18f;
float motorTorqueRamp = 0.15f;
float motorTorqueLimit = 0.25f;
float motorYawTorqueLimit = 0.0f;
float idleYawP = 0.010f;
float idleYawD = 0.008f;
float idleYawTorqueLimit = 0.010f;
float idleYawRampTime = 0.80f;
float batteryCompEnable = 1.0f;
float batteryRefVoltage = 3.80f;
float batteryDividerScale = 43.0f / 33.0f;
float batteryVoltage = NAN;
float batteryVoltageMin = NAN;

extern Quaternion attitude;
extern Vector rates;
extern float motors[4];
extern float t;
extern float dt;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;

void interpretControls();
void failsafe();
void controlAttitude();
void controlRates();
void controlTorque();
float getTorqueScale();
bool isIdleThrottle();
float getBatteryCompensationScale(bool allowBoost = true);
float getMotorSpinMin();
Vector limitTorqueForMotorRange(const Vector& torque, float baseThrust, float minOutput);
void resetRateIntegrators();
void updateBatteryVoltage();
float getIdleYawTorque();
void resetIdleYawHold(float startTime = NAN);
float limitIdleYawTorque(float yawTorque, float baseRL, float baseRR, float baseFR, float baseFL);
void applyMotorIdleOutput(float extraThrust = 0.0f, float yawTorque = 0.0f);
void applyMotorOutputAdjustments(float minOutput);

#define PITCHRATE_P 0.05f
#define PITCHRATE_I 0.2f
#define PITCHRATE_D 0.001f
#define PITCHRATE_I_LIM 0.35f
#define ROLLRATE_P 0.06f
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D 0.02f
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.4f
#define YAWRATE_I 0.01f
#define YAWRATE_D 0.0f
#define YAWRATE_I_LIM 0.3f

#define ROLL_P 7.0f
#define ROLL_I 0.0f
#define ROLL_D 0.0f
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P 3.0f

#define PITCHRATE_MAX radians(360)
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300)
#define TILT_MAX radians(30)
#define RATES_D_LPF_ALPHA 0.2f
#define ARM_GESTURE_HOLD 0.25f
#define DISARM_GESTURE_HOLD 0.06f

extern const int RAW = 0;
extern const int ACRO = 1;
extern const int STAB = 2;
extern const int AUTO = 3;

int mode = STAB;
bool armed = false;
const char *lastStopReason = "not armed";

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra;
Vector torqueTarget;
float thrustTarget;
float idleYawTarget = NAN;
float idleYawControlStart = 0.0f;

void control() {
	interpretControls();
	failsafe();
	updateBatteryVoltage();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	static float armGestureStart = NAN;
	static float disarmGestureStart = NAN;
	const bool throttleValid = valid(controlThrottle);
	if (invalid(controlRoll)) controlRoll = 0.0f;
	if (invalid(controlPitch)) controlPitch = 0.0f;
	if (invalid(controlYaw)) controlYaw = 0.0f;
	if (invalid(controlThrottle)) controlThrottle = 0.0f;
	if (!throttleValid) {
		if (armed) lastStopReason = "RC throttle invalid";
		armed = false;
	}
	controlThrottle = constrain(controlThrottle, 0.0f, 1.0f);
	if (mode != STAB) mode = STAB;

	const bool armGesture = controlThrottle < 0.05f && controlYaw > 0.95f;
	const bool disarmGesture = controlThrottle < 0.05f && controlYaw < -0.95f;

	if (armGesture) {
		if (!isfinite(armGestureStart)) armGestureStart = t;
		if (t - armGestureStart >= ARM_GESTURE_HOLD) armed = true;
	} else {
		armGestureStart = NAN;
	}

	if (disarmGesture) {
		if (!isfinite(disarmGestureStart)) disarmGestureStart = t;
		if (t - disarmGestureStart >= DISARM_GESTURE_HOLD) {
			if (armed) lastStopReason = "RC disarm gesture";
			armed = false;
		}
	} else {
		disarmGestureStart = NAN;
	}

	if (abs(controlYaw) < 0.1f) controlYaw = 0.0f;

	thrustTarget = controlThrottle;
	if (invalid(thrustTarget)) thrustTarget = 0.0f;

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0.0f || isIdleThrottle()) {
			yawTarget = attitude.getYaw();
		}
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z);
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate();
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z;
	}

	if (mode == RAW) {
		attitudeTarget.invalidate();
		ratesTarget.invalidate();
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1f;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || isIdleThrottle()) {
		rollPID.reset();
		pitchPID.reset();
		yawPID.reset();
		ratesTarget.invalidate();
		return;
	}

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
	if (mode == RAW) return;

	const float torqueScale = getTorqueScale();
	if (!armed || ratesTarget.invalid() || isIdleThrottle() || torqueScale <= 0.0f) {
		rollRatePID.reset();
		pitchRatePID.reset();
		yawRatePID.reset();
		torqueTarget = Vector(0, 0, 0);
		return;
	}
	if (torqueScale < 0.99f) resetRateIntegrators();

	Vector error = ratesTarget - rates;

	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
	torqueTarget.x = constrain(torqueTarget.x, -motorTorqueLimit, motorTorqueLimit) * torqueScale;
	torqueTarget.y = constrain(torqueTarget.y, -motorTorqueLimit, motorTorqueLimit) * torqueScale;
	torqueTarget.z = constrain(torqueTarget.z, -motorYawTorqueLimit, motorYawTorqueLimit) * torqueScale;
}

void controlTorque() {
	static bool wasArmed = false;
	static float idleKickUntil = 0.0f;

	if (!torqueTarget.valid()) return;

	if (!armed) {
		memset(motors, 0, sizeof(motors));
		wasArmed = false;
		resetIdleYawHold();
		return;
	}

	if (!wasArmed) {
		idleKickUntil = t + max(idleKickTime, 0.0f);
		resetIdleYawHold(idleKickUntil);
		wasArmed = true;
	}

	if (isIdleThrottle()) {
		float extra = t < idleKickUntil ? max(idleKickThrust, 0.0f) : 0.0f;
		applyMotorIdleOutput(extra, getIdleYawTorque());
		return;
	}

	const float spinMin = getMotorSpinMin();
	const float baseThrust = max(thrustTarget, spinMin);
	Vector limitedTorque = limitTorqueForMotorRange(torqueTarget, baseThrust, spinMin);

	motors[MOTOR_FRONT_LEFT] = baseThrust + limitedTorque.x - limitedTorque.y + limitedTorque.z;
	motors[MOTOR_FRONT_RIGHT] = baseThrust - limitedTorque.x - limitedTorque.y - limitedTorque.z;
	motors[MOTOR_REAR_LEFT] = baseThrust + limitedTorque.x + limitedTorque.y - limitedTorque.z;
	motors[MOTOR_REAR_RIGHT] = baseThrust - limitedTorque.x + limitedTorque.y + limitedTorque.z;

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
	applyMotorOutputAdjustments(getMotorSpinMin());
}

float getTorqueScale() {
	if (motorTorqueRamp <= 0.0f) return 1.0f;
	return constrain((thrustTarget - getMotorSpinMin()) / motorTorqueRamp, 0.0f, 1.0f);
}

bool isIdleThrottle() {
	return invalid(thrustTarget) || thrustTarget <= constrain(idleStickThreshold, 0.0f, 0.25f);
}

float getIdleYawTorque() {
	const float currentYaw = attitude.getYaw();
	if (!isfinite(currentYaw) || !isfinite(rates.z)) return 0.0f;
	if (idleYawTorqueLimit <= 0.0f) {
		idleYawTarget = currentYaw;
		return 0.0f;
	}

	if (abs(controlYaw) > 0.10f) {
		idleYawTarget = currentYaw;
		return 0.0f;
	}

	if (!isfinite(idleYawTarget)) idleYawTarget = currentYaw;

	const float rampTime = max(idleYawRampTime, 0.0f);
	const float ramp = rampTime > 0.0f ? constrain((t - idleYawControlStart) / rampTime, 0.0f, 1.0f) : 1.0f;
	if (ramp <= 0.0f) return 0.0f;

	const float yawError = wrapAngle(idleYawTarget - currentYaw);
	const float yawTorque = yawError * idleYawP - rates.z * idleYawD;
	return constrain(yawTorque, -idleYawTorqueLimit, idleYawTorqueLimit) * ramp;
}

void resetIdleYawHold(float startTime) {
	const float currentYaw = attitude.getYaw();
	idleYawTarget = isfinite(currentYaw) ? currentYaw : NAN;
	idleYawControlStart = isfinite(startTime) ? startTime : t;
}

float limitIdleYawTorque(float yawTorque, float baseRL, float baseRR, float baseFR, float baseFL) {
	const float minYaw = max(max(baseRL - 1.0f, baseFR - 1.0f), max(-baseRR, -baseFL));
	const float maxYaw = min(min(baseRL, baseFR), min(1.0f - baseRR, 1.0f - baseFL));
	if (minYaw > maxYaw) return 0.0f;
	return constrain(yawTorque, minYaw, maxYaw);
}

float getBatteryCompensationScale(bool allowBoost) {
	updateBatteryVoltage();
	if (batteryCompEnable < 0.5f) return 1.0f;
	if (!isfinite(batteryVoltage) || batteryVoltage < 2.5f || batteryVoltage > 5.0f) return 1.0f;

	const float maxScale = allowBoost ? 1.20f : 1.0f;
	return constrain(batteryRefVoltage / batteryVoltage, 0.60f, maxScale);
}

float getMotorSpinMin() {
	return constrain(max(motorMinOutput, idleThrust), 0.0f, 1.0f);
}

Vector limitTorqueForMotorRange(const Vector& torque, float baseThrust, float minOutput) {
	if (torque.invalid()) return Vector(0, 0, 0);

	float scale = 1.0f;
	const float lowRoom = max(baseThrust - minOutput, 0.0f);
	const float highRoom = max(1.0f - baseThrust, 0.0f);
	const float deviations[4] = {
		torque.x - torque.y + torque.z,
		-torque.x - torque.y - torque.z,
		torque.x + torque.y - torque.z,
		-torque.x + torque.y + torque.z,
	};

	for (float deviation : deviations) {
		if (deviation < 0.0f) {
			scale = min(scale, lowRoom / -deviation);
		} else if (deviation > 0.0f) {
			scale = min(scale, highRoom / deviation);
		}
	}

	return torque * constrain(scale, 0.0f, 1.0f);
}

void resetRateIntegrators() {
	rollRatePID.integral = 0.0f;
	pitchRatePID.integral = 0.0f;
	yawRatePID.integral = 0.0f;
}

void updateBatteryVoltage() {
	const int BATTERY_ADC_PIN = 36; // ESP32 VP / ADC1_CH0, net name ADC_1
	static bool configured = false;
	static Rate rate(20);

	if (!configured) {
		analogReadResolution(12);
		analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
		configured = true;
	}
	if (!rate) return;

	const float measured = analogReadMilliVolts(BATTERY_ADC_PIN) * 0.001f * batteryDividerScale;
	if (measured < 2.0f || measured > 5.5f) return;

	if (!isfinite(batteryVoltage)) {
		batteryVoltage = measured;
	} else {
		batteryVoltage = batteryVoltage * 0.90f + measured * 0.10f;
	}
	if (!isfinite(batteryVoltageMin) || batteryVoltage < batteryVoltageMin) batteryVoltageMin = batteryVoltage;
}

void applyMotorIdleOutput(float extraThrust, float yawTorque) {
	const float scale = getBatteryCompensationScale(false);
	const float baseRL = idleThrust + extraThrust + motorTrimRL;
	const float baseRR = idleThrust + extraThrust + motorTrimRR;
	const float baseFR = idleThrust + extraThrust + motorTrimFR;
	const float baseFL = idleThrust + extraThrust + motorTrimFL;
	const float yaw = limitIdleYawTorque(yawTorque, baseRL, baseRR, baseFR, baseFL);
	const float minIdleOutput = getMotorSpinMin();

	motors[MOTOR_REAR_LEFT] = constrain((baseRL - yaw) * scale, minIdleOutput, 1.0f);
	motors[MOTOR_REAR_RIGHT] = constrain((baseRR + yaw) * scale, minIdleOutput, 1.0f);
	motors[MOTOR_FRONT_RIGHT] = constrain((baseFR - yaw) * scale, minIdleOutput, 1.0f);
	motors[MOTOR_FRONT_LEFT] = constrain((baseFL + yaw) * scale, minIdleOutput, 1.0f);
}

void applyMotorOutputAdjustments(float minOutput) {
	const float scale = getBatteryCompensationScale();
	const float minFinalOutput = constrain(minOutput, 0.0f, 1.0f);

	motors[MOTOR_REAR_LEFT] = constrain((motors[MOTOR_REAR_LEFT] + motorTrimRL) * scale, minFinalOutput, 1.0f);
	motors[MOTOR_REAR_RIGHT] = constrain((motors[MOTOR_REAR_RIGHT] + motorTrimRR) * scale, minFinalOutput, 1.0f);
	motors[MOTOR_FRONT_RIGHT] = constrain((motors[MOTOR_FRONT_RIGHT] + motorTrimFR) * scale, minFinalOutput, 1.0f);
	motors[MOTOR_FRONT_LEFT] = constrain((motors[MOTOR_FRONT_LEFT] + motorTrimFL) * scale, minFinalOutput, 1.0f);
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
