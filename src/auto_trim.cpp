// Ground idle motor trim calibration from gyro rates.

#include <Arduino.h>
#include "vector.h"
#include "util.h"

extern Vector gyro;
extern bool armed;
extern bool landed;
extern float motors[4];
extern float idleThrust;
extern float idleKickThrust;
extern float idleKickTime;
extern float motorTrimRL;
extern float motorTrimRR;
extern float motorTrimFR;
extern float motorTrimFL;
extern float controlThrottle;
extern float controlYaw;
extern const int MOTOR_REAR_LEFT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_FRONT_LEFT;

extern void print(const char* format, ...);
bool readRC();
void readIMU();
void sendMotors();
void stopMotors();
void setFlightControlPaused(bool paused);
float getBatteryCompensationScale(bool allowBoost = true);

static constexpr int OFF_SAMPLES = 150;
static constexpr int SETTLE_MS = 900;
static constexpr int SAMPLE_COUNT = 120;
static constexpr int MAX_ITERATIONS = 28;
static constexpr int MIN_ACTIVE_ITERATIONS = 3;
static constexpr int STABLE_ITERATIONS = 4;
static constexpr float SAMPLE_DELAY_MS = 4.0f;
static constexpr float TRIM_LIMIT = 0.070f;
static constexpr float STEP_LIMIT = 0.0035f;
static constexpr float RATE_ABORT = 1.6f;
static constexpr float ROLL_PITCH_TARGET = 0.09f;
static constexpr float YAW_TARGET = 0.07f;
static constexpr float ROLL_PITCH_GAIN = 0.010f;
static constexpr float YAW_GAIN = 0.012f;

bool sampleGyroAverage(int samples, Vector& average);
bool autoTrimAbortRequested();
bool waitWithAbortCheck(uint32_t durationMs);
void writeIdleWithCurrentTrims(float extraThrust = 0.0f);
void applyTrimCorrection(const Vector& rate);
float clampStep(float value);
float trimMean();
void preserveTrimMean(float targetMean);
void printTrimState(const char *prefix);

void autoTrimMotors() {
	print("Auto trim: place aircraft flat on ground, keep sticks low, keep clear of props.\n");
	print("Auto trim: use the normal RC lock gesture to abort immediately.\n");

	const float startRL = motorTrimRL;
	const float startRR = motorTrimRR;
	const float startFR = motorTrimFR;
	const float startFL = motorTrimFL;

	armed = false;
	setFlightControlPaused(true);
	delay(30);
	stopMotors();

	const float targetMean = trimMean();
	Vector gyroZero(0, 0, 0);
	int stable = 0;
	bool aborted = false;
	bool manualAbort = false;
	bool converged = false;

	landed = false;
	if (!sampleGyroAverage(OFF_SAMPLES, gyroZero)) {
		manualAbort = true;
		aborted = true;
	}

	if (!aborted) {
		print("Auto trim: gyro zero %.4f %.4f %.4f rad/s\n", gyroZero.x, gyroZero.y, gyroZero.z);
		printTrimState("Auto trim start");

		float kick = max(idleKickThrust, 0.0f);
		uint32_t kickMs = (uint32_t)(max(idleKickTime, 0.0f) * 1000.0f);
		print("Auto trim: spin-up pulse extra %.3f for %u ms\n", kick, kickMs);
		writeIdleWithCurrentTrims(kick);
		if (!waitWithAbortCheck(kickMs)) {
			manualAbort = true;
			aborted = true;
		}
	}

	if (!aborted) {
		writeIdleWithCurrentTrims();
		if (!waitWithAbortCheck(SETTLE_MS)) {
			manualAbort = true;
			aborted = true;
		}
	}

	for (int i = 0; !aborted && i < MAX_ITERATIONS; i++) {
		landed = false;
		Vector gyroAverage(0, 0, 0);
		if (!sampleGyroAverage(SAMPLE_COUNT, gyroAverage)) {
			manualAbort = true;
			aborted = true;
			break;
		}
		Vector rate = gyroAverage - gyroZero;

		if (abs(rate.x) > RATE_ABORT || abs(rate.y) > RATE_ABORT || abs(rate.z) > RATE_ABORT) {
			print("Auto trim abort: movement too large %.3f %.3f %.3f rad/s\n", rate.x, rate.y, rate.z);
			aborted = true;
			break;
		}

		print("Auto trim iter %02d: rate roll %.3f pitch %.3f yaw %.3f rad/s\n", i + 1, rate.x, rate.y, rate.z);

		if (i >= MIN_ACTIVE_ITERATIONS &&
			abs(rate.x) < ROLL_PITCH_TARGET && abs(rate.y) < ROLL_PITCH_TARGET && abs(rate.z) < YAW_TARGET) {
			stable++;
			if (stable >= STABLE_ITERATIONS) {
				converged = true;
				break;
			}
		} else {
			stable = 0;
		}

		applyTrimCorrection(rate);
		preserveTrimMean(targetMean);
		printTrimState("Auto trim");
		writeIdleWithCurrentTrims();
		if (!waitWithAbortCheck(SETTLE_MS)) {
			manualAbort = true;
			aborted = true;
			break;
		}
	}

	stopMotors();
	armed = false;
	setFlightControlPaused(false);

	if (aborted) {
		motorTrimRL = startRL;
		motorTrimRR = startRR;
		motorTrimFR = startFR;
		motorTrimFL = startFL;
		if (manualAbort) {
			print("Auto trim aborted by RC lock gesture. Old trims restored.\n");
		} else {
			print("Auto trim stopped and restored old trims. Check aircraft movement, prop direction, and motor order before retrying.\n");
		}
	} else if (!converged) {
		printTrimState("Auto trim max-iter");
		print("Auto trim reached max iterations. Verify idle behavior before flight.\n");
	} else {
		printTrimState("Auto trim done");
		print("Auto trim done. Parameters will be saved after motors remain stopped.\n");
	}
}

bool sampleGyroAverage(int samples, Vector& average) {
	Vector sum(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		if (autoTrimAbortRequested()) return false;
		landed = false; // do not adapt gyro bias while measuring motor-induced motion
		readIMU();
		sum = sum + gyro;
		delay((uint32_t)SAMPLE_DELAY_MS);
	}
	average = sum / samples;
	return true;
}

bool autoTrimAbortRequested() {
	readRC();
	if (invalid(controlThrottle) || invalid(controlYaw)) return false;
	return controlThrottle < 0.05f && controlYaw < -0.95f;
}

bool waitWithAbortCheck(uint32_t durationMs) {
	if (autoTrimAbortRequested()) return false;

	uint32_t start = millis();
	while ((uint32_t)(millis() - start) < durationMs) {
		if (autoTrimAbortRequested()) return false;
		delay(20);
	}

	return !autoTrimAbortRequested();
}

void writeIdleWithCurrentTrims(float extraThrust) {
	const float scale = getBatteryCompensationScale(false);
	motors[MOTOR_REAR_LEFT] = constrain((idleThrust + extraThrust + motorTrimRL) * scale, 0.0f, 1.0f);
	motors[MOTOR_REAR_RIGHT] = constrain((idleThrust + extraThrust + motorTrimRR) * scale, 0.0f, 1.0f);
	motors[MOTOR_FRONT_RIGHT] = constrain((idleThrust + extraThrust + motorTrimFR) * scale, 0.0f, 1.0f);
	motors[MOTOR_FRONT_LEFT] = constrain((idleThrust + extraThrust + motorTrimFL) * scale, 0.0f, 1.0f);
	sendMotors();
}

void applyTrimCorrection(const Vector& rate) {
	Vector torque(-rate.x * ROLL_PITCH_GAIN, -rate.y * ROLL_PITCH_GAIN, -rate.z * YAW_GAIN);

	float dFL = clampStep(+torque.x - torque.y + torque.z);
	float dFR = clampStep(-torque.x - torque.y - torque.z);
	float dRL = clampStep(+torque.x + torque.y - torque.z);
	float dRR = clampStep(-torque.x + torque.y + torque.z);

	motorTrimFL = constrain(motorTrimFL + dFL, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimFR = constrain(motorTrimFR + dFR, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimRL = constrain(motorTrimRL + dRL, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimRR = constrain(motorTrimRR + dRR, -TRIM_LIMIT, TRIM_LIMIT);
}

float clampStep(float value) {
	return constrain(value, -STEP_LIMIT, STEP_LIMIT);
}

float trimMean() {
	return (motorTrimRL + motorTrimRR + motorTrimFR + motorTrimFL) * 0.25f;
}

void preserveTrimMean(float targetMean) {
	float delta = trimMean() - targetMean;
	motorTrimRL = constrain(motorTrimRL - delta, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimRR = constrain(motorTrimRR - delta, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimFR = constrain(motorTrimFR - delta, -TRIM_LIMIT, TRIM_LIMIT);
	motorTrimFL = constrain(motorTrimFL - delta, -TRIM_LIMIT, TRIM_LIMIT);
}

void printTrimState(const char *prefix) {
	print("%s: RL %.4f RR %.4f FR %.4f FL %.4f\n",
		prefix, motorTrimRL, motorTrimRR, motorTrimFR, motorTrimFL);
}
