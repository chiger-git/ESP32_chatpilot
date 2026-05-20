// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 使用MOSFET的电机输出控制
// 如果使用ESC，将PWM_STOP、PWM_MIN和PWM_MAX更改为适当的μs值，将PWM_FREQUENCY减小到400
// Motors output control using MOSFETs
// In case of using ESCs, change PWM_STOP, PWM_MIN and PWM_MAX to appropriate values in μs, decrease PWM_FREQUENCY (to 400)
#include <Arduino.h>
#include "util.h"

extern float motors[];
extern void print(const char* format, ...);
extern void pause(float duration);
extern bool armed;
void setFlightControlPaused(bool paused);

float pwmMidRL = 0.50f;
float pwmMidRR = 0.50f;
float pwmMidFR = 0.50f;
float pwmMidFL = 0.50f;
float pwmMaxRL = 1.00f;
float pwmMaxRR = 1.00f;
float pwmMaxFR = 1.00f;
float pwmMaxFL = 1.00f;
float pwmMinRL = 0.00f;
float pwmMinRR = 0.00f;
float pwmMinFR = 0.00f;
float pwmMinFL = 0.00f;

#define MOTOR_0_PIN 12 // RL左后rear left
#define MOTOR_1_PIN 13 // RR右后rear right
#define MOTOR_2_PIN 15 // FR右前front right
#define MOTOR_3_PIN 14 // FL左前front left

extern const int MOTOR_REAR_LEFT = 0;
extern const int MOTOR_REAR_RIGHT = 1;
extern const int MOTOR_FRONT_RIGHT = 2;
extern const int MOTOR_FRONT_LEFT = 3;

#define PWM_FREQUENCY 78000
#define PWM_RESOLUTION 10
#define PWM_STOP 0
#define PWM_MIN 0
#define PWM_MAX 1000000 / PWM_FREQUENCY

void sendMotors();
float mapMotorOutput(float value, float minOutput, float mid, float maxOutput);
float getMappedMotorOutput(int motorId, float value);
void setupMotors() {
	print("Setup Motors\n");

	// configure pins
	ledcAttach(MOTOR_0_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcAttach(MOTOR_3_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

	sendMotors();
	print("Motors initialized\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0, 1);
	float pwm = mapf(value, 0, 1, PWM_MIN, PWM_MAX);
	if (value == 0) pwm = PWM_STOP;
	float duty = mapf(pwm, 0, 1000000 / PWM_FREQUENCY, 0, (1 << PWM_RESOLUTION) - 1);
	return round(duty);
}

float mapMotorOutput(float value, float minOutput, float mid, float maxOutput) {
	if (value <= 0.0f) return 0.0f;
	value = constrain(value, 0.0f, 1.0f);
	const float lowOutput = constrain(minOutput, 0.0f, 1.0f);
	const float midOutput = max(lowOutput, constrain(mid, 0.0f, 1.0f));
	const float topOutput = max(midOutput, constrain(maxOutput, 0.0f, 1.0f));

	if (value <= 0.5f) return lowOutput + value * 2.0f * (midOutput - lowOutput);
	return midOutput + (value - 0.5f) * 2.0f * (topOutput - midOutput);
}

float getMappedMotorOutput(int motorId, float value) {
	switch (motorId) {
		case MOTOR_REAR_LEFT: return mapMotorOutput(value, pwmMinRL, pwmMidRL, pwmMaxRL);
		case MOTOR_REAR_RIGHT: return mapMotorOutput(value, pwmMinRR, pwmMidRR, pwmMaxRR);
		case MOTOR_FRONT_RIGHT: return mapMotorOutput(value, pwmMinFR, pwmMidFR, pwmMaxFR);
		case MOTOR_FRONT_LEFT: return mapMotorOutput(value, pwmMinFL, pwmMidFL, pwmMaxFL);
		default: return 0.0f;
	}
}

void sendMotors() {
	ledcWrite(MOTOR_0_PIN, getDutyCycle(getMappedMotorOutput(MOTOR_REAR_LEFT, motors[MOTOR_REAR_LEFT])));
	ledcWrite(MOTOR_1_PIN, getDutyCycle(getMappedMotorOutput(MOTOR_REAR_RIGHT, motors[MOTOR_REAR_RIGHT])));
	ledcWrite(MOTOR_2_PIN, getDutyCycle(getMappedMotorOutput(MOTOR_FRONT_RIGHT, motors[MOTOR_FRONT_RIGHT])));
	ledcWrite(MOTOR_3_PIN, getDutyCycle(getMappedMotorOutput(MOTOR_FRONT_LEFT, motors[MOTOR_FRONT_LEFT])));
}

void stopMotors() {
	memset(motors, 0, sizeof(float) * 4);
	sendMotors();
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n) {
	if (n < 0 || n > 3) return;
	print("Testing motor %d\n", n);
	armed = false;
	setFlightControlPaused(true);
	delay(20);
	stopMotors();
	motors[n] = 1;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	armed = false;
	setFlightControlPaused(false);
	print("Done\n");
}
