// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Main firmware entry point.

#include <Arduino.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "vector.h"
#include "quaternion.h" 
#include "util.h"

void setupParameters();
void syncParameters();
void setupLED();
void setLED(bool on);
void setupMotors();
void sendMotors();
void setupWiFi();
void setupIMU();
void readIMU();
void setupRC();
void readRC();
void estimate();
void control();
void handleInput();
void processMavlink();
void logData();
void stepFixed(float fixedDt);

extern void print(const char* format, ...);

constexpr uint32_t FLIGHT_LOOP_HZ = 250;
constexpr float FLIGHT_LOOP_DT = 1.0f / FLIGHT_LOOP_HZ;
constexpr uint32_t SERVICE_LOOP_MS = 5;

void flightTask(void *parameter);
void runFlightStep();
void runServiceStep();
void setFlightControlPaused(bool paused);

float t = 0;
float dt = 0;
float controlRoll, controlPitch, controlYaw, controlThrottle;
float controlMode = NAN;
Vector gyro;
Vector acc;
Vector rates;
Quaternion attitude;
bool landed;
float motors[4];
volatile bool flightControlPaused = false;
float flightLoopLastIntervalMs = 0.0f;
float flightLoopMaxIntervalMs = 0.0f;
uint32_t flightLoopOverrunCount = 0;

void setup() {
	Serial.begin(115200);
	print("Program init\n");
	print("Reset reason: %d\n", esp_reset_reason());
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setupRC();
	setLED(false);

	xTaskCreatePinnedToCore(
		flightTask,
		"flight",
		4096,
		NULL,
		4,
		NULL,
		1);

	print("Init done\n");
}

void loop() {
	runServiceStep();
	vTaskDelay(pdMS_TO_TICKS(SERVICE_LOOP_MS));
}

void flightTask(void *parameter) {
	(void)parameter;
	TickType_t lastWake = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(1000 / FLIGHT_LOOP_HZ);

	for (;;) {
		vTaskDelayUntil(&lastWake, period);
		runFlightStep();
	}
}

void runFlightStep() {
	static uint32_t lastRunMicros = 0;
	const uint32_t now = micros();
	if (lastRunMicros != 0) {
		flightLoopLastIntervalMs = (now - lastRunMicros) * 0.001f;
		if (flightLoopLastIntervalMs > flightLoopMaxIntervalMs) flightLoopMaxIntervalMs = flightLoopLastIntervalMs;
		if (flightLoopLastIntervalMs > 6.0f) flightLoopOverrunCount++;
	}
	lastRunMicros = now;

	if (flightControlPaused) return;
	readIMU();
	if (flightControlPaused) return;
	stepFixed(FLIGHT_LOOP_DT);
	readRC();
	estimate();
	control();
	sendMotors();
	logData();
}

void runServiceStep() {
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	syncParameters();
}

void setFlightControlPaused(bool paused) {
	flightControlPaused = paused;
}
