// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Main firmware file 主程序文件20251226
// 源代码使用Arduino IDE 2.3.x以上版本编译，不支持老版本的1.8.19！
// 必须安装ESP32开发板核心库（即esp32 by Espressif Systems）
// 必须安装MAVLINK，FlixPeriph库文件！
// 开发板可选择ESP32 Dev Module或WeMOS D1 MINI ESP32；
// 嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane
// B站微辣火龙果 https://space.bilibili.com/544479100

#include <Arduino.h>
#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "voice/voice.h"
#include "rcwl_1605/rcwl_1605.h"

// 外部函数声明
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
void handleInput();      // cli.cpp
void processMavlink();   // mavlink.cpp
void logData();          // log.cpp
void step();             // time.cpp (计算 dt 的函数)

extern void print(const char* format, ...); 

// 全局变量定义
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

void setup() {
	Serial.begin(115200);
	print("程序初始化！\n");
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
	voice_init();
	setupRCWL1605();
	setLED(false);
	print("初始化完成！\n");
}

void loop() {
	readIMU();
	readRCWL1605();
	step();
	readRC();
	estimate();
	control();
	sendMotors();
	handleInput();
	voice_read_command();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
}
