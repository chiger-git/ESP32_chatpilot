// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 命令行界面的实现，注释修改：B站微辣火龙果 https://space.bilibili.com/544479100   
// 嘉立创开源项目：ESP32迷你无人机 https://oshwhub.com/malagis/esp32-mini-plane
// Implementation of command line interface 

#include <Arduino.h>
#include "pid.h"
#include "vector.h"
#include "util.h"
#include "quaternion.h"

// 这些函数定义在其他 .cpp 文件中，必须在这里声明才能调用
void step();
void processMavlink();
void mavlinkPrint(const char* str);
void printParameters();
float getParameter(const char* name);
bool setParameter(const char* name, float value);
void resetParameters();
void printIMUInfo();
void printIMUCalibration();
const char* getModeName();
void printWiFiInfo();
void printLogHeader();
void printLogData();
void calibrateRC();
void calibrateAccel();
void testMotor(int motorId);
void handleInput();

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern const int RAW, ACRO, STAB, AUTO;
extern float t, dt, loopRate;
extern uint16_t channels[16];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern int mode;
extern bool armed;
extern Quaternion attitude;
extern bool landed;

const char* motd =
"\nWelcome to\n"
"                                      \n"
"    /\\      /\\      /\\      /\\    \n"
"   /  \\    /  \\    /  \\    /  \\   \n"
"  /__M_\\  /__i_\\  /__n_\\  /__i_\\  \n"
" /   F  \\/   L  \\/   i  \\/   x  \\ \n"
"/________\\_______\\_______\\_______\\ \n"
"                                       \n"
"输入命令，然后回车:\n"
"help - 帮助show help\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"preset - 重置参数reset\n"
"mfr, mfl, mrr, mrl - 测试马达 (为了安全不要装桨叶！！！)\n"
"cr - 校准RC遥控calibrate RC\n"
"ca - 校准陀螺仪加速度calibrate accel\n"
"ps - 显示pitch/roll/yaw姿态\n"
"rc - 显示RC遥控数据\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间信息\n"
"wifi - 显示Wi-Fi显示\n"
"mot - 显示motor输出\n"
"sys - 显示系统info信息\n"
"raw/stab/acro/auto - 飞行模式设定\n"
"arm - 解锁无人机arm\n"
"disarm - 锁定无人机disarm\n"
"log [dump] - 打印日志\n"
"reset - 重置无人机状态\n"
"reboot - 重启无人机\n";

extern float motors[];

void print(const char* format, ...) {
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	Serial.print(buf);
#if WIFI_ENABLED
	mavlinkPrint(buf);
#endif
}

void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		step();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
		delay(50);
	}
}

void doCommand(String str, bool echo = false) {
	// parse command
	String command, arg0, arg1;
	splitString(str, command, arg0, arg1);
	if (command.isEmpty()) return;

	// echo command
	if (echo) {
		print("> %s\n", str.c_str());
	}

	command.toLowerCase();

	// execute command
	if (command == "help" || command == "motd") {
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {
		print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {
			print("%s = %g\n", arg0.c_str(), arg1.toFloat());
		} else {
			print("Parameter not found: %s\n", arg0.c_str());
		}
	} else if (command == "preset") {
		resetParameters();
	} else if (command == "time") {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (command == "ps") {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (command == "imu") {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (command == "arm") {
		armed = true;
	} else if (command == "disarm") {
		armed = false;
	} else if (command == "raw") {
		mode = RAW;
	} else if (command == "stab") {
		mode = STAB;
	} else if (command == "acro") {
		mode = ACRO;
	} else if (command == "auto") {
		mode = AUTO;
	} else if (command == "rc") {
		print("channels: ");
		for (int i = 0; i < 16; i++) {
			print("%u ", channels[i]);
		}
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
	} else if (command == "wifi") {
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (command == "mot") {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "log") {
		printLogHeader();
		if (arg0 == "dump") printLogData();
	} else if (command == "cr") {
		calibrateRC();
	} else if (command == "ca") {
		calibrateAccel();
	} else if (command == "mfr") {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {
		testMotor(MOTOR_REAR_LEFT);
	} else if (command == "sys") {
#ifdef ESP32
		print("Chip: %s\n", ESP.getChipModel());
		print("Temperature: %.1f °C\n", temperatureRead());
		print("Free heap: %d\n", ESP.getFreeHeap());
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		// 在 Core 3.x 中，FreeRTOS 的追踪功能默认关闭，无法获取任务列表
		print("Task list tracking is disabled in current ESP32 Core version.\n");
#else
		// 旧版本的任务列表打印逻辑保留在这里
		print("Num  Task                Stack  Prio  Core  CPU%%\n");
		int taskCount = uxTaskGetNumberOfTasks();
		TaskStatus_t *systemState = new TaskStatus_t[taskCount];
		uint32_t totalRunTime;
		uxTaskGetSystemState(systemState, taskCount, &totalRunTime);
		for (int i = 0; i < taskCount; i++) {
			String core = systemState[i].xCoreID == tskNO_AFFINITY ? "*" : String(systemState[i].xCoreID);
			int cpuPercentage = systemState[i].ulRunTimeCounter / (totalRunTime / 100);
			print("%-5d%-20s%-7d%-6d%-6s%d\n",systemState[i].xTaskNumber, systemState[i].pcTaskName,
				systemState[i].usStackHighWaterMark, systemState[i].uxCurrentPriority, core, cpuPercentage);
		}
		delete[] systemState;
#endif // ESP_IDF_VERSION
#endif // ESP32
	} else if (command == "reset") {
		attitude = Quaternion();
	} else if (command == "reboot") {
		ESP.restart();
	} else {
		print("Invalid command: %s\n", command.c_str());
	}
}

void handleInput() {
	static bool showMotd = true;
	static String input;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			doCommand(input);
			input.clear();
		} else {
			input += c;
		}
	}
}
