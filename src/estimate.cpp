// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 基于陀螺仪和加速度计的姿态计算
// Attitude estimation from gyro and accelerometer

#include <Arduino.h>
#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "rcwl_1605/rcwl_1605.h"

// 外部变量与函数声明
// 来自 main.cpp
extern Quaternion attitude;
extern Vector rates;
extern Vector gyro;
extern Vector acc;
extern float dt;
extern bool landed;

// 来自 motors.cpp
bool motorsActive();

// 内部函数前向声明
void applyGyro();
void applyAcc();

// 全局变量定义
float accWeight = 0.003;

// 低通滤波器: cutoff frequency ~ 40 Hz (0.2 * 200Hz loop rate?)
LowPassFilter<Vector> ratesFilter(0.2f);

float alt_est = 0.0f;       // 当前离地高度估计 (m)
float vz_est = 0.0f;        // 当前垂直速度估计 (m/s)
float last_raw_alt = 0.0f;  // 记录上一次有效原始高度，用于微分求速度

void estimateAltitude() {
	float raw_alt = rcwl1605DistanceM(0);

	// 1. 初步异常值剔除（例如未读到数据时测距可能非法）
	if (raw_alt <= 0.02f) return; 

	// 2. 位置一阶低通滤波
	float alpha_alt = 0.2f; 
	if (alt_est == 0.0f) {  
		alt_est = raw_alt;
		last_raw_alt = raw_alt;
	}
	alt_est = alt_est + alpha_alt * (raw_alt - alt_est);

	// 3. 计算垂直运动速度与速度滤波，防止初始dt异常除0报NaN
	if (dt > 0.0001f) {
		float raw_vz = -(raw_alt - last_raw_alt) / dt; // 负号使上升为正速率，下降为负速率（原距离减小=下降）
		
		float alpha_vz = 0.05f; // 求导极易放大噪声，降至0.05
		vz_est = vz_est + alpha_vz * (raw_vz - vz_est);
	}

	last_raw_alt = raw_alt;
}

void estimate() {
	applyGyro();
	applyAcc();
	estimateAltitude();
}

void applyGyro() {
	// filter gyro to get angular rates
	rates = ratesFilter.update(gyro);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	float accNorm = acc.norm();
	// landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;
	landed = abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}
