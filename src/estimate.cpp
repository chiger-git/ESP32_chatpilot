// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// 基于陀螺仪和加速度计的姿态计算
// Attitude estimation from gyro and accelerometer

#include <Arduino.h>
#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

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

void estimate() {
	applyGyro();
	applyAcc();
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
