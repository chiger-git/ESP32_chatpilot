// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
// Attitude estimation from gyro and accelerometer.

#include <Arduino.h>
#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

extern Quaternion attitude;
extern Vector rates;
extern Vector gyro;
extern Vector acc;
extern float dt;
extern bool landed;

void applyGyro();
void applyAcc();

float accWeight = 0.003f;
LowPassFilter<Vector> ratesFilter(0.2f);

void estimate() {
	applyGyro();
	applyAcc();
}

void applyGyro() {
	rates = ratesFilter.update(gyro);
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	float accNorm = acc.norm();
	landed = abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}
