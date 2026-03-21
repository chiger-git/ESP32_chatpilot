#pragma once

#include <Arduino.h>

void setupRCWL1605();
void readRCWL1605();

int rcwl1605SensorCount();
bool rcwl1605SensorValid(int index);
float rcwl1605DistanceM(int index);
float rcwl1605MinDistanceM();
bool rcwl1605ObstacleDetected();
void printRCWL1605Status();

extern float rcwl1605Enabled;
extern float rcwl1605FailsafeEnabled;
extern float rcwl1605ObstacleThresholdM;
extern float rcwl1605MinRangeM;
extern float rcwl1605MaxRangeM;
extern float rcwl1605FilterAlpha;
extern float rcwl1605ReadIntervalMs;
