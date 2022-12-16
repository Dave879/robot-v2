#pragma once

#include <Arduino.h>

struct GyroData
{
	double x, y, z;
} typedef GyroData;

void SignalBuiltinLED(int count, int delayTime);

#define START_TIMER int startTime = millis();
#define END_TIMER                   \
	int endTime = millis();              \
	int deltaTime = endTime - startTime; \
	Serial.println(deltaTime);

int clamp(int8_t value, int8_t min, int8_t max);
