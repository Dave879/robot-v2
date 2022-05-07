#pragma once

#include <Arduino.h>

struct GyroData
{
	double x, y, z;
} typedef GyroData;

void SignalBuiltinLED(int count, int delayTime);

void SetupDebugLEDs();

#define START_TIMER int startTime = millis();
#define END_TIMER                   \
	int endTime = millis();              \
	int deltaTime = endTime - startTime; \
	Serial.println(deltaTime);
