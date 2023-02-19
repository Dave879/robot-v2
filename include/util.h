#pragma once

#include <Arduino.h>

void SignalBuiltinLED(int count, int delayTime);

#define START_TIMER int startTime = millis();
#define END_TIMER                   \
	int endTime = millis();              \
	int deltaTime = endTime - startTime; \
	Serial.println(deltaTime);

#define LOG(x) Serial.println(x)

int32_t clamp(int32_t value, int32_t min, int32_t max);
