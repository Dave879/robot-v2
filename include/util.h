#pragma once

#include <Arduino.h>

void SignalBuiltinLED(int count, int delayTime);

#define START_TIMER int32_t startTime = millis();
#define END_TIMER                   \
	int32_t endTime = millis();              \
	int32_t deltaTime = endTime - startTime; \
	Serial.println(deltaTime);

#define LOG(x) Serial.print(x)

#define LINEGRAPH(title, value) 	Serial.print("{\"0&n&l&"); Serial.print(title); Serial.print("\":"); Serial.print(value); Serial.print("}");
#define LINEGRAPH_MULTI(title, value, id) 	Serial.print("{\""); Serial.print(id); Serial.print("&n&l&"); Serial.print(title); Serial.print("\":"); Serial.print(value); Serial.print("}");

int32_t clamp(int32_t value, int32_t min, int32_t max);
