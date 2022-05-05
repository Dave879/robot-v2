#pragma once

#include <Arduino.h>

struct GyroData
{
	double x, y, z;
} typedef GyroData;

void SignalBuiltinLED(int count, int delayTime);
