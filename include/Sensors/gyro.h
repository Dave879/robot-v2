#pragma once

#include "ICM42688.h"

struct GyroData
{
	double x, y, z;
} typedef GyroData;

class gyro
{
private:
	ICM42688 *IMU;
	uint32_t pastMicros;
public:
	double x, y, z;
	gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin);
	uint8_t UpdateData();
	void ResetZ();
	~gyro();
};
