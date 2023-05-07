#pragma once

#include "ICM42688.h"
#include "Fusion.h"

struct GyroData
{
	double x, y, z;
} typedef GyroData;

class gyro
{
private:
	ICM42688 *IMU;
	uint32_t pastMicros;
	FusionAhrs ahrs;
	FusionVector gyroscope;		 // degrees/s
	FusionVector accelerometer; // g
	FusionEuler euler;
	double delta_seconds = 0;
public:
	double x, y, z;
	gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin);
	uint8_t UpdateData();
	void ResetZ();
	~gyro();
};
