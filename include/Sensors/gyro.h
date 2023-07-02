#pragma once

#include "ICM42688.h"
#include <Fusion.h>
#include "util.h"

struct GyroData
{
	double x, y, z;
} typedef GyroData;

class gyro
{
private:
	ICM42688_FIFO *IMU;
	uint32_t pastMicros;
	FusionAhrs ahrs;
	FusionVector gyroscope;		 // degrees/s
	FusionVector accelerometer; // g
	FusionEuler euler;
public:
	uint32_t count_ = 0;
	uint32_t past_millis_count_;
	double drift_last_sec = 0.0f;
	const double delta_seconds = 0.01f; // 1s/100(odr)
	float x, y, z;
	gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin);
	float GetPosition();
	void ResetPosition();
	uint8_t UpdateData(bool integrate_position = false);
	void ResetZ();
	~gyro();
};
