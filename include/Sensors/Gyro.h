#pragma once

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <Wire.h>

#include "util.h"

class Gyro
{
private:
	MPU6050 mpu;

	// MPU control/status vars
	bool data_ready = false;	// set true if DMP init was successful
	uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;		// expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;		// count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion vars
	Quaternion q;			// [w, x, y, z]         quaternion container
	VectorFloat gravity; // [x, y, z]            gravity vector
	float ypr[3];			// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	GyroData gyro;

public:
	Gyro();

	void SetDataReady(bool is_ready);
	GyroData GetGyroData();
	void ResetGyro();
};
