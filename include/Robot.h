#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include "util.h"
#include "pins.h"
#include "Motors.h"
#include "Sensors/Gyro.h"
#include "Sensors/VL53L5CX_manager.h"
#include "data_formatter.h"

class Robot
{
private:
	Gyro *mpu;
	GyroData mpu_data = {0};
	static volatile bool mpu_data_ready;
	double desired_angle;
	bool ignore_right = false;
	int16_t back_distance_before;

	Motors *ms;

	VL53L5CX_manager *lasers;
	static volatile bool lasers_data_ready[4];

	static void R_MPU6050_int();
	static void R_VL53L5CX_int_0();
	static void R_VL53L5CX_int_1();
	static void R_VL53L5CX_int_2();
	static void R_VL53L5CX_int_3();

public:
	Robot();
	uint8_t TrySensorDataUpdate();
	void UpdateSensorNumBlocking(uint8_t num);
	void UpdateGyroBlocking();
	void PrintSensorData();
	void Run();
	~Robot();
};
