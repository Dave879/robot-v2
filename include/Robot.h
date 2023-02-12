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
	
	bool stop_the_robot = false;
	const int SPEED = 40;
	const int TURN_SPEED = 75;
	int16_t back_distance_before;
	double desired_angle;
	bool ignore_right = false;
	const int MIN_DISTANCE_TO_TURN_RIGHT_MM = 250;
	const int MIN_DISTANCE_TO_SET_IGNORE_RIGHT_FALSE_MM = 120;
	const int MIN_DISTANCE_TO_TURN_LEFT_MM = 250;
	const int MIN_DISTANCE_FROM_FRONT_WALL_MM = 70;
	const int MIN_DISTANCE_FROM_LAST_TILE_MM = 300;

	Motors *ms;

	VL53L5CX_manager *lasers;
	static volatile bool lasers_data_ready[4];

	static void R_MPU6050_int();
	static void R_VL53L5CX_int_0();
	static void R_VL53L5CX_int_1();
	static void R_VL53L5CX_int_2();
	static void R_VL53L5CX_int_3();

	bool StopRobot();
	void Turn(int degree);

public:
	Robot();
	uint8_t TrySensorDataUpdate();
	void UpdateSensorNumBlocking(uint8_t num);
	void UpdateGyroBlocking();
	void PrintSensorData();
	void Run();
	~Robot();
};
