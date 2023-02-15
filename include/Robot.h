#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Servo.h>

#include "util.h"
#include "pins.h"
#include "Motors.h"
#include "Sensors/Gyro.h"
#include "Sensors/VL53L5CX_manager.h"
#include "Sensors/Color.h"
#include "data_formatter.h"

class Robot
{
private:
/**
 * Global configuration variables
 */
#define SPEED 40
#define TURN_SPEED 75
#define MIN_DISTANCE_TO_TURN_RIGHT_MM 250
#define MIN_DISTANCE_TO_SET_IGNORE_RIGHT_FALSE_MM 120
#define MIN_DISTANCE_TO_TURN_LEFT_MM 250
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 70
#define MIN_DISTANCE_FROM_LAST_TILE_MM 300

	/**
	 * Navigation variables
	 */
	bool stop_the_robot = false;
	bool first_time_pressed = false;
	int16_t back_distance_before;
	double desired_angle;
	bool ignore_right = false;

	/**
	 * Navigation utility functions
	 */

	bool StopRobot();
	void Turn(int degree);

	/**
	 * Robot peripherals
	 */
	Motors *ms;

	Gyro *mpu;
	GyroData mpu_data = {0};
	static volatile bool mpu_data_ready;

	VL53L5CX_manager *lasers;
	static volatile bool lasers_data_ready[4];

	Color *cs;

	Servo kit;

	/**
	 * Interrupt functions
	 */
	static void R_MPU6050_int();
	static void R_VL53L5CX_int_0();
	static void R_VL53L5CX_int_1();
	static void R_VL53L5CX_int_2();
	static void R_VL53L5CX_int_3();

public:
	/**
	 * The Robot initialization function. Every sensor gets initialized
	 * @todo Need to implement cold start based on last code upload time
	 * @param cold_start Defaults to true. If true the sensor power source will be deactivated on boot,
	 * and every sensor will need to be configured from scratch. This usually takes ~24 seconds.
	 * If set to false, some sensors may not work sometimes because of faulty connections caused by vibrations,
	 * improper handling... but startup will take a significantly lower amount of time (~4.7 seconds), about 80% less.
	 */
	Robot(bool cold_start = true);
	void Run();
	~Robot();

	/**
	 * Robot peripheral interaction
	 */
	uint8_t TrySensorDataUpdate();
	void UpdateSensorNumBlocking(VL53L5CX num);
	void UpdateGyroBlocking();
	void PrintSensorData();
};
