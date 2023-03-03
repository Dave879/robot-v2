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
#define SPEED 47
#define TURN_SPEED 95
#define MIN_DISTANCE_TO_TURN_MM 220
#define MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM 180
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 50
#define ADDITIONAL_ANGLE_TO_OVERCOME 5
//#define MIN_DISTANCE_FROM_LAST_TILE_MM 300
// Colored tile
#define MIN_VALUE_TO_STOP_COLORED_TILE 18
#define MIN_VALUE_TO_STOP_BLUE 18
#define MIN_VALUE_TO_AVOID_BLACK 9
// PID controller constants
#define KP 0.2//.5 // Proportional gain
#define KI 0.02//.2 // Integral gain
#define KD 0.016//.1 // Derivative gain

	/**
	 * Navigation variables
	 */
	bool stop_the_robot = false;
	bool first_time_pressed = false;
	bool last_turn_right = false;
	// Black tile variables
	bool just_found_black = false;
	int32_t time_after_black_tile_ignore_false = 0;
	// BLue tile variables
	bool ignore_blue = false;
	// Turn variables
	double desired_angle = 0;
	bool ignore_right = false;
	bool ignore_left = false;

	/**
	 * OpenMV varaibles
	*/
	bool openmv_searching = false;
	bool victim_just_found = false;
	int32_t time_after_openmv_can_search_again = 0;
	// OpenMV functions
	void StopOpenMV();


	/**
	 * Navigation utility functions
	 */
	bool StopRobot();
	void Turn(int16_t degree);
	void Straighten();
	void TurnRight();
	void TurnLeft();
	// victim functions
	void DropKit(int8_t number_of_kits);

	/**
	 * PID controller variables
	*/
	uint32_t PID_start_time;
	double PID_previous_error = 0;
	double PID_integral = 0;
	double PID_output = 0;
	double PID_error = 0;
	double PID_derivative = 0;

	/**
	 * PID controller functions
	*/
	double CalculateError(double currentYaw);

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
	static volatile bool color_data_ready;

	Servo kit;

	/**
	 * Interrupt functions
	 */
	static void R_MPU6050_int();
	static void R_VL53L5CX_int_0();
	static void R_VL53L5CX_int_1();
	static void R_VL53L5CX_int_2();
	static void R_VL53L5CX_int_3();
	static void R_TCS34725_int();

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
