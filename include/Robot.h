#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Servo.h>

#include "util.h"
#include "pins.h"
#include "Motors.h"
#include "Sensors/gyro.h"
#include "Sensors/VL53L5CX_manager.h"
#include "Sensors/Color.h"
#include "data_formatter.h"

class Robot
{
private:
/**
 * Global configuration variables
 */
#define SPEED 45
#define TURN_SPEED 75
#define MIN_DISTANCE_TO_TURN_MM 210
#define MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM 180
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 80
#define MIN_DISTANCE_BUMP_BACK_WALL_MM 180
#define ADDITIONAL_ANGLE_TO_OVERCOME 3
// Colored tile
#define MIN_VALUE_BLUE_TILE 22
#define MIN_VALUE_BLACK_TILE 10
// PID controller constants
#define KP 0.45			 //.5 // Proportional gain
#define KI 0.000000001 //.2 // Integral gain
#define KD 0			 //.1 // Derivative gain
// Tile to tile
#define DISTANCE_SENSOR_CELL 27
#define DISTANCE_FRONT_AND_BACK_CENTER_TILE  60
#define MIN_TIME_RAMP 3000
#define RAMP_BACK_DIST 100

	/**
	 * Navigation variables
	 */
	bool stop_the_robot = true;
	bool first_time_pressed = false;
	bool just_found_black = false;
	uint32_t time_in_ramp = 0;
	bool was_in_ramp = false;
	bool going_down_ramp = false;
/*
	bool dont_look_back = false;
*/
	// Tile to tile variables
	int32_t front_distance_to_reach = 0;
	int32_t back_distance_to_reach = 0;
	// Turn variables
	double desired_angle = 0;

	/**
	 * Navigation utility functions
	 */
	bool StopRobot();
	bool NewTile();
	int16_t GetRightDistance();
	int16_t GetLeftDistance();
	int16_t GetFrontDistance();
	int16_t GetBackDistance();
	void SetNewTileDistances();
	void SetCurrentTileDistances();
	bool CanTurnRight();
	bool CanTurnLeft();
	bool CanGoOn();
	bool CanBumpBack();
	bool FrontWall();
	bool BlackTile();
	bool BlueTile();
	void Turn(int16_t degree);
	void Straighten();
	bool NotInRamp();
	void TurnRight();
	void TurnLeft();
	void TurnBack();
	void MotorPowerZGyroAndPID();
	void FakeDelay(uint32_t time);
	// victim variables
	bool just_recived_from_openmv = false;
	uint32_t time_to_wait_after_openmv_search_again = 0;
	int8_t kits_dropped = 0;
	// victim functions
	bool FoundVictim();
	void DropKit(int8_t number_of_kits, bool left_victim);
	void DropKitNoTurn(int8_t number_of_kits);
	void VictimVerify();
	void RemoveVictimU();

	double old_gyro_value; 

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
	int16_t GetPIDOutputAndSec();

	/**
	 * Robot peripherals
	 */
	Motors *ms;

	gyro *imu;
	volatile bool *imu_data_ready;

	VL53L5CX_manager *lasers;
	static volatile bool lasers_data_ready[4];

	Color *cs;
	static volatile bool color_data_ready;

	Servo kit;

	/**
	 * Interrupt functions
	 */
	static void R_IMU_int();
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
	Robot(gyro *imu, volatile bool *imu_dr, bool cold_start = true);
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
