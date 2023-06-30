#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Servo.h>

#include "config.h"
#include "pins.h"
#include "Motors.h"
#include "Sensors/gyro.h"
#include "Sensors/VL53L5CX_manager.h"
#include "Sensors/Color.h"
#include "data_formatter.h"
#include "mapping/graph.h"

class Robot
{
private:

	StaticJsonDocument<5000> json_doc;
	DataFormatter doc_helper;
	JsonArray dist[4];

	/**
	 * Navigation variables
	 */
	uint8_t consecutive_turns = 0;
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

	// Mapping
	/*
	Direction può essere FW = 0, DX = 1, BW = 2, SX = 3
	*/
	enum dir
	{
		FW,
		DX,
		BW,
		SX
	};
	/*
	Graph
	*/
	graph *map;
	int8_t direction = 0;
	int32_t current_x = 0;
	int32_t current_y = 0;
	int32_t current_z = 0;
	bool first_tile = false;
	int32_t previous_tile_y = current_y;
	int32_t previous_tile_x = current_x;
	int32_t previous_tile_z = current_z;
	std::vector<Tile> path_to_tile;
	std::vector<Tile> tile_to_visit;

	Tile last_checkpoint;
	void GoToDirection(int8_t direction_to_go);
	void AddLeftAndFrontTileToTileToVisit();
	void AddRightAndFrontTileToTileToVisit();
	void AddLeftAndRightTileToTileToVisit();
	void AddLeftTileToTileToVisit();
	void AddRightTileToTileToVisit();
	void AddFrontTileToTileToVisit();
	bool InTileToVisit(Tile t);
	void RemoveTileToVisit(Tile t);
	void UpdateAllDistanceSensorsBlocking();
	/**
	 * Navigation utility functions
	 */
	bool StopRobot();
	void FirstTileProcedure();
	void LackOfProgressProcedure();
	void GetAroundTileVisited(bool &left_already_visited, bool &front_already_visited, bool &right_already_visited);
	bool NewTile();

	int16_t GetRightDistance();
	int16_t GetLeftDistance();
	int32_t GetFrontDistance();
	int16_t GetBackDistance();
	void SetNewTileDistances();
	void SetCurrentTileDistances();
	void ChangeMapX(bool increment);
	void ChangeMapY(bool increment);
	void ChangeMapPosition();
	void IncreaseDirection();
	void DecreaseDirection();
	bool CanTurnRight();
	bool CanTurnLeft();
	bool CanGoOn();
	bool CanBumpBack();
	bool FrontWall();
	bool BlackTile();
	bool BlueTile();
	void DecideTurn(bool left_blocked, bool front_blocked, bool right_blocked, bool tile_already_visited, bool blue_tile);
	void Turn(int16_t degree);
	void Straighten();
	bool NotInRamp();
	void TurnRight();
	void TurnLeft();
	void TurnBack();
	void FakeDelay(uint32_t time);
	// victim variables
	bool just_recived_from_openmv = false;
	uint32_t time_to_wait_after_openmv_search_again = 0;
	int8_t kits_dropped = 0;
	// victim functions
	void FindVictim();
	bool VictimFound();
	void DropKit(int8_t number_of_kits, bool left_victim);
	void AfterTurnVictimDetection();

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
	static volatile bool imu_data_ready;

	VL53L5CX_manager *lasers;
	static volatile bool lasers_data_ready[4];

	Color *cs;
	static volatile bool color_data_ready; // As of 12/05/2023, unused - Dave

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
	static void R_ICM_42688_P_int();

public:
	/**
	 * The Robot initialization function. Every sensor gets initialized
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
	void UpdateGyroNonBlocking();
	void PrintSensorData();
	void TestPeripherals();
	void TestButton(uint8_t pin, const char* name);
};
