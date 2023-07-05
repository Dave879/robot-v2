#pragma once


/**
 * Global configuration variables
 */

#define SPEED 45
#define TURN_SPEED 90
#define MIN_DISTANCE_TO_TURN_MM 220
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 100
#define MIN_DISTANCE_BUMP_BACK_WALL_MM 220
#define ADDITIONAL_ANGLE_TO_OVERCOME 5
#define MIN_DISTANCE_TO_CENTER_TILE 65
#define MAX_DISTANCE_TO_CENTER_TILE 145
// Colored tile
#define MAX_VALUE_BLUE_TILE 15
#define MAX_VALUE_BLACK_TILE 9

//#define ROBOT2

#ifdef ROBOT2
  #define DISTANCE_SENSOR_CELL_LEFT 27
  #define DISTANCE_SENSOR_CELL_FRONT 27
  #define DISTANCE_SENSOR_CELL_RIGHT 28
  #define DISTANCE_SENSOR_CELL_BACK 36
#else
  #define DISTANCE_SENSOR_CELL_LEFT 27
  #define DISTANCE_SENSOR_CELL_FRONT 20
  #define DISTANCE_SENSOR_CELL_RIGHT 28
  #define DISTANCE_SENSOR_CELL_BACK 36
#endif

#define DISTANCE_FRONT_TO_CENTER_TILE 74
#define DISTANCE_BACK_TO_CENTER_TILE 20
#define MIN_TIME_RAMP 500
#define RAMP_BACK_DIST 100
#define TIME_TO_TILE 1250
#define MIN_TIME_TO_TILE 850