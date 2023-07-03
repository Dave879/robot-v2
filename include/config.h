#pragma once


/**
 * Global configuration variables
 */

#define SPEED 45
#define TURN_SPEED 90
#define MIN_DISTANCE_TO_TURN_MM 290
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 100
#define MIN_DISTANCE_BUMP_BACK_WALL_MM 180
#define ADDITIONAL_ANGLE_TO_OVERCOME 3
#define MIN_DISTANCE_TO_WIGGLE_WIGGLE 55
// Colored tile
#define MIN_VALUE_BLUE_TILE 16
#define MIN_VALUE_BLACK_TILE 10
// PID controller constants
#define KP 0.0007		//.5 // Proportional gain
#define KI 0 //.2 // Integral gain
#define KD 0 //.1 // Derivative gain
// Tile to tile

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
  #define DISTANCE_SENSOR_CELL_BACK 28
#endif

#define DISTANCE_FRONT_TO_CENTER_TILE 60
#define DISTANCE_BACK_TO_CENTER_TILE 60
#define MIN_TIME_RAMP 3000
#define RAMP_BACK_DIST 100