#pragma once


/**
 * Global configuration variables
 */

#define SPEED 47
#define TURN_SPEED 75
#define MIN_DISTANCE_TO_TURN_MM 210
#define MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM 180
#define MIN_DISTANCE_FROM_FRONT_WALL_MM 100
#define MIN_DISTANCE_BUMP_BACK_WALL_MM 180
#define ADDITIONAL_ANGLE_TO_OVERCOME 3
// Colored tile
#define MIN_VALUE_BLUE_TILE 22
#define MIN_VALUE_BLACK_TILE 11
// PID controller constants
#define KP 0.0007		//.5 // Proportional gain
#define KI 0 //.2 // Integral gain
#define KD 0 //.1 // Derivative gain
// Tile to tile
#define DISTANCE_SENSOR_CELL 27
#define DISTANCE_END_TILE_CENTER 60
#define DISTANCE_FRONT_TO_CENTER_TILE DISTANCE_END_TILE_CENTER + 10
#define DISTANCE_BACK_TO_CENTER_TILE 60
#define MIN_TIME_RAMP 3000
#define RAMP_BACK_DIST 100