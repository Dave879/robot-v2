#pragma once

#include <VL53L1X.h>

#include "config.h"
#include "util.h"


// Table of SPAD locations. Each SPAD has a number which is not obvious.
/*
#define PX_COUNT 16

// clang-format off
const int center[16] = {
	145, 177, 209, 241,
	149, 181, 213, 245,
	110,  78,  46,  14,
	106,  74,  42,  10
};
// clang-format on
int Zone = 0;
uint64_t num = 0;
*/

class Laser
{
private:
	VL53L1X laser;
	int distance;
#if DEBUG == true
	int debugPin;
#endif

public:
	Laser(TwoWire *bus);
#if DEBUG == true
	Laser(TwoWire *bus, int ledPin);
#endif
	~Laser();
	int GetData();
	void SetROICenter(int center);
};
