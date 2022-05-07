#pragma once

#include <VL53L1X.h>

#include "util.h"

class Laser
{
private:
	VL53L1X laser;
	int distance;
public:
	Laser(TwoWire* bus);
	~Laser();
	int GetData();
};
