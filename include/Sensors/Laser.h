#pragma once

#include <VL53L1X.h>

#include "config.h"
#include "util.h"

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
};
