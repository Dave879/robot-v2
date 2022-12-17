#ifndef __ELIA_H__
#define __ELIA_H__

#include <Wire.h>
#include "vl53l5cx_api.h"

/*
	Excellent
	Long (distance)
	Infrared
	Acquisition
*/

class ELIA
{
private:
	VL53L5CX_Configuration config;
	VL53L5CX_ResultsData data;
	uint8_t status;

public:
	enum RangingMode
	{
		kContinuous = VL53L5CX_RANGING_MODE_CONTINUOUS,
		kAutonomous = VL53L5CX_RANGING_MODE_AUTONOMOUS
	};
	// Default address = 0x52 (82dec)
	ELIA(TwoWire &interface, const uint8_t address);
	uint8_t StartRanging(const uint8_t resolution, const uint8_t frequency, const RangingMode mode);
	uint8_t UpdateData();
	VL53L5CX_ResultsData *GetData();
	uint8_t GetStatus();
	~ELIA();
};

#endif // __ELIA_H__