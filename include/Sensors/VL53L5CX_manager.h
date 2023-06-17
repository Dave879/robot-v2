#ifndef __VL53L5CX_manager__
#define __VL53L5CX_manager__

#include "ELIA.h"
#include "pins.h"
#include "util.h"

#define SENSORS_NUM 4

class VL53L5CX_manager
{
public:
	ELIA *sensors[SENSORS_NUM];
	uint8_t resolution;
	volatile bool data_ready[SENSORS_NUM] = {0};
	bool is_disconnected[SENSORS_NUM] = {0};

	VL53L5CX_manager(TwoWire& interface, bool cold_start);
	uint8_t StartRanging(const uint8_t resolution, const uint8_t frequency, const ELIA::RangingMode mode);
	void UpdateData();
	void SetAllDataReady(bool b);
	void GetStatus(uint8_t status[SENSORS_NUM]);
	~VL53L5CX_manager();
};

#endif // __VL53L5CX_manager__