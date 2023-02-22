
#include "ELIA.h"

ELIA::ELIA(TwoWire &interface, const uint8_t address) : status(0)
{
	config = VL53L5CX_Configuration();
	config.platform.i2c_bus = &interface;

	data = VL53L5CX_ResultsData();
	uint8_t is_alive = 0;

	config.platform.address = address;
	status |= vl53l5cx_is_alive(&config, &is_alive); // Try to find sensor and hot reload it
	if (is_alive)
	{
		Serial.println("Old sensor is alive");
		return;
	}
	status = 0; // Reset status, surely vl53l5cx_is_alive function failed

	config.platform.address = 0x52;
	status |= vl53l5cx_is_alive(&config, &is_alive);
	if (is_alive == 0)
	{
		return;
	}
	status |= vl53l5cx_set_i2c_address(&config, address);
	status |= vl53l5cx_is_alive(&config, &is_alive);
	if (is_alive == 0)
	{
		return;
	}
	// This function takes (from 1131ms, 1063ms, 1058ms) 1053ms to execute, it needs to transfer ~84 kbytes (um2884) through the i2c bus
	status |= vl53l5cx_init(&config);
	status |= vl53l5cx_set_sharpener_percent(&config, 20);

	status |= vl53l5cx_is_alive(&config, &is_alive);
}

uint8_t ELIA::StartRanging(const uint8_t resolution, const uint8_t frequency, const RangingMode mode)
{
	status |= vl53l5cx_set_ranging_mode(&config, (uint8_t)mode);
	status |= vl53l5cx_set_resolution(&config, resolution);			  // 4*4 = 16 // 8*8 = 64
	status |= vl53l5cx_set_ranging_frequency_hz(&config, frequency); // Max for 4*4 -> 60Hz // Max for 8*8 -> 15Hz
	status |= vl53l5cx_start_ranging(&config);
	return status;
}

uint8_t ELIA::UpdateData()
{
	return status |= vl53l5cx_get_ranging_data(&config, &data);
}

VL53L5CX_ResultsData *ELIA::GetData()
{
	return &data;
}

uint8_t ELIA::GetStatus()
{
	return status;
}

ELIA::~ELIA()
{
}
