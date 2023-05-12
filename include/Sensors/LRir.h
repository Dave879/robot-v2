#pragma once

#include <TFMPlus.h>

class LRir
{
private:
	TFMPlus tfmP; // Create a TFMini Plus object
	uint32_t past_reading_millis = 0;
	uint16_t delta_measurement_time_ms;

public:
	int16_t tfDist = 0; // Distance to object in centimeters
	int16_t tfFlux = 0; // Strength or quality of return signal
	int16_t tfTemp = 0; // Internal temperature of Lidar sensor chip
	LRir(HardwareSerial &srl, uint32_t framerate);
	uint8_t Read();
	~LRir();
};
