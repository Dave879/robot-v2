#include "Sensors/VL53L5CX_manager.h"

VL53L5CX_manager::VL53L5CX_manager(TwoWire &interface, bool cold_start) : resolution(0)
{

	if (cold_start)
	{
		for (uint8_t i = 0; i < SENSORS_NUM; i++)
		{
			pinMode(VL53L5CX_LPn_pin[i], OUTPUT);
			digitalWriteFast(VL53L5CX_LPn_pin[i], LOW); // Sensor stops listening
		}
	}
	else
	{
		for (uint8_t i = 0; i < SENSORS_NUM; i++)
		{
			pinMode(VL53L5CX_LPn_pin[i], OUTPUT);
			digitalWriteFast(VL53L5CX_LPn_pin[i], HIGH); // Sensor starts listening
		}
	}
	delay(100);
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		digitalWriteFast(VL53L5CX_LPn_pin[i], HIGH); // Sensor starts listening
		Serial.print("Initializing VL53L5CX sensor ");
		Serial.println(i);
		sensors[i] = new ELIA(interface, VL53L5CX_addr[i]);

		if (sensors[i]->GetStatus() == 0)
		{
			Serial.print("VL53L5CX sensor ");
			Serial.print(i);
			Serial.print(" has address: 0x");
			Serial.println(VL53L5CX_addr[i], HEX);
			tone(R_BUZZER_PIN, 3500, 50);
		}
		else
		{
			Serial.print("VL53L5CX sensor ");
			Serial.print(i);
			Serial.println(" is disconnected");
			is_disconnected[i] = true;
		}

		digitalWriteFast(VL53L5CX_LPn_pin[i], LOW); // Sensor stops listening
		delay(100);											  // This delay is ESSENTIAL: If removed, the StartRanging function on the second sensor returns 255!!!
	}
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		digitalWriteFast(VL53L5CX_LPn_pin[i], HIGH); // Sensor starts listening
	}
}

uint8_t VL53L5CX_manager::StartRanging(const uint8_t resolution, const uint8_t frequency, const ELIA::RangingMode mode)
{
	uint8_t status = 0;
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		if (!is_disconnected[i])
		{
			this->resolution = resolution;
			if (sensors[i]->StartRanging(resolution, frequency, mode) == 0)
			{
				Serial.print("Started ranging VL53L5CX ");
				Serial.println(i);
			}
			else
			{
				Serial.print("Failed to start ranging on VL53L5CX ");
				Serial.println(i);
				status++;
			}
		}
	}
	return status;
}

void VL53L5CX_manager::UpdateData()
{
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		sensors[i]->UpdateData();
	}
}

void VL53L5CX_manager::SetAllDataReady(bool b)
{
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		data_ready[i] = b;
	}
}

void VL53L5CX_manager::GetStatus(uint8_t status[SENSORS_NUM])
{
	for (size_t i = 1; i < SENSORS_NUM; i++)
	{
		status[i] = sensors[i]->GetStatus();
	}
}

VL53L5CX_manager::~VL53L5CX_manager()
{
}
