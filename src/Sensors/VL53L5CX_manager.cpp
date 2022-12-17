#include "Sensors/VL53L5CX_manager.h"

VL53L5CX_manager::VL53L5CX_manager(TwoWire &interface) : resolution(0)
{
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		pinMode(VL53L5CX_LPn_pin[i], OUTPUT);
		digitalWrite(VL53L5CX_LPn_pin[i], LOW); // Sensor stops listening
	}

	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		digitalWrite(VL53L5CX_LPn_pin[i], HIGH); // Sensor starts listening
		Serial.print("Initializing VL53L5CX sensor ");
		Serial.println(i);
		sensors[i] = new ELIA(interface, VL53L5CX_addr[i]);

		if (sensors[i]->GetStatus() == 0)
		{
			Serial.print("VL53L5CX sensor ");
			Serial.print(i);
			Serial.print(" has address: 0x");
			Serial.println(VL53L5CX_addr[i], HEX);
		}
		else
		{
			Serial.print("VL53L5CX sensor ");
			Serial.print(i);
			Serial.println(" is disconnected");
			is_disconnected[i] = true;
		}

		digitalWrite(VL53L5CX_LPn_pin[i], LOW); // Sensor stops listening
		delay(100);										 // This delay is ESSENTIAL: If removed, the StartRanging function on the second sensor returns 255!!!
	}
	for (uint8_t i = 0; i < SENSORS_NUM; i++)
	{
		digitalWrite(VL53L5CX_LPn_pin[i], HIGH); // Sensor starts listening
	}
}

void VL53L5CX_manager::StartRanging(const uint8_t resolution, const uint8_t frequency, const ELIA::RangingMode mode)
{
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
			}
		}
	}
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

VL53L5CX_manager::~VL53L5CX_manager()
{
}
