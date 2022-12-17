
#include <Arduino.h>

#include <ArduinoJson.h>

#include <Wire.h>

#include "config.h"
#include "dataname.h"
#include "Sensors/Gyro.h"
#include "data_formatter.h"
#include "Motors.h"
#include "Sensors/VL53L5CX_manager.h"

#include "util.h"
#include "pins.h"

Gyro *mpu;
GyroData gyro = {0};
volatile uint8_t gyro_data_ready = false;
uint16_t calls_a_second = 0;
uint64_t millis_after_one_second = 0;
Motors *ms;
VL53L5CX_manager *lasers;

void VL53L5CX_int_0()
{
	lasers->data_ready[0] = true;
}

void VL53L5CX_int_1()
{
	lasers->data_ready[1] = true;
}

void VL53L5CX_int_2()
{
	lasers->data_ready[2] = true;
}

void VL53L5CX_int_3()
{
	lasers->data_ready[3] = true;
}

void GyroDataIsReady()
{
	gyro_data_ready++;
}

void setup()
{
	LOG("Robot initialization starting...");
	delay(10000);
	LOG("Robot initialization starting...");

	if (CrashReport)
	{
		delay(5000);
		Serial.print(CrashReport);
		delay(5000);
	} else {
		LOG("Good news! No crash report found!");
	}

	LOG("Disabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);													// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
	delay(10);													// Wait for sensors to wake up (especially sensor 0)
	LOG("...done!");

	Wire.begin(); // Gyro
	Wire.setClock(400000);

	LOG("Gyro setup started");
	mpu = new Gyro();
	LOG("Finished gyro setup!");

	attachInterrupt(R_PIN_GYRO_INT, GyroDataIsReady, RISING);

	LOG("Motor setup started");
	ms = new Motors();
	LOG("Finished motor setup!");
	ms->SetPower(30, 30);
	delay(1000);
	ms->SetPower(0, 0);

	Wire2.begin(); // Lasers
	Wire2.setClock(1000000);

	LOG("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2);
	LOG("Laser sensors setup finished");


	/*
		Interrupts *MUST* be attached after the VL53L5CX_manager is instantiated,
		otherwise if the data_ready array isn't initialized and an interrupt is
		fired, the program will crash.
	*/
	attachInterrupt(VL53L5CX_int_pin[0], VL53L5CX_int_0, FALLING); // sensor_0
	attachInterrupt(VL53L5CX_int_pin[1], VL53L5CX_int_1, FALLING); // sensor_1
	attachInterrupt(VL53L5CX_int_pin[2], VL53L5CX_int_2, FALLING); // sensor_2
	attachInterrupt(VL53L5CX_int_pin[3], VL53L5CX_int_3, FALLING); // sensor_3

	lasers->StartRanging(16, 60, ELIA::RangingMode::kContinuous); // 4*4, 60Hz

	millis_after_one_second = millis() + 1000;
}

void loop()
{
	if (millis() > millis_after_one_second)
	{
		Serial.print("Calls last second: ");
		Serial.println(calls_a_second);
		calls_a_second = 0;
		millis_after_one_second = millis() + 1000;
	}

	/*
	DynamicJsonDocument doc(200);
	DataFormatter fm;
	String res = "";

	doc[fm.AddLineGraph("Denis")] = 10;
	doc[fm.AddLineGraph("Noise")] = 69;
	doc[fm.AddLineGraph("Noise1")] = 30;
	doc[fm.AddLineGraph("Noise2")] = 20;
	serializeJson(doc, res);
	Serial.println(res);
	*/

	if (gyro_data_ready > 1)
	{
		gyro = mpu->GetGyroData();



		calls_a_second++;
		gyro_data_ready = 0;
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		if (lasers->data_ready[i])
		{
			lasers->sensors[i]->UpdateData();

			Serial.print("Sensor number ");
			Serial.println(i);

			for (uint8_t j = 0; j < lasers->resolution; j++)
			{
				Serial.print(lasers->sensors[i]->GetData()->distance_mm[j]);
				Serial.print(", \t");
				if (j == 3 || j == 7 || j == 11 || j == 15)
				{
					Serial.println();
				}
			}
			Serial.println();



		Serial.print("gyro.x: \t");
		Serial.print(gyro.x);
		Serial.print(" \t");
		Serial.print("gyro.y: \t");
		Serial.print(gyro.y);
		Serial.print(" \t");
		Serial.print("gyro.z: \t");
		Serial.println(gyro.z);



			lasers->data_ready[i] = false;
		}
	}
}
