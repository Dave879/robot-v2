
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "Robot.h"
#include "data_formatter.h"
#include "util.h"

volatile bool Robot::lasers_data_ready[4] = {0};
volatile bool Robot::color_data_ready = false;

Robot *rb;

gyro *imu;
volatile bool imu_dr;

void IMU_int()
{
	imu_dr = true;
}

// uint16_t times_per_second = 0;
// uint32_t past_millis = 0;

void setup()
{
	LOG("Robot initialization starting... 1");
	LOG("Robot initialization starting... 2");
	LOG("Robot initialization starting... 3");
	delay(3000);
	LOG("Robot initialization starting... 4");

	if (CrashReport)
	{
		delay(5000);
		Serial.print(CrashReport);
		delay(5000);
	}
	else
	{
		LOG("Good news! No crash report found!");
	}

	LOG("Gyro setup started");
	imu_dr = false;
	imu = new gyro(SPI, R_IMU_CS_PIN, R_IMU_EXT_CLK_SPI_PIN);
	attachInterrupt(R_IMU_INT_SPI_PIN, IMU_int, RISING);
	LOG("Finished gyro setup!");

	// If a complete restart of the sensors is needed on every boot, set parameter to true
	rb = new Robot(imu, &imu_dr, true);
	// past_millis = millis();
}

void loop()
{
	rb->TrySensorDataUpdate();
	if (imu_dr)
	{
		imu->UpdateData();
		imu_dr = false;
	}
	rb->PrintSensorData();
	rb->Run();
	if (imu_dr)
	{
		imu->UpdateData();
		imu_dr = false;
	}
	/*
		times_per_second++;
		if (past_millis + 1000 < millis())
		{
			Serial.print("Times per second: ");
			Serial.println(times_per_second);
			times_per_second = 0;
			past_millis = millis();
		}
	*/

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
}
