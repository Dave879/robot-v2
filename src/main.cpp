
#include <Arduino.h>

#include <ArduinoJson.h>

#include <Wire.h>

#include "Robot.h"
#include "data_formatter.h"
#include "util.h"

volatile bool Robot::mpu_data_ready = false;
volatile bool Robot::lasers_data_ready[4] = {0};
Robot *rb;

void setup()
{
	LOG("Robot initialization starting... 1");
	LOG("Robot initialization starting... 2");
	LOG("Robot initialization starting... 3");
	delay(2000);
	LOG("Robot initialization starting... 4");

	if (CrashReport)
	{
		delay(5000);
		Serial.print(CrashReport);
		delay(5000);
	} else {
		LOG("Good news! No crash report found!");
	}

	rb = new Robot();
}

void loop()
{
	rb->TrySensorDataUpdate();
	rb->PrintSensorData();
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
