
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "Robot.h"
#include "data_formatter.h"
#include "util.h"

volatile bool Robot::mpu_data_ready = false;
volatile bool Robot::lasers_data_ready[4] = {0};
volatile bool Robot::color_data_ready = false;

Robot *rb;

uint16_t times_per_second = 0;
uint32_t past_millis = 0;

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
	} else {
		LOG("Good news! No crash report found!");
	}

	// If a complete restart of the sensors is needed on every boot, set parameter to true
	rb = new Robot(false);
	past_millis = millis();
}

void loop()
{

	rb->TrySensorDataUpdate();
	rb->PrintSensorData();

	rb->Run();

	times_per_second++;
	if (past_millis + 1000 < millis())
	{
		Serial.print("Times per second: ");
		Serial.println(times_per_second);
		times_per_second = 0;
		past_millis = millis();
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

}
