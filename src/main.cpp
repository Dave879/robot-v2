
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "Robot.h"
#include "data_formatter.h"
#include "util.h"

volatile bool Robot::lasers_data_ready[4] = {0};
volatile bool Robot::imu_data_ready = false;
// volatile bool Robot::color_data_ready = false;

Robot *rb;

// uint16_t times_per_second = 0;
// uint32_t past_millis = 0;

void setup()
{
	pinMode(R_SW_START_PIN, INPUT);
	pinMode(R_SW_XTRA_PIN, INPUT);

	pinMode(R_COLLISION_SX_PIN, INPUT);
	pinMode(R_COLLISION_DX_PIN, INPUT);

	pinMode(R_LED1_PIN, OUTPUT);
	pinMode(R_LED2_PIN, OUTPUT);
	pinMode(R_LED3_PIN, OUTPUT);
	pinMode(R_LED4_PIN, OUTPUT);

	digitalWriteFast(R_LED1_PIN, LOW);
	digitalWriteFast(R_LED2_PIN, LOW);
	digitalWriteFast(R_LED3_PIN, LOW);
	digitalWriteFast(R_LED4_PIN, LOW);

	delay(1000);
	Serial.println("Robot initialization starting... 1");
	delay(1000);
	Serial.println("Robot initialization starting... 2");

	if (CrashReport)
	{
		delay(10000);
		Serial.print(CrashReport);
		delay(10000);
	}
	else
	{
		Serial.println("Good news! No crash report found!");
		tone(R_BUZZER_PIN, 4000, 69);
	}

	// If a complete restart of the sensors is needed on every boot, set parameter to true
	rb = new Robot(true);
	// past_millis = millis();
}

void loop()
{
	rb->TrySensorDataUpdate();
	rb->PrintSensorData();
	rb->Run();
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
}
