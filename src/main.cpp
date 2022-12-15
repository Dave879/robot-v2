
#include <Arduino.h>

#include <ArduinoJson.h>

#include <Wire.h>

#include "config.h"
#include "dataname.h"
#include "Sensors/Gyro.h"
#include "data_formatter.h"
#include "motors.h"

#include "util.h"
#include "pins.h"

Gyro *mpu;
GyroData gyro = {0};
volatile uint8_t gyro_data_ready = false;
uint16_t calls_a_second = 0;
uint64_t millis_after_one_second = 0;
motors *ms;

void GyroDataIsReady()
{
	gyro_data_ready++;
}

void setup()
{
	if (CrashReport)
	{
		delay(5000);
		Serial.print(CrashReport);
		delay(5000);
	}

	Wire.begin();
	Wire.setClock(400000);

	mpu = new Gyro();

	attachInterrupt(R_PIN_GYRO_INT, GyroDataIsReady, RISING);
	ms = new motors();
	ms->setPower(30, 30);
	delay(1000);
	ms->setPower(0, 0);
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

		Serial.print("gyro.x: \t");
		Serial.print(gyro.x);
		Serial.print(" \t");
		Serial.print("gyro.y: \t");
		Serial.print(gyro.y);
		Serial.print(" \t");
		Serial.print("gyro.z: \t");
		Serial.println(gyro.z);

		calls_a_second++;
		gyro_data_ready = 0;

	}

	//  gyro = mpu->GetGyroData();
	//  laser0_data = l0->GetData();
	//  l1->SetROICenter(center[Zone]);
	//  LaserDataArray.add(l1->GetData());
	/*
		if (Zone >= PX_COUNT - 1)
		{
			// doc[DATANAME_LASER_1] = LaserDataArray;
			serializeJson(doc, res);
			doc.clear();
			// LaserDataArray = doc.createNestedArray("laser");
			Serial.println(res);
			res = "";
		}
		Zone++;
		Zone = Zone % PX_COUNT;
	*/
	/*
	if (digitalRead(33))
	{
		l1 = new Laser(&Wire2);
		delay(1000);
	}
	if (digitalRead(33))
	{
		res += PWM_1;
		res += PWM_2;
		res += dir;
		PWM_1 += 10;
		PWM_2 += 10;
		res += "\n";
		Serial5.print(res);
		res = "";
		if (dir == '0')
			dir = '1';
		else if (dir == '1')
			dir = '2';
		else if (dir == '2')
			dir = '3';
		else
			dir = '0';
		if (PWM_1 >= 127)
			PWM_1 = ' ';
		if (PWM_2 >= 127)
			PWM_2 = ' ';
		delay(500);
	}
	*/

	/*
	START_TIMER
	rgb_sensor.getData();
	END_TIMER

	printColorSensor();
	*/
	/*
	#if DEBUG == true
		doc[DATANAME_GYRO_X] = gyro.x;
		doc[DATANAME_GYRO_Y] = gyro.y;
		doc[DATANAME_GYRO_Z] = gyro.z;
	*/
	// doc[DATANAME_LASER_0] = laser0_data;

	// #endif
}
