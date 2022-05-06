
#include <Arduino.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <VL53L1X.h>

#include "config.h"
#include "dataname.h"
#include "util.h"
#include "Gyro.h"

DynamicJsonDocument doc(1024);
String res = "";

VL53L1X laser0, laser1;

Gyro *mpu;
GyroData gyro = {0.0f, 0.0f, 0.0f};
int laser0_data = 0;
int laser1_data = 0;

void setup()
{
	// configure LED for output
	pinMode(LED_BUILTIN, OUTPUT);
	SignalBuiltinLED(7, 50);
	Wire.begin();
	Wire.setClock(400000);
	Wire2.begin();
	Wire2.setClock(400000);
	Serial.begin(115200);
	while (!Serial)
		SignalBuiltinLED(2, 50);
	; // wait for Leonardo enumeration, others continue immediately
	mpu = new Gyro(115200);

	laser0.setTimeout(500);
	laser1.setBus(&Wire2);
	laser1.setTimeout(500);
	if (!laser0.init())
	{
		SignalBuiltinLED(7, 50);
		Serial.println("Failed to detect and initialize laser0!");
		while (1)
			;
	}
	if (!laser1.init())
	{
		SignalBuiltinLED(7, 50);
		Serial.println("Failed to detect and initialize laser1!");
		while (1)
			;
	}

	// Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
	// You can change these settings to adjust the performance of the sensor, but
	// the minimum timing budget is 20 ms for short distance mode and 33 ms for
	// medium and long distance modes. See the VL53L1X datasheet for more
	// information on range and timing limits.
	laser0.setDistanceMode(VL53L1X::Long);
	laser0.setMeasurementTimingBudget(60000);
	laser1.setDistanceMode(VL53L1X::Long);
	laser1.setMeasurementTimingBudget(60000);

	// Start continuous readings at a rate of one measurement every 50 ms (the
	// inter-measurement period). This period should be at least as long as the
	// timing budget.
	laser0.startContinuous(60);
	laser1.startContinuous(60);
}

void loop()
{
	gyro = mpu->GetGyroData();
#if DEBUG == true
	/*
	Serial.print(gyro.x);
	Serial.print(", ");
	Serial.print(gyro.y);
	Serial.print(", ");
	Serial.print(gyro.z);
	Serial.println();
	*/
	if (laser0.dataReady())
		laser0_data = laser0.read(false);
	if (laser1.dataReady())
		laser1_data = laser1.read(false);
	doc[DATANAME_GYRO_X] = gyro.x;
	doc[DATANAME_GYRO_Y] = gyro.y;
	doc[DATANAME_GYRO_Z] = gyro.z;
	doc[DATANAME_LASER_0] = laser0_data;
	doc[DATANAME_LASER_1] = laser1_data;
	doc["dummy_data"] = 0;

	serializeJson(doc, res);
	Serial.println(res);
	res = "";
	if (laser0.timeoutOccurred())
		SignalBuiltinLED(2, 30);
	//delay(4);
#endif
}
