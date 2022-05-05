
#include <Arduino.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <VL53L1X.h>

#include "config.h"
#include "dataname.h"
#include "util.h"
#include "Gyro.h"

// DynamicJsonDocument doc(1024);

VL53L1X sensor;

Gyro *mpu;
GyroData gyro;

void setup()
{
	// configure LED for output
	pinMode(LED_BUILTIN, OUTPUT);
	SignalBuiltinLED(7, 50);
	Serial.begin(115200);
	while (!Serial)
		; // wait for Leonardo enumeration, others continue immediately
	mpu = new Gyro(115200);

	sensor.setTimeout(500);
	if (!sensor.init())
	{
		Serial.println("Failed to detect and initialize sensor!");
		while (1)
			;
	}

	// Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
	// You can change these settings to adjust the performance of the sensor, but
	// the minimum timing budget is 20 ms for short distance mode and 33 ms for
	// medium and long distance modes. See the VL53L1X datasheet for more
	// information on range and timing limits.
	sensor.setDistanceMode(VL53L1X::Long);
	sensor.setMeasurementTimingBudget(50000);

	// Start continuous readings at a rate of one measurement every 50 ms (the
	// inter-measurement period). This period should be at least as long as the
	// timing budget.
	sensor.startContinuous(50);
}

void loop()
{
	gyro = mpu->GetGyroData();
#if DEBUG == true
	Serial.print(gyro.x);
	Serial.print(", ");
	Serial.print(gyro.y);
	Serial.print(", ");
	Serial.print(gyro.z);
	Serial.println();
	/*
	doc[DATANAME_GYRO_X] = ypr[0] * 180 / M_PI;
	doc[DATANAME_GYRO_Y] = ypr[1] * 180 / M_PI;
	doc[DATANAME_GYRO_Z] = ypr[2] * 180 / M_PI;
	String res = "";
	serializeJson(doc, res);
	Serial.println(res);
	*/
	Serial.print(sensor.read());
	if (sensor.timeoutOccurred())
	{
		Serial.print(" TIMEOUT");
	}

	Serial.println();

	delay(1);
#endif
}
