
#include <Arduino.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <VL53L1X.h>

#include "config.h"
#include "dataname.h"
#include "util.h"
#include "Sensors/Gyro.h"
#include "Sensors/Laser.h"
#include "Sensors/Color.h"

DynamicJsonDocument doc(1024);
String res = "";

tcs34725 rgb_sensor;

Laser *l0, *l1;
Gyro *mpu;
GyroData gyro = {0.0f, 0.0f, 0.0f};
int laser0_data = 0;
int laser1_data = 0;

void setup()
{
	SetupDebugLEDs();
	Serial.begin(115200);
	Wire.begin();
	Wire.setClock(400000);
	Wire2.begin();
	Wire2.setClock(400000);
	SignalBuiltinLED(7, 50);
	while (!Serial)
		SignalBuiltinLED(2, 50);
	;

	rgb_sensor.begin();

#if DEBUG == true
	mpu = new Gyro(LED_0);
	l0 = new Laser(&Wire, LED_1);
	l1 = new Laser(&Wire2, LED_2);
#else
	mpu = new Gyro();
	l0 = new Laser(&Wire);
	l1 = new Laser(&Wire2);
#endif
}

void printColorSensor()
{
	Serial.print(F("Gain:"));
	Serial.print(rgb_sensor.againx);
	Serial.print(F("x "));
	Serial.print(F("Time:"));
	Serial.print(rgb_sensor.atime_ms);
	Serial.print(F("ms (0x"));
	Serial.print(rgb_sensor.atime, HEX);
	Serial.println(F(")"));

	Serial.print(F("Raw R:"));
	Serial.print(rgb_sensor.r);
	Serial.print(F(" G:"));
	Serial.print(rgb_sensor.g);
	Serial.print(F(" B:"));
	Serial.print(rgb_sensor.b);
	Serial.print(F(" C:"));
	Serial.println(rgb_sensor.c);

	Serial.print(F("IR:"));
	Serial.print(rgb_sensor.ir);
	Serial.print(F(" CRATIO:"));
	Serial.print(rgb_sensor.cratio);
	Serial.print(F(" Sat:"));
	Serial.print(rgb_sensor.saturation);
	Serial.print(F(" Sat75:"));
	Serial.print(rgb_sensor.saturation75);
	Serial.print(F(" "));
	Serial.println(rgb_sensor.isSaturated ? "*SATURATED*" : "");

	Serial.print(F("CPL:"));
	Serial.print(rgb_sensor.cpl);
	Serial.print(F(" Max lux:"));
	Serial.println(rgb_sensor.maxlux);

	Serial.print(F("Compensated R:"));
	Serial.print(rgb_sensor.r_comp);
	Serial.print(F(" G:"));
	Serial.print(rgb_sensor.g_comp);
	Serial.print(F(" B:"));
	Serial.print(rgb_sensor.b_comp);
	Serial.print(F(" C:"));
	Serial.println(rgb_sensor.c_comp);

	Serial.print(F("Lux:"));
	Serial.print(rgb_sensor.lux);
	Serial.print(F(" CT:"));
	Serial.print(rgb_sensor.ct);
	Serial.println(F("K"));
}

void loop()
{
	gyro = mpu->GetGyroData();
	laser0_data = l0->GetData();
	laser1_data = l1->GetData();
	START_TIMER
	rgb_sensor.getData();
	END_TIMER

	printColorSensor();

#if DEBUG == true
	doc[DATANAME_GYRO_X] = gyro.x;
	doc[DATANAME_GYRO_Y] = gyro.y;
	doc[DATANAME_GYRO_Z] = gyro.z;
	doc[DATANAME_LASER_0] = laser0_data;
	doc[DATANAME_LASER_1] = laser1_data;
	doc["dummy_data"] = 0;
	serializeJson(doc, res);
	Serial.println(res);
	res = "";
	// delay(4);
#endif
}
