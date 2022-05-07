
#include <Arduino.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <VL53L1X.h>

#include "config.h"
#include "dataname.h"
#include "util.h"
#include "Sensors/Gyro.h"
#include "Sensors/Laser.h"

DynamicJsonDocument doc(1024);
String res = "";

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

void loop()
{
	gyro = mpu->GetGyroData();
	laser0_data = l0->GetData();
	laser1_data = l1->GetData();

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
