
#include "Sensors/Laser.h"

Laser::Laser(TwoWire *bus): distance(0)
{
	laser.setBus(bus);
	laser.setTimeout(800);
	if (!laser.init())
	{
#if DEBUG == true
		SignalBuiltinLED(7, 50);
		Serial.println("Failed to detect and initialize laser!");
//		while (1)
//			;
#endif
	}
	laser.setDistanceMode(VL53L1X::Long);
	laser.setROISize(4, 4);
	laser.setMeasurementTimingBudget(10/*ms*/ * 1000);
	laser.startContinuous(20);
}

void Laser::SetROICenter(int center){
	laser.setROICenter(center);
}

#if DEBUG == true
Laser::Laser(TwoWire *bus, int ledPin): distance(0)
{
	laser.setBus(bus);
	laser.setTimeout(500);
	if (!laser.init())
	{
#if DEBUG == true
		digitalWrite(ledPin, HIGH);
		Serial.println("Failed to detect and initialize laser!");
#endif
	}
	laser.setROISize(4, 4);
	laser.setROICenter(199);
	laser.setDistanceMode(VL53L1X::Long);
	laser.setMeasurementTimingBudget(70000);
	laser.startContinuous(60);
}
#endif

Laser::~Laser()
{
}

int Laser::GetData()
{
	//if (laser.dataReady())
		distance = laser.read(/*false*/);
	Serial.print(distance);
	Serial.print(" -> ");
	Serial.print(laser.ranging_data.range_status);
	Serial.print(" -> ");
	Serial.println(laser.getROICenter());
	return distance;
}
