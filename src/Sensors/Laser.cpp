
#include "Sensors/Laser.h"

Laser::Laser(TwoWire *bus)
{
	laser.setBus(bus);
	laser.setTimeout(500);
	if (!laser.init())
	{
		SignalBuiltinLED(7, 50);
		Serial.println("Failed to detect and initialize laser!");
		while (1)
			;
	}
	laser.setDistanceMode(VL53L1X::Long);
	laser.setMeasurementTimingBudget(60000);
	laser.startContinuous(60);
}

Laser::~Laser()
{
}

int Laser::GetData()
{
	if (laser.dataReady())
		distance = laser.read(false);
	return distance;
}