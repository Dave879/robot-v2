
#include "Sensors/gyro.h"

gyro::gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin) : x(0.0f), y(0.0f), z(0.0f)
{
	pinMode(extClkPin, OUTPUT);
	analogWriteFrequency(extClkPin, 36000);
	analogWrite(extClkPin, 128);

	IMU = new ICM42688(bus, csPin);

	int status = IMU->begin();
	if (status < 0)
	{
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
	}
	else
	{
		Serial.println("Found ICM42688P");
	}
	IMU->disableAccelGyro();
	delay(10);
	IMU->enableDataReadyInterrupt();
	delay(10);
	IMU->enableExternalClock();
	delay(10);
	IMU->setGyroFS(IMU->dps500);
	delay(10);
	IMU->setGyroODR(IMU->odr200);
	delay(10);
	IMU->setAccelODR(IMU->odr200);
	delay(10);
	IMU->enableAccelGyroLN();
	delay(10);
	pastMicros = micros();
}

uint8_t gyro::UpdateData()
{
	uint8_t status = IMU->getAGT();
	uint32_t delta_micros = micros() - pastMicros;
	x += IMU->gyrX() * delta_micros / 1e6;
	y += IMU->gyrY() * delta_micros / 1e6;
	z -= IMU->gyrZ() * delta_micros / 1e6;
	pastMicros = micros();
	return status;
}

void gyro::ResetAxis()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

gyro::~gyro()
{
}
