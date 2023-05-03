
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
	IMU->setGyroFS(IMU->dps1000);
	delay(10);
	IMU->setGyroODR(IMU->odr1k);
	delay(10);
	IMU->setAccelODR(IMU->odr1k);
	delay(10);
	IMU->setAccelFS(IMU->gpm2);
	delay(10);
	IMU->enableAccelGyroLN();
	delay(10);
	pastMicros = micros();
}

uint8_t gyro::UpdateData()
{
	uint8_t status = IMU->getAGT();
	uint32_t delta_micros = micros() - pastMicros;
	double est_x_acc_rad = atanf(IMU->accY() / IMU->accZ());
	double est_y_acc_rad = -asinf(IMU->accX());
	z -= IMU->gyrZ() * delta_micros / 1e6;
	//y += IMU->gyrY() * delta_micros / 1e6;
	//x += IMU->gyrX() * delta_micros / 1e6;
	x = est_x_acc_rad * 57.296;
	y = est_y_acc_rad * 57.296;
	pastMicros = micros();
	return status;
}

void gyro::ResetZ()
{
	z = 0.0f;
}

gyro::~gyro()
{
}
