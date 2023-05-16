
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
	IMU->setGyroODR(IMU->odr2k);
	delay(10);
	IMU->setAccelODR(IMU->odr2k);
	delay(10);
	IMU->enableAccelGyroLN();
	delay(10);
	FusionAhrsInitialise(&ahrs);
	pastMicros = micros();
}

uint8_t gyro::UpdateData()
{
	uint8_t status = IMU->getAGT();
	delta_micros = micros() - pastMicros;
	// double est_x_acc_rad = atanf(IMU->accY() / IMU->accZ());
	// double est_y_acc_rad = -asinf(IMU->accX()); // Works only if stationary
	delta_seconds = delta_micros / 1e6;
	z -= IMU->gyrZ() * delta_seconds;
	gyroscope.axis = {IMU->gyrX(), IMU->gyrY(), IMU->gyrZ()};
	accelerometer.axis = {IMU->accX(), IMU->accY(), IMU->accZ()};
	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_seconds);
	euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

	// printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
	//  y += IMU->gyrY() * delta_micros / 1e6;
	//  x += IMU->gyrX() * delta_micros / 1e6;
	/*
		Serial.print("{\"2&n&l&Gyro Z\":");
	Serial.print(z);
	Serial.print("}");
	Serial.print("{\"3&n&l&Gyro reading time\":");
	Serial.print(delta_micros);
	Serial.print("}");
	x = euler.angle.roll;
	*/
	y = euler.angle.pitch;
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
