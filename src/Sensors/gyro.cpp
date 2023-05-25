
#include "Sensors/gyro.h"

gyro::gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin) : x(0.0f), y(0.0f), z(0.0f)
{
	/*
		pinMode(extClkPin, OUTPUT);
		analogWriteFrequency(extClkPin, 36000);
		analogWrite(extClkPin, 128);
	*/

	IMU = new ICM42688_FIFO(bus, csPin, 24000000);

	uint8_t status = IMU->begin();
	status |= IMU->firstCalibration();
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
	Serial.print("Z gyro bias: ");
	Serial.println(IMU->getGyroBiasZ());

	status |= IMU->enableDataReadyInterrupt();
	status |= IMU->setFilters(true, false);
	status |= IMU->configureUIFilter(AAF_3db_bw_hz::bw_3057, AAF_3db_bw_hz::bw_3057);
	status |= IMU->setGyroFS(IMU->dps500);
	status |= IMU->setGyroODR(IMU->odr100); // If this gets modified, also delta_seconds needs to change in gyro.h
	status |= IMU->setAccelODR(IMU->odr100); // If this gets modified, also delta_seconds needs to change in gyro.h
	status |= IMU->enableFifo();
	status |= IMU->enableAccelGyroLN();

	if (status == 0)
	{
		Serial.println("Everything gyro-related initialized successfully");
	} else {
		Serial.print("An error occured when initializing gyro with status: ");
		Serial.println(status);
	}

	FusionAhrsInitialise(&ahrs);
	past_millis_count_ = millis();
	IMU->readFifo(ahrs, z);
}

uint8_t gyro::UpdateData()
{
	uint8_t status = IMU->readFifo(ahrs, z);
	if (status != 0)
		Serial.println("An error occured while accessing the gyro's FIFO buffer");
	
	euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	x = euler.angle.roll;
	y = euler.angle.pitch;
	return status;
}

void gyro::ResetZ()
{
	z = 0.0f;
}

gyro::~gyro()
{
}
