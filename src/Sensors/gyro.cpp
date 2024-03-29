
#include "Sensors/gyro.h"

gyro::gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin) : x(0.0f), y(0.0f), z(0.0f)
{
	/*
		pinMode(extClkPin, OUTPUT);
		analogWriteFrequency(extClkPin, 36000);
		analogWrite(extClkPin, 128);
	*/

	IMU = new ICM42688_FIFO(bus, csPin);

	uint8_t status = IMU->begin();
	status |= IMU->biasCalibrationRoutine();
	if (status != 0)
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

	Serial.print("X accel bias: ");
	Serial.println(IMU->getAccelBiasX_mss());
	Serial.print("Y accel bias: ");
	Serial.println(IMU->getAccelBiasY_mss());
	Serial.print("Z accel bias: ");
	Serial.println(IMU->getAccelBiasZ_mss());

	status |= IMU->enableDataReadyInterrupt();
	status |= IMU->setFilters(true, false);
	status |= IMU->configureUIFilter(AAF_3db_bw_hz::bw_3057, AAF_3db_bw_hz::bw_3057);
	status |= IMU->setAccelFS(IMU->gpm2);
	status |= IMU->setGyroFS(IMU->dps500);
	status |= IMU->setGyroODR(IMU->odr100);  // If this gets modified, also delta_seconds needs to change in gyro.h
	status |= IMU->setAccelODR(IMU->odr100); // If this gets modified, also delta_seconds needs to change in gyro.h
	status |= IMU->enableFifo();
	status |= IMU->enableAccelGyroLN();

	if (status == 0)
	{
		Serial.println("Everything gyro-related initialized successfully");
	}
	else
	{
		Serial.print("An error occured when initializing gyro with status: ");
		Serial.println(status);
	}

	FusionAhrsInitialise(&ahrs);
	past_millis_count_ = millis();
	IMU->readFifo(ahrs, z, false);
	FusionAhrsReset(&ahrs);
}

uint8_t gyro::UpdateData(bool integrate_position)
{
	uint8_t status = IMU->readFifo(ahrs, z, integrate_position);
	if (status != 0)
		Serial.println("An error occured while accessing the gyro's FIFO buffer");

	euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	x = euler.angle.roll;
	y = euler.angle.pitch;
/*
	LINEGRAPH_MULTI("Degrees X", x, 5);
	LINEGRAPH_MULTI("Degrees Y", y, 5);
	LINEGRAPH_MULTI("Degrees Z", z, 5);
*/
	return status;
}

float gyro::GetPosition(){
	return IMU->getPosition();
}

void gyro::ResetPosition(){
	IMU->resetPosition();
}

void gyro::ResetZ()
{
	z = 0.0f;
}

gyro::~gyro()
{
}
