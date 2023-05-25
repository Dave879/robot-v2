
#include "Sensors/gyro.h"

gyro::gyro(SPIClass &bus, uint8_t csPin, uint8_t extClkPin) : x(0.0f), y(0.0f), z(0.0f)
{
	/*
		pinMode(extClkPin, OUTPUT);
		analogWriteFrequency(extClkPin, 36000);
		analogWrite(extClkPin, 128);
	*/

	IMU = new ICM42688_FIFO(bus, csPin, 24000000);

	int status = IMU->begin();
	IMU->innerCalRoutine____temp();
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

	// Bad solution but i don't care, there are bigger issues to fix - Dave 23/05/23
#define CHECKSTATUS(x)                        \
	if (x != 1)                                \
	{                                          \
		Serial.println("A gyro config failed"); \
	}

	status = IMU->enableDataReadyInterrupt();
	CHECKSTATUS(status)
	status = IMU->setFilters(true, false);
	CHECKSTATUS(status)
	status = IMU->configureUIFilter(AAF_3db_bw_hz::bw_3057, AAF_3db_bw_hz::bw_3057);
	CHECKSTATUS(status)
	status = IMU->setGyroFS(IMU->dps500);
	CHECKSTATUS(status)
	status = IMU->setGyroODR(IMU->odr100); // If this gets modified, also delta_seconds needs to change in gyro.h
	CHECKSTATUS(status)
	status = IMU->setAccelODR(IMU->odr100); // If this gets modified, also delta_seconds needs to change in gyro.h
	CHECKSTATUS(status)
	status = IMU->enableFifo(true, true, false);
	CHECKSTATUS(status)
	status = IMU->enableAccelGyroLN();
	CHECKSTATUS(status)

	FusionAhrsInitialise(&ahrs);
	past_millis_count_ = millis();
	IMU->readFifo();
}

uint8_t gyro::UpdateData()
{
	int8_t status = IMU->readFifo();
	CHECKSTATUS(status)
	for (size_t i = 0; i < IMU->_fifoSize; i++)
	{
		z -= IMU->_gzFifo[i] * delta_seconds;
		drift_last_sec += abs(IMU->_gzFifo[i] * delta_seconds);
		gyroscope.axis = {IMU->_gxFifo[i], IMU->_gyFifo[i], IMU->_gzFifo[i]};
		accelerometer.axis = {IMU->_axFifo[i], IMU->_ayFifo[i], IMU->_azFifo[i]};
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_seconds);
		/*
		LINEGRAPH_MULTI("Z gyro raw", IMU->_gzFifo[i], 1);
		LINEGRAPH_MULTI("X gyro raw", IMU->_gxFifo[i], 1);
		LINEGRAPH_MULTI("Y gyro raw", IMU->_gyFifo[i], 1);
		LINEGRAPH_MULTI("Z accel raw", IMU->_azFifo[i], 2);
		LINEGRAPH_MULTI("X accel raw", IMU->_axFifo[i], 2);
		LINEGRAPH_MULTI("Y accel raw", IMU->_ayFifo[i], 2);
		*/
		count_++;
	}
	//LINEGRAPH("Integrated Z", z);
	euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

/*
	if (past_millis_count_ + 1000 < millis())
	{
		past_millis_count_ = millis();
		Serial.print("Read count: ");
		Serial.print(count_);
		Serial.print("\t Drift last sec: ");
		Serial.print(drift_last_sec);
		Serial.print("°");
		Serial.print("\t z:");
		Serial.print(z);
		Serial.print("°\t ");
		Serial.print(millis() / 1000);
		Serial.println("s");
		drift_last_sec = 0;
		count_ = 0;
	}
*/
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
