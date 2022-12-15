
#include "Sensors/Gyro.h"

Gyro::Gyro() : gyro({-1.0f, -1.0f, -1.0f})
{
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(51);
	mpu.setYGyroOffset(8);
	mpu.setZGyroOffset(21);
	mpu.setXAccelOffset(1150);
	mpu.setYAccelOffset(-50);
	mpu.setZAccelOffset(1060);
	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(10);
		mpu.CalibrateGyro(10);
		mpu.setDMPEnabled(true);
		packetSize = mpu.dmpGetFIFOPacketSize();
		Serial.println();
		Serial.println("Successfully initialized gyro");
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

GyroData Gyro::GetGyroData()
{
	int8_t status = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
	if (status == 1)
	{
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		gyro.x = ypr[0] * 180 / M_PI;
		gyro.y = ypr[1] * 180 / M_PI;
		gyro.z = ypr[2] * 180 / M_PI;
		return gyro;
	}
	else
	{
		return { (double)status, (double)status, (double)status };
	}
}