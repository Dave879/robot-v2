
#include "Sensors/Gyro.h"

Gyro::Gyro() : gyro({-1.0f, -1.0f, -1.0f})
{
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// Changed by Dave on 16/12/2022 - 17:42
	mpu.setXGyroOffset(39);
	mpu.setYGyroOffset(98);
	mpu.setZGyroOffset(-397);
	mpu.setXAccelOffset(-3617);
	mpu.setYAccelOffset(1799);
	mpu.setZAccelOffset(1362);

	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		//mpu.CalibrateAccel(10);
		//mpu.CalibrateGyro(10);
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
		Serial.print("DMP Initialization failed (code ");
		Serial.print(devStatus);
		Serial.println(")");
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