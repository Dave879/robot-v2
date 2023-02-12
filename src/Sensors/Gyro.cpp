
#include "Sensors/Gyro.h"

Gyro::Gyro() : gyro({0.0f, 0.0f, 0.0f}), offset({0.0f, 0.0f, 0.0f})
{
	// mpu.initialize();
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
		// mpu.CalibrateAccel(10);
		// mpu.CalibrateGyro(10);
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
	StartTime = millis();
	uint16_t waitToStabilize = millis() + 15 * 1000; // Current time + 11s
	GyroData temp;
	uint16_t i = 0;
	while (millis() < waitToStabilize)
	{
		GetGyroData(temp);
		Serial.print("Stabilizing gyro: ");
		Serial.println(i++);
		delay(3);
	}
	delay(100);
	Reset();
}

uint8_t Gyro::GetGyroData(GyroData &data)
{
	int8_t status = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
	if (status == 1)
	{
		uint32_t ElapsedTime = millis() - StartTime;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		int16_t v[3] = {0, 0, 0};
		mpu.dmpGetGyro(&v[0], fifoBuffer);
		// gyro.x = ypr[0] * 180 / M_PI;
		double Gx = v[2] / 65.5;
		data.y = ypr[1] * 180 / M_PI - offset.y;
		data.z = ypr[2] * 180 / M_PI - offset.z;
		data.x -= Gx * (ElapsedTime * 0.001);
		gyro = data;
		StartTime = millis();
		return 0;
	}
	else
	{
		return status;
	}
}

void Gyro::Reset()
{
	offset.x = gyro.x;
	offset.y = gyro.y;
	offset.z = gyro.z;
	gyro.x -= offset.x;
}