
#include "Sensors/Gyro.h"

Gyro::Gyro(bool cold_start) : gyro({0.0f, 0.0f, 0.0f}), offset({0.0f, 0.0f, 0.0f})
{
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0)
	{
		// Changed by Dave on 08/03/2023 - 11:49
		mpu.setXGyroOffset(37);
		mpu.setYGyroOffset(93);
		mpu.setZGyroOffset(-393);
		mpu.setXAccelOffset(-3556);
		mpu.setYAccelOffset(1822);
		mpu.setZAccelOffset(1356);
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

	if (cold_start)
	{
		GyroData temp;
		uint16_t i = 0;
		StartTime = millis();
		uint16_t waitToStabilize = millis() + 15 * 1000; // Current time + 11s
		Serial.println("Stabilizing gyro:");
		while (millis() < waitToStabilize)
		{
			Serial.print(".");
			GetGyroData(temp);
			delay(3);
		}
		delay(100);
		ResetX();
	}
	StartTime = millis();
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
		gyro.y = ypr[1] * 180 / M_PI;
		gyro.z = ypr[2] * 180 / M_PI;
		double Gx = v[2] / 16.4;
		gyro.x -= Gx * (ElapsedTime * 0.001);
		//gyro.z += Gz * (ElapsedTime * 0.001);
		//gyro.y -= Gy * (ElapsedTime * 0.001);
		data = gyro;
		StartTime = millis();
		return 0;
	}
	else
	{
		return status;
	}
}

void Gyro::ResetX()
{
	gyro.x = 0;
}