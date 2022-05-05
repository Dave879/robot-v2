
#include "Gyro.h"

Gyro::Gyro(int baudrate) : gyro({-1.0f, -1.0f, -1.0f})
{
	Serial.begin(baudrate);
	pinMode(LED_BUILTIN, OUTPUT);
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
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
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.setDMPEnabled(true);
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
		SignalBuiltinLED(4, 100);
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
		SignalBuiltinLED(15, 100);
	}
}

GyroData Gyro::GetGyroData()
{
	if (!dmpReady)
		return {-1.0f, -1.0f, -1.0f};
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
	{
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		gyro.x = ypr[0] * 180 / M_PI;
		gyro.y = ypr[1] * 180 / M_PI;
		gyro.z = ypr[2] * 180 / M_PI;
	}
	return gyro;
}