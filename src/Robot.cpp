
#include "Robot.h"

Robot::Robot()
{
	LOG("Disabling and then enabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);													// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
	delay(10);													// Wait for sensors to wake up (especially sensor 0)
	LOG("...done!");

	Wire.begin(); // Gyro
	Wire.setClock(400000);

	LOG("Gyro setup started");
	mpu = new Gyro();
	mpu_data_ready = false;
	attachInterrupt(R_PIN_GYRO_INT, R_MPU6050_int, RISING);
	LOG("Finished gyro setup!");

	LOG("Motor setup started");
	ms = new Motors();
	LOG("Finished motor setup!");

	Wire2.begin(); // Lasers
	Wire2.setClock(1000000);

	LOG("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2);
	LOG("Laser sensors setup finished");

	/*
		Interrupts *MUST* be attached after the VL53L5CX_manager is instantiated,
		otherwise if the data_ready array isn't initialized and an interrupt is
		fired, the program will crash.
	*/
	attachInterrupt(VL53L5CX_int_pin[0], R_VL53L5CX_int_0, FALLING); // sensor_0
	attachInterrupt(VL53L5CX_int_pin[1], R_VL53L5CX_int_1, FALLING); // sensor_1
	attachInterrupt(VL53L5CX_int_pin[2], R_VL53L5CX_int_2, FALLING); // sensor_2
	attachInterrupt(VL53L5CX_int_pin[3], R_VL53L5CX_int_3, FALLING); // sensor_3
	lasers->StartRanging(16, 60, ELIA::RangingMode::kContinuous);	  // 4*4, 60Hz

	bool ignore_right = false;

	ms->SetPower(40,40);
	/*
	while (1)
	{
		if (lasers_data_ready[3])
		{
			lasers->sensors[3]->UpdateData();
			lasers_data_ready[3] = false;
		}
		if (lasers_data_ready[0])
		{
			lasers->sensors[0]->UpdateData();
			lasers_data_ready[0] = false;
		}
		if (lasers_data_ready[2])
		{
			lasers->sensors[2]->UpdateData();
			lasers_data_ready[2] = false;
		}

		if(lasers->sensors[0]->GetData()->distance_mm[2] < 50) {
			ms->SetPower(0,0);
			if(lasers->sensors[2]->GetData()->distance_mm[2] > 200) {
				ms->SetPower(-60, 60);
				delay(1200);
				ms->SetPower(50,50);
			} else {
				ms->SetPower(-60, 60);
				delay(2400);
				ms->SetPower(50,50);
			}
		}

		if(lasers->sensors[3]->GetData()->distance_mm[2] > 200 && !ignore_right){
			delay(600);
			ignore_right = true;
			ms->SetPower(60, -60);
			delay(1200);
			ms->SetPower(50,50);
		} else if(lasers->sensors[3]->GetData()->distance_mm[2] < 150 && ignore_right){
			ignore_right = false;
		}
	}
	*/

	// Rampa:
prima:
	ms->SetPower(100,100);
	delay(5000);
	ms->SetPower(0,0);
	
	while (1)
	{
		if(!digitalRead(R_PIN_BUTTON))
		goto prima;
	}
	
}

void Robot::R_MPU6050_int()
{
	mpu_data_ready = true;
}

void Robot::R_VL53L5CX_int_0()
{
	lasers_data_ready[0] = true;
}

void Robot::R_VL53L5CX_int_1()
{
	lasers_data_ready[1] = true;
}

void Robot::R_VL53L5CX_int_2()
{
	lasers_data_ready[2] = true;
}

void Robot::R_VL53L5CX_int_3()
{
	lasers_data_ready[3] = true;
}

uint8_t Robot::TrySensorDataUpdate()
{
	/*
		Status is formed:
		0000 0000
		HIGH  LOW
		Low nibble -> If value is 1 gyro read data
		High nibble -> Each bit represents if data was successfully read from that sensor:
		From	[0] -> 0001
		To 	[3] -> 1000
		If every laser sensor was read successfully -> status >> 4 = 0b1111 = 0xF = 15

		If every sensor was read successfully status = 0b11110001 = 0xF1 = 241 
	*/
	uint8_t status = 0;
	if (mpu_data_ready)
	{
		mpu_data = mpu->GetGyroData();
		mpu_data_ready = false;
		status++;
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		if (lasers_data_ready[i])
		{
			lasers->sensors[i]->UpdateData();
			lasers_data_ready[i] = false;
			status |= 0b10000 << i;
		}
	}
	return status;
}

void Robot::PrintSensorData()
{
	Serial.print("gyro.x: \t");
	Serial.print(mpu_data.x);
	Serial.print(" \t");
	Serial.print("gyro.y: \t");
	Serial.print(mpu_data.y);
	Serial.print(" \t");
	Serial.print("gyro.z: \t");
	Serial.println(mpu_data.z);

	for (uint8_t i = 0; i < 4; i++)
	{

		Serial.print("Sensor number ");
		Serial.println(i);

		for (uint8_t j = 0; j < lasers->resolution; j++)
		{
			Serial.print(lasers->sensors[i]->GetData()->distance_mm[j]);
			Serial.print(", \t");
			if (j == 3 || j == 7 || j == 11 || j == 15)
			{
				Serial.println();
			}
		}
		Serial.println();
	}
}

Robot::~Robot()
{
}
