
#include "Robot.h"

Robot::Robot()
{
	LOG("Disabling and then enabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);										// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWrite(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
	delay(10);										// Wait for sensors to wake up (especially sensor 0)
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
	lasers->StartRanging(16, 60, ELIA::RangingMode::kContinuous);	 // 4*4, 60Hz
	UpdateSensorNumBlocking(1);
	back_distance_before = lasers->sensors[1]->GetData()->distance_mm[2];
	pinMode(R_PIN_BUTTON, INPUT);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(100);
	digitalWrite(LED_BUILTIN, LOW);

}

void Robot::Run()
{
	if (!StopRobot()) {
		bool fermo = false;
		bool test = true;
		if (test) {
			if (fermo) {
				ms->SetPower(0,0);
			} else {
				desired_angle = mpu_data.x + 45;
				if (desired_angle >= 180)
				{
					desired_angle -= 360;
				}
				while (!(mpu_data.x > desired_angle && mpu_data.x * desired_angle > 0))
				{
					UpdateGyroBlocking();
					ms->SetPower(60, 0);
					if (StopRobot()) {
						break;
					}
				}
				ms->SetPower(0,0);
				desired_angle = mpu_data.x + 45;
				if (desired_angle <= -180)
				{
					desired_angle += 360;
				}
				while (!(mpu_data.x > desired_angle && mpu_data.x * desired_angle > 0))
				{
					UpdateGyroBlocking();
					ms->SetPower(0, -60);
					if (StopRobot()) {
						break;
					}
				}
				ms->SetPower(0, 0);
			}
		}
		/*
		bool fermo = true;
		while (fermo) {
			UpdateSensorNumBlocking(SENSOR_BW);
			Serial.println(lasers->sensors[SENSOR_BW]->GetData()->distance_mm[2]);
		}
		
		Serial.print("Destra:");
		Serial.println(lasers->sensors[3]->GetData()->distance_mm[2]);
		Serial.print("Gyro: ");
		Serial.println(mpu_data.x);
		if (lasers->sensors[3]->GetData()->distance_mm[2] >= 250 and not ignore_right)
		{
			Serial.println("Varco a destra------------------------------------------");
			while (!(lasers->sensors[1]->GetData()->distance_mm[2] > back_distance_before + 300))
			{
				UpdateSensorNumBlocking(1);
				Serial.print("Adesso: ");
				Serial.println(lasers->sensors[1]->GetData()->distance_mm[2]);
				Serial.print("Prima: ");
				Serial.println(back_distance_before);
				ms->SetPower(43, 43);
			}
			ms->SetPower(0, 0);
			ignore_right = true;
			desired_angle = mpu_data.x + 90;
			if (desired_angle >= 180)
			{
				desired_angle -= 360;
			}
			while (!(mpu_data.x > desired_angle && mpu_data.x * desired_angle > 0))
			{
				UpdateGyroBlocking();
				Serial.print("Gyro: ");
				Serial.println(mpu_data.x);
				Serial.print("Gyro desiderato: ");
				Serial.println(desired_angle);
				ms->SetPower(43, -43);
			}
			ms->SetPower(0, 0);
			UpdateSensorNumBlocking(1);
			back_distance_before = lasers->sensors[1]->GetData()->distance_mm[2];
		}
		else
		{
			if (lasers->sensors[SENSOR_DX]->GetData()->distance_mm[2] < 150 and ignore_right){
				ignore_right = false;
			}
			if (lasers->sensors[1]->GetData()->distance_mm[2] >= back_distance_before + 300)
			{
				back_distance_before = lasers->sensors[1]->GetData()->distance_mm[2];
			}
			if (lasers->sensors[0]->GetData()->distance_mm[2] < 80)
			{
				Serial.println("Muro frontale");
				if (lasers->sensors[2]->GetData()->distance_mm[2] >= 200)
				{
					desired_angle = mpu_data.x - 90;
					if (desired_angle <= -180)
					{
						desired_angle += 360;
					}
					while (!(mpu_data.x < desired_angle && mpu_data.x * desired_angle > 0))
					{
						UpdateGyroBlocking();
						ms->SetPower(-43, 43);
					}
					ms->SetPower(0, 0);
					UpdateSensorNumBlocking(1);
					back_distance_before = lasers->sensors[1]->GetData()->distance_mm[2];
				}
				else
				{
					desired_angle = mpu_data.x - 180;
					if (desired_angle <= -180)
					{
						desired_angle += 360;
					}
					while (!(mpu_data.x < desired_angle && mpu_data.x * desired_angle > 0))
					{
						UpdateGyroBlocking();
						ms->SetPower(-43, 43);
					}
					ms->SetPower(0, 0);
					UpdateSensorNumBlocking(1);
					back_distance_before = lasers->sensors[1]->GetData()->distance_mm[2];
				}
			}
			UpdateGyroBlocking();
			if (mpu_data.x < desired_angle) {
				ms->SetPower(43 + 20, 43);
			} else if (mpu_data.x > desired_angle){
				ms->SetPower(43, 43 + 20);
			} else{
				ms->SetPower(43, 43);
			}
		}
		*/
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

bool Robot::StopRobot()
{
	stop_the_robot = true;
	if (digitalRead(R_PIN_BUTTON)) {
		stop_the_robot = false;
	}
	Serial.println(stop_the_robot);
	return stop_the_robot;
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
	//if (mpu_data_ready)
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

void Robot::UpdateSensorNumBlocking(uint8_t num)
{
	while (1)
	{
		if(lasers_data_ready[num]){
			lasers->sensors[num]->UpdateData();
			lasers_data_ready[num] = false;
			break;
		}
	}
}

void Robot::UpdateGyroBlocking(){
	while (mpu_data_ready)
	{
		mpu_data = mpu->GetGyroData();
		mpu_data_ready = false;
	}
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
