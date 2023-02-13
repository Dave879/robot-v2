
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
	back_distance_before = lasers->sensors[1]->GetData()->distance_mm[6];
	pinMode(R_PIN_BUTTON, INPUT);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(100);
	digitalWrite(LED_BUILTIN, LOW);
}

void Robot::Run()
{
	if (StopRobot())
	{
		// Controllo la presenza di un varco a destra
		if (lasers->sensors[SENSOR_DX]->GetData()->distance_mm[5] >= MIN_DISTANCE_TO_TURN_RIGHT_MM && !ignore_right)
		{
			Serial.println("Varco a destra!!!");
			Serial.println("Distanza Destra: ");
			Serial.print(lasers->sensors[SENSOR_DX]->GetData()->distance_mm[5]);
			// Varco trovato!
			ms->SetPower(0, 0);
			/*
			int16_t back_distance_to_reach = back_distance_before + MIN_DISTANCE_FROM_LAST_TILE_MM;
			if (!(lasers->sensors[1]->GetData()->distance_mm[6] > back_distance_to_reach)) {
				// Proseguo dritto fino a quando non sono arrivato a 30cm dalla tile precendete
				// ovvero, fino a quando non sono al centro della tile attuale
				ms->SetPower(SPEED, SPEED);
				while (!(lasers->sensors[1]->GetData()->distance_mm[6] > back_distance_to_reach))
				{
					Serial.print("Dio besta!");
					UpdateSensorNumBlocking(1);
				}
				ms->SetPower(0, 0);
			}
			*/
			// Giro a destra (90°)
			Turn(90);
			// Non controllo la destra fino al prossimo muro destro
			ignore_right = true;
		}
		else
		{
			// Se è presente un muro laterale e sto ignorando la destra, smetto di ingorarla
			if (lasers->sensors[SENSOR_DX]->GetData()->distance_mm[5] <= MIN_DISTANCE_TO_SET_IGNORE_RIGHT_FALSE_MM && ignore_right)
			{
				ignore_right = false;
				Serial.println("Muro a destra a tot mm: ");
				Serial.print(lasers->sensors[SENSOR_DX]->GetData()->distance_mm[5]);
			}
			/*
			// Distanza dal muro
			if (lasers->sensors[1]->GetData()->distance_mm[6] >= back_distance_before + MIN_DISTANCE_FROM_LAST_TILE_MM)
			{
				// Aggiorno la distanza dal muro
				back_distance_before = lasers->sensors[1]->GetData()->distance_mm[6];
			}
			*/
			// Controllo la distaza frontale, e se trovo un muro
			if (lasers->sensors[SENSOR_FW]->GetData()->distance_mm[5] <= MIN_DISTANCE_FROM_FRONT_WALL_MM)
			{
				Serial.println("Muro frontale!!!");
				// Controllo la sinistra, se libera giro
				if (lasers->sensors[SENSOR_SX]->GetData()->distance_mm[5] >= MIN_DISTANCE_TO_TURN_LEFT_MM)
				{
					// Giro a sinstra
					Serial.println("Sinistra libera!!!");
					Turn(-90);
				}
				else // Sinistra bloccata, giro di 180 gradi
				{
					Serial.println("Tutto bloccato!!!");
					Turn(180);
					ms->SetPower(-90, -90);
					delay(1000);
					ms->SetPower(0, 0);
					// mpu->Reset();
					// desired_angle = 0;
				}
			}
			else
			{
				// Prosegue "dritto"
				// UpdateGyroBlocking(); // Prendo valori aggiornati del gyro (Non ha senso riprendere il valore, già lo aggiorno prima di entrare in run())
				if (mpu_data.x < desired_angle)
				{ // Se tende a sinistra, più potenza a sinistra
					ms->SetPower(SPEED + 20, SPEED);
				} else if (mpu_data.x > desired_angle){ // Se tende a destra, più potenza a destra
					ms->SetPower(SPEED, SPEED + 20);
				}
				else
				{ // Se è dritto, infatibile, prosegue dritto
					ms->SetPower(SPEED, SPEED);
				}
			}
		}
	}
	else
	{
		Serial.print("Bloccato");
		ms->StopMotors();
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

bool Robot::StopRobot() // TODO: Fix this shit
{
	stop_the_robot = false;
	if (digitalRead(R_PIN_BUTTON)) {
		stop_the_robot = true;
	}
	return stop_the_robot;
}

void Robot::Turn(int degree) {
	Serial.println("Metodo Turn: ");
	// Calcolo il grado da raggiungere
	UpdateGyroBlocking();
	desired_angle = mpu_data.x + degree;
	// Controllo se devo girare a destra o sinistra
	if (degree > 0) // Giro a destra 
	{
		Serial.println("Giro destra ->");
		Serial.print("Angolo da raggiungere: ");
		Serial.print(desired_angle);
		Serial.print("Angolo Attuale: ");
		Serial.print(mpu_data.x);
		// Inizio a girare
		ms->SetPower(TURN_SPEED, -TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x >= desired_angle))
		{
			UpdateGyroBlocking();
			Serial.print("Attuale: ");
			Serial.println(mpu_data.x);
			Serial.print("Desiderato: ");
			Serial.println(desired_angle);
		}
	}
	else // Giro a sinistra
	{
		Serial.println("Giro sinistra <-");
		Serial.print("Angolo da raggiungere: ");
		Serial.print(desired_angle);
		Serial.print("Angolo Attuale: ");
		Serial.print(mpu_data.x);
		// Inizio a girare
		ms->SetPower(-TURN_SPEED, TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x <= desired_angle))
		{
			UpdateGyroBlocking();
			Serial.print("Attuale: ");
			Serial.print(mpu_data.x);
			Serial.print("Desiderato: ");
			Serial.println(desired_angle);
		}
	}
	// Stop dei motori e ricalcolo distanza dal muro posteriore
	Serial.println("Metodo Turn: giro completato!!!");
	ms->SetPower(0, 0);
	/*
	UpdateSensorNumBlocking(1);
	back_distance_before = lasers->sensors[1]->GetData()->distance_mm[6];
	*/
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
		mpu->GetGyroData(mpu_data);
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
	while (true)
	{
		if (mpu_data_ready)
		{
			mpu->GetGyroData(mpu_data);
			mpu_data_ready = false;
			break;
		}
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
	delete ms;
	delete lasers;
	delete mpu;
}
