
#include "Robot.h"

Robot::Robot(bool cold_start)
{
	if (cold_start)
	{
		LOG("Disabling and then enabling sensors power supply...");
		pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
		digitalWrite(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
		delay(10);													// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
		digitalWrite(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
		delay(10);													// Wait for sensors to wake up (especially sensor 0)
		LOG("...done!");
	}

	Wire.begin();			  // Gyro
	Wire.setClock(400000); // 400kHz

	LOG("Gyro setup started");
	mpu = new Gyro(cold_start);
	mpu_data_ready = false;
	attachInterrupt(R_PIN_GYRO_INT, R_MPU6050_int, RISING);
	LOG("Finished gyro setup!");

	LOG("Motor setup started");
	ms = new Motors();
	LOG("Finished motor setup!");

	LOG("Servo setup started");
	kit.attach(R_PIN_SERVO);
	kit.write(0);
	LOG("Finished servo setup!");

	Wire2.begin();				 // Lasers
	Wire2.setClock(1000000); // 1MHz

	LOG("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2);
	LOG("Laser sensors setup finished");

	/*
		Interrupts *MUST* be attached after the VL53L5CX_manager is instantiated,
		otherwise if the data_ready array isn't initialized and an interrupt is
		fired, the program will crash.
	*/
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::FW], R_VL53L5CX_int_0, FALLING); // sensor_0
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::BW], R_VL53L5CX_int_1, FALLING); // sensor_1
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::SX], R_VL53L5CX_int_2, FALLING); // sensor_2
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::DX], R_VL53L5CX_int_3, FALLING); // sensor_3
	lasers->StartRanging(16, 60, ELIA::RangingMode::kContinuous);					 // 4*4, 60Hz

	Wire1.begin();				// Color sensor
	Wire1.setClock(400000); // 400kHz

	Serial.println("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		Serial.println("Initialized color sensor!");
		pinMode(R_PIN_COLOR_INT, INPUT_PULLUP);
		attachInterrupt(R_PIN_COLOR_INT, R_TCS34725_int, FALLING);
		cs->ClearInterrupt();
	}
	else
	{
		Serial.println("Failed to initialize color sensor!");
	}

	pinMode(R_PIN_BUTTON, INPUT);

	/**
	 * Robot ready signal
	 */

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(100);
	digitalWrite(LED_BUILTIN, LOW);

	mpu->ResetX();

	PID_start_time = millis();
	/**
	 * Navigation (dead code?) (DK: Yhep)
	 */
	// UpdateSensorNumBlocking(VL53L5CX::BW);
	// back_distance_before = lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[6];
}

void Robot::Run()
{
	if (!StopRobot())
	{
		// Controllo il colore della tile
		if(cs->c_comp < 20 && mpu_data.z < 3) {
			if (cs->c_comp <= 10)
			{
				just_found_black = true;
				ms->SetPower(-40, -40);
				while (cs->c_comp < 10)
				{
					if (color_data_ready)
					{
						cs->getData();
						color_data_ready = false;
					}
				}

				ms->StopMotors();
				if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_LEFT_MM)
				{
					Turn(-90);
					ms->StopMotors();
					UpdateSensorNumBlocking(VL53L5CX::FW);
					front_distance_after_black_turn = lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[5] - 250;
					ignore_right = true;
				}
				else
				{
					Turn(180);
				}
			}
			else
			{
				ms->SetPower(50, 50);
				while (cs->c_comp < 25 and cs->c_comp >= 10)
				{
					ms->SetPower(50, 50);
					if (color_data_ready)
					{
						cs->getData();
						color_data_ready = false;
					}
				}
				if (cs->c_comp >= 10)
				{
					ms->StopMotors();
					delay(5000);
					ignore_right = false; // Per vedere il varco nel caso in cui non lo abbia visto causa ciclo while
				}
			}
		}

		// Controllo la presenza di un varco a destra
		if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[5] >= MIN_DISTANCE_TO_TURN_RIGHT_MM && !ignore_right)
		{
			// Varco trovato!

			// Output variabili varco
			// Serial.println("Varco a destra!!!");
			// Serial.println("Distanza Destra: ");
			// Serial.print(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[5]);

			// Fermo il robot. TODO: capire se necessario
			ms->SetPower(0, 0);

			// Giro a destra (90°)
			Turn(90);
			// IMposto un vincolo sul varco a destra
			ignore_right = true;
		}
		else
		{
			// Non è stato rilevato nessun varco a destra

			// In presenza di un muro laterale a destra, cambio il vincolo sul varco applicato nella svolta
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[5] <= MIN_DISTANCE_TO_SET_IGNORE_RIGHT_FALSE_MM)  && (ignore_right))
			{
				// Output variabili muro laterale
				Serial.println("Muro a destra a tot mm: ");
				Serial.print(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[5]);
				ignore_right = false;
			}
			if (lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[5] <= (front_distance_after_black_turn) && (just_found_black))
			{
				just_found_black = false;
				ignore_right = false;
			}

			if (lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[5] <= MIN_DISTANCE_FROM_FRONT_WALL_MM)
			{
				// Valutazione azione da svolgere in presenza di muro frontale
				// In presenza di varco a sinistra, svolterò per prosseguire verso quella direzione
				if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_LEFT_MM)
				{
					Turn(-90);
				}
				// Avendo tutte le direzioni bloccate, quindi in presenza di una U, mi girerò del tutto
				else
				{
					Turn(180);
					
					// Manovra da eseguire per ristabilizzare il robot e resettare il giro
					ms->SetPower(-90, -90);
					delay(1000);
					ms->SetPower(0, 0);
					
					// TODO: Valutare se tenerlo con il PID
					mpu->ResetX();
					desired_angle = 0;
				}
			} 
			// In assenza di muro frontale ci troviamo in una strada da prosseguire in modo rettilineo
			else 
			{
				// Calculate error
				PID_error = CalculateError(mpu_data.x);
				// Calculate integral
				PID_integral += PID_error;
				// Calculate derivative
				double derivative = PID_error - PID_previous_error;
				PID_previous_error = PID_error;
				// Calculate output
				PID_output = KP * PID_error + KI * PID_integral + KD * derivative;
				// Update motor powers and apply motor powers to left and right motors
				double elapsed_seconds = millis() - PID_start_time;

				ms->SetPower(SPEED + PID_output * elapsed_seconds, SPEED - PID_output * elapsed_seconds);

				Serial.println("PID_output: ");
				Serial.println(PID_output);
				Serial.println("elapsed_seconds: ");
				Serial.println(elapsed_seconds);
				Serial.println("PID_output * elapsed_seconds: ");
				double corr = PID_output * elapsed_seconds;
				Serial.println(corr);
				// Update start time
				PID_start_time = millis();
			}
		}
	}
	else
	{
		ms->StopMotors();
		Serial.println("Premere il pulsante per far partire il robot!");
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

void Robot::R_TCS34725_int()
{
	color_data_ready = true;
}

bool Robot::StopRobot() // FIXED
{
	if (digitalRead(R_PIN_BUTTON))
	{
		if (!first_time_pressed)
		{
			stop_the_robot = !stop_the_robot;
			first_time_pressed = true;
			if (!stop_the_robot)
			{
				mpu->ResetX();
				desired_angle = 0;
			}
		}
	}
	else
	{
		first_time_pressed = false;
	}
	return stop_the_robot;
}

void Robot::Turn(int16_t degree)
{
	Serial.println("Metodo Turn: ");
	// Calcolo il grado da raggiungere
	UpdateGyroBlocking();
	desired_angle = mpu_data.x + degree; // or desired angle += degree
	// Controllo se devo girare a destra o sinistra
	if (degree > 0) // Giro a destra
	{
		Serial.println("Giro destra ->");
		// Inizio a girare
		ms->SetPower(TURN_SPEED, -TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x >= desired_angle - 5))
		{
			UpdateGyroBlocking();
		}
	}
	else // Giro a sinistra
	{
		Serial.println("Giro sinistra <-");
		// Inizio a girare
		ms->SetPower(-TURN_SPEED, TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x <= desired_angle + 5))
		{
			UpdateGyroBlocking();
		}
	}
	// Stop dei motori e ricalcolo distanza dal muro posteriore
	Serial.println("Metodo Turn: giro completato!!!");
	ms->SetPower(0, 0);
	
	/*
	UpdateSensorNumBlocking(1);
	back_distance_before = lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[6];
	*/
}

double Robot::CalculateError(double currentYaw)
{
	return desired_angle - currentYaw;
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

	if (color_data_ready)
	{
		cs->getData();
	}

	return status;
}

void Robot::UpdateSensorNumBlocking(VL53L5CX num)
{
	while (1)
	{
		if (lasers_data_ready[num])
		{
			lasers->sensors[num]->UpdateData();
			lasers_data_ready[num] = false;
			break;
		}
	}
}

void Robot::UpdateGyroBlocking()
{
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
		const char arr[4] = {'F', 'B', 'S', 'D'};
		Serial.print("Sensor ");
		Serial.print(arr[i]);
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
	Serial.print("c_comp:");
	Serial.print(cs->c_comp);
	Serial.print("\tr_comp: ");
	Serial.print(cs->r_comp);
	Serial.print("\tg_comp: ");
	Serial.print(cs->g_comp);
	Serial.print("\tb_comp:");
	Serial.println(cs->b_comp);
}

Robot::~Robot()
{
	delete ms;
	delete lasers;
	delete mpu;
}
