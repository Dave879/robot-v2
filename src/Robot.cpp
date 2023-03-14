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
	lasers = new VL53L5CX_manager(Wire2, cold_start);
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
	lasers->StartRanging(64, 15, ELIA::RangingMode::kContinuous);					 // 8*8, 15Hz

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

	// Inizializzazione canale di comunicazione con OpenMV, e primo avvio se presente un muro a destra alla partenza
	Serial2.begin(115200);
}

void Robot::Run()
{
	if (!StopRobot()) // Robot in azione
	{
		// Victims detection
		if (Serial2.available() > 0)
		{
			char data = Serial2.read();
			Serial.print("Teensy 4.1 ha ricevuto in seriale: ");
			Serial.println(data);
			// Controllo distanza dal muro laterale, se rilevo il muro con il sensore laser a destra proseguo con il drop dei kit
			// se non trovo nessun muro laterale, scarto il valore
			if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
			{
				switch (data)
				{
					case '0':
						Serial.println("Vittima: 0 kit");
						ms->StopMotors();
						delay(6000);
						break;
					case '1':
						Serial.println("Vittima: 1 kit");
						ms->StopMotors();
						DropKit(1);
						break;
					case '2':
						ms->StopMotors();
						DropKit(2);
						Serial.println("Vittima: 2 kit");
						break;
					case '3':
						ms->StopMotors();
						DropKit(3);
						Serial.println("Vittima: 3 kit");
						break;
					default:
						Serial.println("No vittima");
						break;
				}
			}
		}

		// Colore tile
		if (cs->c_comp <= MIN_VALUE_BLUE_TILE && mpu_data.z < 9 && mpu_data.z > -9 && !ignore_blue) // Ignore blue, non attendo
		{
			// Mando il robot avanti per 100ms, così da vedere con più precisione la luminosità della tile
			delay(150);
			// Aggiorno il valore di luminosità
			cs->getData();
			color_data_ready = false;
			// Controllo il colore della tile
			if (cs->c_comp <= MIN_VALUE_BLACK_TILE) // Tile nera
			{
				Serial.println("Black tile");
				// Torno in dietro fino a quando smetto di vedere nero
				ms->SetPower(-40, -40);
				while (cs->c_comp <= MIN_VALUE_BLACK_TILE)
				{
					if (color_data_ready)
					{
						cs->getData();
						color_data_ready = false;
					}
				}
				// Mando il robot indietro ancorà di più, così da alontanarlo dalla tile nera
				delay(300);
				ms->StopMotors();
				// Giro in base all'ultima svolta effettuata
				if (last_turn_right)
				{
					// Controllo se ho la sinistra libera
					UpdateSensorNumBlocking(VL53L5CX::BW);
					UpdateSensorNumBlocking(VL53L5CX::SX);
					if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[26] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro in dietro
						TurnBack();
					}
					else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro a sinistra
						TurnLeft();

						// Imposto le variabili neccessarie per cambiare l'ignore entro la prossima tile
						just_found_black = true;
						time_after_black_tile_ignore_false = millis() + 500;
					}
					else
					{
						// Giro a destra
						TurnRight();
					}
				}
				// Controllo se arrio da una U
				else if(last_turn_back)
				{
					// Controllo se ho la destra o la sinistra libera
					UpdateSensorNumBlocking(VL53L5CX::DX);
					UpdateSensorNumBlocking(VL53L5CX::SX);
					if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro a destra
						TurnRight();

						// Imposto le variabili neccessarie per cambiare l'ignore entro la prossima tile
						just_found_black = true;
						time_after_black_tile_ignore_false = millis() + 500;
					}
					else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro a sinistra
						TurnLeft();

						// Imposto le variabili neccessarie per cambiare l'ignore entro la prossima tile
						just_found_black = true;
						time_after_black_tile_ignore_false = millis() + 500;
					}
					else
					{
						// Giro in dietro
						TurnBack();
					}
				}
				else
				{
					// Controllo se ho la destra libera
					UpdateSensorNumBlocking(VL53L5CX::DX);
					UpdateSensorNumBlocking(VL53L5CX::BW);
					if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[26] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro in dietro
						TurnBack();
					}
					else if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
					{
						// Giro a destra
						TurnRight();

						// Nero prossima tile ignore = false
						just_found_black = true;
						time_after_black_tile_ignore_false = millis() + 500;
					}
					else
					{
						// Giro a sinistra
						TurnLeft();
					}
				}
				// Una volta svoltato aggiorno tutti i valori
				UpdateSensorNumBlocking(VL53L5CX::SX);
				UpdateSensorNumBlocking(VL53L5CX::DX);
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			else if(cs->c_comp > MIN_VALUE_BLACK_TILE && cs->c_comp <= MIN_VALUE_BLUE_TILE && !ignore_blue)
			{
				Serial.println("Blue tile");
				// Imposto i valori per ignorare il blue
				ignore_blue = true;
				// proseguo dirtto fino a quando vedo blue
				ms->SetPower(40, 40);
				while (cs->c_comp > MIN_VALUE_BLACK_TILE && cs->c_comp <= MIN_VALUE_BLUE_TILE)
				{
					// In presenza di un muro laterale a destra, cambio il vincolo sul varco applicato nella svolta
					UpdateSensorNumBlocking(VL53L5CX::DX);
					if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_right)
					{
						ignore_right = false;
					}
					// In presenza di un muro laterale a sinistra, cambio il vincolo sul varco applicato nella svolta
					UpdateSensorNumBlocking(VL53L5CX::SX);
					if ((lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_left)
					{
						ignore_left = false;
					}

					if (color_data_ready)
					{
						cs->getData();
						Serial.println("Data ready:");
						Serial.print("Luminosità: ");
						Serial.println(cs->c_comp);
						color_data_ready = false;
					}
				}
				ms->StopMotors();
				// Una volta arrivato alla fine della tile, attendo 5s prima di fare qualsiasi altra cosa
				delay(5000);
				
				cs->getData();
				if (cs->c_comp <= MIN_VALUE_BLACK_TILE)
				{
					ignore_blue = false;
				}

				// Una volta svoltato aggiorno tutti i valori
				UpdateSensorNumBlocking(VL53L5CX::SX);
				UpdateSensorNumBlocking(VL53L5CX::DX);
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
		}
		// Se ho ignorato il nero, controllo il tempo, se passato il tempo necessario, in base a dove avevo svoltato l'ultima volta, imposto l'ignore a false
		else if (just_found_black)
	{
		if (millis() > time_after_black_tile_ignore_false)
		{
			if (last_turn_right)
			{
				ignore_right = false;
			}
			else
			{
				ignore_left = false;
			}
			just_found_black = false;
		}
	}

		// Controllo se devo girare
		if (NeedToTurn())
		{
			// Varco trovato!
			delay(100);
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);

			// Controllo se ho entrabi i lati liberi
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right) && (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left))
			{
				// Output variabili varco
				Serial.println("Varco Trovato!!!");
				// Varco Destra
				Serial.print("Distanza Destra: ");
				Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27]);
				// Varco Sinistra
				Serial.print("Distanza Sinistra: ");
				Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27]);

				// Scelgo quale direzione prendere tra destra e sinistra
				if (millis() % 2 == 1)
				// Giro a destra
				{
					// Giro a destra(90°)
					TurnRight();
				}
				// Giro a sinistra
				else
				{
					// Giro a sinistra(-90°)
					TurnLeft();
				}
			}
			// Non è stato rilevato un varco simultaneo, ma solo a destra o sinistra
			else if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right) || (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left))
			{	
				// Giro a destra
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right)
				{
					if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
					{
						if (millis() % 2 == 1)
						{
							// Giro a destra
							TurnRight();

							UpdateSensorNumBlocking(VL53L5CX::BW);
							if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[28] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
							{
								// Manovra da eseguire per ristabilizzare il robot e resettare il giro
								// Straighten();
							}
						}
						else
						{
							ignore_right = true;
							ignore_left = true;
						}
					}
					else
					{
						// Output variabili varco
						Serial.println("Varco Trovato!!!");
						Serial.print("Distanza Destra: ");
						Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27]);

						// Giro a destra(90°)
						TurnRight();
					}
				}
				// Giro a sinistra
				else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left)
				{
					if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
					{
						if (millis() % 2 == 1)
						{
							// Giro a sinistra
							TurnLeft();

							UpdateSensorNumBlocking(VL53L5CX::BW);
							if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[28] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
							{
								// Manovra da eseguire per ristabilizzare il robot e resettare il giro
								// Straighten();
							}
						}
						else
						{
							ignore_right = true;
							ignore_left = true;
						}
					}
					else
					{
						// Output variabili varco
						Serial.println("Varco Trovato!!!");
						Serial.print("Distanza Sinistra: ");
						Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27]);

						// Giro a sinistra(-90°)
						TurnLeft();
					}
				}
			}
			// Non dovrebbe aver trovato varchi
		}
		// Se non devo girare, prosseguo dritto, e controllo la presenza di un muro frontale
		else
		{
			// Se non sono su una tile blue e sto ignorando il blue, toglo l'ignora blue
			if (cs->c_comp > MIN_VALUE_BLUE_TILE && ignore_blue)
			{
				ignore_blue = false;
			}
			// In presenza di un muro laterale a destra, cambio il vincolo sul varco applicato nella svolta
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_right)
			{
				ignore_right = false;
			}
			// In presenza di un muro laterale a sinistra, cambio il vincolo sul varco applicato nella svolta
			if ((lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_left)
			{
				ignore_left = false;
			}
			// Controllo la distanza frontale
			if (lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[19] <= MIN_DISTANCE_FROM_FRONT_WALL_MM)
			{
				ms->StopMotors();
				UpdateSensorNumBlocking(VL53L5CX::SX);
				UpdateSensorNumBlocking(VL53L5CX::DX);
				// Valutazione azione da svolgere in presenza di muro frontale
				// In presenza di varco a destra, svolterò per prosseguire verso quella direzione
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
				{
					// Giro a destra
					TurnRight();

					// Manovra da eseguire per ristabilizzare il robot e resettare il giro
					// Straighten();
				}
				// In presenza di varco a sinistra, svolterò per prosseguire verso quella direzione
				else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM)
				{
					// Giro a left
					TurnLeft();

					// Manovra da eseguire per ristabilizzare il robot e resettare il giro
					// Straighten();
				}
				// Avendo tutte le direzioni bloccate, quindi in presenza di una U, mi girerò del tutto
				else
				{
					// Giro in dietro
					TurnBack();

					// Manovra da eseguire per ristabilizzare il robot e resettare il giro
					Straighten();
				}
			} 
			// In assenza di muro frontale proseguiamo in modo rettilineo
			else
			{ 
				MotorPowerZGyroAndPID();
			}
		}
	}
	else // Roboto fermo
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

bool Robot::StopRobot()
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

void Robot::DropKit(int8_t number_of_kits)
{
	if (number_of_kits > 1)
	{
		ms->SetPower(-50, -50);
		delay(400);
	}

	Turn(-90);
	ms->SetPower(-90, -80);
	delay(500);
	ms->SetPower(40, 40);
	delay(200);
	ms->StopMotors();
	
	for (int8_t i = 0; i < number_of_kits; i++)
	{
		kit.write(180);
		delay(1000);
		kit.write(0);
		delay(1000);
	}

	Turn(90);
	delay(7000 - (2000 * number_of_kits));
}

void Robot::Straighten()
{
	ms->SetPower(-90, -90);
	delay(1200);
	ms->StopMotors();
	mpu->ResetX();
	desired_angle = 0;
}

int16_t Robot::GetPIDOutputAndSec()
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

	// Update start time
	PID_start_time = millis();

	Serial.println("PID_output: ");
	Serial.println(PID_output);
	Serial.println("elapsed_seconds: ");
	Serial.println(elapsed_seconds);
	Serial.println("PID_output * elapsed_seconds: ");
	double corr = PID_output * elapsed_seconds;
	Serial.println(corr);

	return PID_output * elapsed_seconds;
}

void Robot::MotorPowerZGyroAndPID()
{
	int16_t pid_speed = GetPIDOutputAndSec() / 100;
	if (mpu_data.z > 0) {
		ms->SetPower(SPEED + pid_speed + mpu_data.z, SPEED - pid_speed + mpu_data.z);
	}
	else if (mpu_data.z < -10)
	{
		ms->SetPower(SPEED + pid_speed - 5, SPEED - pid_speed - 5);
	}
	else
	{
		ms->SetPower(SPEED + pid_speed, SPEED - pid_speed);
	}
}

bool Robot::NeedToTurn(){
	return (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right) || (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left);
}

void Robot::TurnRight()
{
	// Aggiorno i dati sull'ultima curva
	last_turn_right = true;
	last_turn_back = false;
	just_found_black = false;

	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro a destra (90°)
	Turn(90);

	// Imposto un vincolo sul varco a destra e snistra
	ignore_right = true;
	ignore_left = true;
}

void Robot::TurnLeft()
{	
	// Aggiorno i dati sull'ultima curva
	last_turn_right = false;
	last_turn_back = false;
	just_found_black = false;

	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro a sinistra (-90°)
	Turn(-90);

	// Imposto un vincolo sul varco a destra e snistra
	ignore_right = true;
	ignore_left = true;
}

void Robot::TurnBack()
{
	// Aggiorno i dati sull'ultima curva
	last_turn_right = false;
	last_turn_back = true;
	just_found_black = false;

	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro totaale (-180°)
	Turn(-180);

	// Imposto un vincolo sul varco a destra e snistra
	ignore_right = true;
	ignore_left = true;
}

void Robot::Turn(int16_t degree)
{
	// Output metodo Turn()
	Serial.println("Metodo Turn: Inizio la manovra!");

	// Calcolo il grado da raggiungere
	UpdateGyroBlocking();
	desired_angle = mpu_data.x + degree;
	// desired_angle += degree;

	// Controllo se devo girare a destra o sinistra
	if (degree > 0) // Giro a destra
	{
		Serial.println("Giro destra ->");
		while (mpu_data.x <= desired_angle)
		{
			int16_t gyro_speed = GetPIDOutputAndSec();
			UpdateGyroBlocking();
			Serial.print("Gyyro: ");
			Serial.println(mpu_data.x);
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(gyro_speed + mpu_data.z, -gyro_speed - mpu_data.z);
		}
	}
	else // Giro a sinistra o indietro
	{
		Serial.println("Giro sinistra <-");
		while (mpu_data.x >= desired_angle)
		{
			int16_t gyro_speed = GetPIDOutputAndSec();
			UpdateGyroBlocking();
			Serial.print("Gyyro: ");
			Serial.println(mpu_data.x);
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(gyro_speed + mpu_data.z, -gyro_speed - mpu_data.z);
		}
	}

	
	// Stop dei motori
	ms->StopMotors();
	PID_integral = 0;
	Serial.println("Metodo Turn: giro completato!!!");
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
			float how_many_tiles = lasers->sensors[i]->GetData()->distance_mm[j]/300.0f;
			Serial.print(how_many_tiles);
			Serial.print(", \t");
			if ((j + 1) % 8 == 0)
			{
				Serial.print("\t");
				for (uint8_t k = abs(7-j); k <= j; k++)
				{
					Serial.print(lasers->sensors[i]->GetData()->target_status[k]);
					Serial.print(", \t");
				}
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


	Serial.println(Serial2.available());
}

Robot::~Robot()
{
	delete ms;
	delete lasers;
	delete mpu;
}
