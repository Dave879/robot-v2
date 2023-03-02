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
	lasers->StartRanging(16, 60, ELIA::RangingMode::kContinuous);					 // 8*8, 15Hz

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

	// Inizializzazione canale di comunicazione con OpenMV, e primo avvio se presente un muro a destra alla partenza
	Serial2.begin(115200);
}

void Robot::Run()
{
	if (!StopRobot()) // Robot in azione
	{
		// Victims detection
		if (openmv_searching)
		{
			if (Serial2.available() > 0)
			{
				openmv_searching = false;
				victim_just_found = true;
				char data = Serial2.read();
				Serial.print("Teensy 4.1 ha ricevuto in seriale: ");
				Serial.println(data);
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
				time_after_openmv_can_search_again = millis() + 500;
			}
		}
		else
		{
			if (!victim_just_found)
			{
				if(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
				{
					openmv_searching = true;
					Serial2.println(1);
					time_after_openmv_can_search_again = millis();
					Serial.println("Teensy 4.1 ha inviato in seriale: 1. OpenMV ha iniziato a cercare!");
				}
				else
				{
					Serial.println("E' presente un varco a destra, OpenMV non viene attivata");
				}
			}
			else
			{
				if (millis() > time_after_openmv_can_search_again)
				{
					victim_just_found = false;
					Serial.println("Tempo minimo d'attesa da attendere dopo aver trovato una vittima passato.");
				}
			}
		}

	// Colore tile
	if (cs->c_comp <= MIN_VALUE_TO_STOP_COLORED_TILE)
	{
		ms->StopMotors();
		delay(100);
		cs->getData();
		color_data_ready = false;
		if (cs->c_comp <= MIN_VALUE_TO_AVOID_BLACK)
		{
			ms->SetPower(-40, -40);
			while (cs->c_comp <= MIN_VALUE_TO_AVOID_BLACK)
			{
				if (color_data_ready)
				{
					cs->getData();
					color_data_ready = false;
				}
			}
			delay(200);
			ms->StopMotors();
			if (last_turn_right)
			{
				UpdateSensorNumBlocking(VL53L5CX::SX);
				if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM)
				{
					// Nero prossima tile ignore = false
					just_found_black = true;
					time_after_black_tile_ignore_false = millis() + 500;

					// Fermo l'openMV
					StopOpenMV();

					// Giro a destra (90°)
					Turn(-90);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
				else
				{
					// Fermo l'openMV
					StopOpenMV();

					// Giro del tutto (-180°)
					Turn(-180);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
			}
			else
			{
				UpdateSensorNumBlocking(VL53L5CX::DX);
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM)
				{
					// Nero prossima tile ignore = false
					just_found_black = true;
					time_after_black_tile_ignore_false = millis() + 500;

					// Fermo l'openMV
					StopOpenMV();

					// Giro del tutto (-180°)
					Turn(90);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
				else
				{
					// Fermo l'openMV
					StopOpenMV();

					// Giro del tutto (-180°)
					Turn(-180);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
			}
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);
		}
		else if(cs->c_comp > MIN_VALUE_TO_AVOID_BLACK && cs->c_comp <= MIN_VALUE_TO_STOP_BLUE)
		{
			ms->SetPower(40, 40);
			while (cs->c_comp <= MIN_VALUE_TO_STOP_BLUE && cs->c_comp > MIN_VALUE_TO_AVOID_BLACK)
			{
				if (color_data_ready)
				{
					cs->getData();
					color_data_ready = false;
				}
			}
			ms->StopMotors();
			if (!(cs->c_comp <= MIN_VALUE_TO_AVOID_BLACK))
			{
				delay(5000);
				ignore_right = false;
				ignore_left = false;
			}
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);
		}
	}
	// Inogra con nero, controllo il tempo
	else if(just_found_black)
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

		// Controllo la presenza di un varco
		if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right) && (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left))
		{
			// Varco trovato!

			// Output variabili varco
			Serial.println("Varco Trovato!!!");
			Serial.print("Distanza Destra: ");
			Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6]);


			Serial.print("Distanza Sinistra: ");
			Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6]);

			if (millis() % 2 == 1)
			// Giro a destra
			{
				Serial.print("millis() % 2: ");
				Serial.println(millis() % 2);
				// Fermo l'openMV
				StopOpenMV();

				// Fermo il robot prima di girare
				ms->StopMotors();

				// Giro a destra (90°)
				Turn(90);
				// Imposto un vincolo sul varco a destra e snistra
				ignore_right = true;
				ignore_left = true;
			}
			// Giro a sinistra
			else
			{
				Serial.print("millis() % 2: ");
				Serial.println(millis() % 2);
				// Fermo l'openMV
				StopOpenMV();

				// Fermo il robot prima di girare
				ms->StopMotors();

				// Giro a sinistra (-90°)
				Turn(-90);
				// Imposto un vincolo sul varco a destra e snistra
				ignore_right = true;
				ignore_left = true;
			}
		}
		// Non è stato rilevato varco simultaneo a destra e sinistra
		else if((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right) || (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left))
		{
			// Varco trovato!
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_right)
			// Giro a destra
			{
				if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
				{
					if (millis() % 2 == 1)
					{
						// Giro a destra
						Serial.print("millis() % 2: ");
						Serial.println(millis() % 2);

						// Fermo l'openMV
						StopOpenMV();

						// Fermo il robot prima di girare
						ms->StopMotors();

						// Giro a destra (90°)
						Turn(90);

						// Radrizzo
						Straighten();

						// Imposto un vincolo sul varco a destra e snistra
						ignore_right = true;
						ignore_left = true;
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
					Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6]);

					// Fermo l'openMV
					StopOpenMV();

					// Fermo il robot prima di girare
					ms->StopMotors();


					// Giro a destra (90°)
					Turn(90);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
			}
			// Giro a sinistra
			else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM && !ignore_left)
			{
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
				{
					if (millis() % 2 == 1)
					{
						Serial.print("millis() % 2: ");
						Serial.println(millis() % 2);

						// Fermo il robot prima di girare
						ms->StopMotors();

						// Fermo l'openMV
						StopOpenMV();

						// Giro a sinistra (-90°)
						Turn(-90);

						// Radrizzo
						Straighten();

						// Imposto un vincolo sul varco a destra e snistra
						ignore_right = true;
						ignore_left = true;
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
					Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6]);

					// Fermo l'openMV
					StopOpenMV();

					// Fermo il robot prima di girare
					ms->StopMotors();

					// Giro a sinistra (-90°)
					Turn(-90);
					// Imposto un vincolo sul varco a destra e snistra
					ignore_right = true;
					ignore_left = true;
				}
			}
		}
		// Non dovrebbe aver trovato varchi
		else
		{
			// In presenza di un muro laterale a destra, cambio il vincolo sul varco applicato nella svolta
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_right)
			{
				ignore_right = false;
			}
			// In presenza di un muro laterale a sinistra, cambio il vincolo sul varco applicato nella svolta
			if ((lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)  && ignore_left)
			{
				ignore_left = false;
			}
			// Controllo la distanza frontale
			if (lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[6] <= MIN_DISTANCE_FROM_FRONT_WALL_MM)
			{
				ms->StopMotors();
				UpdateSensorNumBlocking(VL53L5CX::SX);
				UpdateSensorNumBlocking(VL53L5CX::DX);
				// Valutazione azione da svolgere in presenza di muro frontale
				// In presenza di varco a destra, svolterò per prosseguire verso quella direzione
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM)
				{
					// Fermo l'openMV
					StopOpenMV();

					Turn(90);
					Straighten();
				}
				// In presenza di varco a sinistra, svolterò per prosseguire verso quella direzione
				else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[6] >= MIN_DISTANCE_TO_TURN_MM)
				{
					Turn(-90);
					Straighten();
				}
				// Avendo tutte le direzioni bloccate, quindi in presenza di una U, mi girerò del tutto
				else
				{
					Turn(-180);
					
					// Manovra da eseguire per ristabilizzare il robot e resettare il giro
					Straighten();
				}
			} 
			// In assenza di muro frontale proseguiamo in modo rettilineo
			else // PID Controll
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

				if (mpu_data.z > 0) {
					ms->SetPower(SPEED + (PID_output * elapsed_seconds) + mpu_data.z, SPEED - (PID_output * elapsed_seconds) + mpu_data.z);
				}
				else if (mpu_data.z < 45)
				{
					ms->SetPower(SPEED + (PID_output * elapsed_seconds) - 5, SPEED - (PID_output * elapsed_seconds) - 5);
				}
				else
				{
					ms->SetPower(SPEED + (PID_output * elapsed_seconds), SPEED - (PID_output * elapsed_seconds));
				}

				Serial.println("PID_output: ");
				Serial.println(PID_output);
				Serial.println("elapsed_seconds: ");
				Serial.println(elapsed_seconds);
				Serial.println("PID_output * elapsed_seconds: ");
				double corr = PID_output * elapsed_seconds;
				Serial.println(corr);
				// Update start time
				PID_start_time = millis();

			/*
				// Provisorio
				if (mpu_data.x > desired_angle)
				{
					ms->SetPower(SPEED - 5 + mpu_data.z, SPEED + 5  + mpu_data.z);
				}
				else
				{
					ms->SetPower(SPEED + 5 + mpu_data.z, SPEED - 5  + mpu_data.z);
				}
			*/
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

void Robot::DropKit(int8_t number_of_kits)
{
	if (number_of_kits > 1)
	{
		ms->SetPower(-50, -50);
		delay(400);
	}

	Turn(-90);
	for (int8_t i = 0; i < number_of_kits; i++)
	{
		ms->SetPower(-90, -80);
		delay(1000);
		ms->SetPower(40, 40);
		delay(200);
		ms->StopMotors();
		kit.write(180);
		delay(1000);
		kit.write(0);
		delay(1000);
	}

	Turn(90);
	delay(7000 - (2000 * number_of_kits));
}

void Robot::StopOpenMV()
{
		// Fermo l'openMV
		Serial2.println(2);
		openmv_searching = false;
}

void Robot::Straighten()
{
	ms->SetPower(-90, -90);
	delay(1200);
	ms->StopMotors();
	mpu->ResetX();
	desired_angle = 0;
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
		last_turn_right = true;
		Serial.println("Giro destra ->");
		// Inizio a girare
		ms->SetPower(TURN_SPEED, -TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x >= desired_angle /*+ ADDITIONAL_ANGLE_TO_OVERCOME*/))
		{
			UpdateGyroBlocking();
		}
	}
	else // Giro a sinistra
	{
		last_turn_right = false;
		Serial.println("Giro sinistra <-");
		// Inizio a girare
		ms->SetPower(-TURN_SPEED, TURN_SPEED);
		// Controllo se l'angolo raggiunto è quello desiderato e aspetto nuovi valori del gyro
		while (!(mpu_data.x <= desired_angle /*- ADDITIONAL_ANGLE_TO_OVERCOME*/))
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
