#include "Robot.h"

Robot::Robot(gyro *imu, volatile bool *imu_dr, bool cold_start)
{
	this->imu = imu;
	imu_data_ready = imu_dr;
	if (cold_start)
	{
		LOG("Disabling and then enabling sensors power supply...");
		pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
		delay(10);																					// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
		delay(10);																					// Wait for sensors to wake up (especially sensor 0)
		LOG("...done!");
	}

	/*
		LOG("Gyro setup started");
		imu = new gyro(SPI, R_IMU_CS_PIN, R_IMU_EXT_CLK_SPI_PIN);
		imu_data_ready = false;
		attachInterrupt(R_IMU_INT_SPI_PIN, R_IMU_int, RISING);
		LOG("Finished gyro setup!");
	*/

	LOG("Motor setup started");
	ms = new Motors();
	LOG("Finished motor setup!");

	LOG("Servo setup started");
	kit.attach(R_PIN_SERVO);
	kit.write(0);
	LOG("Finished servo setup!");

	Wire2.begin();					 // Lasers
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
	lasers->StartRanging(64, 15, ELIA::RangingMode::kContinuous);								// 8*8, 15Hz

	Wire1.begin();					// Color sensor
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

	pinMode(R_SW_START_PIN, INPUT);
	pinMode(R_SW_XTRA_PIN, INPUT);

	/**
	 * Robot ready signal
	 */

	pinMode(R_LED1_PIN, OUTPUT);
	pinMode(R_LED2_PIN, OUTPUT);
	pinMode(R_LED3_PIN, OUTPUT);
	pinMode(R_LED4_PIN, OUTPUT);

	digitalWriteFast(R_LED1_PIN, HIGH);
	delay(100);
	digitalWriteFast(R_LED1_PIN, LOW);

	imu->ResetAxis();

	PID_start_time = millis();

	// Inizializzazione canale di comunicazione con OpenMV SX, e primo avvio se presente un muro a destra alla partenza
	Serial2.begin(115200);
	Serial2.print('9');

	// Inizializzazione canale di comunicazione con OpenMV DX, e primo avvio se presente un muro a destra alla partenza
	Serial8.begin(115200);
	Serial8.print('9');

	// Initialize front/back distance to reach
	front_distance_to_reach = lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] - DISTANCE_TO_TILE;
	back_distance_to_reach = lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] + DISTANCE_TO_TILE;
}

void Robot::Run()
{
	if (!StopRobot()) // Robot in azionew
	{
		// Victims detection
		if (FoundVictim())
		{
			char kits_number;
			bool left_victim;
			if (Serial2.available() > 0)
			{
				kits_number = Serial2.read();
				left_victim = true;
			}
			else
			{
				kits_number = Serial8.read();
				left_victim = false;
			}
			Serial.print("Teensy 4.1 ha ricevuto in seriale: ");
			Serial.println(kits_number);
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM && !left_victim) || (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[27] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM && left_victim))
			{
				switch (kits_number)
				{
				case '0':
					Serial.println("Vittima: 0 kit");
					ms->StopMotors();
					delay(6000);
					break;
				case '1':
					Serial.println("Vittima: 1 kit");
					ms->StopMotors();
					DropKit(1, left_victim);
					break;
				case '2':
					ms->StopMotors();
					DropKit(2, left_victim);
					Serial.println("Vittima: 2 kit");
					break;
				case '3':
					ms->StopMotors();
					DropKit(3, left_victim);
					Serial.println("Vittima: 3 kit");
					break;
				default:
					Serial.println("No vittima");
					break;
				}
			}
			else
			{
				just_recived_from_openmv = false;
				Serial8.print('9');
				Serial2.print('9');
			}
		just_recived_from_openmv = true;
		time_to_wait_after_openmv_search_again = millis() + 800;
		}
		else if (just_recived_from_openmv && millis() > time_to_wait_after_openmv_search_again)
		{
			just_recived_from_openmv = false;
			Serial8.print('9');
			Serial2.print('9');
		}
		
		// Black tile
		if (cs->c_comp <= MIN_VALUE_BLACK_TILE && imu->y < 20 && imu->y > -20)
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
			delay(400);
			ms->StopMotors();
			Turn(-180);
			just_found_black = true;
		}

		// Controllo se ho raggiunto una nuova tile
		if (NewTile() || lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_FROM_FRONT_WALL_MM || just_found_black)
		{
			digitalWriteFast(R_LED1_PIN, HIGH);
			digitalWriteFast(R_LED2_PIN, HIGH);
			digitalWriteFast(R_LED3_PIN, HIGH);
			digitalWriteFast(R_LED4_PIN, HIGH);

			if (just_found_black )
			{
				just_found_black = false;
			}
			else if (cs->c_comp <= MIN_VALUE_BLUE_TILE && imu->y < 20 && imu->y > -20)
			{
				ms->StopMotors();
				delay(5000);
			}

			// ms->StopMotors();
			// delay(500);
			
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);

			// Controllo se ho entrabi i lati liberi
			if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[27] >= MIN_DISTANCE_TO_TURN_MM) && (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= MIN_DISTANCE_TO_TURN_MM))
			{
				// Output variabili varco
				Serial.println("Varco Trovato!!!");
				// Varco Destra
				Serial.print("Distanza Destra: ");
				Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL]);
				// Varco Sinistra
				Serial.print("Distanza Sinistra: ");
				Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL]);

				// Scelgo quale direzione prendere tra destra e sinistra
				if (millis() % 2)
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
			else if ((lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= MIN_DISTANCE_TO_TURN_MM) || (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= MIN_DISTANCE_TO_TURN_MM))
			{
				// Giro a destra
				if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= MIN_DISTANCE_TO_TURN_MM)
				{
					if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
					{
						if (millis() % 2)
						{
							// Giro a destra
							TurnRight();

							UpdateSensorNumBlocking(VL53L5CX::BW);
							if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
							{
								// Manovra da eseguire per ristabilizzare il robot e resettare il giro
								Straighten();
							}
						}
					}
					else
					{
						// Output variabili varco
						Serial.println("Varco Trovato!!!");
						Serial.print("Distanza Destra: ");
						Serial.println(lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL]);

						// Giro a destra(90°)
						TurnRight();
					}
				}
				// Giro a sinistra
				else if (lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= MIN_DISTANCE_TO_TURN_MM)
				{
					if (lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
					{
						if (millis() % 2)
						{
							// Giro a sinistra
							TurnLeft();

							UpdateSensorNumBlocking(VL53L5CX::BW);
							if (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_TO_SET_IGNORE_FALSE_MM)
							{
								// Manovra da eseguire per ristabilizzare il robot e resettare il giro
								Straighten();
							}
						}
					}
					else
					{
						// Output variabili varco
						Serial.println("Varco Trovato!!!");
						Serial.print("Distanza Sinistra: ");
						Serial.println(lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL]);

						// Giro a sinistra(-90°)
						TurnLeft();
					}
				}
			}
			else if ((lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= MIN_DISTANCE_FROM_FRONT_WALL_MM && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5))
			{
				TurnBack();
			}
			
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);
			
			front_distance_to_reach = lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] - DISTANCE_TO_TILE;
			back_distance_to_reach = lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] + DISTANCE_TO_TILE;

			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
		}
		else
		{
			ms->SetPower(SPEED, SPEED);
		}
	}
	else // Roboto fermo
	{
		ms->StopMotors();
		Serial.println("Premere il pulsante per far partire il robot!");
	}
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
	if (digitalReadFast(R_SW_START_PIN))
	{
		if (!first_time_pressed)
		{
			stop_the_robot = !stop_the_robot;
			if (!stop_the_robot)
			{
			}
				// OpenMV discard old data
				while (Serial2.available())
				{
					Serial2.read();
				}
				while (Serial8.available())
				{
					Serial8.read();
				}
				Serial2.print('9');
				Serial8.print('9');
				// Reset gyro
				imu->ResetAxis();
				desired_angle = 0;
				// Set new front/back distance to reach
				front_distance_to_reach = lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] - DISTANCE_TO_TILE;
				back_distance_to_reach = lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] + DISTANCE_TO_TILE;
			first_time_pressed = true;
		}
	}
	else
	{
		first_time_pressed = false;
	}
	return stop_the_robot;
}

bool Robot::NewTile()
{
	return (lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] <= front_distance_to_reach && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5) || (lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL] >= back_distance_to_reach && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5);
}

bool Robot::FoundVictim()
{
	return Serial2.available() > 0 || Serial8.available() > 0;
}

void Robot::DropKit(int8_t number_of_kits, bool left_victim)
{
	if (number_of_kits > 1)
	{
		ms->SetPower(-50, -50);
		delay(400);
	}

	int8_t side = 1;

	if (left_victim)
	{
		side = -1;
	}

	Turn(-90 * side);
	ms->SetPower(-90, -80);
	delay(500);
	ms->SetPower(40, 40);
	delay(200);
	ms->StopMotors();

	for (int8_t i = 0; i < number_of_kits; i++)
	{
		digitalWriteFast(R_LED1_PIN, HIGH);
		digitalWriteFast(R_LED2_PIN, HIGH);
		digitalWriteFast(R_LED3_PIN, HIGH);
		digitalWriteFast(R_LED4_PIN, HIGH);
		kit.write(180);
		delay(1000);
		kit.write(0);
		digitalWriteFast(R_LED1_PIN, LOW);
		digitalWriteFast(R_LED2_PIN, LOW);
		digitalWriteFast(R_LED3_PIN, LOW);
		digitalWriteFast(R_LED4_PIN, LOW);
		delay(1000);
	}

	Turn(90 * side);

	delay(5000 - (1000 * number_of_kits));
}

void Robot::Straighten()
{
	ms->SetPower(-90, -90);
	delay(1200);
	ms->StopMotors();
	imu->ResetAxis();
	desired_angle = 0;
	ms->SetPower(40, 40);
	delay(400);
}

int16_t Robot::GetPIDOutputAndSec()
{
	// Calculate error
	PID_error = CalculateError(imu->z);
	// Calculate integral
	PID_integral += PID_error;
	// Calculate derivative
	double derivative = PID_error - PID_previous_error;
	PID_previous_error = PID_error;
	// Calculate output
	PID_output = KP * PID_error + KI * PID_integral + KD * derivative;
	// e motor powers and apply motor powers to left and right motors
	uint32_t elapsed_seconds = micros() - PID_start_time;

	// Update start time
	PID_start_time = micros();

	Serial.print("Desired angle: ");
	Serial.print(desired_angle);
	Serial.print("\tGyro: ");
	Serial.print(imu->z);
	Serial.print("\tPID_output: ");
	Serial.print(PID_output);
	Serial.print(".\tElapsed_seconds: ");
	Serial.print(elapsed_seconds);
	Serial.print(".\tPID_output * elapsed_seconds: ");
	int16_t corr = PID_output * elapsed_seconds;
	Serial.println(corr);

	return corr;
}

void Robot::MotorPowerZGyroAndPID()
{
	/*
	int16_t pid_speed = GetPIDOutputAndSec() / 1100;
	ms->SetPower(SPEED - pid_speed, SPEED + pid_speed);
	*/
	Serial.print("Desired angle: ");
	Serial.print(desired_angle);
	Serial.print("\tGyro: ");
	Serial.println(imu->z);

	int16_t gyro_diff = (desired_angle - imu->z);
	ms->SetPower(SPEED - gyro_diff, SPEED + gyro_diff);
}

void Robot::TurnRight()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro a destra (90°)
	Turn(90);
}

void Robot::TurnLeft()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro a sinistra (-90°)
	Turn(-90);
}

void Robot::TurnBack()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro totaale (-180°)
	Turn(-180);

	// Radrizzo il robot
	Straighten();
}

void Robot::Turn(int16_t degree)
{
	// Output metodo Turn()
	Serial.println("Metodo Turn: Inizio la manovra!");

	// desired_angle = mpu_data.x + degree;
	desired_angle += degree;

	// Controllo se devo girare a destra o sinistra
	if (degree > 0) // Giro a destra
	{
		Serial.println("Giro destra ->");
		while (imu->z <= desired_angle - ADDITIONAL_ANGLE_TO_OVERCOME)
		{
			UpdateGyroBlocking();
			Serial.print("Stiamo girando a destra");
			Serial.print("\tGyro: ");
			Serial.print(imu->z);
			Serial.print("\tAngolo desiderato: ");
			Serial.println(desired_angle);
			int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(-gyro_speed, +gyro_speed);
		}
	}
	else // Giro a sinistra o indietro
	{
		Serial.println("Giro sinistra <-");
		while (imu->z >= desired_angle + ADDITIONAL_ANGLE_TO_OVERCOME)
		{
			UpdateGyroBlocking();
			Serial.print("Stiamo girando a sinistra");
			Serial.print("\tGyro: ");
			Serial.print(imu->z);
			Serial.print("\tAngolo desiderato: ");
			Serial.println(desired_angle);
			int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(-gyro_speed, +gyro_speed);
		}
	}

	PID_integral = 0;

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

	if (imu_data_ready)
	{
		imu->UpdateData();
		*imu_data_ready = false;
	}
	for (uint8_t i = 0; i < 4; i++)
	{
		if (lasers_data_ready[i])
		{
			lasers->sensors[i]->UpdateData();
			lasers_data_ready[i] = false;
			status |= 0b10000 << i;
		}
		if (imu_data_ready)
		{
			imu->UpdateData();
			*imu_data_ready = false;
		}
	}

	if (color_data_ready)
	{
		cs->getData();
	}
	if (imu_data_ready)
	{
		imu->UpdateData();
		*imu_data_ready = false;
	}
	return status;
}

void Robot::UpdateSensorNumBlocking(VL53L5CX num)
{
	while (1)
	{
		if (imu_data_ready)
		{
			imu->UpdateData();
			*imu_data_ready = false;
		}
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
		if (imu_data_ready)
		{
			imu->UpdateData();
			*imu_data_ready = false;
			break;
		}
	}
}

void Robot::PrintSensorData()
{

	Serial.print("gyro.x: \t");
	Serial.print(imu->x);
	Serial.print(" \t");
	Serial.print("gyro.y: \t");
	Serial.print(imu->y);
	Serial.print(" \t");
	Serial.print("gyro.z: \t");
	Serial.println(imu->z);

	for (uint8_t i = 0; i < 4; i++)
	{
		const char arr[4] = {'F', 'B', 'S', 'D'};
		Serial.print("Sensor ");
		Serial.print(arr[i]);
		Serial.println(i);

		for (uint8_t j = 0; j < lasers->resolution; j++)
		{
			// float how_many_tiles = lasers->sensors[i]->GetData()->distance_mm[j] / 300.0f;
			Serial.print(lasers->sensors[i]->GetData()->distance_mm[j]);
			Serial.print(", \t");
			if ((j + 1) % 8 == 0)
			{
				Serial.print("\t");
				for (uint8_t k = abs(7 - j); k <= j; k++)
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

	Serial.print("Serial8 bits available for read (OpenMV DX): ");
	Serial.println(Serial8.available());

	Serial.print("Serial2 bits available for read (OpenMV SX): ");
	Serial.println(Serial2.available());
}

Robot::~Robot()
{
	delete ms;
	delete lasers;
	delete imu;
}
