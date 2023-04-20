#include "Robot.h"

Robot::Robot(gyro *imu, volatile bool *imu_dr, bool cold_start)
{

	LOG("Servo setup started");
	kit.attach(R_PIN_SERVO);
	kit.write(0);
	LOG("Finished servo setup!");

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

	pinMode(R_COLLISION_SX_PIN, INPUT);
	pinMode(R_COLLISION_DX_PIN, INPUT);

	LOG("Motor setup started");
	ms = new Motors();
	LOG("Finished motor setup!");

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
	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous);								// 8*8, 12Hz

	Wire1.begin();					// Color sensor
	Wire1.setClock(400000); // 400kHz

	LOG("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		LOG("Initialized color sensor!");
		pinMode(R_PIN_COLOR_INT, INPUT_PULLUP);
		attachInterrupt(R_PIN_COLOR_INT, R_TCS34725_int, FALLING);
		cs->ClearInterrupt();
	}
	else
	{
		LOG("Failed to initialize color sensor!");
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

	imu->ResetZ();

	maze = new Map();

	FakeDelay(2000);

	// Inizializzazione canale di comunicazione con OpenMV SX, e primo avvio se presente un muro a destra alla partenza
	Serial2.begin(115200);
	Serial2.print('9');

	// Inizializzazione canale di comunicazione con OpenMV DX, e primo avvio se presente un muro a destra alla partenza
	Serial8.begin(115200);
	Serial8.print('9');

	// Get first old_gyro value for check drift
	old_gyro_value = imu->z;

	// Initialize front/back distance to reach
	SetNewTileDistances();
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
			just_recived_from_openmv = true;
			if (NotInRamp() && ((!CanTurnRight() && !left_victim) || (!CanTurnLeft() && left_victim)))
			{
				switch (kits_number)
				{
				case '0':
					Serial.println("Vittima: 0 kit");
					ms->StopMotors();
					DropKitNoTurn(0);
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
				if (left_victim)
				{
					Serial2.print('9');
				}
				else
				{
					Serial8.print('9');
				}
			}
			time_to_wait_after_openmv_search_again = millis() + 2500;
		}
		else if (just_recived_from_openmv && millis() > time_to_wait_after_openmv_search_again)
		{
			just_recived_from_openmv = false;
			Serial8.print('9');
			Serial2.print('9');
		}
		

		if (!NotInRamp())
		{
			if (!was_in_ramp)
			{
				was_in_ramp = true;
				time_in_ramp = millis();
			}
			if (imu->y < -10)
			{
				going_down_ramp = true;
			}
			Serial.println("Rampa");
		}
		else if(was_in_ramp)
		{
			if (millis() - time_in_ramp > 2500)
			{
				ms->SetPower(45,45);
				while (imu->y > 1 || imu->y < -1)
				{
					UpdateGyroBlocking();
				}
				Serial.println(imu->y);
				FakeDelay(350);

/*
				ms->StopMotors();
				Serial.println("Rampa fatta");
				FakeDelay(1000);
*/

				//SetNewTileDistances();
				/*
				maze->clear();
				current_x = 0;
				current_y = 0;
				*/
				SetCurrentTileDistances();
				if (going_down_ramp)
				{
					going_down_ramp = false;
					UpdateSensorNumBlocking(VL53L5CX::BW);
					back_distance_to_reach = (((GetBackDistance()/ 300)) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE + GetBackDistance() - (GetBackDistance() / 300) * 320;                                      					//back_distance_to_reach = GetBackDistance() + 60 /*260*/;
				}
				Serial.println("Distanze per nuova tile");
				Serial.print("Front to reach: ");
				Serial.print(front_distance_to_reach);
				Serial.print("\tBack to reach: ");
				Serial.print(back_distance_to_reach);
				//FakeDelay(1000);
			}
			was_in_ramp = false;
		}

		// Black tile
		if (BlackTile() && NotInRamp())
		{
			Serial.println("Black tile");
			// Torno in dietro fino a quando smetto di vedere nero
			ms->SetPower(-45, -45);
			while (cs->c_comp <= MIN_VALUE_BLUE_TILE)
			{
				if (color_data_ready)
				{
					cs->getData();
					color_data_ready = false;
				}
			}
			SetCurrentTileDistances();
			while (NewTile())
			{
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			
			if (direction == 0)
			{
				int16_t black_tile_y = current_x + 1;
				maze->push({current_x , black_tile_y, current_x, current_y});
			}
			else if (direction == 1)
			{
				int16_t black_tile_x = current_x + 1;
				maze->push({black_tile_x, current_y, current_x, current_y});
			}
			else if (direction == 2)
			{
				int16_t black_tile_y = current_x -1;
				maze->push({current_x , black_tile_y, current_x, current_y});
			}
			else if (direction == 3)
			{
				int16_t black_tile_x = current_x - 1;
				maze->push({black_tile_x, current_y, current_x, current_y});
			}


			// Mando il robot indietro al centro della tile
			TurnBack();
			SetCurrentTileDistances();
		}

		// Se ho colpito un muretto con gli switch
		if (digitalReadFast(R_COLLISION_SX_PIN) && NotInRamp())
		{
			ms->SetPower(-30, -100);

			FakeDelay(350);

			ms->StopMotors();
			UpdateGyroBlocking();
			while (imu->z <= desired_angle - ADDITIONAL_ANGLE_TO_OVERCOME)
			{
				UpdateGyroBlocking();
				ms->SetPower(-TURN_SPEED, TURN_SPEED);
			}
			ms->StopMotors();
		}
		else if (digitalReadFast(R_COLLISION_DX_PIN)  && NotInRamp())
		{
			ms->SetPower(-100, -30);

			FakeDelay(350);

			ms->StopMotors();
			UpdateGyroBlocking();
			while (imu->z >= desired_angle + ADDITIONAL_ANGLE_TO_OVERCOME)
			{	
				UpdateGyroBlocking();
				ms->SetPower(TURN_SPEED, -TURN_SPEED);
			}
			ms->StopMotors();
		}

		// Controllo se ho raggiunto una nuova tile
		if ((NewTile() && NotInRamp()) || FrontWall())
		{

			/*
			digitalWriteFast(R_LED1_PIN, HIGH);
			digitalWriteFast(R_LED2_PIN, HIGH);
			digitalWriteFast(R_LED3_PIN, HIGH);
			digitalWriteFast(R_LED4_PIN, HIGH);
			*/
			int16_t old_tile_x = current_x;
			int16_t old_tile_y = current_y;
			ChangeMapPosition();
			Serial.print("Direction: ");
			Serial.println(direction);
			maze->push({current_x, current_y, old_tile_x, old_tile_y});

			maze->print();

			if (BlueTile())
			{
				ms->StopMotors();
				Serial.println("Tile blue");
				FakeDelay(5000);
			}

			// Per fermare il robot per 0.5s ogni tile (sono per fase di test)
			// ms->StopMotors();
			// FakeDelay(500);

			Serial.println("Nuova tile");
			Serial.print("Front: ");
			Serial.print(GetFrontDistance());
			Serial.print("\tRight: ");
			Serial.print(GetRightDistance());
			Serial.print("\tLeft: ");
			Serial.print(GetLeftDistance());
			Serial.print("\tBack: ");
			Serial.println(GetBackDistance());

			Serial.println("Distanze per nuova tile");
			Serial.print("Front to reach: ");
			Serial.print(front_distance_to_reach);
			Serial.print("\tBack to reach: ");
			Serial.print(back_distance_to_reach);

			ms->StopMotors();
			FakeDelay(250);

			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);

			int16_t next_tile = 0;
			bool right_already_visited = false;
			bool left_already_visited = false;
			bool front_already_visited = false;
			if (direction == 0)
			{
				next_tile = current_x + 1;
				right_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_x - 1;
				left_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_y + 1;
				front_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
			}
			else if (direction == 1)
			{
				next_tile = current_y - 1;
				right_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_y + 1;
				left_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_x + 1;
				front_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
			}
			else if (direction == 2)
			{
				next_tile = current_x - 1;
				right_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_x + 1;
				left_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_y - 1;
				front_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
			}
			else if (direction == 3)
			{
				next_tile = current_y + 1;
				right_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_y - 1;
				left_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_x - 1;
				front_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
			}
			
			bool right_blocked = !CanTurnRight() || right_already_visited;
			bool left_blocked = !CanTurnLeft() || left_already_visited;
			bool front_blocked = !CanGoOn() || front_already_visited;

			if (right_blocked && left_blocked && front_blocked)
			{
				Tile previus_tile = maze->get({current_x, current_y, old_tile_x, old_tile_y});
				if (current_x == previus_tile.a)
				{
					if (current_y > previus_tile.b)
					{
						GoToDIrection(2);
					}
					else
					{
						GoToDIrection(0);
					}
				}
				else
				{
					if (current_x > previus_tile.x)
					{
						GoToDIrection(1);
					}
					else
					{
						GoToDIrection(3);
					}
				}
			}
			else if (!right_blocked && !left_blocked)
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
					if (!front_blocked)
					{
						// Giro o continuo ad andare dritto
						if (millis() % 2)
						{
							// Giro a destra
							TurnRight();
						}
					}
					else
					{
						// Giro a destra
						TurnRight();
					}
				}
				// Giro a sinistra
				else
				{
					if (!front_blocked)
					{
						// Giro o continuo ad andare dritto
						if (millis() % 2)
						{
							// Giro a sinistra
							TurnLeft();
						}
					}
					else
					{
						// Giro a sinistra
						TurnLeft();
					}
				}
			}
			// Non è stato rilevato un varco simultaneo, ma solo a destra o sinistra
			else if (!right_blocked || ! left_blocked)
			{
				// Giro a destra
				if (!right_blocked)
				{
					if (!front_blocked)
					{
						// Giro o continuo ad andare dritto
						if (millis() % 2)
						{
							// Giro a destra
							TurnRight();
						}
					}
					else
					{
						// Giro a destra
						TurnRight();
					}
				}
				// Giro a sinistra
				else if (!left_blocked)
				{
					if (!front_blocked)
					{
						// Giro o continuo ad andare dritto
						if (millis() % 2)
						{
							// Giro a sinistra
							TurnLeft();
						}
					}
					else
					{
						// Giro a sinistra
						TurnLeft();
					}
				}
			}
			/*
			else if (!CanGoOn() && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
			{
				TurnBack();
			}
			*/

			FakeDelay(250);
			SetNewTileDistances();
			
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
		}
		// Proseguo diretto
		else
		{
			int16_t power_to_add = imu->y / 1.5;
			if (power_to_add < -10)
			{
				power_to_add = -10;
			}
			if (imu->y > 15)
			{
				power_to_add = 25;
			}
			if (imu->z > desired_angle + 3)
			{
				ms->SetPower(SPEED + power_to_add + 5, SPEED + power_to_add - 5);
			}
			else if(imu->z < desired_angle - 3)
			{
				ms->SetPower(SPEED + power_to_add - 5, SPEED + power_to_add + 5);
			}
			else
			{
				ms->SetPower(SPEED + power_to_add, SPEED + power_to_add);
			}
		}
	}
	else // Roboto fermo
	{
		ms->StopMotors();
		Serial.println("Premere il pulsante per far partire il robot!");
		UpdateGyroBlocking();
		if (imu->z - old_gyro_value > 0.1 || imu->z - old_gyro_value < -0.1)
		{
			digitalToggleFast(R_LED3_PIN);
			old_gyro_value = imu->z;
		}
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
				imu->ResetZ();
				desired_angle = 0;

				current_x = 0;
				current_y = 0;
				direction = 0;
				maze->clear();
				// Set new front/back distance to reach
				SetNewTileDistances();
				first_time_pressed = true;
				SetCurrentTileDistances();
				digitalWriteFast(R_LED3_PIN, LOW);
		}
	}
	else
	{
		first_time_pressed = false;
	}
	return stop_the_robot;
}

int16_t Robot::GetRightDistance()
{
	return lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL];
}

int16_t Robot::GetLeftDistance()
{
	return lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL];
}

int16_t Robot::GetFrontDistance()
{
	return lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL];
}

int16_t Robot::GetBackDistance()
{
	return lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL];
}

void Robot::SetNewTileDistances()
{
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance()/ 300) - 1) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance()/ 300) + 1) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	if (GetBackDistance() - (GetBackDistance() / 300) * 320 > 300 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = (((GetBackDistance()/ 300) + 1) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE + GetBackDistance() - (GetBackDistance() / 300) * 320;
	}
}

void Robot::SetCurrentTileDistances()
{
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance()/ 300)) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance()/ 300)) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	if (GetBackDistance() - (GetBackDistance() / 300) * 320 > 300 && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = (((GetBackDistance()/ 300)) * 320 ) + DISTANCE_FRONT_AND_BACK_CENTER_TILE + GetBackDistance() - (GetBackDistance() / 300) * 320;
	}
}

void Robot::ChangeMapX(bool increment)
{
	if (increment)
	{
		current_x++;
	}
	else
	{
		current_x--;
	}

}

void Robot::ChangeMapY(bool increment)
{
	if (increment)
	{
		current_y++;
	}
	else
	{
		current_y--;
	}
}

void Robot::ChangeMapPosition()
{
	switch (direction)
	{
	case 0:
		ChangeMapY(true);
		break;
	case 1:
		ChangeMapX(true);
		break;
	case 2:
		ChangeMapY(false);
		break;
	case 3:
		ChangeMapX(false);
		break;
	}
}

void Robot::IncreaseDirection()
{
	direction++;
	direction %= 4;
}

void Robot::DecreaseDirection()
{
	direction += 3;
	direction %= 4;
}

void Robot::GoToDIrection(int8_t direction_to_go)
{
	int8_t delta_dir = direction - direction_to_go;
	if (abs(delta_dir) == 2)
	{
		TurnBack();
	}
	else if (delta_dir == -3 || delta_dir == 1)
	{
		TurnLeft();
	}
	else if (delta_dir == 3 || delta_dir == -1)
	{
		TurnRight();
	}
}

bool Robot::CanTurnRight()
{
	return GetRightDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::DX]->GetData()->target_status[DISTANCE_SENSOR_CELL] != 5;
}

bool Robot::CanTurnLeft()
{
	return GetLeftDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::SX]->GetData()->target_status[DISTANCE_SENSOR_CELL] != 5;
}

bool Robot::CanGoOn()
{
	return GetFrontDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] != 5;
}

bool Robot::CanBumpBack()
{
	return GetBackDistance() <= MIN_DISTANCE_BUMP_BACK_WALL_MM && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5;
}

bool Robot::FrontWall()
{
	return GetFrontDistance() <= MIN_DISTANCE_FROM_FRONT_WALL_MM;
}

bool Robot::BlackTile()
{
	return cs->c_comp <= MIN_VALUE_BLACK_TILE;
}

bool Robot::BlueTile()
{
	return cs->c_comp <= MIN_VALUE_BLUE_TILE;
}

bool Robot::NewTile()
{
	return (GetFrontDistance() <= front_distance_to_reach && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5) || (GetBackDistance() >= back_distance_to_reach && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5);
}

bool Robot::FoundVictim()
{
	return Serial2.available() > 0 || Serial8.available() > 0;
}

void Robot::VictimVerify()
{
	if (FoundVictim())
	{
		Serial.println("Verifico vittima");
		// Fermo il robot, in alcuni casi speciali potrebbe andare avanti e buttare kit se tolto
		ms->StopMotors();
		ms->SetPower(-45, -45);
		FakeDelay(100);
		ms->StopMotors();
		int kits_number;
		if (Serial2.available() > 0)
		{
			kits_number = int(Serial2.read() - '0');
			Serial2.print('9');
			FakeDelay(1000);
			if (!Serial2.available())
			{
				Serial.println("Vittima dietro (prima a sinistra)");
				DropKitNoTurn(kits_number);
				ms->SetPower(45, 45);
				FakeDelay(200);
				ms->StopMotors();
			}
		}
		else
		{
			kits_number = int(Serial8.read() - '0');
			Serial8.print('9');
			FakeDelay(1000);
			if (!Serial8.available())
			{
				Serial.println("Vittima dietro (prima a destra)");
				DropKitNoTurn(kits_number);
				ms->SetPower(45, 45);
				FakeDelay(200);
				ms->StopMotors();
			}
		}
	}
}

void Robot::RemoveVictimU()
{
	if (FoundVictim())
	{
		if (Serial2.available() > 0)
		{
			Serial2.read();
			Serial2.print('9');;
		}
		else
		{
			Serial8.read();
			Serial8.print('9');
		}
	}
}

void Robot::DropKit(int8_t number_of_kits, bool left_victim) 
{
	Serial8.print('7');
	Serial2.print('7');	
	if (number_of_kits > 1 && kits_dropped < 12)
	{
		ms->SetPower(-45, -45);
		FakeDelay(200);
		ms->StopMotors();
	}


	int8_t seconds_to_wait = 6;
	for (int8_t i = 0; i < seconds_to_wait; i++)
	{
		digitalWriteFast(R_LED1_PIN, HIGH);
		digitalWriteFast(R_LED2_PIN, HIGH);
		digitalWriteFast(R_LED3_PIN, HIGH);
		digitalWriteFast(R_LED4_PIN, HIGH);
		FakeDelay(500);
		digitalWriteFast(R_LED1_PIN, LOW);
		digitalWriteFast(R_LED2_PIN, LOW);
		digitalWriteFast(R_LED3_PIN, LOW);
		digitalWriteFast(R_LED4_PIN, LOW);
		FakeDelay(500);
	}
	
	if (kits_dropped < 12)
	{
		int8_t side = 1;

		if (left_victim)
		{
			side = -1;
		}

		Turn(-90 * side);
		bool bumped = false;
		UpdateSensorNumBlocking(VL53L5CX::BW);
		if (CanBumpBack())
		{
			ms->SetPower(-90, -90);
			FakeDelay(500);
			ms->StopMotors();
			bumped = true;
		}


		for (int8_t i = 0; i < number_of_kits; i++)
		{
			Serial.println("Sono nel blocco del drop kit con svolta");
			kit.write(180);
			FakeDelay(1000);
			kit.write(0);
			FakeDelay(1000);
			kits_dropped++;
		}

		if (bumped)
		{
			ms->SetPower(45, 45);
			FakeDelay(250);
		}
		
		Turn(90 * side);

		if (left_victim)
		{
			Serial8.print('9');
		}
		else
		{
			Serial2.print('9');
		}
	}

	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
}

void Robot::DropKitNoTurn(int8_t number_of_kits)
{
	ms->StopMotors();
	int8_t seconds_to_wait = 6;
	for (int8_t i = 0; i < seconds_to_wait; i++)
	{
		digitalWriteFast(R_LED1_PIN, HIGH);
		digitalWriteFast(R_LED2_PIN, HIGH);
		digitalWriteFast(R_LED3_PIN, HIGH);
		digitalWriteFast(R_LED4_PIN, HIGH);
		FakeDelay(500);
		digitalWriteFast(R_LED1_PIN, LOW);
		digitalWriteFast(R_LED2_PIN, LOW);
		digitalWriteFast(R_LED3_PIN, LOW);
		digitalWriteFast(R_LED4_PIN, LOW);
		FakeDelay(500);
	}

	if (kits_dropped < 12)
	{
		for (int8_t i = 0; i < number_of_kits; i++)
		{
			Serial.println("Sono nel blocco del drop kit senza svolta");
			kit.write(180);
			FakeDelay(1000);
			kit.write(0);
			FakeDelay(1000);
			kits_dropped++;
		}
	}
}

void Robot::Straighten()
{
	ms->SetPower(-100, -100);
	FakeDelay(1200);
	ms->StopMotors();
	imu->ResetZ();
	desired_angle = 0;
	SetCurrentTileDistances();
	ms->SetPower(45, 45);
	while (!NewTile())
	{
		UpdateSensorNumBlocking(VL53L5CX::FW);
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}
	
}

bool Robot::NotInRamp()
{
	return (imu->y <= 10 && imu->y >= -10);
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

	IncreaseDirection();

	// Controllo se posso adrizzare il robot dopo la svolta
	UpdateSensorNumBlocking(VL53L5CX::BW);
	if (CanBumpBack())
	{
		// Manovra da eseguire per ristabilizzare il robot e resettare il giro
		Straighten();
	}

	// Controllo se sono presenti vittime, più eventuale conferma
	VictimVerify();
}

void Robot::TurnLeft()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro a sinistra (-90°)
	Turn(-90);

	DecreaseDirection();

	// Controllo se posso adrizzare il robot dopo la svolta
	UpdateSensorNumBlocking(VL53L5CX::BW);
	if (CanBumpBack())
	{
		// Manovra da eseguire per ristabilizzare il robot e resettare il giro
		Straighten();
	}

	// Controllo se sono presenti vittime, più eventuale conferma
	VictimVerify();
}

void Robot::TurnBack()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro totale (-180°)
	//Turn(-180);
	//New solution
	Turn(-90);
	RemoveVictimU();
	FakeDelay(1000);
	if (FoundVictim())
	{
		Serial.println("Cerco vittima in U");
		// Fermo il robot, in alcuni casi speciali potrebbe andare avanti e buttare kit se tolto
		ms->StopMotors();
		ms->SetPower(-45, -45);
		FakeDelay(100);
		ms->StopMotors();
		int kits_number;
		if (Serial8.available() > 0)
		{
			kits_number = int(Serial8.read() - '0');
			DropKit(kits_number, false);
		}
	}

	Turn(-90);
	Serial8.print('9');
	DecreaseDirection();
	DecreaseDirection();

	// Controllo se posso adrizzare il robot dopo la svolta
	UpdateSensorNumBlocking(VL53L5CX::BW);
	if (CanBumpBack())
	{
		// Manovra da eseguire per ristabilizzare il robot e resettare il giro
		Straighten();
	}
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
			/*
			Serial.print("Stiamo girando a destra");
			Serial.print("\tGyro: ");
			Serial.print(imu->z);
			Serial.print("\tAngolo desiderato: ");
			Serial.println(desired_angle);
			*/
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
		/*
		if (degree == -180)
		{
			RemoveVictimU();
		}
		*/
	}
	
	// Stop dei motori
	ms->StopMotors();
	
	PID_integral = 0;
	Serial.println("Metodo Turn: giro completato!!!");
}

void Robot::FakeDelay(uint32_t time)
{
	uint32_t time_to_wait = millis() + time;
	while (millis() < time_to_wait)
	{
		if (imu_data_ready)
		{
			imu->UpdateData();
			*imu_data_ready = false;
		}
	}
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
	uint32_t time_end = millis() + 500;
	while (millis() < time_end)
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
			return;
		}
	}

	ms->StopMotors();

	digitalWriteFast(R_LED1_PIN, LOW);
	digitalWriteFast(R_LED2_PIN, LOW);
	digitalWriteFast(R_LED3_PIN, LOW);
	digitalWriteFast(R_LED4_PIN, HIGH);

	LOG("Disabling and then enabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);																					// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
	delay(10);																					// Wait for sensors to wake up (especially sensor 0)
	LOG("...done!");

	LOG("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, true);
	LOG("Laser sensors setup finished");	

	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous);								// 8*8, 12Hz

	LOG("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		LOG("Initialized color sensor!");
		cs->ClearInterrupt();
	}
	else
	{
		LOG("Failed to initialize color sensor!");
	}

	digitalWriteFast(R_LED4_PIN, LOW);

	imu->ResetZ();	
	
	// Initialize front/back distance to reach
	SetCurrentTileDistances();
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
