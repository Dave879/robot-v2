#include "Robot.h"

Robot::Robot(gyro *imu, volatile bool *imu_dr, bool cold_start)
{

	PRINTLN("Servo setup started");
	kit.attach(R_PIN_SERVO);
	kit.write(0);
	PRINTLN("Finished servo setup!");

	this->imu = imu;
	imu_data_ready = imu_dr;
	if (cold_start)
	{
		PRINTLN("Disabling and then enabling sensors power supply...");
		pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
		delay(10);																					// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
		delay(10);																					// Wait for sensors to wake up (especially sensor 0)
		PRINTLN("...done!");
	}

	/*
		PRINTLN("Gyro setup started");
		imu = new gyro(SPI, R_IMU_CS_PIN, R_IMU_EXT_CLK_SPI_PIN);
		imu_data_ready = false;
		attachInterrupt(R_IMU_INT_SPI_PIN, R_IMU_int, RISING);
		PRINTLN("Finished gyro setup!");
	*/

	pinMode(R_COLLISION_SX_PIN, INPUT);
	pinMode(R_COLLISION_DX_PIN, INPUT);

	PRINTLN("Motor setup started");
	ms = new Motors();
	PRINTLN("Finished motor setup!");

	Wire2.begin();					 // Lasers
	Wire2.setClock(1000000); // 1MHz

	PRINTLN("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, cold_start);
	PRINTLN("Laser sensors setup finished");

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

	PRINTLN("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		PRINTLN("Initialized color sensor!");
		pinMode(R_PIN_COLOR_INT, INPUT_PULLUP);
		attachInterrupt(R_PIN_COLOR_INT, R_TCS34725_int, FALLING);
		cs->ClearInterrupt();
	}
	else
	{
		PRINTLN("Failed to initialize color sensor!");
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
		if (!NotInRamp())
		{
			if (!was_in_ramp)
			{
				digitalWriteFast(R_LED3_PIN, HIGH);
				was_in_ramp = true;
				time_in_ramp = millis();
			}
			if (imu->y < -10)
			{
				going_down_ramp = true;
			}
			PRINTLN("Rampa");
		}
		else if (was_in_ramp)
		{
			digitalWriteFast(R_LED3_PIN, LOW);
			was_in_ramp = false;
			if (millis() - time_in_ramp > 3000)
			{
				ms->SetPower(45, 45);
				while (imu->y > 0.5 || imu->y < -0.5)
				{
					UpdateGyroBlocking();
				}
				//PRINTLN(imu->y);
/*
				ms->StopMotors();
				PRINTLN("Rampa fatta");
				FakeDelay(1000);

				maze->clear();
				current_x = 1000;
				current_y = 1000;
*/
				SetCurrentTileDistances();
				if (going_down_ramp)
				{
					going_down_ramp = false;
					UpdateSensorNumBlocking(VL53L5CX::BW);
					back_distance_to_reach = DISTANCE_FRONT_AND_BACK_CENTER_TILE + (GetBackDistance() - DISTANCE_FRONT_AND_BACK_CENTER_TILE);
				}
/*
				PRINTLN("Distanze per nuova tile");
				PRINT("Front to reach: ");
				PRINT(front_distance_to_reach);
				PRINT("\tBack to reach: ");
				PRINT(back_distance_to_reach);
*/
			}
		}

		// Black tile
		if (BlackTile() && NotInRamp())
		{
			PRINTLN("Black tile");
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
			ms->SetPower(SPEED, SPEED);
			SetCurrentTileDistances();
			while (!NewTile())
			{
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			// Adding the black tile into the map
			if (direction == 0)
			{
				maze->push({current_x, ++current_y, current_x, current_y});
			}
			else if (direction == 1)
			{
				maze->push({++current_x, current_y, current_x, current_y});
			}
			else if (direction == 2)
			{
				maze->push({current_x, --current_y, current_x, current_y});
			}
			else if (direction == 3)
			{
				maze->push({--current_x, current_y, current_x, current_y});
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
			while (imu->z <= desired_angle /*- ADDITIONAL_ANGLE_TO_OVERCOME*/)
			{
				UpdateGyroBlocking();
				ms->SetPower(-TURN_SPEED, TURN_SPEED);
			}
			ms->StopMotors();
		}
		else if (digitalReadFast(R_COLLISION_DX_PIN) && NotInRamp())
		{
			ms->SetPower(-100, -30);

			FakeDelay(350);

			ms->StopMotors();
			UpdateGyroBlocking();
			while (imu->z >= desired_angle /*+ ADDITIONAL_ANGLE_TO_OVERCOME*/)
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
			// Send signal to watch for victims to OpenMV
			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM){
				Serial8.print('9');
			}
			if (GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM){
				Serial2.print('9');
			}
			// Mapping things
			int16_t old_tile_x = current_x;
			int16_t old_tile_y = current_y;
			ChangeMapPosition();
/*
			PRINT("Direction: ");
			PRINTLN(direction);
			PRINT("current x: ");
			PRINT(current_x);
			PRINT("\tcurrent y: ");
			PRINTLN(current_y);
*/
			maze->push({current_x, current_y, old_tile_x, old_tile_y});
			maze->print();
			// Blue tile check
			if (BlueTile())
			{
				ms->StopMotors();
				PRINTLN("Tile blue");
				for (int8_t i = 0; i < 5; i++)
				{
					digitalWriteFast(R_LED2_PIN, HIGH);
					digitalWriteFast(R_LED4_PIN, HIGH);
					FakeDelay(500);
					digitalWriteFast(R_LED1_PIN, HIGH);
					digitalWriteFast(R_LED3_PIN, HIGH);
					FakeDelay(500);
					digitalWriteFast(R_LED1_PIN, LOW);
					digitalWriteFast(R_LED2_PIN, LOW);
					digitalWriteFast(R_LED3_PIN, LOW);
					digitalWriteFast(R_LED4_PIN, LOW);
				}
			}

			// Per fermare il robot per 0.5s ogni tile (sono per fase di test)
			// ms->StopMotors();
			// FakeDelay(500);

			PRINTLN("Nuova tile");
			PRINT("Front: ");
			PRINT(GetFrontDistance());
			PRINT("\tRight: ");
			PRINT(GetRightDistance());
			PRINT("\tLeft: ");
			PRINT(GetLeftDistance());
			PRINT("\tBack: ");
			PRINTLN(GetBackDistance());

			PRINTLN("Distanze per nuova tile");
			PRINT("Front to reach: ");
			PRINT(front_distance_to_reach);
			PRINT("\tBack to reach: ");
			PRINTLN(back_distance_to_reach);

			ms->StopMotors();
			FakeDelay(250);

			// Victims detection
			while (FoundVictim())
			{
				int kits_number;
				bool left_victim;
				if (Serial2.available() > 0)
				{
					kits_number = int(Serial2.read() - '0');
					left_victim = true;
				}
				else
				{
					kits_number = int(Serial8.read() - '0');
					left_victim = false;
				}
				PRINT("Teensy 4.1 ha ricevuto in seriale: ");
				PRINTLN(kits_number);
				DropKit(kits_number, left_victim);
			}
			Serial8.print('7');
			Serial2.print('7');

			UpdateSensorNumBlocking(VL53L5CX::SX);
			UpdateSensorNumBlocking(VL53L5CX::DX);
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);

			int16_t next_tile = 0;
			bool right_already_visited = false;
			bool left_already_visited = false;
			bool front_already_visited = false;
			switch (direction)
			{
			case 0:
				next_tile = current_x + 1;
				right_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_x - 1;
				left_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_y + 1;
				front_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				break;
			case 1:
				next_tile = current_y - 1;
				right_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_y + 1;
				left_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_x + 1;
				front_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				break;
			case 2:
				next_tile = current_x - 1;
				right_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_x + 1;
				left_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				next_tile = current_y - 1;
				front_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				break;
			case 3:
				next_tile = current_y + 1;
				right_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_y - 1;
				left_already_visited = maze->find({current_x, next_tile, old_tile_x, old_tile_y});
				next_tile = current_x - 1;
				front_already_visited = maze->find({next_tile, current_y, old_tile_x, old_tile_y});
				break;
			default:
				break;
			}

			bool right_blocked = !CanTurnRight() || right_already_visited;
			bool left_blocked = !CanTurnLeft() || left_already_visited;
			bool front_blocked = !CanGoOn() || front_already_visited;

			PRINT("Right blocked: ");
			PRINT(right_blocked);
			PRINT("\tleft blocked: ");
			PRINT(left_blocked);
			PRINT("\tfront blocked: ");
			PRINTLN(front_blocked);

			if (right_blocked && left_blocked && front_blocked)
			{
				Tile previous_tile = maze->get({current_x, current_y, old_tile_x, old_tile_y});
				if (previous_tile.a == 1000 && previous_tile.b == 1000)
				{
					ms->StopMotors();
					for (int8_t i = 0; i < 10; i++)
					{
						digitalWriteFast(R_LED1_PIN, HIGH);
						FakeDelay(250);
						digitalWriteFast(R_LED2_PIN, HIGH);
						FakeDelay(250);
						digitalWriteFast(R_LED4_PIN, HIGH);
						FakeDelay(250);
						digitalWriteFast(R_LED3_PIN, HIGH);
						FakeDelay(250);
						digitalWriteFast(R_LED1_PIN, LOW);
						digitalWriteFast(R_LED2_PIN, LOW);
						digitalWriteFast(R_LED3_PIN, LOW);
						digitalWriteFast(R_LED4_PIN, LOW);
					}
				}
				if (current_x == previous_tile.a)
				{
					if (current_y > previous_tile.b)
					{
						GoToDirection(2);
					}
					else
					{
						GoToDirection(0);
					}
				}
				else
				{
					if (current_x > previous_tile.a)
					{
						GoToDirection(3);
					}
					else
					{
						GoToDirection(1);
					}
				}
			}
			else if (!right_blocked && !left_blocked)
			{
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
							ms->StopMotors();
							AfterTurnVictimDetection();
						}
					}
					else
					{
						// Giro a destra
						TurnRight();
						ms->StopMotors();
						AfterTurnVictimDetection();
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
							ms->StopMotors();
							AfterTurnVictimDetection();
						}
					}
					else
					{
						// Giro a sinistra
						TurnLeft();
						ms->StopMotors();
						AfterTurnVictimDetection();
					}
				}
			}
			// Non è stato rilevato un varco simultaneo, ma solo a destra o sinistra
			else if (!right_blocked || !left_blocked)
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
							ms->StopMotors();
							AfterTurnVictimDetection();
						}
					}
					else
					{
						// Giro a destra
						TurnRight();
						ms->StopMotors();
						AfterTurnVictimDetection();
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
							ms->StopMotors();
							AfterTurnVictimDetection();
						}
					}
					else
					{
						// Giro a sinistra
						TurnLeft();
						ms->StopMotors();
						AfterTurnVictimDetection();
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

/*
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
*/
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
			else if (imu->z < desired_angle - 3)
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
			first_time_pressed = true;
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
			// Reset gyro
			imu->ResetZ();
			desired_angle = 0;
			// Reset Map
			current_x = 1000;
			current_y = 1000;
			direction = 0;
			maze->clear();
			// Set current front/back distance to reach
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
	front_distance_to_reach = (((GetFrontDistance() / 300) - 1) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300) + 1) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = ((((GetBackDistance() / 300) + 1) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE));
	}
}

void Robot::SetCurrentTileDistances()
{
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance() / 300)) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300)) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE;
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = (((GetBackDistance() / 300) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_FRONT_AND_BACK_CENTER_TILE));
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

void Robot::GoToDirection(int8_t direction_to_go)
{
	int8_t delta_dir = direction - direction_to_go;
	if (delta_dir != 0)
	{
		switch (direction)
		{
		case 0:
			if (delta_dir == -1)
			{
				TurnRight();
			}
			else if (delta_dir == -3)
			{
				TurnLeft();
			}
			else if (delta_dir == -2)
			{
				TurnBack();
			}
			break;
		case 1:
			if (delta_dir == -1)
			{
				TurnRight();
			}
			else if (delta_dir == 1)
			{
				TurnLeft();
			}
			else if (delta_dir == -2)
			{
				TurnBack();
			}
			break;
		case 2:
			if (delta_dir == -1)
			{
				TurnRight();
			}
			else if (delta_dir == 1)
			{
				TurnLeft();
			}
			else if (delta_dir == 2)
			{
				TurnBack();
			}
			break;
		case 3:
			if (delta_dir == 3)
			{
				TurnRight();
			}
			else if (delta_dir == 1)
			{
				TurnLeft();
			}
			else if (delta_dir == 2)
			{
				TurnBack();
			}
			break;
		default:
			break;
		}
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
	return GetFrontDistance() <= MIN_DISTANCE_FROM_FRONT_WALL_MM && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5;
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

void Robot::AfterTurnVictimDetection()
{
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM){
		Serial8.print('9');
	}
	if (GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM){
		Serial2.print('9');
	}
	FakeDelay(250);
	while (FoundVictim())
	{
		int kits_number;
		bool left_victim;
		if (Serial2.available() > 0)
		{
			kits_number = int(Serial2.read() - '0');
			left_victim = true;
		}
		else
		{
			kits_number = int(Serial8.read() - '0');
			left_victim = false;
		}
		PRINT("Teensy 4.1 ha ricevuto in seriale: ");
		PRINTLN(kits_number);
		DropKit(kits_number, left_victim);
	}
	Serial8.print('7');
	Serial2.print('7');
}

void Robot::DropKit(int8_t number_of_kits, bool left_victim)
{
	// TODO: Verificare se tenere o meno le due righe sotto
	Serial8.print('7');
	Serial2.print('7');
	// -------------------

	int8_t seconds_to_wait = 6;
	if (left_victim)
	{
		for (int8_t i = 0; i < seconds_to_wait; i++)
		{
			digitalWriteFast(R_LED1_PIN, HIGH);
			digitalWriteFast(R_LED2_PIN, HIGH);
			FakeDelay(250);
			digitalWriteFast(R_LED3_PIN, HIGH);
			digitalWriteFast(R_LED4_PIN, HIGH);
			FakeDelay(250);
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			FakeDelay(250);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
			FakeDelay(250);
		}
	}
	else
	{
		for (int8_t i = 0; i < seconds_to_wait; i++)
		{
			digitalWriteFast(R_LED3_PIN, HIGH);
			digitalWriteFast(R_LED4_PIN, HIGH);
			FakeDelay(250);
			digitalWriteFast(R_LED1_PIN, HIGH);
			digitalWriteFast(R_LED2_PIN, HIGH);
			FakeDelay(250);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
			FakeDelay(250);
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			FakeDelay(250);
		}
	}

	if (kits_dropped < 12 && number_of_kits > 0)
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
			kit.write(180);
			FakeDelay(1000);
			kit.write(0);
			FakeDelay(1000);
			kits_dropped++;
		}

		if (bumped)
		{
			SetCurrentTileDistances();
			ms->SetPower(SPEED, SPEED);
			while (!NewTile())
			{
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			ms->StopMotors();
		}

		Turn(90 * side);
	}
	// TODO: Verificare se tenere o meno le due righe sotto
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	// -------------------
}

void Robot::Straighten()
{
	ms->SetPower(-100, -100);
	FakeDelay(1200);
	ms->StopMotors();
	imu->ResetZ();
	desired_angle = 0;
	SetCurrentTileDistances();
	ms->SetPower(SPEED, SPEED);
	while (!NewTile())
	{
		UpdateSensorNumBlocking(VL53L5CX::FW);
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}
	ms->StopMotors();
}

bool Robot::NotInRamp()
{
	return (imu->y <= 20 && imu->y >= -20);
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

	/*
		PRINT("Desired angle: ");
		PRINT(desired_angle);
		PRINT("\tGyro: ");
		PRINT(imu->z);
		PRINT("\tPID_output: ");
		PRINT(PID_output);
		PRINT(".\tElapsed_seconds: ");
		PRINT(elapsed_seconds);
		PRINT(".\tPID_output * elapsed_seconds: ");
	*/
	int16_t corr = PID_output * elapsed_seconds;
	//	PRINTLN(corr);

	return corr;
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
}

void Robot::TurnBack()
{
	// Fermo il robot prima di girare
	ms->StopMotors();

	// Giro totale (-180°)
	Turn(-90);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM){
		Serial8.print('9');
	}
	FakeDelay(1000);
	while (FoundVictim())
	{
		PRINTLN("Cerco vittima in U");
		int kits_number;
		if (Serial8.available() > 0)
		{
			kits_number = int(Serial8.read() - '0');
			DropKit(kits_number, false);
		}
	}
	Serial8.print('7');

	Turn(-90);
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
	PRINTLN("Metodo Turn: Inizio la manovra!");

	// desired_angle = mpu_data.x + degree;
	desired_angle += degree;

	// Controllo se devo girare a destra o sinistra
	if (degree > 0) // Giro a destra
	{
		PRINTLN("Giro destra ->");
		while (imu->z <= desired_angle - ADDITIONAL_ANGLE_TO_OVERCOME)
		{
			UpdateGyroBlocking();
			/*
			PRINT("Stiamo girando a destra");
			PRINT("\tGyro: ");
			PRINT(imu->z);
			PRINT("\tAngolo desiderato: ");
			PRINTLN(desired_angle);
			*/
			int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(-gyro_speed, +gyro_speed);
		}
	}
	else // Giro a sinistra o indietro
	{
		PRINTLN("Giro sinistra <-");
		while (imu->z >= desired_angle + ADDITIONAL_ANGLE_TO_OVERCOME)
		{
			UpdateGyroBlocking();
			/*
				PRINT("Stiamo girando a sinistra");
				PRINT("\tGyro: ");
				PRINT(imu->z);
				PRINT("\tAngolo desiderato: ");
				PRINTLN(desired_angle);
			*/
			int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			ms->SetPower(-gyro_speed, +gyro_speed);
		}
	}

	// Stop dei motori
	ms->StopMotors();

	PID_integral = 0;
	PRINTLN("Metodo Turn: giro completato!!!");
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

	PRINTLN("Disabling and then enabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);																					// Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	// Enable power supply output to sensors
	delay(10);																					// Wait for sensors to wake up (especially sensor 0)
	PRINTLN("...done!");

	PRINTLN("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, true);
	PRINTLN("Laser sensors setup finished");

	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous); // 8*8, 12Hz

	PRINTLN("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		PRINTLN("Initialized color sensor!");
		cs->ClearInterrupt();
	}
	else
	{
		PRINTLN("Failed to initialize color sensor!");
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

	json_doc[doc_helper.AddLineGraph("Gyro X", -180, 180)] = imu->x;
	json_doc[doc_helper.AddLineGraph("Gyro Y", -180, 180)] = imu->y;
	json_doc[doc_helper.AddLineGraph("Gyro Z")] = imu->z;
	dist[VL53L5CX::FW] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX FW", 8, 8, 0, 1000));
	dist[VL53L5CX::BW] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX BW", 8, 8, 0, 1000));
	dist[VL53L5CX::SX] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX SX", 8, 8, 0, 1000));
	dist[VL53L5CX::DX] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX DX", 8, 8, 0, 1000));

	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 64; j++)
		{
			dist[i].add(lasers->sensors[i]->GetData()->distance_mm[j]);
		}
	}

	/*
		for (uint8_t i = 0; i < 4; i++)
		{
			const char arr[4] = {'F', 'B', 'S', 'D'};
			PRINT("Sensor ");
			PRINT(arr[i]);
			PRINTLN(i);

			for (uint8_t j = 0; j < lasers->resolution; j++)
			{
				// float how_many_tiles = lasers->sensors[i]->GetData()->distance_mm[j] / 300.0f;
				PRINT(lasers->sensors[i]->GetData()->distance_mm[j]);
				PRINT(", \t");
				if ((j + 1) % 8 == 0)
				{
					PRINT("\t");
					for (uint8_t k = abs(7 - j); k <= j; k++)
					{
						PRINT(lasers->sensors[i]->GetData()->target_status[k]);
						PRINT(", \t");
					}
					PRINTLN();
				}
			}
			PRINTLN();
		}
	*/

	json_doc[doc_helper.AddLineGraph("Color: c_comp", 0, 1050)] = cs->c_comp;
	//json_doc[doc_helper.AddLineGraph("Color: r_comp", 0, 500)] = cs->r_comp;
	//json_doc[doc_helper.AddLineGraph("Color: g_comp", 0, 500)] = cs->g_comp;
	//json_doc[doc_helper.AddLineGraph("Color: b_comp", 0, 500)] = cs->b_comp;

	json_doc[doc_helper.AddPacketIndex()] = doc_helper.GetAndIncrementPacketIdx();

	doc_helper.ResetIdx();
	serializeJson(json_doc, Serial);
	json_doc.clear();
	dist->clear();
/*
	PRINT("Serial8 bits available for read (OpenMV DX): ");
	PRINTLN(Serial8.available());

	PRINT("Serial2 bits available for read (OpenMV SX): ");
	PRINTLN(Serial2.available());
*/
}

Robot::~Robot()
{
	delete ms;
	delete lasers;
	delete imu;
}
