#include "Robot.h"

Robot::Robot(gyro *imu, volatile bool *imu_dr, bool cold_start)
{
	/**
	 * The following two lines MUST be first, because Robot::FakeDelay() needs the gyro pointer to be initialized correctly
	 */
	this->imu = imu;
	imu_data_ready = imu_dr;

	// Inizializzazione canale di comunicazione con OpenMV SX
	OPENMV_SX.begin(115200);

	// Inizializzazione canale di comunicazione con OpenMV DX
	OPENMV_DX.begin(115200);

	// TODO: check if OpenMV cams are alive

	if (digitalReadFast(R_SW_XTRA_PIN))
		TestPeripherals();

	Serial.println("Servo setup started");
	kit.attach(R_PIN_SERVO);
	kit.write(0);
	Serial.println("Finished servo setup!");

	if (cold_start)
	{
		Serial.println("Disabling and then enabling sensors power supply...");
		pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
		delay(10);														 // Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
		digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	 // Enable power supply output to sensors
		delay(10);														 // Wait for sensors to wake up (especially sensor 0)
		Serial.println("...done!");
	}

	/*
		Serial.println("Gyro setup started");
		imu = new gyro(SPI, R_IMU_CS_PIN, R_IMU_EXT_CLK_SPI_PIN);
		imu_data_ready = false;
		attachInterrupt(R_IMU_INT_SPI_PIN, R_IMU_int, RISING);
		Serial.println("Finished gyro setup!");
	*/

	Serial.println("Motor setup started");
	ms = new Motors();
	Serial.println("Finished motor setup!");

	Wire2.begin();				 // Lasers
	Wire2.setClock(1000000); // 1MHz

	Serial.println("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, cold_start);
	Serial.println("Laser sensors setup finished");

	Serial.println("Front ir sensor setup started");
	ir_front = new LRir(Serial3, FRAME_100);
	if (ir_front->Read() == 2)
	{
		Serial.println("ir_front->Read() was successful");
		tone(R_BUZZER_PIN, 3500, 50);
	}
	Serial.println("Front ir sensor setup finished");

	/*
		Interrupts *MUST* be attached after the VL53L5CX_manager is instantiated,
		otherwise if the data_ready array isn't initialized and an interrupt is
		fired, the program will crash.
	*/
	// attachInterrupt(VL53L5CX_int_pin[VL53L5CX::FW], R_VL53L5CX_int_0, FALLING); // sensor_0
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::BW], R_VL53L5CX_int_1, FALLING); // sensor_1
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::SX], R_VL53L5CX_int_2, FALLING); // sensor_2
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::DX], R_VL53L5CX_int_3, FALLING); // sensor_3
	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous);					 // 8*8, 12Hz

	Wire1.begin();				// Color sensor
	Wire1.setClock(400000); // 400kHz

	Serial.println("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		Serial.println("Initialized color sensor!");
		tone(R_BUZZER_PIN, 3500, 50);
	}
	else
	{
		Serial.println("Failed to initialize color sensor!");
	}

	// Crea il grafo vuoto
	map = new graph();

	FakeDelay(100);
	tone(R_BUZZER_PIN, 4000, 69);

	imu->ResetZ();
	// Get first old_gyro value for check drift
	old_gyro_value = imu->z;
}

void Robot::Run()
{
	if (!StopRobot()) // Robot in azione
	{
		/*
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
			Serial.println("Rampa");
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
				// Serial.println(imu->y);

				SetCurrentTileDistances();
				// TODO: DA TOGLIERE
				if (going_down_ramp)
				{
					going_down_ramp = false;
					UpdateSensorNumBlocking(VL53L5CX::BW);
					back_distance_to_reach = DISTANCE_BACK_TO_CENTER_TILE + (GetBackDistance() - DISTANCE_BACK_TO_CENTER_TILE);
				}
			}
		}

		// Black tile
		if (BlackTile() && NotInRamp())
		{
			Serial.println("Black tile");
			// Torno in dietro fino a quando smetto di vedere nero
			ms->SetPower(-45, -45);
			while (cs->c_comp <= MIN_VALUE_BLUE_TILE)
			{
				// if (color_data_ready)
				{
					cs->getData();
					// color_data_ready = false;
				}
			}
			SetCurrentTileDistances();
			while (NewTile())
			{
				UpdateFrontBlocking();
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			ms->SetPower(SPEED, SPEED);
			SetCurrentTileDistances();
			while (!NewTile())
			{
				UpdateFrontBlocking();
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
		*/

		// Se ho colpito un muretto con gli switch
		if (digitalReadFast(R_COLLISION_SX_PIN) && NotInRamp())
		{
			ms->SetPower(-30, -100);
			FakeDelay(400);
			ms->StopMotors();
			UpdateGyroBlocking();
			while (imu->z <= desired_angle)
			{
				UpdateGyroBlocking();
				ms->SetPower(-TURN_SPEED, TURN_SPEED);
			}
			ms->StopMotors();
		}
		else if (digitalReadFast(R_COLLISION_DX_PIN) && NotInRamp())
		{
			ms->SetPower(-100, -30);

			FakeDelay(400);

			ms->StopMotors();
			UpdateGyroBlocking();
			while (imu->z >= desired_angle)
			{
				UpdateGyroBlocking();
				ms->SetPower(TURN_SPEED, -TURN_SPEED);
			}
			ms->StopMotors();
		}

		// Controllo se ho raggiunto una nuova tile
		if ((NewTile() && NotInRamp()) || FrontWall())
		{
			digitalWriteFast(R_LED1_PIN, HIGH);
			digitalWriteFast(R_LED2_PIN, HIGH);
			digitalWriteFast(R_LED3_PIN, HIGH);
			digitalWriteFast(R_LED4_PIN, HIGH);

			// Blue tile check
			bool blue_tile = false;
			if (BlueTile())
			{
				ms->StopMotors();
				blue_tile = true;
				Serial.println("Tile blue");
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

			if (!path_to_tile.empty())
			{
				ChangeMapPosition();
				if (current_x == path_to_tile.at(0).x)
				{
					if (current_y > path_to_tile.at(0).y)
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
					if (current_x > path_to_tile.at(0).x)
					{
						GoToDirection(3);
					}
					else
					{
						GoToDirection(1);
					}
				}
				path_to_tile.erase(path_to_tile.begin());
				map->PrintMazePath(path_to_tile);
			}
			else
			{
				// Invio il segnale alle openMV
				bool need_to_stop = false;
				if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM)
				{
					OPENMV_DX.print('9');
					need_to_stop = true;
				}
				if (GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM)
				{
					OPENMV_SX.print('9');
					need_to_stop = true;
				}

				// Aggiungo angolo tra le due tile
				int16_t previous_tile_y = current_y;
				int16_t previous_tile_x = current_x;
				int16_t previous_tile_z = current_z;
				if (!first_tile)
				{
					Serial.println("Cambiato posizione");
					ChangeMapPosition();
				}
				else
				{
					Serial.println("Prima tile");
					first_tile = false;
				}

				// Aggiungo il vertice corrente
				map->AddVertex(Tile{current_y, current_x, current_z});
				if (!blue_tile)
				{
					Serial.println("Arco aggiunto");
					map->AddEdge(Tile{previous_tile_y, previous_tile_x, previous_tile_z}, Tile{current_y, current_x, current_z}, 1);
				}
				else
				{
					Serial.println("Arco blue aggiunto");
					map->AddEdge(Tile{previous_tile_y, previous_tile_x, previous_tile_z}, Tile{current_y, current_x, current_z}, 5);
				}

				// Mi fermo nella tile solo se la tile non è mai stata visitata e sono presenti muri
				bool already_visited = map->GetNode(Tile{current_y, current_x, current_z}) != -1;
				if (!already_visited && need_to_stop)
				{
					ms->StopMotors();
					FakeDelay(250);
				}
				UpdateSensorNumBlocking(VL53L5CX::SX);
				UpdateSensorNumBlocking(VL53L5CX::DX);
				UpdateFrontBlocking();
				UpdateSensorNumBlocking(VL53L5CX::BW);

				int16_t next_tile = 0;
				bool right_already_visited = false;
				bool left_already_visited = false;
				bool front_already_visited = false;
				switch (direction)
				{
				case 0:
					next_tile = current_x + 1;
					right_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanTurnRight())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					next_tile = current_x - 1;
					left_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanTurnLeft())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					next_tile = current_y + 1;
					front_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanGoOn())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					break;
				case 1:
					next_tile = current_y - 1;
					right_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanTurnRight())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					next_tile = current_y + 1;
					left_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanTurnLeft())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					next_tile = current_x + 1;
					front_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanGoOn())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					break;
				case 2:
					next_tile = current_x - 1;
					right_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanTurnRight())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					next_tile = current_x + 1;
					left_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanTurnLeft())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					next_tile = current_y - 1;
					front_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanGoOn())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					break;
				case 3:
					next_tile = current_y + 1;
					right_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanTurnRight())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					next_tile = current_y - 1;
					left_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1;
					if (CanTurnLeft())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
					next_tile = current_x - 1;
					front_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1;
					if (CanGoOn())
						map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
					break;
				default:
					break;
				}

				bool right_blocked = !CanTurnRight() || right_already_visited;
				bool left_blocked = !CanTurnLeft() || left_already_visited;
				bool front_blocked = !CanGoOn() || front_already_visited;

				Serial.print("Right blocked: ");
				Serial.print(right_blocked);
				Serial.print("\\tleft blocked: ");
				Serial.print(left_blocked);
				Serial.print("\\tfront blocked: ");
				Serial.println(front_blocked);

				if (right_blocked && left_blocked && front_blocked)
				{
					int len;
					if (tile_to_visit.empty())
					{
						map->FindPathAStar(Tile{current_y, current_x, current_z}, Tile{0, 0, 0}, path_to_tile, len, direction);
						// Da fare alla fine quando la tile da raggiungere era la 0,0,0 e non ci sono elementi nella lista tile_to_visit
						/*
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
						*/
					}
					else
					{
						Serial.println("bababooeyyyyy");
						map->FindPathAStar(Tile{current_y, current_x, current_z}, tile_to_visit.front(), path_to_tile, len, direction);
						tile_to_visit.pop_back();
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
								switch (direction)
								{
								case 0:
									next_tile = current_x - 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_y + 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 1:
									next_tile = current_y + 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_x + 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 2:
									next_tile = current_x + 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_y - 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 3:
									next_tile = current_y - 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_x - 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								default:
									break;
								}
								// Giro a destra
								TurnRight();
								ms->StopMotors();
								if (!already_visited)
								{
									AfterTurnVictimDetection();
								}
							}
							else
							{
								switch (direction)
								{
								case 0:
									next_tile = current_x + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_x - 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 1:
									next_tile = current_y - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_y + 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 2:
									next_tile = current_x - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_x + 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 3:
									next_tile = current_y + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_y - 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								default:
									break;
								}
							}
						}
						else
						{
							// Giro a destra
							switch (direction)
							{
							case 0:
								next_tile = current_x - 1;
								if (!left_blocked)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								next_tile = current_y + 1;
								if (!front_blocked)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								break;
							case 1:
								next_tile = current_y + 1;
								if (!left_blocked)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								next_tile = current_x + 1;
								if (!front_blocked)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								break;
							case 2:
								next_tile = current_x + 1;
								if (!left_blocked)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								next_tile = current_y - 1;
								if (!front_blocked)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								break;
							case 3:
								next_tile = current_y - 1;
								if (!left_blocked)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								next_tile = current_x - 1;
								if (!front_blocked)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								break;
							default:
								break;
							}
							TurnRight();
							ms->StopMotors();
							if (!already_visited)
							{
								AfterTurnVictimDetection();
							}
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
								switch (direction)
								{
								case 0:
									next_tile = current_x + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_y + 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 1:
									next_tile = current_y - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_x + 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 2:
									next_tile = current_x - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_y - 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 3:
									next_tile = current_y + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_x - 1;
									if (!front_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								default:
									break;
								}
								TurnLeft();
								ms->StopMotors();
								if (!already_visited)
								{
									AfterTurnVictimDetection();
								}
							}
							else
							{
								switch (direction)
								{
								case 0:
									next_tile = current_x + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_x - 1;
									if (!left_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 1:
									next_tile = current_y - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_y + 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 2:
									next_tile = current_x - 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									next_tile = current_x + 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 3:
									next_tile = current_y + 1;
									if (!right_blocked)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									next_tile = current_y - 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								default:
									break;
								}
							}
						}
						else
						{
							// Giro a sinistra
							switch (direction)
							{
							case 0:
								next_tile = current_x + 1;
								if (!right_already_visited)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								next_tile = current_y + 1;
								if (!front_already_visited)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								break;
							case 1:
								next_tile = current_y - 1;
								if (!right_already_visited)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								next_tile = current_x + 1;
								if (!front_already_visited)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								break;
							case 2:
								next_tile = current_x - 1;
								if (!right_already_visited)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								next_tile = current_y - 1;
								if (!front_already_visited)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								break;
							case 3:
								next_tile = current_y + 1;
								if (!right_already_visited)
								{
									tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
								}
								next_tile = current_x - 1;
								if (!front_already_visited)
								{
									tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
								}
								break;
							default:
								break;
							}
							TurnLeft();
							ms->StopMotors();
							if (!already_visited)
							{
								AfterTurnVictimDetection();
							}
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
								switch (direction)
								{
								case 0:
									next_tile = current_y + 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 1:
									next_tile = current_x + 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 2:
									next_tile = current_y - 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 3:
									next_tile = current_x - 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								default:
									break;
								}
								TurnRight();
								ms->StopMotors();
								if (!already_visited)
								{
									AfterTurnVictimDetection();
								}
							}
							else
							{
								switch (direction)
								{
								case 0:
									next_tile = current_x + 1;
									if (!right_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 1:
									next_tile = current_y - 1;
									if (!right_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 2:
									next_tile = current_x - 1;
									if (!right_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 3:
									next_tile = current_y + 1;
									if (!right_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								default:
									break;
								}
							}
						}
						else
						{
							// Giro a destra
							TurnRight();
							ms->StopMotors();
							if (!already_visited)
							{
								AfterTurnVictimDetection();
							}
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
								switch (direction)
								{
								case 0:
									next_tile = current_y + 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 1:
									next_tile = current_x + 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 2:
									next_tile = current_y - 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 3:
									next_tile = current_x - 1;
									if (!front_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								default:
									break;
								}
								TurnLeft();
								ms->StopMotors();
								if (!already_visited)
								{
									AfterTurnVictimDetection();
								}
							}
							else
							{
								switch (direction)
								{
								case 0:
									next_tile = current_x - 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 1:
									next_tile = current_y + 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								case 2:
									next_tile = current_x + 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
									}
									break;
								case 3:
									next_tile = current_y - 1;
									if (!left_already_visited)
									{
										tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
									}
									break;
								default:
									break;
								}
							}
						}
						else
						{
							// Giro a sinistra
							TurnLeft();
							ms->StopMotors();
							if (!already_visited)
							{
								AfterTurnVictimDetection();
							}
						}
					}
				}
				/*
							else if (!CanGoOn() && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
							{
								TurnBack();
							}
				*/
				if (!already_visited && need_to_stop)
				{
					FakeDelay(250);
				}
				map->PrintMaze({current_y, current_x, current_z});
			}

			/*
			// Victims detection
			while (FoundVictim())
			{
				int kits_number;
				bool left_victim;
				if (OPENMV_SX.available() > 0)
				{
					kits_number = int(OPENMV_SX.read() - '0');
					left_victim = true;
				}
				else
				{
					kits_number = int(OPENMV_DX.read() - '0');
					left_victim = false;
				}
				Serial.print("Teensy 4.1 ha ricevuto in seriale: ");
				Serial.println(kits_number);
				DropKit(kits_number, left_victim);
			}
			OPENMV_DX.print('7');
			OPENMV_SX.print('7');
			*/

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
			digitalToggleFast(R_LED4_PIN);
			old_gyro_value = imu->z;
		}
		if (!NotInRamp())
		{
			digitalWriteFast(R_LED3_PIN, HIGH);
		}
		else
		{
			digitalWriteFast(R_LED3_PIN, LOW);
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
			// Gestione stop robot
			first_time_pressed = true;
			stop_the_robot = !stop_the_robot;
			// OpenMV discard old data
			while (OPENMV_SX.available())
			{
				OPENMV_SX.read();
			}
			while (OPENMV_DX.available())
			{
				OPENMV_DX.read();
			}
			if (map->NumVertices() == 0)
			{
				// Reset gyro
				imu->ResetZ();
				desired_angle = 0;
				first_tile = true;
				last_checkpoint = Tile{current_y, current_x, current_z};
			}
			else
			{
				current_y = last_checkpoint.y;
				current_x = last_checkpoint.x;
				current_z = last_checkpoint.z;
				direction = 0;
			}
			// Set current front/back distance to reach
			SetCurrentTileDistances();
			// Spengo tutti i led per sicurezza
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
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

int32_t Robot::GetFrontDistance()
{
	return ir_front->tfDist * 10;
}

int16_t Robot::GetBackDistance()
{
	return lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL];
}

void Robot::SetNewTileDistances()
{
	UpdateFrontBlocking();
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance() / 300) - 1) * 320) + DISTANCE_FRONT_TO_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300) + 1) * 320) + DISTANCE_BACK_TO_CENTER_TILE;
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = ((((GetBackDistance() / 300) + 1) * 320) + DISTANCE_BACK_TO_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE));
	}
}

void Robot::SetCurrentTileDistances()
{
	UpdateFrontBlocking();
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance() / 300)) * 320) + DISTANCE_FRONT_TO_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300)) * 320) + DISTANCE_BACK_TO_CENTER_TILE;
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE));
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
	return GetFrontDistance() >= MIN_DISTANCE_TO_TURN_MM;
}

bool Robot::CanBumpBack()
{
	return GetBackDistance() <= MIN_DISTANCE_BUMP_BACK_WALL_MM && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5;
}

bool Robot::FrontWall()
{
	return GetFrontDistance() <= MIN_DISTANCE_FROM_FRONT_WALL_MM && ir_front->tfFlux != 0;
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
	return (GetFrontDistance() <= front_distance_to_reach && GetFrontDistance() != 0) || (GetBackDistance() >= back_distance_to_reach && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5);
}

bool Robot::FoundVictim()
{
	return OPENMV_SX.available() > 0 || OPENMV_DX.available() > 0;
}

void Robot::AfterTurnVictimDetection()
{
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM)
	{
		OPENMV_DX.print('9');
	}
	if (GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM)
	{
		OPENMV_SX.print('9');
	}
	FakeDelay(250);
	while (FoundVictim())
	{
		int kits_number;
		bool left_victim;
		if (OPENMV_SX.available() > 0)
		{
			kits_number = int(OPENMV_SX.read() - '0');
			left_victim = true;
		}
		else
		{
			kits_number = int(OPENMV_DX.read() - '0');
			left_victim = false;
		}
		Serial.print("Teensy 4.1 ha ricevuto in seriale: ");
		Serial.println(kits_number);
		DropKit(kits_number, left_victim);
	}
	OPENMV_DX.print('7');
	OPENMV_SX.print('7');
}

void Robot::DropKit(int8_t number_of_kits, bool left_victim)
{
	// TODO: Verificare se tenere o meno le due righe sotto
	OPENMV_DX.print('7');
	OPENMV_SX.print('7');
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
				UpdateFrontBlocking();
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			ms->StopMotors();
		}

		Turn(90 * side);
	}
	// TODO: Verificare se tenere o meno le due righe sotto
	UpdateFrontBlocking();
	UpdateSensorNumBlocking(VL53L5CX::BW);
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	// -------------------
}

void Robot::Straighten()
{
	consecutive_turns++;
	if (consecutive_turns < 5)
	{
		return;
	}
	consecutive_turns = 0;
	ms->SetPower(-100, -100);
	FakeDelay(1200);
	ms->StopMotors();
	imu->ResetZ();
	desired_angle = 0;
	SetCurrentTileDistances();
	ms->SetPower(SPEED, SPEED);
	while (!NewTile())
	{
		UpdateFrontBlocking();
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}
	ms->StopMotors();
}

bool Robot::NotInRamp()
{
	return (imu->y <= 5 && imu->y >= -5);
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
		Serial.print("Desired angle: ");
		Serial.print(desired_angle);
		Serial.print("\tGyro: ");
		Serial.print(imu->z);
		Serial.print("\tPID_output: ");
		Serial.print(PID_output);
		Serial.print(".\tElapsed_seconds: ");
		Serial.print(elapsed_seconds);
		Serial.print(".\tPID_output * elapsed_seconds: ");
	*/
	int16_t corr = PID_output * elapsed_seconds;

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
	if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM)
	{
		OPENMV_DX.print('9');
	}
	FakeDelay(1000);
	while (FoundVictim())
	{
		Serial.println("Cerco vittima in U");
		int kits_number;
		if (OPENMV_DX.available() > 0)
		{
			kits_number = int(OPENMV_DX.read() - '0');
			DropKit(kits_number, false);
		}
	}
	OPENMV_DX.print('7');

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
			/*
				Serial.print("Stiamo girando a sinistra");
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
		UpdateGyroNonBlocking();
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

	for (uint8_t i = 0; i < 4; i++)
	{
		if (lasers_data_ready[i])
		{
			lasers->sensors[i]->UpdateData();
			lasers_data_ready[i] = false;
			status |= 0b10000 << i;
		}
	}

	status |= ir_front->Read();

	// if (color_data_ready)
	{
		cs->getData();
	}
	return status;
}

void Robot::UpdateSensorNumBlocking(VL53L5CX num)
{
	uint32_t time_end = millis() + 500;
	while (millis() < time_end)
	{
		UpdateGyroNonBlocking();
		if (lasers_data_ready[num])
		{
			lasers->sensors[num]->UpdateData();
			lasers_data_ready[num] = false;
			return;
		}
	}

	ms->StopMotors();

	Serial.println("Disabling and then enabling sensors power supply...");
	pinMode(R_PIN_SENSORS_POWER_ENABLE, OUTPUT);
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, HIGH); // Disable power supply output to sensors
	delay(10);														 // Wait for sensors to shutdown - 10ms from UM2884 Sensor reset management (VL53L5CX)
	digitalWriteFast(R_PIN_SENSORS_POWER_ENABLE, LOW);	 // Enable power supply output to sensors
	delay(10);														 // Wait for sensors to wake up (especially sensor 0)
	Serial.println("...done!");

	Serial.println("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, true);
	Serial.println("Laser sensors setup finished");

	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous); // 8*8, 12Hz

	Serial.println("Initializing color sensor");
	cs = new Color();
	if (cs->begin(&Wire1))
	{
		Serial.println("Initialized color sensor!");
	}
	else
	{
		Serial.println("Failed to initialize color sensor!");
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
		if (*imu_data_ready)
		{
			imu->UpdateData();
			*imu_data_ready = false;
			break;
		}
	}
}

void Robot::UpdateGyroNonBlocking()
{
	if (*imu_data_ready)
	{
		imu->UpdateData();
		*imu_data_ready = false;
	}
}

void Robot::UpdateFrontBlocking()
{
	while (ir_front->Read() != 2)
	{
		FakeDelay(1);
	}
}

void Robot::PrintSensorData()
{
	json_doc[doc_helper.AddLineGraph("Gyro X", 1)] = imu->x;
	json_doc[doc_helper.AddLineGraph("Gyro Y", 1)] = imu->y;
	json_doc[doc_helper.AddLineGraph("Gyro Z", 1)] = imu->z;
	json_doc[doc_helper.AddLineGraph("Front distance (cm)", 2)] = ir_front->tfDist;
	json_doc[doc_helper.AddLineGraph("Front strength", 2)] = ir_front->tfFlux;
	// dist[VL53L5CX::FW] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX FW", 8, 8, 0, 1000));
	dist[VL53L5CX::BW] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX BW", 8, 8, 0, 1000));
	dist[VL53L5CX::SX] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX SX", 8, 8, 0, 1000));
	dist[VL53L5CX::DX] = json_doc.createNestedArray(doc_helper.AddHeatmap("VL53L5LX DX", 8, 8, 0, 1000));

	for (size_t i = 1; i < 4; i++)
	{
		for (size_t j = 0; j < 64; j++)
		{
			dist[i].add(lasers->sensors[i]->GetData()->distance_mm[j]);
		}
	}
	json_doc[doc_helper.AddLineGraph("Color: b_comp", 3)] = cs->b_comp;
	json_doc[doc_helper.AddLineGraph("Color: c_comp", 3)] = cs->c_comp;
	json_doc[doc_helper.AddLineGraph("Color: g_comp", 3)] = cs->g_comp;
	json_doc[doc_helper.AddLineGraph("Color: r_comp", 3)] = cs->r_comp;

	json_doc[doc_helper.AddPacketIndex()] = doc_helper.GetAndIncrementPacketIdx();

	doc_helper.ResetIdx();
	serializeJson(json_doc, Serial);
	json_doc.clear();
}

void Robot::TestPeripherals()
{ // TODO: send special char to OPENMV to signal ping
	Serial.println("----- Additional testing procedure -----");
	FakeDelay(300);
	tone(R_BUZZER_PIN, 5000, 100);
	FakeDelay(200);
	tone(R_BUZZER_PIN, 5000, 100);

	if (OPENMV_SX.available())
	{
		Serial.print("OpenMV_SX: ");
		while (OPENMV_SX.available())
		{
			Serial.print((char)OPENMV_SX.read());
			FakeDelay(1);
		}
		Serial.println();
	}

	if (OPENMV_DX.available())
	{
		Serial.print("OpenMV_DX: ");
		while (OPENMV_DX.available())
		{
			Serial.print((char)OPENMV_DX.read());
			FakeDelay(1);
		}
		Serial.println();
	}

	TestButton(R_COLLISION_DX_PIN, "Bumper DX");
	TestButton(R_COLLISION_SX_PIN, "Bumper SX");
	TestButton(R_SW_START_PIN, "Start Button");
	TestButton(R_SW_XTRA_PIN, "Extra Button");

	FakeDelay(300);
	tone(R_BUZZER_PIN, 5000, 100);
	FakeDelay(200);
	tone(R_BUZZER_PIN, 5000, 100);
	Serial.println("----- Additional testing procedure end -----");
}

void Robot::TestButton(uint8_t pin, const char *name)
{
	uint32_t past_time = millis();
	while (true)
	{
		if (past_time + 1000 < millis())
		{
			Serial.print("Waiting for \"");
			Serial.print(name);
			Serial.println("\" to be pressed");
			past_time = millis();
		}
		if (digitalReadFast(pin))
		{
			break;
		}
		FakeDelay(1);
	}
	Serial.print("Pressed ");
	Serial.println(name);
	tone(R_BUZZER_PIN, 3500, 100);
	FakeDelay(100);
}

Robot::~Robot()
{
	delete ms;
	delete lasers;
	delete imu;
}
