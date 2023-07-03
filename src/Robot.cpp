#include "Robot.h"

Robot::Robot(bool cold_start)
{
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

	Serial.println("Motor setup started");
	ms = new Motors();
	Serial.println("Finished motor setup!");

	Wire2.begin();				 // Lasers
	Wire2.setClock(1000000); // 1MHz

	Serial.println("Laser sensors setup started");
	lasers = new VL53L5CX_manager(Wire2, cold_start);
	Serial.println("Laser sensors setup finished");

	/*
		Interrupts *MUST* be attached after the VL53L5CX_manager is instantiated,
		otherwise if the data_ready array isn't initialized and an interrupt is
		fired, the program will crash.
	*/
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::FW], R_VL53L5CX_int_0, FALLING); // sensor_0
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::BW], R_VL53L5CX_int_1, FALLING); // sensor_1
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::SX], R_VL53L5CX_int_2, FALLING); // sensor_2
	attachInterrupt(VL53L5CX_int_pin[VL53L5CX::DX], R_VL53L5CX_int_3, FALLING); // sensor_3
	lasers->StartRanging(64, 15, ELIA::RangingMode::kContinuous);					 // 8*8, 20Hz

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

	Serial.println("Gyro setup started");
	imu = new gyro(SPI, R_IMU_CS_PIN, R_IMU_EXT_CLK_SPI_PIN);
	attachInterrupt(R_IMU_INT_SPI_PIN, R_ICM_42688_P_int, RISING);
	Serial.println("Finished gyro setup!");
	tone(R_BUZZER_PIN, 3500, 50);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//												FROM NOW ON FakeDelay() can and should be used
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Crea il grafo vuoto
	map = new graph();

	FakeDelay(100);
	tone(R_BUZZER_PIN, 4000, 69);

	imu->ResetZ();
	// Get first old_gyro value for check drift
	old_gyro_value = imu->z;

	while (OPENMV_SX.available())
	{
		OPENMV_SX.read();
	}
	while (OPENMV_DX.available())
	{
		OPENMV_DX.read();
	}
}

void Robot::Run()
{
	if (!StopRobot()) // Robot in azione
	{
		if (!NotInRamp())
		{
			if (!was_in_ramp && (imu->y < -15 || imu->y > 15))
			{
				digitalWriteFast(R_LED3_PIN, HIGH);
				was_in_ramp = true;
				time_in_ramp = millis();
				bumper_stair_while_going_to_tile = true;
			}
			if (imu->y < -15)
			{
				going_down_ramp = true;
			}
		}
		else if (was_in_ramp)
		{
			digitalWriteFast(R_LED3_PIN, LOW);
			was_in_ramp = false;
			if (millis() - time_in_ramp > 3000)
			{
				Serial.println("Rampa");
				// ms->SetPower(45, 45);
				while (imu->y > 0.5 || imu->y < -0.5)
				{
					UpdateGyroBlocking();
				}

				/*
				// Aggiungo angolo tra le due tile
				int16_t previous_tile_y = current_y;
				int16_t previous_tile_x = current_x;
				int16_t previous_tile_z = current_z;
				ChangeMapPosition();
				// Se la tile corrente è in TileToVisit() la tolgo
				RemoveTileToVisit(Tile{current_y, current_x, current_z});

				// Aggiungo il vertice corrente
				map->AddVertex(Tile{current_y, current_x, current_z});
				if (!going_down_ramp)
				{
					map->AddEdge(Tile{previous_tile_y, previous_tile_x, previous_tile_z}, Tile{current_y, current_x, current_z-1}, 1);
				}
				else
				{
					map->AddEdge(Tile{previous_tile_y, previous_tile_x, previous_tile_z}, Tile{current_y, current_x, current_z+1}, 1);
				}
				*/

/*
				previous_tile_y = current_y;
				previous_tile_x = current_x;
				previous_tile_z = current_z;
				ChangeMapPosition();
				// Aggiungo il vertice corrente
				map->AddVertex(Tile{current_y, current_x, current_z});
				map->AddEdge(Tile{previous_tile_y, previous_tile_x, previous_tile_z}, Tile{current_y, current_x, current_z}, 1);
*/

				SetCurrentTileDistances();

				going_down_ramp = false;
			}
		}
	
		// Black tile
		if (BlackTile() && NotInRamp())
		{
			Serial.println("Black tile");
			// Torno in dietro fino a quando smetto di vedere nero
			ms->SetPower(-45, -45);
			while (cs->c_comp <= MIN_VALUE_BLACK_TILE)
			{
				// if (color_data_ready)
				{
					cs->getData();
					// color_data_ready = false;
				}
			}
			ms->StopMotors();
			SetCurrentTileDistances();
			ms->SetPower(-45, -45);
			while (NewTile())
			{
				UpdateAllDistanceSensorsBlocking();
			}
			ms->StopMotors();
			SetCurrentTileDistances();
			ms->SetPower(SPEED, SPEED);
			while (!NewTile())
			{
				UpdateSensorNumBlocking(VL53L5CX::FW);
				UpdateSensorNumBlocking(VL53L5CX::BW);
			}
			ms->StopMotors();

			ChangeMapPosition();

			map->RemoveTileAdjacencyList(Tile{current_y, current_x, current_z});
			RemoveTileToVisit(Tile{current_y, current_x, current_z});

			IncreaseDirection();
			IncreaseDirection();
			ChangeMapPosition();
			ChangeMapPosition();
			IncreaseDirection();
			IncreaseDirection();
			path_to_tile.clear();
			SetCurrentTileDistances();
			cs->getData();
		}

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
		if (NewTile() && NotInRamp())
		{
			Serial.print("Direzione: ");
			Serial.println(direction);
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
				// Lampeggio per indicare la tile blue
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
				RemoveTileToVisit(Tile{current_y, current_x, current_z});
				ChangeMapPosition();
				if (InTileToVisit(Tile{current_y, current_x, current_z}))
				{
					FindVictim();
				}
				RemoveTileToVisit(Tile{current_y, current_x, current_z});
				if (!blue_tile && analogRead(R_SHARP_VOUT) < 300)
				{
					last_checkpoint = Tile{current_y, current_x, current_z};
				}
				map->PrintMazePath(path_to_tile);
				map->PrintMaze({current_y, current_x, current_z});
				path_to_tile.erase(path_to_tile.begin());
				if (!path_to_tile.empty())
				{
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
				}
				if (path_to_tile.empty())
				{
					bool left_already_visited = false;
					bool front_already_visited = false;
					bool right_already_visited = false;
					AddEdges(blue_tile, left_already_visited, front_already_visited, right_already_visited);

					bool right_blocked = !CanTurnRight() || right_already_visited;
					bool left_blocked = !CanTurnLeft() || left_already_visited;
					bool front_blocked = !CanGoOn() || front_already_visited;

					DecideTurn(left_blocked, front_blocked, right_blocked, false, blue_tile);
				}
				else
				{
					SetNewTileDistances();
				}
			}
			else
			{
				// Cambio coordinate con quelle della tile corrente
				// Se la tile corrente è InTileToVisit() la tolgo
				Serial.print("(Prima di rimuovere)Tutte le tile da vedere: ");
				for (size_t i = 0; i < tile_to_visit.size(); i++)
				{
					Serial.println(tile_to_visit.at(i));
				}
				Serial.print("Vecchia tile rimossa: ");
				Serial.println(Tile{current_y, current_x, current_z});
				RemoveTileToVisit(Tile{current_y, current_x, current_z});
				ChangeMapPosition();

				// Mi fermo nella tile solo se la tile non è mai stata visitata e sono presenti muri
				bool current_tile_already_visited = map->GetNode(Tile{current_y, current_x, current_z}) != -1 && !(InTileToVisit(Tile{current_y, current_x, current_z}));

				// Aggiungo il vertice corrente
				map->AddVertex(Tile{current_y, current_x, current_z});

				if (!blue_tile && analogRead(R_SHARP_VOUT) < 300)
				{
					last_checkpoint = Tile{current_y, current_x, current_z};
				}

				bool left_already_visited = false;
				bool front_already_visited = false;
				bool right_already_visited = false;
				AddEdges(blue_tile, left_already_visited, front_already_visited, right_already_visited);

				// Se la tile corrente è in TileToVisit() la tolgo
				Serial.print("Tile da rimuovere: ");
				Serial.println(Tile{current_y, current_x, current_z});
				RemoveTileToVisit(Tile{current_y, current_x, current_z});

				Serial.print("(Dopo la rimozione)Tutte le tile da vedere: ");
				for (size_t i = 0; i < tile_to_visit.size(); i++)
				{
					Serial.println(tile_to_visit.at(i));
				}

				if (!current_tile_already_visited && !blue_tile)
				{
					FindVictim();
				}

				// Aggiorno i sensori di distanza
				UpdateAllDistanceSensorsBlocking();

				bool left_blocked = !CanTurnLeft() || left_already_visited;
				bool front_blocked = !CanGoOn() || front_already_visited;
				bool right_blocked = !CanTurnRight() || right_already_visited;

				Serial.print("Right blocked: ");
				Serial.print(right_blocked);
				Serial.print("\\tleft blocked: ");
				Serial.print(left_blocked);
				Serial.print("\\tfront blocked: ");
				Serial.println(front_blocked);

				DecideTurn(left_blocked, front_blocked, right_blocked, current_tile_already_visited, blue_tile);

				map->PrintMaze({current_y, current_x, current_z});
			}

			bumper_stair_while_going_to_tile = false;
			digitalWriteFast(R_LED1_PIN, LOW);
			digitalWriteFast(R_LED2_PIN, LOW);
			digitalWriteFast(R_LED3_PIN, LOW);
			digitalWriteFast(R_LED4_PIN, LOW);
		}
		// Proseguo diretto
		else
		{
			int16_t power_to_add = imu->y / 3;
			if (power_to_add < -10)
			{
				power_to_add = -10;
			}
			if (imu->y > 15)
			{
				power_to_add = 10;
			}
			if (imu->z > desired_angle)
			{
				ms->SetPower(SPEED + power_to_add + 5, SPEED + power_to_add - 5);
			}
			else if (imu->z < desired_angle)
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

void Robot::R_ICM_42688_P_int()
{
	imu_data_ready = true;
}

bool Robot::StopRobot()
{
	if (digitalReadFast(R_SW_START_PIN))
	{
		if (!first_time_pressed)
		{
			FakeDelay(500);
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
				last_checkpoint = Tile{current_y, current_x, current_z};
				FirstTileProcedure();
			}
			else
			{
				imu->ResetZ();
				direction = 0;
				current_y = last_checkpoint.y;
				current_x = last_checkpoint.x;
				current_z = last_checkpoint.z;
				LackOfProgressProcedure();
			}
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

void Robot::FirstTileProcedure()
{
	// Set current front/back distance to reach
	SetCurrentTileDistances();
	ms->SetPower(SPEED, SPEED);
	while (!NewTile())
	{
		UpdateSensorNumBlocking(VL53L5CX::FW);
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}

	map->AddVertex({current_y, current_x, current_z});
	UpdateAllDistanceSensorsBlocking();
	bool left_already_visited = false;
	bool front_already_visited = false;
	bool right_already_visited = false;
	AddEdges(false, left_already_visited, front_already_visited, right_already_visited);

	bool right_blocked = !CanTurnRight() || right_already_visited;
	bool left_blocked = !CanTurnLeft() || left_already_visited;
	bool front_blocked = !CanGoOn() || front_already_visited;
	/*
		Serial.print("Right blocked: ");
		Serial.print(right_blocked);
		Serial.print("\\tleft blocked: ");
		Serial.print(left_blocked);
		Serial.print("\\tfront blocked: ");
		Serial.println(front_blocked);
	*/
	// Ultimo parametro a true pk non ha senso fermarmi a controllare vittime nella tile di partenza
	DecideTurn(left_blocked, front_blocked, right_blocked, true, false);
}

void Robot::LackOfProgressProcedure()
{
	// Set current front/back distance to reach
	path_to_tile.clear();
	SetCurrentTileDistances();
	ms->SetPower(SPEED, SPEED);
	while (!NewTile())
	{
		UpdateSensorNumBlocking(VL53L5CX::FW);
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}

	UpdateAllDistanceSensorsBlocking();
	bool left_already_visited = false;
	bool front_already_visited = false;
	bool right_already_visited = false;
	AddEdges(false, left_already_visited, front_already_visited, right_already_visited);

	DecideTurn(true, true, true, true, false);
}

void Robot::GetAroundTileVisited(bool &left_already_visited, bool &front_already_visited, bool &right_already_visited)
{
	int32_t next_tile = 0;
	switch (direction)
	{
	case 0:
		// Destra
		next_tile = current_x + 1;
		right_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanTurnRight())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		// Sinistra
		next_tile = current_x - 1;
		left_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanTurnLeft())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		// Frontale
		next_tile = current_y + 1;
		front_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanGoOn())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		break;
	case 1:
		// Destra
		next_tile = current_y - 1;
		right_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanTurnRight())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		// Sinistra
		next_tile = current_y + 1;
		left_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanTurnLeft())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		// Frontale
		next_tile = current_x + 1;
		front_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanGoOn())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		break;
	case 2:
		// Destra
		next_tile = current_x - 1;
		right_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanTurnRight())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		// Sinistra
		next_tile = current_x + 1;
		left_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanTurnLeft())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		// Frontale
		next_tile = current_y - 1;
		front_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanGoOn())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		break;
	case 3:
		// Destra
		next_tile = current_y + 1;
		right_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanTurnRight())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		// Sinistra
		next_tile = current_y - 1;
		left_already_visited = map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && !InTileToVisit(Tile{next_tile, current_x, current_z});
		if (CanTurnLeft())
		{
			map->AddVertex(Tile{next_tile, current_x, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, 1);
		}
		// Frontale
		next_tile = current_x - 1;
		front_already_visited = map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && !InTileToVisit(Tile{current_y, next_tile, current_z});
		if (CanGoOn())
		{
			map->AddVertex(Tile{current_y, next_tile, current_z});
			map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, 1);
		}
		break;
	default:
		break;
	}
}

void Robot::DecideTurn(bool left_blocked, bool front_blocked, bool right_blocked, bool tile_already_visited, bool blue_tile)
{
	if (right_blocked && left_blocked && front_blocked)
	{
		// Se è tutto bloccato calcola il percorso migliore

		// Se sono in una U controllo se ci sono vittime
		if (!CanGoOn() && (!CanTurnLeft() || BlackTileLeft()) && (!CanTurnRight() || BlackTileRight()))
		{
			// Fermo il robot prima di girare
			ms->StopMotors();

			// Giro totale (-180°)
			Turn(-90);
			AfterTurnVictimDetection();
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
		Serial.print("Numero Tile da vedere: ");
		Serial.println(tile_to_visit.size());
		int len;
		bool path_calculated = false;
		if (tile_to_visit.empty())
		{
			if (!(Tile{current_y, current_x, current_z} == Tile{0, 0, 0}))
			{
				map->FindPathAStar(Tile{current_y, current_x, current_z}, Tile{0, 0, 0}, path_to_tile, len, direction);
				path_calculated = true;
			}
			else
			{
				// Da fare alla fine quando la tile da raggiungere era la 0,0,0 e non ci sono elementi nella lista tile_to_visit
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
		}
		else
		{
			Serial.print("Tile da vedere: ");
			Serial.println(tile_to_visit.at(tile_to_visit.size() - 1));
			Serial.print("Tutte le tile da vedere: ");
			for (size_t i = 0; i < tile_to_visit.size(); i++)
			{
				Serial.println(tile_to_visit.at(i));
			}
			if ((tile_to_visit.at(tile_to_visit.size() - 1)) == Tile{0,0,0})
			{
				tile_to_visit.pop_back();
			}
			map->FindPathAStar(Tile{current_y, current_x, current_z}, tile_to_visit.at(tile_to_visit.size() - 1), path_to_tile, len, direction);
			path_calculated = true;
			// tile_to_visit.pop_back();
		}
		// Lo faccio per la prima tile
		if (path_calculated)
		{
			Serial.print("Tutte le tile da seguire: ");
			for (size_t i = 0; i < path_to_tile.size(); i++)
			{
				Serial.println(path_to_tile.at(i));
			}
			map->PrintMazePath(path_to_tile);
			path_to_tile.erase(path_to_tile.begin());
			int8_t old_dir = direction;
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
			if (old_dir!=direction && old_dir!=(direction + 2) && (old_dir!=direction - 2) && !tile_already_visited)
			{
				AfterTurnVictimDetection();
			}
			ms->StopMotors();
			SetNewTileDistances();
			return;
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
					if (!tile_already_visited && !blue_tile)
					{
						AfterTurnVictimDetection();
					}
				}
				// Proseguo dritto
			}
			else
			{
				// Giro a destra
				TurnRight();
				ms->StopMotors();
				if (!tile_already_visited  && !blue_tile)
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
					TurnLeft();
					ms->StopMotors();
					if (!tile_already_visited  && !blue_tile)
					{
						AfterTurnVictimDetection();
					}
				}
				// Proseguo dritto
			}
			else
			{
				// Giro a sinistra
				TurnLeft();
				ms->StopMotors();
				if (!tile_already_visited && !blue_tile)
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
					TurnRight();
					ms->StopMotors();
					if (!tile_already_visited && !blue_tile)
					{
						AfterTurnVictimDetection();
					}
				}
				// Proseguo dritto
			}
			else
			{
				// Giro a destra
				TurnRight();
				ms->StopMotors();
				if (!tile_already_visited && !blue_tile)
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
					TurnLeft();
					ms->StopMotors();
					if (!tile_already_visited && !blue_tile)
					{
						AfterTurnVictimDetection();
					}
				}
				// Proseguo dritto
			}
			else
			{
				// Giro a sinistra
				TurnLeft();
				ms->StopMotors();
				if (!tile_already_visited && !blue_tile)
				{
					AfterTurnVictimDetection();
				}
			}
		}
	}
	SetNewTileDistances();
}

void Robot::AddEdges(bool blue_tile, bool &left_already_visited, bool &front_already_visited, bool &right_already_visited)
{
	int32_t next_tile = 0;
	uint16_t weight = (blue_tile) ? 5 : 1;
	UpdateAllDistanceSensorsBlocking();
	switch (direction)
	{
	case 0:
		// Destra
		next_tile = current_x + 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				right_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Sinistra
		next_tile = current_x - 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				left_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Frontale
		next_tile = current_y + 1;
		if (CanGoOn())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				front_already_visited = true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Posteriore
		if (blue_tile)
		{
			next_tile = current_y - 1;
			map->ChangeTileWeight(Tile{next_tile, current_x, current_z}, Tile{current_y, current_x, current_z}, weight);
		}
		break;
	case 1:
		// Destra
		next_tile = current_y - 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				right_already_visited = true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Sinistra
		next_tile = current_y + 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				left_already_visited = true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Frontale
		next_tile = current_x + 1;
		if (CanGoOn())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				front_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Posteriore
		if (blue_tile)
		{
			next_tile = current_x - 1;
			map->ChangeTileWeight(Tile{current_y, next_tile, current_z}, Tile{current_y, current_x, current_z}, weight);
		}
		break;
	case 2:
		// Destra
		next_tile = current_x - 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				right_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Sinistra
		next_tile = current_x + 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				left_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Frontale
		next_tile = current_y - 1;
		if (CanGoOn())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				front_already_visited =true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Posteriore
		if (blue_tile)
		{
			next_tile = current_y + 1;
			map->ChangeTileWeight(Tile{next_tile, current_x, current_z}, Tile{current_y, current_x, current_z}, weight);
		}
		break;
	case 3:
		// Destra
		next_tile = current_y + 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				right_already_visited = true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Sinistra
		next_tile = current_y - 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && (!InTileToVisit(Tile{next_tile, current_x, current_z}) || map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty()))
			{
				left_already_visited = true;
			}
			else if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1 && !InTileToVisit(Tile{next_tile, current_x, current_z}))
			{
				tile_to_visit.push_back(Tile{next_tile, current_x, current_z});
			}
			if (map->GetNode(Tile{next_tile, current_x, current_z}) == -1)
			{
				map->AddVertex(Tile{next_tile, current_x, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{next_tile, current_x, current_z}, weight);
			}
		}
		// Frontale
		next_tile = current_x - 1;
		if (CanGoOn())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && (!InTileToVisit(Tile{current_y, next_tile, current_z}) || map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty()))
			{
				front_already_visited = true;
			}
			else if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1 && !InTileToVisit(Tile{current_y, next_tile, current_z}))
			{
				tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
			}
			if (map->GetNode(Tile{current_y, next_tile, current_z}) == -1)
			{
				map->AddVertex(Tile{current_y, next_tile, current_z});
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
			else if (!map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				map->AddEdge(Tile{current_y, current_x, current_z}, Tile{current_y, next_tile, current_z}, weight);
			}
		}
		// Posteriore
		if (blue_tile)
		{
			next_tile = current_x + 1;
			map->ChangeTileWeight(Tile{current_y, next_tile, current_z}, Tile{current_y, current_x, current_z}, weight);
		}
		break;
	default:
		break;
	}
}

bool Robot::InTileToVisit(Tile t)
{
	for (size_t i = 0; i < tile_to_visit.size(); i++)
	{
		if (tile_to_visit.at(i) == t)
		{
			return true;
		}
	}
	return false;
}

void Robot::RemoveTileToVisit(Tile t)
{
	for (size_t i = 0; i < tile_to_visit.size(); i++)
	{
		if (tile_to_visit.at(i) == t)
		{
			tile_to_visit.erase(tile_to_visit.begin() + i);
			return;
		}
	}
}

void Robot::UpdateAllDistanceSensorsBlocking()
{
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
}

int16_t Robot::GetRightDistance()
{
	return lasers->sensors[VL53L5CX::DX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL_RIGHT];
}

int16_t Robot::GetLeftDistance()
{
	return lasers->sensors[VL53L5CX::SX]->GetData()->distance_mm[DISTANCE_SENSOR_CELL_LEFT];
}

int32_t Robot::GetFrontDistance()
{
	return lasers->sensors[VL53L5CX::FW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL_FRONT];
}

int16_t Robot::GetBackDistance()
{
	return lasers->sensors[VL53L5CX::BW]->GetData()->distance_mm[DISTANCE_SENSOR_CELL_BACK];
}

void Robot::SetNewTileDistances()
{
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance() / 300) - 1) * 320) + DISTANCE_FRONT_TO_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300) + 1) * 320) + DISTANCE_BACK_TO_CENTER_TILE;
	/*
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = ((((GetBackDistance() / 300) + 1) * 320) + DISTANCE_BACK_TO_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE));
	}
	*/
}

void Robot::SetCurrentTileDistances()
{
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	front_distance_to_reach = (((GetFrontDistance() / 300)) * 320) + DISTANCE_FRONT_TO_CENTER_TILE;
	back_distance_to_reach = (((GetBackDistance() / 300)) * 320) + DISTANCE_BACK_TO_CENTER_TILE;
	/*
	if ((GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE)) > 250 && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL] == 5)
	{
		back_distance_to_reach = (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE) + (GetBackDistance() - (((GetBackDistance() / 300) * 320) + DISTANCE_BACK_TO_CENTER_TILE));
	}
	*/
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
	Serial.print("Direzione dove andare: ");
	Serial.println(direction_to_go);
	int8_t delta_dir = direction - direction_to_go;
	Serial.print("Delta Direzione: ");
	Serial.println(delta_dir);
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
	return GetRightDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::DX]->GetData()->target_status[DISTANCE_SENSOR_CELL_RIGHT] != 5;
}

bool Robot::CanTurnLeft()
{
	return GetLeftDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::SX]->GetData()->target_status[DISTANCE_SENSOR_CELL_LEFT] != 5;
}

bool Robot::CanGoOn()
{
	return GetFrontDistance() >= MIN_DISTANCE_TO_TURN_MM || lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL_FRONT] != 5;
}

bool Robot::CanBumpBack()
{
	return GetBackDistance() <= MIN_DISTANCE_BUMP_BACK_WALL_MM && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL_BACK] == 5;
}

bool Robot::FrontWall()
{
	return GetFrontDistance() <= MIN_DISTANCE_FROM_FRONT_WALL_MM && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[20] == 5;
}

bool Robot::BlackTileRight()
{
	int32_t next_tile = 0;
	UpdateAllDistanceSensorsBlocking();
	switch (direction)
	{
	case 0:
		// Destra
		next_tile = current_x + 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 1:
		// Destra
		next_tile = current_y - 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 2:
		// Destra
		next_tile = current_x - 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 3:
		// Destra
		next_tile = current_y + 1;
		if (CanTurnRight())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 &&  map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	default:
		return false;
		break;
	}
}

bool Robot::BlackTileLeft()
{
	int32_t next_tile = 0;
	UpdateAllDistanceSensorsBlocking();
	switch (direction)
	{
	case 0:
		// Sinistra
		next_tile = current_x - 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 1:
		// Sinistra
		next_tile = current_y + 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 && map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 2:
		// Sinistra
		next_tile = current_x + 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{current_y, next_tile, current_z}) != -1 && map->GetAdjacencyList(Tile{current_y, next_tile, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	case 3:
		// Sinistra
		next_tile = current_y - 1;
		if (CanTurnLeft())
		{
			if (map->GetNode(Tile{next_tile, current_x, current_z}) != -1 &&  map->GetAdjacencyList(Tile{next_tile, current_x, current_z}).empty())
			{
				return true;
			}
			return false;
		}
		return false;
		break;
	default:
		return false;
		break;
	}
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
	if (GetFrontDistance() <= front_distance_to_reach && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL_FRONT] == 5)
	{
		Serial.print("Frontale: ");
		Serial.println(GetFrontDistance());
		Serial.print("Frontale da raggiungere: ");
		Serial.println(front_distance_to_reach);
	}
	if (GetBackDistance() >= back_distance_to_reach && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL_BACK] == 5)
	{
		Serial.print("Posteriore: ");
		Serial.println(GetBackDistance());
		Serial.print("Posteriore da raggiungere: ");
		Serial.println(back_distance_to_reach);
	}
	UpdateGyroBlocking();
	if ((((GetFrontDistance() <= front_distance_to_reach && lasers->sensors[VL53L5CX::FW]->GetData()->target_status[DISTANCE_SENSOR_CELL_FRONT] == 5) || (GetBackDistance() >= back_distance_to_reach && lasers->sensors[VL53L5CX::BW]->GetData()->target_status[DISTANCE_SENSOR_CELL_BACK] == 5))))
	{

		if (((CalculateError(imu->z) < 5) || (CalculateError(imu->z) > -5)))
		{
			return true;
		}
		Turn(0);
		return false;
	}
	return false;
}

void Robot::FindVictim()
{
	bool signal_sent = false;
	UpdateAllDistanceSensorsBlocking();
	// bool centred = false;
	if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM || GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM)
	{
		CenterTile();

		if (GetRightDistance() < MIN_DISTANCE_TO_TURN_MM)
		{
			/*
			while(GetRightDistance() < MIN_DISTANCE_TO_WIGGLE_WIGGLE)
			{
				Turn(-90);
				CenterTile();
				Turn(90);
			}
			centred = true;
			*/
			Serial.println("controllo vittima destrAAAAA!!!!");
			OPENMV_DX.print('9');
			signal_sent = true;
		}
		
		if (GetLeftDistance() < MIN_DISTANCE_TO_TURN_MM)
		{
			/*
			while(GetLeftDistance() < MIN_DISTANCE_TO_WIGGLE_WIGGLE && !centred)
			{
				Turn(90);
				CenterTile();
				Turn(-90);
			}
			*/
			Serial.println("controllo vittima sinistrAAAAA!!!!");
			OPENMV_SX.print('9');
			signal_sent = true;
		}
	}
	
	if (!signal_sent)
	{
		return;
	}

	Serial.println("controllo vittima!!!!");
	
	ms->StopMotors();
	FakeDelay(500);

	while (VictimFound())
	{
		int kits_number;
		bool left_victim;
		if (OPENMV_SX.available() > 0)
		{
			kits_number = OPENMV_SX.read() - '0';
			left_victim = true;
		}
		else
		{
			kits_number = OPENMV_DX.read() - '0';
			left_victim = false;
		}
		if (kits_number <= 3 && kits_number >= 0)
		{
			DropKit(kits_number, left_victim);
		}
		else if (kits_number == 9)
		{
			if (left_victim)
			{
				OPENMV_SX.print('8');
			}
			else
			{
				OPENMV_DX.print('8');
			}
			FakeDelay(500);
		}
	}
}

bool Robot::VictimFound()
{
	return OPENMV_SX.available() > 0 || OPENMV_DX.available() > 0;
}

void Robot::AfterTurnVictimDetection()
{
	FindVictim();
}

void Robot::DropKit(int8_t number_of_kits, bool left_victim)
{
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
		consecutive_turns++;
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
		consecutive_turns++;
	}
	digitalWriteFast(R_LED1_PIN, HIGH);
	digitalWriteFast(R_LED2_PIN, HIGH);
	digitalWriteFast(R_LED3_PIN, HIGH);
	digitalWriteFast(R_LED4_PIN, HIGH);
	FakeDelay(250);
	tone(R_BUZZER_PIN, 5000, 100);
	digitalWriteFast(R_LED1_PIN, LOW);
	digitalWriteFast(R_LED2_PIN, LOW);
	digitalWriteFast(R_LED3_PIN, LOW);
	digitalWriteFast(R_LED4_PIN, LOW);
	// TODO: Verificare se tenere o meno le due righe sotto
	UpdateSensorNumBlocking(VL53L5CX::FW);
	UpdateSensorNumBlocking(VL53L5CX::BW);
	UpdateSensorNumBlocking(VL53L5CX::SX);
	UpdateSensorNumBlocking(VL53L5CX::DX);
	// -------------------
}

void Robot::Straighten()
{
	consecutive_turns++;
	if (consecutive_turns < 2)
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
		UpdateSensorNumBlocking(VL53L5CX::FW);
		UpdateSensorNumBlocking(VL53L5CX::BW);
	}
	ms->StopMotors();
}

void Robot::CenterTile()
{
	ms->StopMotors();
	UpdateGyroBlocking();
	if (NotInRamp() && !bumper_stair_while_going_to_tile)
	{
		Turn(0);
		ms->StopMotors();
		
		SetCurrentTileDistances();
		ms->SetPower(-45, -45);
		while (NewTile())
		{
			UpdateAllDistanceSensorsBlocking();
		}
		ms->StopMotors();
		SetCurrentTileDistances();
		ms->SetPower(SPEED, SPEED);
		while (!NewTile())
		{
			UpdateSensorNumBlocking(VL53L5CX::FW);
			UpdateSensorNumBlocking(VL53L5CX::BW);
		}
		ms->StopMotors();
		UpdateAllDistanceSensorsBlocking();
	}
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
	Turn(-180);

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
			// int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			//ms->SetPower(-gyro_speed, +gyro_speed);
			ms->SetPower(-90, 90);
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
			// int16_t gyro_speed = GetPIDOutputAndSec();
			// Potenza gestita da PID e Gyro-z
			// ms->SetPower(-gyro_speed, +gyro_speed);
			ms->SetPower(90, -90);
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

	if (imu_data_ready)
	{
		imu->UpdateData();
		imu_data_ready = false;
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

	lasers->StartRanging(64, 12, ELIA::RangingMode::kContinuous); // 8*8, 12Hz

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
			imu_data_ready = false;
			break;
		}
	}
}

void Robot::UpdateGyroNonBlocking()
{
	if (imu_data_ready)
	{
		imu->UpdateData();
		imu_data_ready = false;
	}
}

void Robot::PrintSensorData()
{
	json_doc[doc_helper.AddLineGraph("Gyro X", 1)] = imu->x;
	json_doc[doc_helper.AddLineGraph("Gyro Y", 1)] = imu->y;
	json_doc[doc_helper.AddLineGraph("Gyro Z", 1)] = imu->z;

	json_doc[doc_helper.AddLineGraph("Sharp sensor")] = analogRead(R_SHARP_VOUT);

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
