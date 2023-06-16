# Navigazione senza gyro

Funziona male, bisgona implementare il gyro

``` cpp
bool ignore_right = false;

ms->SetPower(40,40);

while (1)
{
    if (lasers_data_ready[3])
    {
        lasers->sensors[3]->UpdateData();
        lasers_data_ready[3] = false;
    }
    if (lasers_data_ready[0])
    {
        lasers->sensors[0]->UpdateData();
        lasers_data_ready[0] = false;
    }
    if (lasers_data_ready[2])
    {
        lasers->sensors[2]->UpdateData();
        lasers_data_ready[2] = false;
    }

    if(lasers->sensors[0]->GetData()->distance_mm[2] < 50) {
        ms->SetPower(0,0);
        if(lasers->sensors[2]->GetData()->distance_mm[2] > 200) {
            ms->SetPower(-60, 60);
            delay(1200);
            ms->SetPower(50,50);
        } else {
            ms->SetPower(-60, 60);
            delay(2400);
            ms->SetPower(50,50);
        }
    }

    if(lasers->sensors[3]->GetData()->distance_mm[2] > 200 && !ignore_right){
        delay(600);
        ignore_right = true;
        ms->SetPower(60, -60);
        delay(1200);
        ms->SetPower(50,50);
    } else if(lasers->sensors[3]->GetData()->distance_mm[2] < 150 && ignore_right){
        ignore_right = false;
    }
}
```

# Gyro

``` cpp
// Turn right:
desired_angle = mpu_data + 90;
if (desired_angle >= 180) {
    desired_angle -= 360;
}
// Turn left:
desired_angle = mpu_data -90;
if (desired_angle <= -180) {
    desired_angle += 360;
}

// L'angolo in rotazione deve essere maggiore dell'angolo voluto e dello stesso segno
while(desired_angle * mpu_data > 0 and mpu_data > desired_angle) {
    ms->SetPower(+K, -k);
}
```


# 90° destra, 180° sinistra

``` cpp
# Da mettere come prima cosa nel Run()
bool fermo = true;
bool test = false;
while (test) {
    if (fermo) {
        ms->SetPower(0,0);
    } else {
        desired_angle = mpu_data.x + 90;
        if (desired_angle >= 180)
        {
            desired_angle -= 360;
        }
        while (!(mpu_data.x > desired_angle && mpu_data.x * desired_angle > 0))
        {
            UpdateGyroBlocking();
            ms->SetPower(100, -100);
        }
        ms->SetPower(0,0);
        desired_angle = mpu_data.x - 180;
        if (desired_angle <= -180)
        {
            desired_angle += 360;
        }
        while (!(mpu_data.x < desired_angle && mpu_data.x * desired_angle > 0))
        {
            UpdateGyroBlocking();
            ms->SetPower(-100, 100);
        }
        ms->SetPower(0, 0);
    }
}
```

``` cpp
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
}
```

### Turn right

``` cpp
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
```

## Old method to make the robot go straight
``` cpp
// In assenza di muro frontale ci troviamo in una strada da prosseguire in modo rettilineo
else
{
    if (mpu_data.x < desired_angle)
    { // Se tende a sinistra, più potenza a sinistra
        ms->SetPower(SPEED + 20, SPEED);
    }
    else if (mpu_data.x > desired_angle)
    { // Se tende a destra, più potenza a destra
        ms->SetPower(SPEED, SPEED + 20);
    }
    else
    { // Se è dritto, infatibile, prosegue dritto
        ms->SetPower(SPEED, SPEED);
    }
}
```

## Set speed via serial

```cpp
int16_t receivedSpeed;
bool newData = false;
while (true)
{
    if (Serial.available() > 0) {
        receivedSpeed = Serial.parseInt();
        newData = true;
        ms->SetPower(receivedSpeed, receivedSpeed);
    }
    if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedSpeed);
    newData = false;
    }
}
```
## Test PID curva

```cpp
bool newData = false;
while (true)
{
    if (Serial.available() > 0) {
            desired_angle = Serial.parseInt();
            newData = true;
    }
    else
    {
        UpdateGyroBlocking();
        Serial.println("Gyro: ");
        Serial.println(imu->z);

        // Calculate error
        PID_error = CalculateError(imu->z);
        // Calculate integral
        PID_integral += PID_error;
        // Calculate derivative
        double derivative = PID_error - PID_previous_error;
        PID_previous_error = PID_error;
        // Calculate output
        PID_output = KP * PID_error + KI * PID_integral + KD * derivative;
        // Update motor powers and apply motor powers to left and right motors
        double elapsed_seconds = micros() - PID_start_time;

        // Update start time
        PID_start_time = micros();

        Serial.println("PID_output: ");
        Serial.println(PID_output);
        Serial.println("elapsed_seconds: ");
        Serial.println(elapsed_seconds);
        Serial.println("PID_output * elapsed_seconds: ");
        double corr = PID_output * elapsed_seconds;
        Serial.println(corr);

        ms->SetPower(-PID_output * elapsed_seconds, +PID_output * elapsed_seconds);
    }
    if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(desired_angle);
    newData = false;
    }
}
```

## New snippet

``` cpp

		while (true)
		{
			if (Serial.available() > 0)
			{
				desired_angle = Serial.parseInt();
			}
			else
			{
				UpdateGyroBlocking();
				Serial.println("Gyro: ");
				Serial.println(imu->z);

				int16_t gyro_speed = GetPIDOutputAndSec();
				// Potenza gestita da PID e Gyro-z
				ms->SetPower(-gyro_speed, +gyro_speed);
			}
		}
```


## Send data to OPenMV via serial

```cpp
char receivedChar;
bool newData = false;
while (true)
{
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        newData = true;
        Serial2.print(receivedChar);
    }
    if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChar);
    newData = false;
    }
}
```

``` cpp
switch (direction)
{
    case 0:
        next_tile = current_x + 1;
        if (!right_already_visited)
        {
            tile_to_visit.push_back(Tile{current_y, next_tile, current_z});
        }
        next_tile = current_x - 1;
        if (!left_already_visited)
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
        next_tile = current_y + 1;
        if (!left_already_visited)
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
        next_tile = current_x + 1;
        if (!left_already_visited)
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
        next_tile = current_y - 1;
        if (!left_already_visited)
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
```