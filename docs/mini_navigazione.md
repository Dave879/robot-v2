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