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