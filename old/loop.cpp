	//  gyro = mpu->GetGyroData();
	//  laser0_data = l0->GetData();
	//  l1->SetROICenter(center[Zone]);
	//  LaserDataArray.add(l1->GetData());
	/*
		if (Zone >= PX_COUNT - 1)
		{
			// doc[DATANAME_LASER_1] = LaserDataArray;
			serializeJson(doc, res);
			doc.clear();
			// LaserDataArray = doc.createNestedArray("laser");
			Serial.println(res);
			res = "";
		}
		Zone++;
		Zone = Zone % PX_COUNT;
	*/
	/*
	if (digitalRead(33))
	{
		l1 = new Laser(&Wire2);
		delay(1000);
	}
	if (digitalRead(33))
	{
		res += PWM_1;
		res += PWM_2;
		res += dir;
		PWM_1 += 10;
		PWM_2 += 10;
		res += "\n";
		Serial5.print(res);
		res = "";
		if (dir == '0')
			dir = '1';
		else if (dir == '1')
			dir = '2';
		else if (dir == '2')
			dir = '3';
		else
			dir = '0';
		if (PWM_1 >= 127)
			PWM_1 = ' ';
		if (PWM_2 >= 127)
			PWM_2 = ' ';
		delay(500);
	}
	*/

	/*
	START_TIMER
	rgb_sensor.getData();
	END_TIMER

	printColorSensor();
	*/
	/*
	#if DEBUG == true
		doc[DATANAME_GYRO_X] = gyro.x;
		doc[DATANAME_GYRO_Y] = gyro.y;
		doc[DATANAME_GYRO_Z] = gyro.z;
	*/
	// doc[DATANAME_LASER_0] = laser0_data;

	// #endif