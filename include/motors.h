#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "pins.h"

class motors
{
private:
	int8_t l_motor_power;
	int8_t r_motor_power;
	bool l_motor_dir;
	bool r_motor_dir;
public:
	motors();
	void setPower(int8_t left, int8_t right);
	void stopMotors();
	~motors();
};
