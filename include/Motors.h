#pragma once
#include <stdint.h>	// int8_t
#include <Arduino.h> // PinMode ecc...
#include "pins.h"		// Motor pins
#include "util.h"		// clamp

/**
 * \brief A class to handle the movement of the robot
 * \author Davide Rorato
 * \version 1.0
 * \date 16/12/2022
*/
class Motors
{
private:
	int8_t l_motor_power_;
	int8_t r_motor_power_;

public:
	/**
	 * Initializes motors, configures pins and PWM frequency
	 */
	Motors();
	/**
	 * Sets the power to the left and right motor banks
	 * \param left A value from -100 to 100, respectively to go backward and forward with the left motor bank
	 * \param right A value from -100 to 100, respectively to go backward and forward with the right motor bank
	 */
	void SetPower(int8_t left, int8_t right);
	/**
	 * Sets the power to the left motor bank
	 * \param left A value from -100 to 100, respectively to go backward and forward with the left motor bank
	 */
	void SetPowerLeft(int8_t power);
	/**
	 * Sets the power to the right motor bank
	 * \param power A value from -100 to 100, respectively to go backward and forward with the right motor bank
	 */
	void SetPowerRight(int8_t power);
	/**
	 * Stops both motors
	 */
	void StopMotors();
	~Motors();
};
