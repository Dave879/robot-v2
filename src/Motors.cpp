#include "Motors.h"

Motors::Motors(): l_motor_power_(0), r_motor_power_(0)
{
	pinMode(R_PIN_MOTOR_L_DIR, OUTPUT);
	pinMode(R_PIN_MOTOR_R_DIR, OUTPUT);
	pinMode(R_PIN_MOTOR_L_PWM, OUTPUT);
	pinMode(R_PIN_MOTOR_R_PWM, OUTPUT);

	digitalWrite(R_PIN_MOTOR_L_DIR, LOW);
	digitalWrite(R_PIN_MOTOR_R_DIR, LOW);

	analogWriteFrequency(R_PIN_MOTOR_L_PWM, 90000);
	analogWriteFrequency(R_PIN_MOTOR_R_PWM, 90000);
}

void Motors::SetPower(int8_t left, int8_t right)
{
	l_motor_power_ = clamp(left, -100, 100);
	r_motor_power_ = clamp(right, -100, 100);
	if (left > 0)
		digitalWrite(R_PIN_MOTOR_L_DIR, HIGH);
	else
		digitalWrite(R_PIN_MOTOR_L_DIR, LOW);
	if (right > 0)
		digitalWrite(R_PIN_MOTOR_R_DIR, HIGH);
	else
		digitalWrite(R_PIN_MOTOR_R_DIR, LOW);

	analogWrite(R_PIN_MOTOR_L_PWM, abs(map(left, -100, 100, -255, 255)));
	analogWrite(R_PIN_MOTOR_R_PWM, abs(map(right, -100, 100, -255, 255)));
}

void Motors::SetPowerLeft(int8_t power)
{
	l_motor_power_ = clamp(power, -100, 100);
	if (power > 0)
		digitalWrite(R_PIN_MOTOR_L_DIR, HIGH);
	else
		digitalWrite(R_PIN_MOTOR_L_DIR, LOW);

	analogWrite(R_PIN_MOTOR_L_PWM, abs(map(power, -100, 100, -255, 255)));
}

void Motors::SetPowerRight(int8_t power)
{
	r_motor_power_ = clamp(power, -100, 100);
	if (power > 0)
		digitalWrite(R_PIN_MOTOR_R_DIR, HIGH);
	else
		digitalWrite(R_PIN_MOTOR_R_DIR, LOW);

	analogWrite(R_PIN_MOTOR_R_PWM, abs(map(power, -100, 100, -255, 255)));
}

Motors::~Motors()
{
}
