#include "Motors.h"

//#define ROBOT_2

Motors::Motors(): l_motor_power_(0), r_motor_power_(0)
{
	pinMode(R_PIN_MOTOR_L_DIR, OUTPUT);
	pinMode(R_PIN_MOTOR_R_DIR, OUTPUT);
	pinMode(R_PIN_MOTOR_L_PWM, OUTPUT);
	pinMode(R_PIN_MOTOR_R_PWM, OUTPUT);

	digitalWriteFast(R_PIN_MOTOR_L_DIR, LOW);
	digitalWriteFast(R_PIN_MOTOR_R_DIR, LOW);

	analogWriteFrequency(R_PIN_MOTOR_L_PWM, 90000);
	analogWriteFrequency(R_PIN_MOTOR_R_PWM, 90000);
}

void Motors::SetPower(int32_t left, int32_t right)
{
	l_motor_power_ = clamp(left, -100, 100);
	r_motor_power_ = clamp(right, -100, 100);
	#ifdef ROBOT_2
		if (l_motor_power_ != 0)
		{
			(l_motor_power_ < 0) ? l_motor_power_ += 20 : l_motor_power_ -= 20;
		}
		if (r_motor_power_ != 0)
		{
			(r_motor_power_ < 0) ? r_motor_power_ += 20 : r_motor_power_ -= 20;
		}
	#endif
	if (left > 0)
		digitalWriteFast(R_PIN_MOTOR_L_DIR, HIGH);
	else
		digitalWriteFast(R_PIN_MOTOR_L_DIR, LOW);
	if (right > 0)
		digitalWriteFast(R_PIN_MOTOR_R_DIR, HIGH);
	else
		digitalWriteFast(R_PIN_MOTOR_R_DIR, LOW);

	analogWrite(R_PIN_MOTOR_L_PWM, abs(map(l_motor_power_, -100, 100, -255, 255)));
	analogWrite(R_PIN_MOTOR_R_PWM, abs(map(r_motor_power_, -100, 100, -255, 255)));
}

void Motors::SetPowerLeft(int8_t power)
{
	l_motor_power_ = clamp(power, -100, 100);
	if (power > 0)
		digitalWriteFast(R_PIN_MOTOR_L_DIR, HIGH);
	else
		digitalWriteFast(R_PIN_MOTOR_L_DIR, LOW);

	analogWrite(R_PIN_MOTOR_L_PWM, abs(map(l_motor_power_, -100, 100, -255, 255)));
}

void Motors::SetPowerRight(int8_t power)
{
	r_motor_power_ = clamp(power, -100, 100);
	if (power > 0)
		digitalWriteFast(R_PIN_MOTOR_R_DIR, HIGH);
	else
		digitalWriteFast(R_PIN_MOTOR_R_DIR, LOW);

	analogWrite(R_PIN_MOTOR_R_PWM, abs(map(r_motor_power_, -100, 100, -255, 255)));
}

void Motors::StopMotors(){
	SetPower(0, 0);
}

Motors::~Motors()
{
}
