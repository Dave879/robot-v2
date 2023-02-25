
#ifndef __PINS_H__
#define __PINS_H__

#include <stdint.h>

#define R_PIN_MOTOR_L_PWM 23 // M2
#define R_PIN_MOTOR_R_PWM 22 // M1

#define R_PIN_MOTOR_L_DIR 21 // M2
#define R_PIN_MOTOR_R_DIR 20 // M1

#define R_PIN_SENSORS_POWER_ENABLE 33
#define R_PIN_BUTTON 27
#define R_PIN_GYRO_INT 15

#define R_PIN_SERVO 0
#define R_PIN_COLOR_INT 14

const uint8_t VL53L5CX_LPn_pin[4] = {
	31,	// Forward
	2,		// Backward
	11,	// Left
	29		// Right
};

const uint8_t VL53L5CX_int_pin[4] = {
	30,	// Forward
	1,		// Backward
	12,	// Left
	28		// Right
};

enum VL53L5CX {
	FW,
	BW,
	SX,
	DX
};

// TODO: explain why close i2c adresses are bad, example: 0x52, 0x53....
// https://community.st.com/s/feed/0D53W00001GQQVaSAP
const uint8_t VL53L5CX_addr[4]{
	0x50U,
	0x60U,
	0x70U,
	0x80U,
};

#endif // __PINS_H__