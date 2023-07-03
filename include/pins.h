
#ifndef __PINS_H__
#define __PINS_H__

#include <stdint.h>

#define R_PIN_MOTOR_L_PWM 23 // M2
#define R_PIN_MOTOR_R_PWM 22 // M1

#define R_PIN_MOTOR_L_DIR 21 // M2
#define R_PIN_MOTOR_R_DIR 20 // M1

#define R_PIN_SENSORS_POWER_ENABLE 33

#define R_PIN_SERVO 2

#define R_LED1_PIN 38
#define R_LED2_PIN 39
#define R_LED3_PIN 34
#define R_LED4_PIN 35

#define R_SW_START_PIN 31
#define R_SW_XTRA_PIN 32

#define R_COLLISION_SX_PIN 36
#define R_COLLISION_DX_PIN 37

#define R_IMU_CS_PIN 10
#define R_IMU_EXT_CLK_SPI_PIN 9
#define R_IMU_INT_SPI_PIN 41

#define R_SHARP_VOUT 40

#define R_BUZZER_PIN 3

#define OPENMV_SX Serial2
#define OPENMV_DX Serial1

const uint8_t VL53L5CX_LPn_pin[4] = {
	29,	// Forward
	5,	// Backward
	6,		// Left
	27		// Right
};

const uint8_t VL53L5CX_int_pin[4] = {
	30,	// Forward
	4,	// Backward
	26,		// Left
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