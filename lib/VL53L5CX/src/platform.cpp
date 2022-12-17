/*******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
 * either 'STMicroelectronics Proprietary license'
 * or 'BSD 3-clause "New" or "Revised" License' , at your option.
 *
 ********************************************************************************
 *
 * 'STMicroelectronics Proprietary license'
 *
 ********************************************************************************
 *
 * License terms: STMicroelectronics Proprietary in accordance with licensing
 * terms at www.st.com/sla0081
 *
 * STMicroelectronics confidential
 * Reproduction and Communication of this document is strictly prohibited unless
 * specifically authorized in writing by STMicroelectronics.
 *
 *
 ********************************************************************************
 *
 * Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
 * terms of 'BSD 3-clause "New" or "Revised" License', in which case the
 * following provisions apply instead of the ones mentioned above :
 *
 ********************************************************************************
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************/

#include "platform.h"

#define I2C_BUFFER_LEN 128
#define TIMEOUT_LOOPS 10

/*
	i2c Error structure:
	0000 0000
	RDER WRER
	RDER = Read error
	WRER = Write error

	status WRER:
	0: success
	1: data too long to fit in transmit buffer
	2: received NACK on transmit of address
	3: received NACK on transmit of data
	4: other error
	5: timeout
	6: write() returned 0

	status RDER:

	0: success
	1: data too long to fit in transmit buffer
	2: received NACK on transmit of address
	3: received NACK on transmit of data
	4: other error
	5: timeout
	8: read timeuot

	The errors can be signaled together:
	1001 (High byte -> RDER) -> read timeout + data too long to fit in trasmit buffer
*/

enum I2C_ERROR
{
	NO_ERROR,
	DATA_TOO_LONG_FOR_TRANSMIT_BUFFER,
	NACK_ON_TRANSMIT_OF_ADDRESS,
	NACK_ON_TRANSMIT_OF_DATA,
	OTHER_ERROR,
	TIMEOUT,
	WRITE_RETURNED_0,
	FAILED_TO_READ_EVERY_BYTE,
	READ_TIMEOUT
};

#define GET_VALID_I2C_ADDRESS(x) uint8_t(x >> 1 & 0x7F)

uint8_t
RdByte(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAddress,
	uint8_t *p_value)
{
	/* Alternative implementation
		uint8_t status;
		p_platform->i2c_bus->beginTransmission(GET_VALID_I2C_ADDRESS(p_platform->address));
		p_platform->i2c_bus->write(highByte(RegisterAddress));
		p_platform->i2c_bus->write(lowByte(RegisterAddress));
		status = p_platform->i2c_bus->endTransmission();
		p_platform->i2c_bus->requestFrom(GET_VALID_I2C_ADDRESS(p_platform->address), 1U);
		*p_value = p_platform->i2c_bus->read();
		return status;
	*/
	return RdMulti(p_platform, RegisterAddress, p_value, 1);
}

uint8_t WrByte(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAddress,
	uint8_t value)
{
	return WrMulti(p_platform, RegisterAddress, &value, 1);
}

uint8_t WrMulti(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAddress,
	uint8_t *p_values,
	uint32_t size)
{
	uint32_t i = 0;
	uint8_t buffer[2];

	while (i < size)
	{
		// If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
		// else the remaining number of bytes
		uint8_t current_write_size = (size - i > I2C_BUFFER_LEN ? I2C_BUFFER_LEN : size - i);

		p_platform->i2c_bus->beginTransmission(GET_VALID_I2C_ADDRESS(p_platform->address));

		// Target register address for transfer
		buffer[0] = highByte(RegisterAddress + i);
		buffer[1] = lowByte(RegisterAddress + i);
		p_platform->i2c_bus->write(buffer, 2);
		if (p_platform->i2c_bus->write(p_values + i, current_write_size) == 0)
		{
			return I2C_ERROR::WRITE_RETURNED_0;
		}
		else
		{
			i += current_write_size;
			if (size - i)
			{
				// Flush buffer but do not send stop bit so we can keep going
				p_platform->i2c_bus->endTransmission(false);
			}
		}
	}

	return p_platform->i2c_bus->endTransmission(true);
}

uint8_t RdMulti(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAddress,
	uint8_t *p_values,
	uint32_t size)
{
	int status = I2C_ERROR::NO_ERROR;
	uint8_t buffer[2];

	// Loop until the port is transmitted correctly or timeout is over
	uint8_t timeout = 0;
	do
	{
		p_platform->i2c_bus->beginTransmission(GET_VALID_I2C_ADDRESS(p_platform->address));
		// Target register address for transfer
		buffer[0] = highByte(RegisterAddress);
		buffer[1] = lowByte(RegisterAddress);
		p_platform->i2c_bus->write(buffer, 2);
		status = p_platform->i2c_bus->endTransmission(false);

		timeout++;
	} while (status != 0 && timeout < TIMEOUT_LOOPS);
	if (status != 0 && timeout == TIMEOUT_LOOPS) // Failed to successfully transmit
	{
		status |= I2C_ERROR::READ_TIMEOUT;
		return status << 4;
	}

	uint32_t i = 0;
	if (size > I2C_BUFFER_LEN)
	{
		while (i < size)
		{
			// If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
			// else the remaining number of bytes
			uint8_t current_read_size = (size - i > I2C_BUFFER_LEN ? I2C_BUFFER_LEN : size - i);
			p_platform->i2c_bus->requestFrom(GET_VALID_I2C_ADDRESS(p_platform->address), current_read_size);
			while (p_platform->i2c_bus->available())
			{
				p_values[i] = p_platform->i2c_bus->read();
				i++;
			}
		}
	}
	else
	{
		p_platform->i2c_bus->requestFrom(GET_VALID_I2C_ADDRESS(p_platform->address), size);
		while (p_platform->i2c_bus->available() > 0)
		{
			p_values[i] = p_platform->i2c_bus->read();
			i++;
		}
	}

	return i == size ? I2C_ERROR::NO_ERROR : I2C_ERROR::FAILED_TO_READ_EVERY_BYTE << 4;
}

uint8_t Reset_Sensor(
	VL53L5CX_Platform *p_platform)
{
	uint8_t status = 255;

	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);

	return status;
}

void SwapBuffer(
	uint8_t *buffer,
	uint16_t size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for (i = 0; i < size; i = i + 4)
	{
		tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(
	VL53L5CX_Platform *p_platform,
	uint32_t TimeMs)
{
	delay(TimeMs);
	return 0;
}
