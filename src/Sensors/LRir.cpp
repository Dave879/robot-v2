#include "Sensors/LRir.h"

/*
	Long Range ir
*/

LRir::LRir(HardwareSerial &srl, uint32_t framerate)
{
	srl.begin(115200); // Initialize TFMPLus device serial port.
	delay(20);			 // Give port time to initalize
	tfmP.begin(&srl);	 // Initialize device library object and...
							 // pass device serial port to the object.

	// Send some example commands to the TFMini-Plus
	// - - Perform a system reset - - - - - - - - - - -
	Serial.print("Soft reset: ");
	if (tfmP.sendCommand(SOFT_RESET, 0))
	{
		Serial.println("passed.");
	}
	else
		tfmP.printReply();

	delay(500); // added to allow the System Rest enough time to complete

	// - - Display the firmware version - - - - - - - - -
	Serial.print("Firmware version: ");
	if (tfmP.sendCommand(GET_FIRMWARE_VERSION, 0))
	{
		Serial.print(tfmP.version[0]); // print three single numbers
		Serial.print(".");				 // print three single numbers
		Serial.print(tfmP.version[1]); // each separated by a dot
		Serial.print(".");				 // print three single numbers
		Serial.println(tfmP.version[2]);
	}
	else
		tfmP.printReply();
	// - - Set the data frame-rate to 20Hz - - - - - - - -
	Serial.print("Data-Frame rate: ");
	if (tfmP.sendCommand(SET_FRAME_RATE, framerate))
	{
		Serial.print(framerate);
		Serial.println("Hz");
		delta_measurement_time_ms = 1000 / framerate;
	}
	else
		tfmP.printReply();
}

uint8_t LRir::Read()
{
	if (millis() > past_reading_millis + delta_measurement_time_ms)
	{
		if (tfmP.getData(tfDist, tfFlux, tfTemp)) // Get data from the device.
		{
			return 2;
		}
		else // If the command fails...
		{
			tfmP.printFrame(); // display the error and HEX data
			return 0;
		}
	}
	return 4;
}

LRir::~LRir()
{
}
