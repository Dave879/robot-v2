
#include "config.h"
#include "util.h"

void SignalBuiltinLED(int count, int delayTime)
{
	bool blinkState = true;
	for (int i = 0; i < count; i++)
	{
		digitalWrite(LED_BUILTIN, blinkState);
		delay(delayTime);
		blinkState = !blinkState;
	}
}

void SetupDebugLEDs()
{
	pinMode(LED_0, OUTPUT);
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_0, LOW);
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, LOW);
	digitalWrite(LED_3, LOW);
}
