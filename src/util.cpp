
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

