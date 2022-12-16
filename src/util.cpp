
#include "config.h"
#include "util.h"

void SignalBuiltinLED(int count, int delayTime)
{
	for (int i = 0; i < count; i++)
	{
		digitalToggle(LED_BUILTIN);
		delay(delayTime);
	}
}

int clamp(int8_t value, int8_t min, int8_t max){
	if (value > max)
		return max;
	else if (value < min)
		return min;
	return value;
}