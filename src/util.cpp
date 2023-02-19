
#include "util.h"

void SignalBuiltinLED(int count, int delayTime)
{
	for (int i = 0; i < count; i++)
	{
		digitalToggle(LED_BUILTIN);
		delay(delayTime);
	}
}

int32_t clamp(int32_t value, int32_t min, int32_t max){
	if (value > max)
		return max;
	else if (value < min)
		return min;
	return value;
}