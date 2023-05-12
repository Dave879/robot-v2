
#include "Sensors/Color.h"

//
// Gain/time combinations to use and the min/max limits for hysteresis
// that avoid saturation. They should be in order from dim to bright.
//
// Also set the first min count and the last max count to 0 to indicate
// the start and end of the list.
//
const Color::tcs_agc Color::agc_lst[] = {
    {TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_614MS, 0, 20000},
    {TCS34725_GAIN_60X, TCS34725_INTEGRATIONTIME_154MS, 4990, 63000},
    {TCS34725_GAIN_16X, TCS34725_INTEGRATIONTIME_154MS, 16790, 63000},
    {TCS34725_GAIN_4X, TCS34725_INTEGRATIONTIME_154MS, 15740, 63000},
    {TCS34725_GAIN_1X, TCS34725_INTEGRATIONTIME_154MS, 15740, 0}};
Color::Color() : agc_cur(0), isAvailable(0), isSaturated(0), last_reading_time(0)
{
}

// initialize the sensor
boolean Color::begin(TwoWire *wireInterface)
{
  tcs = Adafruit_TCS34725(agc_lst[agc_cur].at, agc_lst[agc_cur].ag);
  if ((isAvailable = tcs.begin(0x29, wireInterface)))
  {
    tcs.setGain(tcs34725Gain_t::TCS34725_GAIN_1X);
    tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS);
    tcs.setInterrupt(false);
    // setGainTime();
  }
  return (isAvailable);
}

// Set the gain and integration time
void Color::setGainTime()
{
  tcs.setGain(agc_lst[agc_cur].ag);
  tcs.setIntegrationTime(agc_lst[agc_cur].at);
  atime = int(agc_lst[agc_cur].at);
  atime_ms = ((256 - atime) * 2.4);
  switch (agc_lst[agc_cur].ag)
  {
  case TCS34725_GAIN_1X:
    againx = 1;
    break;
  case TCS34725_GAIN_4X:
    againx = 4;
    break;
  case TCS34725_GAIN_16X:
    againx = 16;
    break;
  case TCS34725_GAIN_60X:
    againx = 60;
    break;
  }
}

// Retrieve data from the sensor and do the calculations
void Color::getData()
{
  if (millis() > last_reading_time + 7)
  {
    last_reading_time = millis();
    // read the sensor and autorange if necessary
    tcs.getRawData(&r, &g, &b, &c);
    //tcs.clearInterrupt();

    r_comp = (float)r / c * 255.0;
    g_comp = (float)g / c * 255.0;
    b_comp = (float)b / c * 255.0;
    c_comp = c;

    // DN40 calculations
    // ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;
    /*
      r_comp = r - ir;
      g_comp = g - ir;
      b_comp = b - ir;
      c_comp = c - ir;

    */

    /*
      cratio = float(ir) / float(c);

      saturation = ((256 - atime) > 63) ? 65535 : 1024 * (256 - atime);
      saturation75 = (atime_ms < 150) ? (saturation - saturation / 4) : saturation;
      isSaturated = (atime_ms < 150 && c > saturation75) ? 1 : 0;
      cpl = (atime_ms * againx) / (TCS34725_GA * TCS34725_DF);
      maxlux = 65535 / (cpl * 3);

      lux = (TCS34725_R_Coef * float(r_comp) + TCS34725_G_Coef * float(g_comp) + TCS34725_B_Coef * float(b_comp)) / cpl;
      ct = TCS34725_CT_Coef * float(b_comp) / float(r_comp) + TCS34725_CT_Offset;
    */
  }

}

void Color::ClearInterrupt()
{
  tcs.clearInterrupt();
}