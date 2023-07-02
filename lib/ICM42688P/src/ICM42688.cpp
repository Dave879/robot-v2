#include "ICM42688.h"

using namespace ICM42688reg;

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK)
{
  _spi = &bus;    // SPI bus
  _csPin = csPin; // chip select pin
  SPI_HS_CLOCK = SPI_HS_CLK;

  acc_earth.axis.x = 0.0f;
  acc_earth.axis.y = 0.0f;
  acc_earth.axis.z = 0.0f;

  acc_earth_prev.axis.x = 0.0f;
  acc_earth_prev.axis.y = 0.0f;
  acc_earth_prev.axis.z = 0.0f;
}

/* starts communication with the ICM42688 */
uint8_t ICM42688::begin()
{
  status = 0;
  // using SPI for communication
  // use low speed SPI for register setting
  _useSPIHS = false;
  // setting CS pin to output
  pinMode(_csPin, OUTPUT);
  // setting CS pin high
  digitalWrite(_csPin, HIGH);
  // begin SPI communication
  _spi->begin();

  // reset the ICM42688
  status |= reset();

  // check the WHO AM I byte
  if (whoAmI() != WHO_AM_I)
  {
    return 0x10;
  }

  // 16G is default -- do this to set up accel resolution scaling
  status |= setAccelFS(gpm16);

  // 2000DPS is default -- do this to set up gyro resolution scaling
  status |= setGyroFS(dps2000);

  return status;
}

uint8_t ICM42688::enableAccelGyroLN()
{
  if (gyro_accel_started)
    return 0;

  _useSPIHS = false;

  status = 0;
  // turn on accel and gyro in Low Noise (LN) Mode
  status |= writeRegister(UB0_REG_PWR_MGMT0, 0x0F);
  delay(45); // Gyroscope needs to be kept ON for a minimum of 45ms + From datasheet, do not issue any register writes for 200μs
  if (status == 0)
    gyro_accel_started = true;
  return status;
}

uint8_t ICM42688::disableAccelGyro()
{
  if (!gyro_accel_started)
    return 0;

  _useSPIHS = false;
  status = 0;
  status |= writeRegister(UB0_REG_PWR_MGMT0, 0x00);
  delayMicroseconds(200); // From datasheet, do not issue any register writes for 200μs
  if (status == 0)
    gyro_accel_started = false;
  return status;
}

uint8_t ICM42688::enableExternalClock()
{
  _useSPIHS = false;
  status = 0;
  status |= disableAccelGyro();
  status |= setBank(0);
  status |= writeRegister(UB0_REG_INTF_CONFIG1, 0x95);
  status |= setBank(1);
  status |= writeRegister(UB1_REG_INTF_CONFIG5, 0x04);
  return status;
}

bool ICM42688::dataIsReady() // TODO: Considering deleting or modifying this function
{
  uint8_t reg;
  _useSPIHS = true;
  setBank(0);
  readRegisters(UB0_REG_INT_STATUS, 1, &reg);
  if (reg >= 0x08)
  {
    return true;
  }
  return false;
}

/* sets the accelerometer full scale range to values other than default */
uint8_t ICM42688::setAccelFS(AccelFS fssel)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  status = 0;
  status |= setBank(0);

  uint8_t reg;
  // read current register value
  readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg);

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  status |= writeRegister(UB0_REG_ACCEL_CONFIG0, reg);

  _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
  _accelFS = fssel;
  return status;
}

/* sets the gyro full scale range to values other than default */
uint8_t ICM42688::setGyroFS(GyroFS fssel)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  status = 0;

  status |= setBank(0);

  // read current register value
  uint8_t reg;
  readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg);

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  status |= writeRegister(UB0_REG_GYRO_CONFIG0, reg);

  _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
  _gyroFS = fssel;

  return status;
}

uint8_t ICM42688::setAccelODR(ODR odr)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  status = 0;

  status |= setBank(0);

  // read current register value
  uint8_t reg;
  readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg);

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  status |= writeRegister(UB0_REG_ACCEL_CONFIG0, reg);

  return status;
}

uint8_t ICM42688::setGyroODR(ODR odr)
{
  // use low speed SPI for register setting
  _useSPIHS = false;
  status = 0;

  status |= setBank(0);

  // read current register value
  uint8_t reg;
  readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg);

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  status |= writeRegister(UB0_REG_GYRO_CONFIG0, reg);

  return status;
}

uint8_t ICM42688::configureNotchFilterBandwidth(Filter_BW f_bw)
{
  status = 0;
  status |= disableAccelGyro();
  status |= setBank(1);

  uint8_t v = 0x01 | f_bw << 4;
  status |= writeRegister(UB1_REG_GYRO_CONFIG_STATIC10, v);

  return status;
}

uint8_t ICM42688::configureUIFilter(AAF_3db_bw_hz accel_bw, AAF_3db_bw_hz gyro_bw)
{
  status = 0;
  status |= disableAccelGyro();
  status |= configureUIFilterAccel(accel_bw);
  status |= configureUIFilterGyro(gyro_bw);
  return status;
}

uint8_t ICM42688::configureUIFilterAccel(AAF_3db_bw_hz accel_bw)
{
  _useSPIHS = false;
  status = 0;
  status |= disableAccelGyro();

  status |= setBank(2);

  uint8_t accel_reg_val;
  readRegisters(UB2_REG_ACCEL_CONFIG_STATIC2, 1, &accel_reg_val);

  accel_reg_val &= 0x81; // Clear bits 1-6
  accel_reg_val |= accel_bw << 1;
  status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, accel_reg_val); // Write ACCEL_AAF_DELT

  status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC3, (uint8_t)aaf_reg_val[accel_bw].AAF_DELTSQR); // Write to ACCEL_AAF_DELTSQR

  accel_reg_val = aaf_reg_val[accel_bw].AAF_BITSHIFT << 4 | highByte(aaf_reg_val[accel_bw].AAF_DELTSQR); // Write higher nibble of ACCEL_AAF_DELTSQR and ACCEL_AAF_BITSHIFT
  status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC4, accel_reg_val);

  return status;
}

uint8_t ICM42688::configureUIFilterGyro(AAF_3db_bw_hz gyro_bw)
{
  _useSPIHS = false;
  status = 0;
  status |= disableAccelGyro();

  status |= setBank(1);

  uint8_t gyro_reg_val;
  readRegisters(UB1_REG_GYRO_CONFIG_STATIC3, 1, &gyro_reg_val);
  gyro_reg_val &= 0xC0; // Clear bits 0-5
  gyro_reg_val |= gyro_bw;
  status |= writeRegister(UB1_REG_GYRO_CONFIG_STATIC3, gyro_reg_val); // Write GYRO_AAF_DELT

  status |= writeRegister(UB1_REG_GYRO_CONFIG_STATIC4, (uint8_t)aaf_reg_val[gyro_bw].AAF_DELTSQR); // Write lower bits of GYRO_AAF_DELTSQR

  gyro_reg_val = aaf_reg_val[gyro_bw].AAF_BITSHIFT << 4 | highByte(aaf_reg_val[gyro_bw].AAF_DELTSQR);
  status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC4, gyro_reg_val); // Write GYRO_AAF_DELTSQR in the lower nibble and GYRO_AAF_BITSHIFT in the higher one

  return status;
}

uint8_t ICM42688::setFilters(bool gyroFilters, bool accFilters)
{
  _useSPIHS = false;
  status = 0;
  status |= disableAccelGyro();

  status |= setBank(1);

  if (gyroFilters == true)
  {
    status |= writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE);
  }
  else
  {
    status |= writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE);
  }

  status |= setBank(2);

  if (accFilters == true)
  {
    status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE);
  }
  else
  {
    status |= writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE);
  }
  return status;
}

uint8_t ICM42688::enableDataReadyInterrupt()
{
  // use low speed SPI for register setting
  status = 0;
  _useSPIHS = false;
  status |= disableAccelGyro();

  // push-pull, pulsed, active HIGH interrupts
  status |= writeRegister(UB0_REG_INT_CONFIG, 0x8 | 0x03);

  // need to clear bit 4 to allow proper INT1 and INT2 operation
  uint8_t reg;
  readRegisters(UB0_REG_INT_CONFIG1, 1, &reg);

  reg &= ~0x10;
  status |= writeRegister(UB0_REG_INT_CONFIG1, reg);

  // route UI data ready interrupt to INT1
  status |= writeRegister(UB0_REG_INT_SOURCE0, 0x18);

  return status;
}

uint8_t ICM42688::disableDataReadyInterrupt()
{
  // use low speed SPI for register setting
  status = 0;
  _useSPIHS = false;
  status |= disableAccelGyro();

  // set pin 4 to return to reset value
  uint8_t reg;
  readRegisters(UB0_REG_INT_CONFIG1, 1, &reg);

  reg |= 0x10;
  status |= writeRegister(UB0_REG_INT_CONFIG1, reg);

  // return reg to reset value
  status |= writeRegister(UB0_REG_INT_SOURCE0, 0x10);

  return status;
}

/* reads the most current data from ICM42688 and stores in buffer */
bool ICM42688::ReadRawMeasurements()
{
  _useSPIHS = false;
  uint8_t data_ready;
  readRegisters(UB0_REG_INT_STATUS, 1, &data_ready);
  data_ready = (data_ready & (1 << 3)) != 0;
  _useSPIHS = true; // use the high speed SPI for data readout

  if (data_ready)
  {
    readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer);

    // combine bytes into 16 bit values
    for (size_t i = 0; i < 7; i++)
    {
      _rawMeas[i] = ((int16_t)_buffer[i * 2] << 8) | _buffer[i * 2 + 1];
    }

    //_t = (static_cast<float>(_rawMeas[0]) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

    _acc[0] = ((_rawMeas[1] * _accelScale) - _accB[0]);
    _acc[1] = ((_rawMeas[2] * _accelScale) - _accB[1]);
    _acc[2] = ((_rawMeas[3] * _accelScale) - _accB[2]);

    _gyr[0] = (_rawMeas[4] * _gyroScale) - _gyrB[0];
    _gyr[1] = (_rawMeas[5] * _gyroScale) - _gyrB[1];
    _gyr[2] = (_rawMeas[6] * _gyroScale) - _gyrB[2];

    return true;
  }
  return false;
}

/* configures and enables the FIFO buffer  */
uint8_t ICM42688_FIFO::enableFifo()
{
  // use low speed SPI for register setting
  status = 0;
  _useSPIHS = false;

  status |= disableAccelGyro();
  status |= setBank(0);
  status |= writeRegister(UB0_REG_FIFO_CONFIG, 0x40);  // Set FIFO mode to Stream-to-FIFO
  status |= writeRegister(UB0_REG_FIFO_CONFIG1, 0x03); // Enable gyro and accel data output

  readRegisters(UB0_REG_INTF_CONFIG0, 1, _buffer);
  status |= writeRegister(UB0_REG_INTF_CONFIG0, _buffer[0] | 0x40); // Set FIFO_COUNT to record count and not bytes

  return status;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
uint8_t ICM42688_FIFO::readFifo(FusionAhrs &ahrs, float &euler_yaw, bool integrate_position)
{
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(UB0_REG_FIFO_COUNTH, 2, _buffer);
  _fifoSize = (_buffer[0] << 8) | _buffer[1];
  // read and parse the buffer
  for (uint32_t i = 0; i < _fifoSize; i++)
  {
    // grab the data from the ICM42688
    readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer);
    // combine into 16 bit values
    int16_t rawMeas[3];
    rawMeas[0] = (((int16_t)_buffer[1]) << 8) | _buffer[2];
    rawMeas[1] = (((int16_t)_buffer[3]) << 8) | _buffer[4];
    rawMeas[2] = (((int16_t)_buffer[5]) << 8) | _buffer[6];
    // transform and convert to float values
    _acc[0] = ((rawMeas[0] * _accelScale) - _accB[0]);
    _acc[1] = ((rawMeas[1] * _accelScale) - _accB[1]);
    _acc[2] = ((rawMeas[2] * _accelScale) - _accB[2]);

    // NEED TO TEST
    //_t = (static_cast<float>(_buffer[13]) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

    rawMeas[0] = (((int16_t)_buffer[7]) << 8) | _buffer[8];
    rawMeas[1] = (((int16_t)_buffer[9]) << 8) | _buffer[10];
    rawMeas[2] = (((int16_t)_buffer[11]) << 8) | _buffer[12];
    // transform and convert to float values
    _gyr[0] = (rawMeas[0] * _gyroScale) - _gyrB[0];
    _gyr[1] = (rawMeas[1] * _gyroScale) - _gyrB[1];
    _gyr[2] = (rawMeas[2] * _gyroScale) - _gyrB[2];

    //LINEGRAPH_MULTI("Gyro raw X", _gyr[0], 3);
    //LINEGRAPH_MULTI("Gyro raw Y", _gyr[1], 3);
    //LINEGRAPH_MULTI("Gyro raw Z", _gyr[2], 3);

    /*
      AHRS: Madgwick estimation
    */
    if (abs(_gyr[2]) > 0.1) // Filter out small values, hopefully reduces drift
    {
      euler_yaw -= _gyr[2] * delta_seconds;
    }

    gyroscope.axis = {_gyr[0], _gyr[1], _gyr[2]};
    accelerometer.axis = {_acc[0], _acc[1], _acc[2]};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_seconds);

    if (integrate_position)
    {
      acc_earth = FusionAhrsGetEarthAcceleration(&ahrs);
      acc_earth.axis.x *= 9.80665;
      acc_earth.axis.y *= 9.80665;
      acc_earth.axis.z *= 9.80665;

      if (abs(acc_earth.axis.x) < 0.05) 
      {
        _velocity = 0.0f;
      }

      //if (abs(acc_earth.axis.x - acc_earth_prev.axis.x) > 0.003)
      {
        _velocity -= acc_earth.axis.x * delta_seconds;
        _position += _velocity * delta_seconds * 100; // Position in cm
      }
      acc_earth_prev.axis.x = acc_earth.axis.x;
    }

/*
    LINEGRAPH("Velocity X", _velocity);
    LINEGRAPH("Position X", _position);

    LINEGRAPH_MULTI("Accel X", _acc[0], 2);
    LINEGRAPH_MULTI("Accel Y", _acc[1], 2);
    LINEGRAPH_MULTI("Accel Z", _acc[2], 2);

    LINEGRAPH_MULTI("Earth X", acc_earth.axis.x, 4);
    LINEGRAPH_MULTI("Earth Y", acc_earth.axis.y, 4);
    LINEGRAPH_MULTI("Earth Z", acc_earth.axis.z, 4);
*/

    /*
    count_++;
    if (past_millis_count_ + 1000 < millis())
    {
      past_millis_count_ = millis();
      Serial.print("Read count: ");
      Serial.print(count_);
      Serial.print("\t z:");
      Serial.print(euler_yaw);
      Serial.print("°\t ");
      Serial.print(millis() / 1000);
      Serial.println("s");
      count_ = 0;
    }
    */
  }
  return 0;
}

float ICM42688::getPosition(){
  return _position;
}

void ICM42688::resetPosition()
{
  _position = 0.0f;
  _velocity = 0.0f;
}

uint8_t ICM42688::biasCalibrationRoutine()
{
  // set at a lower range (more resolution) since IMU not moving
  status = 0;
  status |= disableAccelGyro();

  const GyroFS current_fssel = _gyroFS;
  status |= setGyroFS(dps500);
  status |= setGyroODR(odr100);
  status |= setAccelODR(odr100);
  status |= enableAccelGyroLN();

  _useSPIHS = true;

  float acc_average[3] = {0.0f};
  float gyr_average[3] = {0.0f};

  uint16_t count = 0;
  while (count < NUM_CALIB_SAMPLES)
  {
    if (ReadRawMeasurements() && accZ() > 0.1f) // Strangely, some Z accelerometer readings are 0.0f, skewing the average. This happens randomly and is obviously undesirable. This could be happening for the other axis as well, but it isn't noticeable
    {

      gyr_average[0] += gyrX();
      gyr_average[1] += gyrY();
      gyr_average[2] += gyrZ();

      acc_average[0] += accX();
      acc_average[1] += accY();
      acc_average[2] += accZ();

/*
      LINEGRAPH_MULTI("Gyro raw X", _gyr[0], 3);
      LINEGRAPH_MULTI("Gyro raw Y", _gyr[1], 3);
      LINEGRAPH_MULTI("Gyro raw Z", _gyr[2], 3);

      LINEGRAPH_MULTI("Accel X", _acc[0] * 9.81, 2);
      LINEGRAPH_MULTI("Accel Y", _acc[1] * 9.81, 2);
      LINEGRAPH_MULTI("Accel Z", _acc[2] * 9.81, 2);
*/
      count++;
    }
    delay(5);
  }

  _gyrB[0] = gyr_average[0] / NUM_CALIB_SAMPLES;
  _gyrB[1] = gyr_average[1] / NUM_CALIB_SAMPLES;
  _gyrB[2] = gyr_average[2] / NUM_CALIB_SAMPLES;

  _accB[0] = acc_average[0] / NUM_CALIB_SAMPLES;
  _accB[1] = acc_average[1] / NUM_CALIB_SAMPLES;
  _accB[2] = acc_average[2] / NUM_CALIB_SAMPLES - 1; // Gravity

  // recover the full scale setting
  status |= setGyroFS(current_fssel);
  status |= disableAccelGyro();

  return status;
}

/* returns the gyro bias in the X direction, dps */
float ICM42688::getGyroBiasX()
{
  return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float ICM42688::getGyroBiasY()
{
  return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float ICM42688::getGyroBiasZ()
{
  return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void ICM42688::setGyroBiasX(float bias)
{
  _gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void ICM42688::setGyroBiasY(float bias)
{
  _gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void ICM42688::setGyroBiasZ(float bias)
{
  _gyrB[2] = bias;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float ICM42688::getAccelBiasX_mss()
{
  return _accB[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float ICM42688::getAccelBiasY_mss()
{
  return _accB[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float ICM42688::getAccelBiasZ_mss()
{
  return _accB[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM42688::setAccelCalX(float bias)
{
  _accB[0] = bias;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM42688::setAccelCalY(float bias)
{
  _accB[1] = bias;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM42688::setAccelCalZ(float bias)
{
  _accB[2] = bias;
}

/* writes a byte to ICM42688 register given a register address and data */
uint8_t ICM42688::writeRegister(uint8_t subAddress, uint8_t data)
{

  _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
  digitalWrite(_csPin, LOW);                                              // select the ICM42688 chip
  _spi->transfer(subAddress);                                             // write the register address
  _spi->transfer(data);                                                   // write the data
  digitalWrite(_csPin, HIGH);                                             // deselect the ICM42688 chip
  _spi->endTransaction();                                                 // end the transaction

  delay(10);

  /* read back the register */
  readRegisters(subAddress, 1, _buffer);
  /* check the read back register against the written register */
  if (_buffer[0] == data)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
void ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  // begin the transaction
  if (_useSPIHS)
  {
    _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
  }
  else
  {
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
  }
  digitalWrite(_csPin, LOW);         // select the ICM42688 chip
  _spi->transfer(subAddress | 0x80); // specify the starting register address
  for (uint8_t i = 0; i < count; i++)
  {
    dest[i] = _spi->transfer(0x00); // read the data
  }
  digitalWrite(_csPin, HIGH); // deselect the ICM42688 chip
  _spi->endTransaction();     // end the transaction
}

uint8_t ICM42688::setBank(uint8_t bank)
{
  // if we are already on this bank, bail
  if (_bank == bank)
    return 0;

  _bank = bank;

  return writeRegister(REG_BANK_SEL, bank);
}

uint8_t ICM42688::reset()
{
  status = 0;
  status |= setBank(0);

  status |= writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

  // wait for ICM42688 to come back up
  delay(1);
  return status;
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t ICM42688::whoAmI()
{
  setBank(0);

  // read the WHO AM I register
  readRegisters(UB0_REG_WHO_AM_I, 1, _buffer);
  // return the register value
  return _buffer[0];
}
