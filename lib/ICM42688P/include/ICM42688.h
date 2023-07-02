#ifndef ICM42688_H
#define ICM42688_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Fusion.h>
#include "registers.h"

class ICM42688
{
public:
  enum GyroFS : uint8_t
  {
    dps2000 = 0x00,
    dps1000 = 0x01,
    dps500 = 0x02,
    dps250 = 0x03,
    dps125 = 0x04,
    dps62_5 = 0x05,
    dps31_25 = 0x06,
    dps15_625 = 0x07
  };

  enum AccelFS : uint8_t
  {
    gpm16 = 0x00,
    gpm8 = 0x01,
    gpm4 = 0x02,
    gpm2 = 0x03
  };

  enum ODR : uint8_t
  {
    odr32k = 0x01, // LN mode only
    odr16k = 0x02, // LN mode only
    odr8k = 0x03,  // LN mode only
    odr4k = 0x04,  // LN mode only
    odr2k = 0x05,  // LN mode only
    odr1k = 0x06,  // LN mode only
    odr200 = 0x07,
    odr100 = 0x08,
    odr50 = 0x09,
    odr25 = 0x0A,
    odr12_5 = 0x0B,
    odr6a25 = 0x0C,   // LP mode only (accel only)
    odr3a125 = 0x0D,  // LP mode only (accel only)
    odr1a5625 = 0x0E, // LP mode only (accel only)
    odr500 = 0x0F,
  };

  enum Filter_BW : uint8_t
  {
    bw_1449,
    bw_680,
    bw_329,
    bw_162,
    bw_80,
    bw_40,
    bw_20,
    bw_10
  };

  /**
   * @brief      Constructor for SPI communication
   *
   * @param      bus    SPI bus
   * @param[in]  csPin  Chip Select pin
   */
  ICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK = 8000000);

  uint8_t begin();

  uint8_t setAccelFS(AccelFS fssel);

  uint8_t setGyroFS(GyroFS fssel);

  uint8_t setAccelODR(ODR odr);

  uint8_t setGyroODR(ODR odr);

  uint8_t configureNotchFilterBandwidth(Filter_BW f_bw);
  uint8_t configureUIFilter(AAF_3db_bw_hz accel_bw, AAF_3db_bw_hz gyro_bw);
  uint8_t configureUIFilterAccel(AAF_3db_bw_hz accel_bw);
  uint8_t configureUIFilterGyro(AAF_3db_bw_hz gyro_bw);

  uint8_t setFilters(bool gyroFilters, bool accFilters);

  uint8_t enableDataReadyInterrupt();

  uint8_t disableDataReadyInterrupt(); // NEED TO CHECK

  bool ReadRawMeasurements();

  /**
   * @brief      Get accelerometer data, per axis
   *
   * @return     Acceleration in g's
   */
  float accX() const { return _acc[0]; }
  float accY() const { return _acc[1]; }
  float accZ() const { return _acc[2]; }

  /**
   * @brief      Get gyro data, per axis
   *
   * @return     Angular velocity in dps
   */
  float gyrX() const { return _gyr[0]; }
  float gyrY() const { return _gyr[1]; }
  float gyrZ() const { return _gyr[2]; }

  /**
   * @brief      Get temperature of gyro die
   *
   * @return     Temperature in Celsius
   */
  float temp() const { return _t; }

  int16_t getAccelX_count();
  int16_t getAccelY_count();
  int16_t getAccelZ_count();
  int16_t getGyroX_count();
  int16_t getGyroY_count();
  int16_t getGyroZ_count();

  uint8_t biasCalibrationRoutine();
  float getGyroBiasX();
  float getGyroBiasY();
  float getGyroBiasZ();
  void setGyroBiasX(float bias);
  void setGyroBiasY(float bias);
  void setGyroBiasZ(float bias);
  float getAccelBiasX_mss();
  float getAccelBiasY_mss();
  float getAccelBiasZ_mss();
  void setAccelCalX(float bias);
  void setAccelCalY(float bias);
  void setAccelCalZ(float bias);
  uint8_t enableExternalClock();
  uint8_t enableAccelGyroLN();
  uint8_t disableAccelGyro();
  void resetPosition();
  float getPosition();
  bool dataIsReady();

protected:
  uint8_t status = 0;

  int16_t _rawMeas[7]; // temp, accel xyz, gyro xyz

  ///\brief SPI Communication
  SPIClass *_spi = {};
  uint8_t _csPin = 0;
  bool _useSPIHS = false;
  static constexpr uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
  uint32_t SPI_HS_CLOCK = 8000000;                  // 8 MHz

  // buffer for reading from sensor
  uint8_t _buffer[16] = {0};

  bool gyro_accel_started = false;

  // data buffer
  float _t = 0.0f;
  float _acc[3] = { 0.0f }; // [ x, y, z ]
  float _gyr[3] = { 0.0f }; // [ x, y, z ]

  FusionVector acc_earth;
  FusionVector acc_earth_prev;

  float _velocity = 0.0f;

  float _position = 0.0f;

  ///\brief Full scale resolution factors
  float _accelScale = 0.0f;
  float _gyroScale = 0.0f;

  ///\brief Full scale selections
  AccelFS _accelFS;
  GyroFS _gyroFS;

  ///\brief Accel calibration
  float _accB[3] = { 0.0f };

  ///\brief Gyro calibration
  float _gyrB[3] = { 0.0f };

  ///\brief Constants
  static constexpr uint8_t WHO_AM_I = 0x47;           ///< expected value in UB0_REG_WHO_AM_I reg
  static constexpr uint16_t NUM_CALIB_SAMPLES = 100; ///< for gyro/accel bias calib

  ///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
  static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
  static constexpr float TEMP_OFFSET = 25.0f;

  uint8_t _bank = 0; ///< current user bank

  const uint8_t FIFO_EN = 0x23;
  const uint8_t FIFO_TEMP_EN = 0x04;
  const uint8_t FIFO_GYRO = 0x02;
  const uint8_t FIFO_ACCEL = 0x01;
  // const uint8_t FIFO_COUNT = 0x2E;
  // const uint8_t FIFO_DATA = 0x30;

  // BANK 1
  // const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
  const uint8_t GYRO_NF_ENABLE = 0x00;
  const uint8_t GYRO_NF_DISABLE = 0x01;
  const uint8_t GYRO_AAF_ENABLE = 0x00;
  const uint8_t GYRO_AAF_DISABLE = 0x02;

  // BANK 2
  // const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
  const uint8_t ACCEL_AAF_ENABLE = 0x00;
  const uint8_t ACCEL_AAF_DISABLE = 0x01;

  // private functions
  uint8_t writeRegister(uint8_t subAddress, uint8_t data);
  void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
  uint8_t setBank(uint8_t bank);

  /**
   * @brief      Software reset of the device
   */
  uint8_t reset();

  /**
   * @brief      Read the WHO_AM_I register
   *
   * @return     Value of WHO_AM_I register
   */
  uint8_t whoAmI();
};

class ICM42688_FIFO : public ICM42688
{
public:
  using ICM42688::ICM42688;
  uint8_t enableFifo();
  uint8_t readFifo(FusionAhrs &ahrs, float &euler_yaw, bool integrate_position);
  FusionVector gyroscope;             // degrees/s
  FusionVector accelerometer;         // g
  const double delta_seconds = 0.01f; // 1s/100(odr)
  uint32_t _fifoSize = 0;
  const uint8_t _fifoFrameSize = 16; // If Accel + Gyro enabled packet size = 16 bytes as per datasheet
};

#endif // ICM42688_H
