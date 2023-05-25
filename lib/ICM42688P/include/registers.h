#ifndef ICM42688_REGISTERS_H_
#define ICM42688_REGISTERS_H_

#include <cstdint>

namespace ICM42688reg
{

  // Accesible from all user banks
  static constexpr uint8_t REG_BANK_SEL = 0x76;

  // User Bank 0
  static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;
  // break
  static constexpr uint8_t UB0_REG_DRIVE_CONFIG = 0x13;
  static constexpr uint8_t UB0_REG_INT_CONFIG = 0x14;
  // break
  static constexpr uint8_t UB0_REG_FIFO_CONFIG = 0x16;
  // break
  static constexpr uint8_t UB0_REG_TEMP_DATA1 = 0x1D;
  static constexpr uint8_t UB0_REG_TEMP_DATA0 = 0x1E;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_X1 = 0x1F;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_X0 = 0x20;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Y1 = 0x21;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Y0 = 0x22;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Z1 = 0x23;
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Z0 = 0x24;
  static constexpr uint8_t UB0_REG_GYRO_DATA_X1 = 0x25;
  static constexpr uint8_t UB0_REG_GYRO_DATA_X0 = 0x26;
  static constexpr uint8_t UB0_REG_GYRO_DATA_Y1 = 0x27;
  static constexpr uint8_t UB0_REG_GYRO_DATA_Y0 = 0x28;
  static constexpr uint8_t UB0_REG_GYRO_DATA_Z1 = 0x29;
  static constexpr uint8_t UB0_REG_GYRO_DATA_Z0 = 0x2A;
  static constexpr uint8_t UB0_REG_TMST_FSYNCH = 0x2B;
  static constexpr uint8_t UB0_REG_TMST_FSYNCL = 0x2C;
  static constexpr uint8_t UB0_REG_INT_STATUS = 0x2D;
  static constexpr uint8_t UB0_REG_FIFO_COUNTH = 0x2E;
  static constexpr uint8_t UB0_REG_FIFO_COUNTL = 0x2F;
  static constexpr uint8_t UB0_REG_FIFO_DATA = 0x30;
  static constexpr uint8_t UB0_REG_APEX_DATA0 = 0x31;
  static constexpr uint8_t UB0_REG_APEX_DATA1 = 0x32;
  static constexpr uint8_t UB0_REG_APEX_DATA2 = 0x33;
  static constexpr uint8_t UB0_REG_APEX_DATA3 = 0x34;
  static constexpr uint8_t UB0_REG_APEX_DATA4 = 0x35;
  static constexpr uint8_t UB0_REG_APEX_DATA5 = 0x36;
  static constexpr uint8_t UB0_REG_INT_STATUS2 = 0x37;
  static constexpr uint8_t UB0_REG_INT_STATUS3 = 0x38;
  // break
  static constexpr uint8_t UB0_REG_SIGNAL_PATH_RESET = 0x4B;
  static constexpr uint8_t UB0_REG_INTF_CONFIG0 = 0x4C;
  static constexpr uint8_t UB0_REG_INTF_CONFIG1 = 0x4D;
  static constexpr uint8_t UB0_REG_PWR_MGMT0 = 0x4E;
  static constexpr uint8_t UB0_REG_GYRO_CONFIG0 = 0x4F;
  static constexpr uint8_t UB0_REG_ACCEL_CONFIG0 = 0x50;
  static constexpr uint8_t UB0_REG_GYRO_CONFIG1 = 0x51;
  static constexpr uint8_t UB0_REG_GYRO_ACCEL_CONFIG0 = 0x52;
  static constexpr uint8_t UB0_REG_ACCEFL_CONFIG1 = 0x53;
  static constexpr uint8_t UB0_REG_TMST_CONFIG = 0x54;
  // break
  static constexpr uint8_t UB0_REG_APEX_CONFIG0 = 0x56;
  static constexpr uint8_t UB0_REG_SMD_CONFIG = 0x57;
  // break
  static constexpr uint8_t UB0_REG_FIFO_CONFIG1 = 0x5F;
  static constexpr uint8_t UB0_REG_FIFO_CONFIG2 = 0x60;
  static constexpr uint8_t UB0_REG_FIFO_CONFIG3 = 0x61;
  static constexpr uint8_t UB0_REG_FSYNC_CONFIG = 0x62;
  static constexpr uint8_t UB0_REG_INT_CONFIG0 = 0x63;
  static constexpr uint8_t UB0_REG_INT_CONFIG1 = 0x64;
  static constexpr uint8_t UB0_REG_INT_SOURCE0 = 0x65;
  static constexpr uint8_t UB0_REG_INT_SOURCE1 = 0x66;
  // break
  static constexpr uint8_t UB0_REG_INT_SOURCE3 = 0x68;
  static constexpr uint8_t UB0_REG_INT_SOURCE4 = 0x69;
  // break
  static constexpr uint8_t UB0_REG_FIFO_LOST_PKT0 = 0x6C;
  static constexpr uint8_t UB0_REG_FIFO_LOST_PKT1 = 0x6D;
  // break
  static constexpr uint8_t UB0_REG_SELF_TEST_CONFIG = 0x70;
  // break
  static constexpr uint8_t UB0_REG_WHO_AM_I = 0x75;

  // User Bank 1
  static constexpr uint8_t UB1_REG_SENSOR_CONFIG0 = 0x03;
  // break
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC2 = 0x0B;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC3 = 0x0C;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC4 = 0x0D;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC5 = 0x0E;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC6 = 0x0F;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC7 = 0x10;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC8 = 0x11;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC9 = 0x12;
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC10 = 0x13;
  // break
  static constexpr uint8_t UB1_REG_XG_ST_DATA = 0x5F;
  static constexpr uint8_t UB1_REG_YG_ST_DATA = 0x60;
  static constexpr uint8_t UB1_REG_ZG_ST_DATA = 0x61;
  static constexpr uint8_t UB1_REG_TMSTVAL0 = 0x62;
  static constexpr uint8_t UB1_REG_TMSTVAL1 = 0x63;
  static constexpr uint8_t UB1_REG_TMSTVAL2 = 0x64;
  // break
  static constexpr uint8_t UB1_REG_INTF_CONFIG4 = 0x7A;
  static constexpr uint8_t UB1_REG_INTF_CONFIG5 = 0x7B;
  static constexpr uint8_t UB1_REG_INTF_CONFIG6 = 0x7C;

  // User Bank 2
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC2 = 0x03;
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC3 = 0x04;
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC4 = 0x05;
  // break
  static constexpr uint8_t UB2_REG_XA_ST_DATA = 0x3B;
  static constexpr uint8_t UB2_REG_YA_ST_DATA = 0x3C;
  static constexpr uint8_t UB2_REG_ZA_ST_DATA = 0x3D;

  // User Bank 4
  static constexpr uint8_t UB4_REG_APEX_CONFIG1 = 0x40;
  static constexpr uint8_t UB4_REG_APEX_CONFIG2 = 0x41;
  static constexpr uint8_t UB4_REG_APEX_CONFIG3 = 0x42;
  static constexpr uint8_t UB4_REG_APEX_CONFIG4 = 0x43;
  static constexpr uint8_t UB4_REG_APEX_CONFIG5 = 0x44;
  static constexpr uint8_t UB4_REG_APEX_CONFIG6 = 0x45;
  static constexpr uint8_t UB4_REG_APEX_CONFIG7 = 0x46;
  static constexpr uint8_t UB4_REG_APEX_CONFIG8 = 0x47;
  static constexpr uint8_t UB4_REG_APEX_CONFIG9 = 0x48;
  // break
  static constexpr uint8_t UB4_REG_ACCEL_WOM_X_THR = 0x4A;
  static constexpr uint8_t UB4_REG_ACCEL_WOM_Y_THR = 0x4B;
  static constexpr uint8_t UB4_REG_ACCEL_WOM_Z_THR = 0x4C;
  static constexpr uint8_t UB4_REG_INT_SOURCE6 = 0x4D;
  static constexpr uint8_t UB4_REG_INT_SOURCE7 = 0x4E;
  static constexpr uint8_t UB4_REG_INT_SOURCE8 = 0x4F;
  static constexpr uint8_t UB4_REG_INT_SOURCE9 = 0x50;
  static constexpr uint8_t UB4_REG_INT_SOURCE10 = 0x51;
  // break
  static constexpr uint8_t UB4_REG_OFFSET_USER0 = 0x77;
  static constexpr uint8_t UB4_REG_OFFSET_USER1 = 0x78;
  static constexpr uint8_t UB4_REG_OFFSET_USER2 = 0x79;
  static constexpr uint8_t UB4_REG_OFFSET_USER3 = 0x7A;
  static constexpr uint8_t UB4_REG_OFFSET_USER4 = 0x7B;
  static constexpr uint8_t UB4_REG_OFFSET_USER5 = 0x7C;
  static constexpr uint8_t UB4_REG_OFFSET_USER6 = 0x7D;
  static constexpr uint8_t UB4_REG_OFFSET_USER7 = 0x7E;
  static constexpr uint8_t UB4_REG_OFFSET_USER8 = 0x7F;

} // ns ICM42688reg

enum AAF_3db_bw_hz : uint8_t
{
  bw_42,
  bw_84,
  bw_126,
  bw_170,
  bw_213,
  bw_258,
  bw_303,
  bw_348,
  bw_394,
  bw_441,
  bw_488,
  bw_536,
  bw_585,
  bw_634,
  bw_684,
  bw_734,
  bw_785,
  bw_837,
  bw_890,
  bw_943,
  bw_997,
  bw_1051,
  bw_1107,
  bw_1163,
  bw_1220,
  bw_1277,
  bw_1336,
  bw_1395,
  bw_1454,
  bw_1515,
  bw_1577,
  bw_1639,
  bw_1702,
  bw_1766,
  bw_1830,
  bw_1896,
  bw_1962,
  bw_2029,
  bw_2097,
  bw_2166,
  bw_2235,
  bw_2306,
  bw_2377,
  bw_2449,
  bw_2522,
  bw_2596,
  bw_2671,
  bw_2746,
  bw_2823,
  bw_2900,
  bw_2978,
  bw_3057,
  bw_3137,
  bw_3217,
  bw_3299,
  bw_3381,
  bw_3464,
  bw_3548,
  bw_3633,
  bw_3718,
  bw_3805,
  bw_3892,
  bw_3979
};

struct aaf_reg_cont
{
  uint16_t AAF_DELTSQR;
  uint8_t AAF_BITSHIFT;
};

const aaf_reg_cont aaf_reg_val[63] = {
    {1, 15},
    {4, 13},
    {9, 12},
    {16, 11},
    {25, 10},
    {36, 10},
    {49, 9},
    {64, 9},
    {81, 9},
    {100, 8},
    {122, 8},
    {144, 8},
    {170, 8},
    {196, 7},
    {224, 7},
    {256, 7},
    {288, 7},
    {324, 7},
    {360, 6},
    {400, 6},
    {440, 6},
    {488, 6},
    {528, 6},
    {576, 6},
    {624, 6},
    {680, 6},
    {736, 5},
    {784, 5},
    {848, 5},
    {896, 5},
    {960, 5},
    {1024, 5},
    {1088, 5},
    {1152, 5},
    {1232, 5},
    {1296, 5},
    {1376, 4},
    {1440, 4},
    {1536, 4},
    {1600, 4},
    {1696, 4},
    {1760, 4},
    {1856, 4},
    {1952, 4},
    {2016, 4},
    {2112, 4},
    {2208, 4},
    {2304, 4},
    {2400, 4},
    {2496, 4},
    {2592, 4},
    {2720, 4},
    {2816, 3},
    {2944, 3},
    {3008, 3},
    {3136, 3},
    {3264, 3},
    {3392, 3},
    {3456, 3},
    {3584, 3},
    {3712, 3},
    {3840, 3},
    {3968, 3},
};

#endif // ICM42688_REGISTERS_H_
