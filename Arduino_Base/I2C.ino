

#ifdef IMU_V5

#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 gyro_acc;
LIS3MDL mag;

#else // older IMUs through v4

#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

#endif


void I2C_Init()
{
  /*
   * This function: 
   * 1. Initializes I2C
   */
  Wire.begin();
}

void Gyro_Init()
{
  /*
   * This function: 
   * 1. Initializes gyroscope in a way that depends on the type of IMU used
   */
#ifdef IMU_V5
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
#else
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
#endif
}

void Read_Gyro()
{
  /*
   * This function: 
   * 1. Reads data from gyroscope and stores them in AN. 
   * 2. Uses sensor offset to get correct readings of gyroscope
   */
#ifdef IMU_V5
  gyro_acc.readGyro();

  AN[0] = gyro_acc.g.x;
  AN[1] = gyro_acc.g.y;
  AN[2] = gyro_acc.g.z;
#else
  gyro.read();

  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
#endif

  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
  /*
   * This function:
   * 1. Initializes accelerometer
   * 2. Initializes compass
   */
#ifdef IMU_V5
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
#else
  compass.init();
  compass.enableDefault();
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
#endif
}

void Read_Accel()
/*
 * This function:
 * 1. Reads x,y,z data from accelerometer, either by gyro_acc unit or by compass and stores them in AN
 * 2. Uses sensor offset to get correct readings of accelerometer
 */
{
#ifdef IMU_V5
  gyro_acc.readAcc();

  AN[3] = gyro_acc.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = gyro_acc.a.y >> 4;
  AN[5] = gyro_acc.a.z >> 4;
#else
  compass.readAcc();

  AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
#endif
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{
  /*
   * This function:
   * 1. Initializes compass and, by extension megnetometer
   */
#ifdef IMU_V5
  mag.init();
  mag.enableDefault();
#else
  // LSM303: doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
#endif
}

void Read_Compass()
{
  /*
   * This function:
   * 1. Reads data from compass or directly from magnetometer depending on IMU version
   */
#ifdef IMU_V5
  mag.read();

  magnetom_x = SENSOR_SIGN[6] * mag.m.x;
  magnetom_y = SENSOR_SIGN[7] * mag.m.y;
  magnetom_z = SENSOR_SIGN[8] * mag.m.z;
#else
  compass.readMag();

  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
#endif
}

