
#ifndef DEF_GY521_H
#define DEF_GH521_H

#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R

#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_I2C_ADDRESS        0x68   // I2C
#define MPU6050_GYRO_CONFIG        0x1B
#define MPU6050_ACCEL_CONFIG       0x1C
#define MPU6050_WHO_AM_I           0x75




#include <stdint.h>
// SOURCE: https://www.kernel.org/doc/Documentation/i2c/dev-interface
#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include "I2C_device.h"

class MPU6050:public  I2C_device
{
  public:
    MPU6050();
    MPU6050(int adapterNumber);
    void initGyro();
   bool isConnected();
    int8_t getGyroX();
    int8_t getGyroY();
    int8_t getGyroZ();
    int8_t getAccelX();
    int8_t getAccelY();
    int8_t getAccelZ();
    float getAngleX();
    float getAngleY();
    float getAngleZ();

  private:
    int I2CAddress;
    int adapterNumber=1;

    int8_t x_gyro;
    int8_t y_gyro;
    int8_t z_gyro;

    int8_t x_init_gyro;
    int8_t y_init_gyro;
    int8_t z_init_gyro;

    int8_t x_accel;
    int8_t y_accel;
    int8_t z_accel;
};

#endif