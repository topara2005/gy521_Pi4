
#include <stdio.h>
#include <cmath>
#include "gy521.h"
 

 MPU6050::MPU6050()
 {
   fd = -1;
   I2CAddress = MPU6050_I2C_ADDRESS;
   x_gyro = 0;
   y_gyro = 0;
   z_gyro = 0;
   x_init_gyro = 0;
   y_init_gyro = 0;
   z_init_gyro = 0;
   x_accel = 0;
 }

