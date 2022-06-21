
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

    MPU6050::MPU6050(int i2cAdapterNumber):MPU6050()
    {
        adapterNumber=i2cAdapterNumber;
    }
    check_MPU6050()
    {
        if(fd==-1)
        {
            printf("MPU6050 not connected\n");
            return false;
        }
        return true;
    }
 
    bool GY521::isConnected()
    {
    _wire->beginTransmission(_address);
    return (_wire->endTransmission() == 0);
    }
    void MPU6050::initGyro()
    {
        if (fd == -1)
        {
            char filename[20];
            snprintf(filename, 19, "/dev/i2c-%d", adapterNumber);
            fd = open(filename, O_RDWR);
            if (fd == -1)
            {
                printf("Error opening file\n");
                return;
            }
            if (ioctl(fd, I2C_SLAVE, I2CAddress) < 0)
            {
                printf("Error setting slave address\n");
                return;
            }
        }
        //First we reset the device...according to datasheet, this is done by setting the PWR_MGMT_1 register to 0x00
        //Set SLEEP bit = 0 in the Power Management Register (107) OR Clear the Register
        uint8_t data[2];
        data[0] = MPU6050_PWR_MGMT_1;
        data[1] = 0x00;
        if (write(fd, data, 2) != 2)
        {
            printf("Error writing to MPU6050\n");
            return;
        }
        //Then we set the gyroscope to full scale range of 250 degrees/sec)
        //Set the GYRO_CONFIG register to 0x18
        data[0] = MPU6050_GYRO_CONFIG;
        data[1] = 0x18;
        if (write(fd, data, 2) != 2)
        {
            printf("Error writing to MPU6050\n");
            return;
        }
        //Then we set the accelerometer to full scale range of 2g
        //Set the ACCEL_CONFIG register to 0x01
        data[0] = MPU6050_ACCEL_CONFIG;
        data[1] = 0x01;
        if (write(fd, data, 2) != 2)
        {
            printf("Error writing to MPU6050\n");
            return;
        }
        //Then we set the clock source to use the gyroscope 
        //Set the PWR_MGMT_1 register to 0x01   
        data[0] = MPU6050_PWR_MGMT_1;   
        data[1] = 0x01;
        if (write(fd, data, 2) != 2)
        {
            printf("Error writing to MPU6050\n");
            return;
        }
        read(fd, data, 14);
    }

    int talk_to_device(int busfd, int addr)
    {
        int ret = ioctl(busfd, I2C_SLAVE, addr);
        if (ret < 0)
            printf("Failed to acquire bus access and/or talk to slave.\n");

        return ret;
    }
    int read_bus(int busfd, unsigned char *buf, int bufsize)
    {
        return read(busfd, buf, bufsize);
    }
    int read_register(int busfd, __uint16_t reg, unsigned char *buf, int bufsize)
    {
        unsigned char reg_buf[2];
        int ret;

        reg_buf[0] = (reg >> 0) & 0xFF;
        reg_buf[1] = (reg >> 8) & 0xFF;

        ret = write_bus(busfd, reg_buf, 2);
        if (ret < 0) {
            printf("Failed to write [0x%02x 0x%02x] (reg: %04x).\n", reg_buf[0], reg_buf[1], reg);
            return ret;
        }

        printf("wrote %d bytes\n", ret);
        return read_bus(busfd, buf, bufsize);

    }
    int write_bus(int busfd, unsigned char *buf, int bufsize)
    {
        return write(busfd, buf, bufsize);
    }

