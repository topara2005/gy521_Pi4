#ifndef DEF_I2C_DEVICE_H
#define DEF_I2C_DEVICE_H
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define I2C_NODE	"/dev/i2c-"
#define BUFSIZE			512

class I2C_device
{
private:
   char filename[64];
public:
    I2C_device(int deviceNumber);
    ~I2C_device();
    virtual int openBus();
    int setSlaveAddress(int bus);
    int write(uint8_t address, uint8_t *data, int length);
    int read(uint8_t address, uint8_t *data, int length);
    int read_reg(uint8_t address, uint8_t reg, uint8_t *data, int length);
protected:
    int fd;
    virtual int read_bus(unsigned char *buf, int bufsize);
    virtual int read_register( __uint16_t reg, unsigned char *buf, int bufsize)
};

I2C_device::I2C_device(int bus)
{
     snprintf(filename, 64, "%s%d", I2C_NODE, bus);
}

I2C_device::~I2C_device()
{
    close(fd);
}

///
/// \brief I2C_device::read_bus - read from the bus into the buffer provided   
/// \param buf      buffer to read into
/// \param bufsize  size of buffer
I2C_device::read_bus(unsigned char *buf, int bufsize)
{
    return read(fd, buf, bufsize);
}
///
/// \brief I2C_device::read_register    Read a register from the I2C device 
/// \param reg  register to read
/// \param buf  buffer to read into
/// \param bufsize  size of buffer
I2C_device::read_register( __uint16_t reg, unsigned char *buf, int bufsize)
{
    unsigned char reg_buf[2];
    int ret;

    reg_buf[0] = (reg >> 0) & 0xFF;
    reg_buf[1] = (reg >> 8) & 0xFF;
    ret = write_bus(fd, reg_buf, 2);
	if (ret < 0) {
		printf("Failed to write [0x%02x 0x%02x] (reg: %04x).\n", reg_buf[0], reg_buf[1], reg);
		return ret;
	}

	printf("wrote %d bytes\n", ret);
	return read_bus( buf, bufsize);
    
}
/// \brief I2C_device::openBus - open the bus   
/// \return 0 on success, -1 on failure
I2C_device::openBus()
{
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("Failed to open the i2c bus.\n");
        return -1;
    }
    return fd;
}
///
/// \brief  I2C_device::setSlaveAddress  set the slave address   of the device
/// \param bus  bus number
I2C_device::setSlaveAddress(int address)
{
    int ret = ioctl(fd, I2C_SLAVE, address);
    if (ret < 0)
        printf("Failed to acquire bus access and/or talk to slave.\n");
    return ret;
}
///     /brief I2C_device::write  write to the device   
/// \param address  address of the device   
/// \param data  data to write  
/// \param length  length of data
I2C_device::write(uint8_t address, uint8_t *data, int length)
{
    int ret = write(fd, data, length);
    if (ret < 0)
        printf("Failed to write to the i2c bus.\n");
    return ret;
}
I2C_device::read( uint8_t *data, int length)
{
    int ret = read(fd, data, length);
    if (ret < 0)
        printf("Failed to read from the i2c bus.\n");
    return ret;
}


#endif