#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include "i2c.h"

//#define M_PI 3.14159265358979323846


unsigned char i2c_buffer[16];

void selectDevice(int *fd, int *addr, char * name) {
    if (ioctl(*fd, I2C_SLAVE, *addr) < 0) {
        fprintf(stderr, "%s not present\n", name);
    }
}

void writeToDevice(int *fd, int reg, int val) {
    char buf[2];
    buf[0] = reg;
    buf[1] = val;

    if (write(*fd, buf, 2) != 2) {
        fprintf(stderr, "Can't write to ADXL345\n");
    }
}

enum e_i2c_status init_i2c(int *fd, int addr) {
    *fd = -1;

    if ((*fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        fprintf(stderr, "Failed to open i2c bus\n");

        return I2C_ERROR;
    }

    /* initialise ADXL345 */
    selectDevice(fd, &addr, "HMC5883L");

    //writeToDevice(fd, 0x01, 0);
    writeToDevice(fd, 0x01, 32);
    writeToDevice(fd, 0x02, 0);

    return I2C_SUCCESS;
}

enum e_i2c_status read_i2c(int *fd, short *x, short *y, short *z) {
    i2c_buffer[0] = 0x03;

    if ((write(*fd, i2c_buffer, 1)) != 1) {
        // Send the register to read from
        fprintf(stderr, "Error writing to i2c slave\n");
        return I2C_WRITE;
    }

    if (read(*fd, i2c_buffer, 6) != 6) {
        fprintf(stderr, "Unable to read from HMC5883L\n");
        return I2C_READ;
    } else {
        *x = (i2c_buffer[0] << 8) | i2c_buffer[1];
        *z = (i2c_buffer[2] << 8) | i2c_buffer[3];
        *y = (i2c_buffer[4] << 8) | i2c_buffer[5];
        //float angle = atan2(*y, *x) * 180.0f / M_PI;
    }

    return I2C_SUCCESS;
}

