#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <limits.h>
#include "sensor.h"

#define HTS221_I2C  0x5F

char reg;
char data[4] = {0};
int H0, H1, H2, H3;
int T0, T1, T2, T3;

enum e_i2c_status sensor_hum_init(int * file) {
    assert((*file = open("/dev/i2c-1", O_RDWR)) != -1);
    ioctl(*file, I2C_SLAVE, HTS221_I2C);

    // Select average configuration register(0x10)
    // Temperature average samples = 16, humidity average samples = 32(0x1B)
    char config[2] = {0x10, 0x1B};
    //config[0] = 0x10;
    //config[1] = 0x1B;
    write(*file, config, 2);
    sleep(1);
    // Select control register1(0x20)
    // Power on, block data update, data rate o/p = 1 Hz(0x85)
    config[0] = 0x20;
    config[1] = 0x85;
    write(*file, config, 2);
    sleep(1);

    // Read Calibration values from the non-volatile memory of the device
    // Humidity Calibration values
    //Read 1 byte of data from address(0x30)
    reg = 0x30;
    write(*file, &reg, 1);
    //char data[1] = {0};
    assert(read(*file, data, 1) == 1);
    char data_0 = data[0];
    // Read 1 byte of data from address(0x31)
    reg = 0x31;
    write(*file, &reg, 1);
    read(*file, data, 1);
    char data_1 = data[0];

    H0 = data_0 / 2;
    H1 = data_1 / 2;

    //Read 1 byte of data from address(0x36)
    reg = 0x36;
    write(*file, &reg, 1);
    assert(read(*file, data, 1) == 1);

    data_0 = data[0];
    // Read 1 byte of data from address(0x37)
    reg = 0x37;
    write(*file, &reg, 1);
    read(*file, data, 1);
    data_1 = data[0];

    H2 = (data_1 * 256) + data_0;

    //Read 1 byte of data from address(0x3A)
    reg = 0x3A;
    write(*file, &reg, 1);
    assert(read(*file, data, 1) == 1);

    data_0 = data[0];
    // Read 1 byte of data from address(0x3B)
    reg = 0x3B;
    write(*file, &reg, 1);
    read(*file, data, 1);
    data_1 = data[0];

    H3 = (data_1 * 256) + data_0;

    // Temperature Calibration values
    // Read 1 byte of data from address(0x32)
    reg = 0x32;
    write(*file, &reg, 1);
    read(*file, data, 1);

    T0 = data[0];

    // Read 1 byte of data from address(0x33)
    reg = 0x33;
    write(*file, &reg, 1);
    read(*file, data, 1);

    T1 = data[0];

    // Read 1 byte of data from address(0x35)
    reg = 0x35;
    write(*file, &reg, 1);
    read(*file, data, 1);

    int raw = data[0];

    // Convert the temperature Calibration values to 10-bits
    T0 = ((raw & 0x03) * 256) + T0;
    T1 = ((raw & 0x0C) * 64) + T1;

    //Read 1 byte of data from address(0x3C)
    reg = 0x3C;
    write(*file, &reg, 1);
    assert(read(*file, data, 1) == 1);
    data_0 = data[0];
    // Read 1 byte of data from address(0x3D)
    reg = 0x3D;
    write(*file, &reg, 1);
    read(*file, data, 1);
    data_1 = data[0];

    T2 = (data_1 * 256) + data_0;

    //Read 1 byte of data from address(0x3E)
    reg = 0x3E;
    write(*file, &reg, 1);
    assert(read(*file, data, 1) == 1);
    data_0 = data[0];
    // Read 1 byte of data from address(0x3F)
    reg = 0x3F;
    write(*file, &reg, 1);
    read(*file, data, 1);
    data_1 = data[0];

    T3 = (data_1 * 256) + data_0;

    return I2C_SUCCESS;
}

enum e_i2c_status sensor_hum_read(int *file, float *humidity, float *cTemp) {
    // Read 4 bytes of data(0x28 | 0x80)
    // hum msb, hum lsb, temp msb, temp lsb
    reg = 0x28 | 0x80;
    write(*file, &reg, 1);

    assert(read(*file, data, 4) == 4);

    // Convert the data
    int hum = (data[1] * 256) + data[0];
    int temp = (data[3] * 256) + data[2];
    if (temp > SHRT_MAX) {
        temp -= USHRT_MAX;
    }
    *humidity = ((1.0 * H1) - (1.0 * H0)) * (1.0 * hum - 1.0 * H2) / (1.0 * H3 - 1.0 * H2) + (1.0 * H0);
    *cTemp = ((T1 - T0) / 8.0) * (temp - T2) / (T3 - T2) + (T0 / 8.0);

    //printf("\rRelative humidity : %.2f RH \n", *humidity);
    //printf("\rTemperature in Celsius : %.2f C \n", *cTemp);

    return I2C_SUCCESS;
}