/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   i2c.h
 * Author: florian
 *
 * Created on 31. August 2016, 10:40
 */

#ifndef I2C_H
#define I2C_H

#define HMC5883L_I2C_ADDR 0x1E

enum e_i2c_status {
    I2C_SUCCESS,
    I2C_WRITE,
    I2C_READ,
    I2C_ERROR
};

enum e_i2c_status init_i2c(int *fd,int addr);
enum e_i2c_status read_i2c(int *fd, short *x, short *y, short *z);

#endif /* I2C_H */

