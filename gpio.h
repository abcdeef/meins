#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#define GPIO_IN  0
#define GPIO_OUT 1

#define GPIO_LOW  0
#define GPIO_HIGH 1

#define BL_ON "0"
#define BL_OFF "1"

#define GPIO_PIN 21

#define MAXBUFFER 100

int gpio_export(int pin);
int gpio_unexport(int pin);
int gpio_direction(int pin, int dir);
int gpio_read(int pin);
int gpio_write(int pin, int value);

int bl_write(char *value);

#endif /* GPIO_H */

