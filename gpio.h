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

#define BL_ON "0"
#define BL_OFF "1"

#define GPIO_LOW "0"
#define GPIO_HIGH "1"

#define MAXBUFFER 100

#define LCD_LINE_1  0x80    
#define LCD_LINE_2  0xC0 
#define E_PULSE  8
#define E_DELAY  8

void gpio_lcd_init();
void gpio_open(int *pins, int *fds, int count);
void gpio_export(int *pins, int count);
void gpio_unexport(int *pins, int count);
int gpio_direction(int pin, int dir);
int gpio_read(int pin);
int gpio_write(int pin, char *value);

int bl_write(char *value);

void delay(unsigned long millis);

void gpio_lcd_send_byte(char bits, char *mode);
void gpio_lcd_init();


#endif /* GPIO_H */

