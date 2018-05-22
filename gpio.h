#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#define GPIO_IN  0
#define GPIO_OUT 1

#define BL_ON "0"
#define BL_OFF "1"

#define GPIO_LOW "0"
#define GPIO_HIGH "1"

#define MAXBUFFER 100

#define LCD_LINE_1  0x80    
#define LCD_LINE_2  0xC0 
#define E_PULSE  4
#define E_DELAY  4

int bl_write(char *value);
void gpio_init();
void gpio_lcd_send_byte(char bits, char *mode);
void gpio_lcd_init();
void gpio_get_button(unsigned int *button);
void gpio_set_lcd_maske(uint_fast8_t display, float *speed);
void gpio_button_led(uint_fast8_t port, uint_fast8_t mode);
void gpio_lcd_shutdown(void);

#endif /* GPIO_H */

