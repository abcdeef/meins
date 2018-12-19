#include "gpio.h"
#include "bcm2835.h"

extern int meins_delay(unsigned long millis);

static int32_t BpiPin[] = {
    257, 256, //BCM GPIO0,1
    53, 52, //BCM GPIO2,3
    226, 35, //BCM GPIO4,5
    277, 270, //BCM GPIO6,7
    266, 269, //BCM GPIO8,9
    268, 267, //BCM GPIO10,11
    276, 45, //BCM GPIO12,13
    228, 229, //BCM GPIO14,15
    38, 275, //BCM GPIO16,17
    259, 39, //BCM GPIO18,19
    44, 40, //BCM GPIO20,21
    273, 244, //BCM GPIO22,23
    245, 272, //BCM GPIO24,25
    37, 274, //BCM GPIO26,27
};

int LCD_BUS[11] = {21, 20, 6, 5, 19, 9, 16, 10, 13, 22, 12};

typedef struct {
    uint8_t bcm;
    int32_t pin;
    uint32_t mask;
    uint32_t offset;
} T_GPIO;

T_GPIO a[11];

volatile unsigned int *gpio;
volatile uint32_t *sunxi_gpio = (volatile uint32_t *)MAP_FAILED;

#define GPIO_RS 0
#define GPIO_E  1
#define GPIO_RW 2
#define GPIO_DATA0 3
#define GPIO_DATA1 4
#define GPIO_DATA2 5
#define GPIO_DATA3 6
#define GPIO_DATA4 7
#define GPIO_DATA5 8
#define GPIO_DATA6 9
#define GPIO_DATA7 10

int bl_write(char *value) {
    int fd;
    int res;

    fd = open("/sys/class/backlight/rpi_backlight/bl_power", O_WRONLY);

    if (fd < 0) {
        perror("\rKann auf bl_power nicht schreiben (open)!\n");
        return (-1);
    }
    res = write(fd, value, 1);

    if (res < 0) {
        perror("\rKann auf bl_power nicht schreiben (write)!\n");
        return (-1);
    }
    close(fd);
    return (0);
}

int gpio_init(void) {
    int memfd = -1;
    int ok = 0;
    uint32_t mmap_base;
    uint32_t mmap_seek;
    volatile uint32_t *sunxi_tmp;

    // Open the master /dev/memory device
    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        return -1;
    }

    mmap_base = (SUNXI_GPIO_BASE & (~MAP_MASK));
    mmap_seek = (SUNXI_GPIO_BASE - mmap_base) >> 2;
    sunxi_gpio = (uint32_t *) mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, mmap_base);
    if (MAP_FAILED == sunxi_gpio) {
        return -1;
    }
    sunxi_tmp = sunxi_gpio;
    /*gpio register base address*/
    sunxi_gpio += mmap_seek;

    close(memfd);

    for (uint8_t n = 0; n < 11; n++) {
        a[n].bcm = LCD_BUS[n];
        a[n].pin = BpiPin[LCD_BUS[n]];
        a[n].offset = ((SUNXI_GPIO_DAT_OFFSET + (a[n].pin / 32) * 0x24) >> 2);
        a[n].mask = 0x01 << (a[n].pin % 32);
    }

    return 1;
}

uint32_t bcm2835_peri_read1(volatile uint32_t* paddr) {
    uint32_t ret = *paddr;
    *paddr; // Read without assigneing to an unused variable
    return ret;
}

void bcm2835_gpio_fsel1(uint8_t pin, uint8_t mode) {
    int32_t bpipin = BpiPin[pin];
    volatile uint32_t* paddr = sunxi_gpio + ((SUNXI_GPIO_CFG_OFFSET + (bpipin / 32) * 0x24 + ((bpipin % 32) / 8) *0x04) >> 2);
    uint8_t shift = (bpipin % 8) * 4;
    uint32_t mask = BCM2835_GPIO_FSEL_MASK << shift;
    uint32_t value = mode << shift;

    bcm2835_peri_set_bits1(paddr, value, mask);
}

void bcm2835_peri_set_bits1(volatile uint32_t* paddr, uint32_t value, uint32_t mask) {
    uint32_t v = bcm2835_peri_read1(paddr);
    v = (v & ~mask) | (value & mask); // Nur das entsprechende Bit setzten. Die anderen Bits müssen übernommen werden
    bcm2835_peri_write1(paddr, v);
}

void bcm2835_peri_write1(volatile uint32_t* paddr, uint32_t value) {
    *paddr = value;
    *paddr = value;
}

void bcm2835_gpio_set1(uint8_t pin) {
    int32_t bpipin = BpiPin[pin];

    volatile uint32_t* paddr = sunxi_gpio + ((SUNXI_GPIO_DAT_OFFSET + (bpipin / 32) * 0x24) >> 2);
    uint8_t shift = bpipin % 32;
    uint32_t mask = 0x01 << shift;
    uint32_t value = 1 << shift;
    bcm2835_peri_set_bits1(paddr, value, mask);

}

void bcm2835_gpio_clr1(uint8_t pin) {
    int32_t bpipin = BpiPin[pin];
    volatile uint32_t* paddr = sunxi_gpio + ((SUNXI_GPIO_DAT_OFFSET + (bpipin / 32) * 0x24) >> 2);
    uint8_t shift = bpipin % 32;
    uint32_t mask = 0x01 << shift;
    uint32_t value = 0 << shift;
    bcm2835_peri_set_bits1(paddr, value, mask);
}

void bcm2835_gpio_write1(uint8_t pin, uint8_t on) {
    if (on)
        bcm2835_gpio_set1(pin);
    else
        bcm2835_gpio_clr1(pin);
}

void gpio_lcd_send_byte(char bits, char *mode) {
    if (*mode == 48) {
        bcm2835_gpio_clr1(LCD_BUS[GPIO_RS]);
    } else {
        bcm2835_gpio_write1(LCD_BUS[GPIO_RS], HIGH);
    }
    /*
    LCD_BUS     bcm     offset  mask
    GPIO_RS     21      13      00000000000000000000000100000000 256
    GPIO_E      20      13      00000000000000000001000000000000 4096
    GPIO_DATA0  5       13      00000000000000000000000000001000 8
    GPIO_DATA1  19      13      00000000000000000000000010000000 128
    GPIO_DATA3  16      13      00000000000000000000000001000000 64
    GPIO_DATA5  13      13      00000000000000000010000000000000 8192
                                00000000000000000011000111001000 12744

    GPIO_RW     6       76      00000000001000000000000000000000 2097152
    GPIO_DATA2  9       76      00000000000000000010000000000000 8192
    GPIO_DATA4  10      76      00000000000000000001000000000000 4096
    GPIO_DATA6  22      76      00000000000000100000000000000000 131072
    GPIO_DATA7  12      76      00000000000100000000000000000000 1048576
                                00000000001100100011000000000000 3289088

     */
    volatile uint32_t* paddr = sunxi_gpio + 76; // ((SUNXI_GPIO_DAT_OFFSET + (BpiPin[6] / 32) * 0x24) >> 2);
    bcm2835_peri_set_bits1(paddr, 0, 3289088);

    paddr = sunxi_gpio + 13; // ((SUNXI_GPIO_DAT_OFFSET + (BpiPin[5] / 32) * 0x24) >> 2);
    bcm2835_peri_set_bits1(paddr, 0, 8392);


    paddr = sunxi_gpio + 13; //((SUNXI_GPIO_DAT_OFFSET + (BpiPin[5] / 32) * 0x24) >> 2);
    uint32_t value = 0;

    if ((bits & 0x01) == 0x01) {
        value |= 8;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA0], HIGH);
    }
    if ((bits & 0x02) == 0x02) {
        value |= 128;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA1], HIGH);
    }
    if ((bits & 0x08) == 0x08) {
        value |= 64;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA3], HIGH);
    }
    if ((bits & 0x20) == 0x20) {
        value |= 8192;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA5], HIGH);
    }
    bcm2835_peri_set_bits1(paddr, value, 8392);


    paddr = sunxi_gpio + 76; //((SUNXI_GPIO_DAT_OFFSET + (BpiPin[9] / 32) * 0x24) >> 2);
    value = 0;
    if ((bits & 0x04) == 0x04) {
        value |= 8192;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA2], HIGH);
    }
    if ((bits & 0x10) == 0x10) {
        value |= 4096;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA4], HIGH);
    }
    if ((bits & 0x40) == 0x40) {
        value |= 131072;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA6], HIGH);
    }
    if ((bits & 0x80) == 0x80) {
        value |= 1048576;
        //bcm2835_gpio_write1(LCD_BUS[GPIO_DATA7], HIGH);
    }
    bcm2835_peri_set_bits1(paddr, value, 1191936);

    meins_delay(E_DELAY);
    bcm2835_gpio_write1(LCD_BUS[GPIO_E], HIGH);
    meins_delay(E_PULSE);
    bcm2835_gpio_clr1(LCD_BUS[GPIO_E]);
    meins_delay(E_DELAY);
}

void gpio_lcd_shutdown(void) {
    gpio_lcd_send_byte(0x08, GPIO_LOW); // Display off
}

void gpio_lcd_init() {
    // 8-Bit Bus OLED
    for (uint_fast8_t n = 0; n < 11; n++) {
        bcm2835_gpio_fsel1(LCD_BUS[n], BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_fsel1(LCD_BUS[n], BCM2835_GPIO_FSEL_OUTP);
    }
    sleep(1);

    bcm2835_gpio_fsel1(17, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel1(18, BCM2835_GPIO_FSEL_INPT);
    sleep(1);

    gpio_lcd_send_byte(0x39, GPIO_LOW); // Function Set, western european character set,8-Bit
    gpio_lcd_send_byte(0x08, GPIO_LOW); // Display off
    gpio_lcd_send_byte(0x06, GPIO_LOW); // Entry mode set, increment cursor by 1 not shifting display
    gpio_lcd_send_byte(0x17, GPIO_LOW); // Character mode and internel power on (have to turnon internel power to get the best brightness)
    gpio_lcd_send_byte(0x01, GPIO_LOW); // Clear display
    gpio_lcd_send_byte(0x02, GPIO_LOW); // Return home
    gpio_lcd_send_byte(0x0C, GPIO_LOW); // Display on
}

/*
 Input für Pin 17 und 18 ermitteln. Beider liegen auf der gleichen Adresse!
 */
uint32_t gpio_get_button(void) {
    volatile uint32_t* paddr = sunxi_gpio + 76; // ((SUNXI_GPIO_DAT_OFFSET + ((bpipin=17 oder 18) / 32) * 0x24) >> 2)
    uint32_t v = bcm2835_peri_read1(paddr);
    return (v & 524296); // 10000000000000001000 = 10000000000000000000 -> PIN 17; 1000 -> PIN 18
}

//void gpio_button_led(uint_fast8_t port, uint_fast8_t mode) {
//    GPIO_SET = mode << port;
//}

void gpio_set_lcd_maske(uint_fast8_t display, float *speed) {
    gpio_lcd_send_byte(0x01, GPIO_LOW); // Clear display
    gpio_lcd_send_byte(0x0C, GPIO_LOW); // Display on

    if (display == 0) {
        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        gpio_lcd_send_byte('U', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 7, GPIO_LOW);
        gpio_lcd_send_byte('V', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 18, GPIO_LOW);
        gpio_lcd_send_byte(210, GPIO_HIGH); // "°"
        gpio_lcd_send_byte('C', GPIO_HIGH);

        if (*speed == 0.0f) {
            gpio_lcd_send_byte(LCD_LINE_1 + 11, GPIO_LOW);
            gpio_lcd_send_byte('L', GPIO_HIGH);
            gpio_lcd_send_byte('/', GPIO_HIGH);
            gpio_lcd_send_byte('H', GPIO_HIGH);
            gpio_lcd_send_byte(':', GPIO_HIGH);
        } else {
            gpio_lcd_send_byte(LCD_LINE_1 + 10, GPIO_LOW);
            gpio_lcd_send_byte('K', GPIO_HIGH);
            gpio_lcd_send_byte('M', GPIO_HIGH);
            gpio_lcd_send_byte('/', GPIO_HIGH);
            gpio_lcd_send_byte('L', GPIO_HIGH);
            gpio_lcd_send_byte(':', GPIO_HIGH);

        }
        //gpio_lcd_send_byte(LCD_LINE_2 + 19, GPIO_LOW);
        //gpio_lcd_send_byte('%', GPIO_HIGH);
    } else if (display == 1) {
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        gpio_lcd_send_byte('O', GPIO_HIGH);
        gpio_lcd_send_byte('2', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_1 + 11, GPIO_LOW);
        gpio_lcd_send_byte('V', GPIO_HIGH);


        gpio_lcd_send_byte(LCD_LINE_1 + 15, GPIO_LOW);
        gpio_lcd_send_byte('I', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte('i', GPIO_HIGH);
        gpio_lcd_send_byte('A', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 12, GPIO_LOW);
        gpio_lcd_send_byte('M', GPIO_HIGH);
        gpio_lcd_send_byte('A', GPIO_HIGH);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);
    } else if (display == 2) {
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte('I', GPIO_HIGH);
        gpio_lcd_send_byte('X', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_1 + 11, GPIO_LOW);
        gpio_lcd_send_byte('S', GPIO_HIGH);
        gpio_lcd_send_byte('A', GPIO_HIGH);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        gpio_lcd_send_byte('H', GPIO_HIGH);
        gpio_lcd_send_byte('D', GPIO_HIGH);
        gpio_lcd_send_byte('O', GPIO_HIGH);
        gpio_lcd_send_byte('P', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

    } else if (display == 3) {
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        gpio_lcd_send_byte('N', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);
    } else if (display == 4) {
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        gpio_lcd_send_byte('S', GPIO_HIGH);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_1 + 14, GPIO_LOW);
        gpio_lcd_send_byte('C', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        gpio_lcd_send_byte('L', GPIO_HIGH);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

    } else if (display == 5) {
        gpio_lcd_send_byte(LCD_LINE_1 + 3, GPIO_LOW);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte('E', GPIO_HIGH);
        gpio_lcd_send_byte('H', GPIO_HIGH);
        gpio_lcd_send_byte('L', GPIO_HIGH);
        gpio_lcd_send_byte('E', GPIO_HIGH);
        gpio_lcd_send_byte('R', GPIO_HIGH);
    } else if (display == 6) {
        gpio_lcd_send_byte(LCD_LINE_1 + 2, GPIO_LOW);
        gpio_lcd_send_byte('/', GPIO_HIGH);
        gpio_lcd_send_byte(LCD_LINE_1 + 7, GPIO_LOW);
        gpio_lcd_send_byte('/', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_1 + 17, GPIO_LOW);
        gpio_lcd_send_byte('F', GPIO_HIGH);
        gpio_lcd_send_byte('P', GPIO_HIGH);
        gpio_lcd_send_byte('S', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 3, GPIO_LOW);
        gpio_lcd_send_byte('/', GPIO_HIGH);
        gpio_lcd_send_byte(LCD_LINE_2 + 7, GPIO_LOW);
        gpio_lcd_send_byte('/', GPIO_HIGH);
        gpio_lcd_send_byte(LCD_LINE_2 + 18, GPIO_LOW);
        gpio_lcd_send_byte(210, GPIO_HIGH); // "°"
        gpio_lcd_send_byte('C', GPIO_HIGH);
    }
}