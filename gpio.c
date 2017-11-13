#include "gpio.h"

extern int delay(unsigned long millis);

int mem_fd;
void *gpio_map;
volatile unsigned int *gpio;
#define BLOCK_SIZE (4*1024)
#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE  (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
//#define GET_GPIO(g) (*(gpio+13)&(1<<g))>>g 
#define GET_GPIO(g) *(gpio+13)


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

/* RS, E, RW, DATA0, DATA1, DATA2, DATA3, DATA4, DATA5, DATA6, DATA7 */
int LCD_BUS[11] = {20, 19, 25, 24, 16, 23, 13, 22, 12, 27, 6};
//int GPIO_FD[11];

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

void gpio_init() {
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }

    gpio_map = mmap(
            NULL, //Any adddress in our space will do
            BLOCK_SIZE, //Map length
            PROT_READ | PROT_WRITE, // Enable reading & writting to mapped memory
            MAP_SHARED, //Shared with other processes
            mem_fd, //File to map
            GPIO_BASE //Offset to GPIO peripheral
            );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int) gpio_map); //errno also set!
        exit(-1);
    }

    gpio = (volatile unsigned *) gpio_map;
}

#ifdef __RASPI__

void gpio_lcd_send_byte(char bits, char *mode) {
    if (*mode == 48)
        GPIO_CLR = 1 << LCD_BUS[GPIO_RS];
    else
        GPIO_SET = 1 << LCD_BUS[GPIO_RS];

    GPIO_CLR = 1 << LCD_BUS[GPIO_RW];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA0];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA1];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA2];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA3];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA4];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA5];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA6];
    GPIO_CLR = 1 << LCD_BUS[GPIO_DATA7];

    if ((bits & 0x01) == 0x01)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA0];
    if ((bits & 0x02) == 0x02)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA1];
    if ((bits & 0x04) == 0x04)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA2];
    if ((bits & 0x08) == 0x08)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA3];
    if ((bits & 0x10) == 0x10)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA4];
    if ((bits & 0x20) == 0x20)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA5];
    if ((bits & 0x40) == 0x40)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA6];
    if ((bits & 0x80) == 0x80)
        GPIO_SET = 1 << LCD_BUS[GPIO_DATA7];

    delay(E_DELAY);
    GPIO_SET = 1 << LCD_BUS[GPIO_E];
    delay(E_PULSE);
    GPIO_CLR = 1 << LCD_BUS[GPIO_E];
    delay(E_DELAY);
}
#else
char oled_line1[20], oled_line2[20];
int oled_cursor = 0;

void gpio_lcd_send_byte(char bits, char *mode) {

    if (*mode == 0) {
        oled_cursor = bits;
    } else {
        if (oled_cursor >= LCD_LINE_2) {
            oled_line2[oled_cursor - LCD_LINE_2] = bits;
        } else {
            oled_line1[oled_cursor - LCD_LINE_1] = bits;
        }

        printf("\r|");
        for (int a = 0; a < 20; a++) {
            printf("%c", oled_line1[a]);
        }
        printf("|    |");
        for (int a = 0; a < 20; a++) {
            printf("%c", oled_line2[a]);
        }
        printf("|");
    }

}
#endif

void gpio_lcd_init() {
    gpio_init();

    // Buttons
    INP_GPIO(17);
    INP_GPIO(18);
    // BUTTON LED    
    INP_GPIO(5);
    OUT_GPIO(5);

    // 8-Bit Bus OLED
    for (uint_fast8_t n = 0; n < 11; n++) {
        INP_GPIO(LCD_BUS[n]);
        OUT_GPIO(LCD_BUS[n]);
    }

    gpio_lcd_send_byte(0x39, GPIO_LOW); // Function Set, western european character set,8-Bit
    gpio_lcd_send_byte(0x08, GPIO_LOW); // Display off
    gpio_lcd_send_byte(0x06, GPIO_LOW); // Entry mode set, increment cursor by 1 not shifting display
    gpio_lcd_send_byte(0x17, GPIO_LOW); // Character mode and internel power on (have to turnon internel power to get the best brightness)
    gpio_lcd_send_byte(0x01, GPIO_LOW); // Clear display
    gpio_lcd_send_byte(0x02, GPIO_LOW); // Return home
    gpio_lcd_send_byte(0x0C, GPIO_LOW); // Display on
}

#ifdef __RASPI__

void gpio_get_button(unsigned int *button) {
    *button = GET_GPIO(18);
}

void gpio_button_led(uint_fast8_t port, uint_fast8_t mode) {
    GPIO_SET = mode << port;
}
#else

void gpio_get_button(unsigned int *button) {
    //*button = GET_GPIO(18);
}

void gpio_button_led(uint_fast8_t port, uint_fast8_t mode) {
    //GPIO_SET = mode << port;
}
#endif

void gpio_set_lcd_maske(uint_fast8_t display) {
    gpio_lcd_send_byte(0x01, GPIO_LOW); // Clear display
    gpio_lcd_send_byte(0x0C, GPIO_LOW); // Display on

    if (display == 0) {
        gpio_lcd_send_byte(LCD_LINE_1 + 15, GPIO_LOW);
        gpio_lcd_send_byte('T', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        gpio_lcd_send_byte('U', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 7, GPIO_LOW);
        gpio_lcd_send_byte('V', GPIO_HIGH);

        gpio_lcd_send_byte(LCD_LINE_2 + 11, GPIO_LOW);
        gpio_lcd_send_byte('R', GPIO_HIGH);
        gpio_lcd_send_byte('H', GPIO_HIGH);
        gpio_lcd_send_byte(':', GPIO_HIGH);
        gpio_lcd_send_byte(LCD_LINE_2 + 19, GPIO_LOW);
        gpio_lcd_send_byte('%', GPIO_HIGH);
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

        /*        gpio_lcd_send_byte(LCD_LINE_1 + 13, GPIO_LOW);
                gpio_lcd_send_byte('T', GPIO_HIGH);
                gpio_lcd_send_byte(':', GPIO_HIGH);*/
    } else if (display == 4) {
        gpio_lcd_send_byte(0x08, GPIO_LOW);
    }
}