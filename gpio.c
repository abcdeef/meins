#include "gpio.h"

#define GPIO_RS 0
#define GPIO_E  1
#define GPIO_DATA4 2
#define GPIO_DATA5 3
#define GPIO_DATA6 4
#define GPIO_DATA7 5

/* RS, E, DATA4, DATA5, DATA6, DATA7 */
int LCD_BUS[6] = {4, 17, 18, 22, 23, 24};
int GPIO_FD[6];

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

void gpio_lcd_init() {
    gpio_unexport(LCD_BUS, 6);
    gpio_export(LCD_BUS, 6);
    for (int n = 0; n < 6; n++)
        gpio_direction(LCD_BUS[n], GPIO_OUT);
    gpio_open(LCD_BUS, GPIO_FD, 6);

    gpio_lcd_send_byte(0x33, GPIO_LOW);
    gpio_lcd_send_byte(0x32, GPIO_LOW);
    gpio_lcd_send_byte(0x28, GPIO_LOW);
    gpio_lcd_send_byte(0x0C, GPIO_LOW);
    gpio_lcd_send_byte(0x06, GPIO_LOW);
    gpio_lcd_send_byte(0x01, GPIO_LOW);
}

void gpio_open(int *pins, int *fds, int count) {
    char path[MAXBUFFER];
    for (int n = 0; n < count; n++) {
        snprintf(path, MAXBUFFER, "/sys/class/gpio/gpio%d/value", pins[n]);
        fds[n] = open(path, O_WRONLY);
        if (fds[n] < 0) {
            perror("\rKann auf GPIO nicht schreiben (open)!\n");
        }
    }
}

void gpio_export(int *pins, int count) {
    char buffer[MAXBUFFER]; /* Output Buffer   */
    ssize_t bytes; /* Datensatzlaenge */
    int fd; /* Filedescriptor  */
    int res; /* Ergebnis von write */

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("\rKann nicht auf export schreiben!\n");
        return (-1);
    }
    for (int n = 0; n < count; n++) {
        bytes = snprintf(buffer, MAXBUFFER, "%d", pins[n]);
        res = write(fd, buffer, bytes);
        if (res < 0) {
            perror("\rKann Pin nicht aktivieren (write)!\n");
            return (-1);
        }
    }
    close(fd);
}

void gpio_unexport(int *pins, int count) {
    char buffer[MAXBUFFER]; /* Output Buffer   */
    ssize_t bytes; /* Datensatzlaenge */
    int fd; /* Filedescriptor  */
    int res; /* Ergebnis von write */

    fd = open("/sys/class/gpio/unexport", O_WRONLY);

    if (fd < 0) {
        printf("\rKann nicht auf unexport schreiben!\n");
        return (-1);
    }

    for (int n = 0; n < count; n++) {
        bytes = snprintf(buffer, MAXBUFFER, "%d", pins[n]);
        res = write(fd, buffer, bytes);
        if (res < 0) {
            perror("\rKann Pin nicht deaktivieren (write)!\n");
            return (-1);
        }
    }
    close(fd);
}

/* Datenrichtung GPIO-Pin festlegen
 * Schreiben Pinnummer nach /sys/class/gpioXX/direction
 * Richtung dir: 0 = Lesen, 1 = Schreiben
 * Ergebnis: 0 = O.K., -1 = Fehler
 */
int gpio_direction(int pin, int dir) {
    char path[MAXBUFFER]; /* Buffer fuer Pfad   */
    int fd; /* Filedescriptor     */
    int res; /* Ergebnis von write */

    snprintf(path, MAXBUFFER, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("\rKann Datenrichtung nicht setzen (open)!\n");
        return (-1);
    }
    switch (dir) {
        case GPIO_IN: res = write(fd, "in", 2);
            break;
        case GPIO_OUT: res = write(fd, "out", 3);
            break;
    }
    if (res < 0) {
        perror("\rKann Datenrichtung nicht setzen (write)!\n");
        return (-1);
    }
    close(fd);
    return (0);
}

/* vom GPIO-Pin lesen
 * Ergebnis: -1 = Fehler, 0/1 = Portstatus
 */
int gpio_read(int pin) {
    char path[MAXBUFFER]; /* Buffer fuer Pfad     */
    int fd; /* Filedescriptor       */
    char result[MAXBUFFER] = {0}; /* Buffer fuer Ergebnis */

    snprintf(path, MAXBUFFER, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("\rKann vom GPIO nicht lesen (open)!\n");
        return (-1);
    }
    if (read(fd, result, 3) < 0) {
        perror("\rKann vom GPIO nicht lesen (read)!\n");
        return (-1);
    }
    close(fd);
    return (atoi(result));
}

/* auf GPIO schreiben
 * Ergebnis: -1 = Fehler, 0 = O.K.
 */
int gpio_write(int pin, char *value) {
    char path[MAXBUFFER]; /* Buffer fuer Pfad   */
    int fd; /* Filedescriptor     */
    int res; /* Ergebnis von write */

    snprintf(path, MAXBUFFER, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("\rKann auf GPIO nicht schreiben (open)!\n");
        return (-1);
    }
    res = write(fd, value, 1);

    if (res < 0) {
        perror("\rKann auf GPIO nicht schreiben (write)!\n");
        return (-1);
    }
    close(fd);
    return (0);
}

/*
 * Delay (warten), Zeitangabe in Millisekunden
 */
void delay(unsigned long millis) {
    struct timespec ts;

    ts.tv_sec = millis / 1000;
    ts.tv_nsec = (millis % 1000) * 1000000L;

    int err = nanosleep(&ts, (struct timespec *) NULL);
}

void gpio_lcd_send_byte(char bits, char *mode) {
    write(GPIO_FD[GPIO_RS], mode, 1);
    write(GPIO_FD[GPIO_DATA4], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA5], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA6], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA7], GPIO_LOW, 1);

    if ((bits & 0x10) == 0x10)
        write(GPIO_FD[GPIO_DATA4], GPIO_HIGH, 1);
    if ((bits & 0x20) == 0x20)
        write(GPIO_FD[GPIO_DATA5], GPIO_HIGH, 1);
    if ((bits & 0x40) == 0x40)
        write(GPIO_FD[GPIO_DATA6], GPIO_HIGH, 1);
    if ((bits & 0x80) == 0x80)
        write(GPIO_FD[GPIO_DATA7], GPIO_HIGH, 1);

    delay(E_DELAY);
    write(GPIO_FD[GPIO_E], GPIO_HIGH, 1);
    delay(E_PULSE);
    write(GPIO_FD[GPIO_E], GPIO_LOW, 1);
    delay(E_DELAY);

    write(GPIO_FD[GPIO_DATA4], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA5], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA6], GPIO_LOW, 1);
    write(GPIO_FD[GPIO_DATA7], GPIO_LOW, 1);

    if ((bits & 0x01) == 0x01)
        write(GPIO_FD[GPIO_DATA4], GPIO_HIGH, 1);
    if ((bits & 0x02) == 0x02)
        write(GPIO_FD[GPIO_DATA5], GPIO_HIGH, 1);
    if ((bits & 0x04) == 0x04)
        write(GPIO_FD[GPIO_DATA6], GPIO_HIGH, 1);
    if ((bits & 0x08) == 0x08)
        write(GPIO_FD[GPIO_DATA7], GPIO_HIGH, 1);

    delay(E_DELAY);
    write(GPIO_FD[GPIO_E], GPIO_HIGH, 1);
    delay(E_PULSE);
    write(GPIO_FD[GPIO_E], GPIO_LOW, 1);
    delay(E_DELAY);
}
