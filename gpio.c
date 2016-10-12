#include "gpio.h"

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

int gpio_export(int pin) {
    char buffer[MAXBUFFER]; /* Output Buffer   */
    ssize_t bytes; /* Datensatzlaenge */
    int fd; /* Filedescriptor  */
    int res; /* Ergebnis von write */

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("\rKann nicht auf export schreiben!\n");
        return (-1);
    }
    bytes = snprintf(buffer, MAXBUFFER, "%d", pin);
    res = write(fd, buffer, bytes);
    if (res < 0) {
        perror("\rKann Pin nicht aktivieren (write)!\n");
        return (-1);
    }
    close(fd);
    return (0);
}

/* GPIO-Pin deaktivieren
 * Schreiben der Pinnummer nach /sys/class/gpio/unexport
 * Ergebnis: 0 = O.K., -1 = Fehler
 */
int gpio_unexport(int pin) {
    char buffer[MAXBUFFER];
    ssize_t bytes;
    int fd;
    int res;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        return (-1);
    }
    bytes = snprintf(buffer, MAXBUFFER, "%d", pin);
    res = write(fd, buffer, bytes);
    close(fd);
    return (0);
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
int gpio_write(int pin, int value) {
    char path[MAXBUFFER]; /* Buffer fuer Pfad   */
    int fd; /* Filedescriptor     */
    int res; /* Ergebnis von write */

    snprintf(path, MAXBUFFER, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("\rKann auf GPIO nicht schreiben (open)!\n");
        return (-1);
    }
    switch (value) {
        case GPIO_LOW: res = write(fd, "0", 1);
            break;
        case GPIO_HIGH: res = write(fd, "1", 1);
            break;
    }
    if (res < 0) {
        perror("\rKann auf GPIO nicht schreiben (write)!\n");
        return (-1);
    }
    close(fd);
    return (0);
}