/*
 * obd.c
 *
 *  Created on: 17.08.2016
 *      Author: florian
 */
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include "obd.h"

#define OBDCMD_NEWLINE "\r"
#define OBDCOMM_TIMEOUT 10000000l

int modifybaud(int fd, long baudrate);

float obdConvert_04(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_05(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A - 40.0f;
}

float obdConvert_06_09(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A - 128.0f) * 100.0f / 128.0f;
}

float obdConvert_0A(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 3;
}

float obdConvert_0B(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A;
}

float obdConvert_0C(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (((float) A * 256.0)+(float) B) / 4.0f;
}

float obdConvert_0D(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A;
}

float obdConvert_0E(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A / 2.0f - 64.0f;
}

float obdConvert_0F(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A - 40.0f;
}

float obdConvert_10(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (((float) A * 256.0f)+(float) B) / 100;
}

float obdConvert_11(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_14_1B(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 0.005f;
}

float obdConvert_1F(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 256.0f + (float) B;
}

float obdConvert_21(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 256.0f + (float) B;
}

float obdConvert_22(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*0.079f;
}

float obdConvert_23(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*10.0f;
}

float obdConvert_24_2B(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*0.0000305f;
}

float obdConvert_2C(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_2D(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 0.78125f - 100.0f;
}

float obdConvert_2E(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_2F(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_30(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A;
}

float obdConvert_31(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 256.0f + (float) B;
}

float obdConvert_32(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B) / 4.0f - 8192.0f;
}

float obdConvert_33(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A;
}

float obdConvert_34_3B(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*0.0000305f;
}

float obdConvert_3C_3F(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B) / 10.0f - 40.0f;
}

float obdConvert_42(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B) / 1000.0f;
}

float obdConvert_43(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*100.0f / 255.0f;
}

float obdConvert_44(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return ((float) A * 256.0f + (float) B)*0.0000305f;
}

float obdConvert_45(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_46(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A - 40.0f;
}

float obdConvert_47_4B(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_4C(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

float obdConvert_4D(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 256.0f + (float) B;
}

float obdConvert_4E(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 256.0f + (float) B;
}

float obdConvert_52(unsigned int A, unsigned int B, unsigned int C, unsigned int D) {
    return (float) A * 100.0f / 255.0f;
}

int obdRevConvert_04(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_05(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (val + 40);
    return 1;
}

int obdRevConvert_06_09(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (128.0f * val / 100.0f) + 128;
    return 1;
}

int obdRevConvert_0A(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (val / 3.0f);
    return 1;
}

int obdRevConvert_0B(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) val;
    return 1;
}

int obdRevConvert_0C(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val * 4;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_0D(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) val;
    return 1;
}

int obdRevConvert_0E(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((val + 64.0f) * 2.0f);
    return 1;
}

int obdRevConvert_0F(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (val + 40);
    return 1;
}

int obdRevConvert_10(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val * 100;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_11(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_14_1B(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (val / 0.005f);
    return 1;
}

int obdRevConvert_1F(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_21(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_22(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val / 0.079f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_23(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val / 10.0f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_24_2B(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val / 0.0000305f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_2C(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_2D(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((val + 100.0f) / 0.78125f);
    return 1;
}

int obdRevConvert_2E(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_2F(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_30(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) val;
    return 1;
}

int obdRevConvert_31(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_32(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val + 8192.0f;
    val = val * 4.0f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_33(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) val;
    return 1;
}

int obdRevConvert_34_3B(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val / 0.0000305f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_3C_3F(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val + 40.0f;
    val = val * 10.0f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_42(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val * 1000.0f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_43(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val * 255.0f / 100.0f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_44(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    val = val / 0.0000305f;
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_45(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_46(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (val + 40);
    return 1;
}

int obdRevConvert_47_4B(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_4C(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

int obdRevConvert_4D(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_4E(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) ((long) val / 256);
    *B = (unsigned int) ((long) val % 256);
    return 2;
}

int obdRevConvert_52(float val, unsigned int *A, unsigned int *B, unsigned int *C, unsigned int *D) {
    *A = (unsigned int) (255.0f * val / 100.0f);
    return 1;
}

long guessbaudrate(int fd) {
    const char testcmd[] = "0100\r\n";
    long guesses[] = {9600, 38400, 115200, 57600, 2400, 1200};
    int i;

    printf("Baudrate guessing: ");

    for (i = 0; i < sizeof (guesses) / sizeof (guesses[0]); i++) {
        long guess = guesses[i];

        printf("%li, ", guess);
        if (-1 == modifybaud(fd, guess)) {
            fprintf(stderr, "Error setting baudrate %li\n", guess);
            return -1;
        }

        char retbuf[1024];
        memset(retbuf, '\0', sizeof (retbuf));

        int nbytes = write(fd, testcmd, sizeof (testcmd));
        if (-1 == nbytes) {
            perror("Error writing to serial port guessing baudrate");
            return -1;
        }

        sleep(1); // CHEESY
        nbytes = read(fd, retbuf, sizeof (retbuf));
        if (-1 == nbytes) {
            perror("Error reading from serial port guessing baudrate");
            return -1;
        }
        if (NULL != strstr(retbuf, ">")) {
            printf("success at %li\n", guess);
            return guess;
        }
    }
    fprintf(stderr, "Couldn't guess baudrate\n");
    return -1;
}

int modifybaud(int fd, long baudrate) {
    // printf("Baudrate: %i\n", (int)baudrate);
    if (baudrate == -1)
        return 0;

    if (baudrate == 0) {
        return guessbaudrate(fd) > 0 ? 0 : 1;
    }

    struct termios options;
    if (0 != tcgetattr(fd, &options)) {
        perror("tcgetattr");
        return -1;
    }

    speed_t speedreq = B38400;

    switch (baudrate) {
#ifdef B4000000
        case 4000000:
            speedreq = B4000000;
            break;
#endif //B4000000
#ifdef B3500000
        case 3500000:
            speedreq = B3500000;
            break;
#endif //B3500000
#ifdef B3000000
        case 3000000:
            speedreq = B3000000;
            break;
#endif //B3000000
#ifdef B2500000
        case 2500000:
            speedreq = B2500000;
            break;
#endif //B2500000
#ifdef B2000000
        case 2000000:
            speedreq = B2000000;
            break;
#endif //B2000000
#ifdef B1500000
        case 1500000:
            speedreq = B1500000;
            break;
#endif //B1500000
#ifdef B1152000
        case 1152000:
            speedreq = B1152000;
            break;
#endif //B1152000
#ifdef B1000000
        case 1000000:
            speedreq = B1000000;
            break;
#endif //B1000000
#ifdef B9210600
        case 9210600:
            speedreq = B9210600;
            break;
#endif //B9210600
#ifdef B576000
        case 576000:
            speedreq = B576000;
            break;
#endif //B576000
#ifdef B500000
        case 500000:
            speedreq = B500000;
            break;
#endif //B500000
#ifdef B460800
        case 460800:
            speedreq = B460800;
            break;
#endif //B460800
#ifdef B230400
        case 230400:
            speedreq = B230400;
            break;
#endif //B230400
#ifdef B115200
        case 115200:
            speedreq = B115200;
            break;
#endif //B115200
#ifdef B76800
        case 76800:
            speedreq = B76800;
            break;
#endif //B76800
#ifdef B57600
        case 57600:
            speedreq = B57600;
            break;
#endif //B57600
        case 38400:
            speedreq = B38400;
            break;
#ifdef B28800
        case 28800:
            speedreq = B28800;
            break;
#endif //B28800
        case 19200:
            speedreq = B19200;
            break;
#ifdef B14400
        case 14400:
            speedreq = B14400;
            break;
#endif //B14400
        case 9600:
            speedreq = B9600;
            break;
#ifdef B7200
        case 7200:
            speedreq = B7200;
            break;
#endif //B7200
        case 4800:
            speedreq = B4800;
            break;
        case 2400:
            speedreq = B2400;
            break;
        case 1200:
            speedreq = B1200;
            break;
        case 600:
            speedreq = B600;
            break;
        case 300:
            speedreq = B300;
            break;
        case 150:
            speedreq = B150;
            break;
        case 134:
            speedreq = B134;
            break;
        case 110:
            speedreq = B110;
            break;
        case 75:
            speedreq = B75;
            break;
        case 50:
            speedreq = B50;
            break;
        case 0: // Don't look at me like *I* think it's a good idea
            speedreq = B0;
            break;
        default:
            fprintf(stderr, "Uknown baudrate: %li\n", baudrate);
            return -1;
            break;

    }

    if (0 != cfsetispeed(&options, speedreq)) {
        perror("cfsetispeed");
        return -1;
    }

    if (0 != cfsetospeed(&options, speedreq)) {
        perror("cfsetospeed");
        return -1;
    }

    if (0 != tcsetattr(fd, TCSANOW, &options)) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}
#define SERIAL_OUT 1
#define SERIAL_IN 0
static FILE *seriallog = NULL;

static void appendseriallog(const char *line, int out) {
    if (NULL != seriallog) {
        char timestr[200];
        time_t t;
        struct tm *tmp;

        t = time(NULL);
        tmp = localtime(&t);
        if (tmp == NULL) {
            snprintf(timestr, sizeof (timestr), "Unknown time");
        }

        if (strftime(timestr, sizeof (timestr), "%H:%M:%S", tmp) == 0) {
            snprintf(timestr, sizeof (timestr), "Unknown time");
        }

        fprintf(seriallog, "%s(%s): '%s'\n", timestr, out == SERIAL_OUT ? "out" : "in", line);
        fflush(seriallog);
    }
}

int readserialdata(int fd, char *buf, int n) {
    char *bufptr = buf; // current position in buf

    struct timeval start, curr; // For timing out
    if (0 != gettimeofday(&start, NULL)) {
        perror("Couldn't gettimeofday");
        return -1;
    }
    memset((void *) buf, '\0', n);
    int retval = 0; // Value to return
    int nbytes; // Number of bytes read
    do {
        nbytes = read(fd, bufptr, buf + n - bufptr - 1);
        if (-1 == nbytes && EAGAIN != errno) {
            perror("Error in readserialdata");
        }
        if (-1 != nbytes) {
            printf("Read bytes '%s'\n", bufptr);
            retval += nbytes; // Increment bytecount
            bufptr += nbytes; // Move pointer forward
        }
        if (0 != gettimeofday(&curr, NULL)) {
            perror("Couldn't gettimeofday");
            return -1;
        }
        if (OBDCOMM_TIMEOUT < 1000000l * (curr.tv_sec - start.tv_sec) + (curr.tv_usec - start.tv_usec)) {
            printf("Timeout!\n");
            return -1;
        }
    } while (retval == 0 || bufptr[-1] != '>');

    appendseriallog(buf, SERIAL_IN);
    return retval;
}

void readtonextprompt(int fd) {
    char retbuf[4096]; // Buffer to store returned stuff
    readserialdata(fd, retbuf, sizeof (retbuf));
}

void blindcmd(int fd, const char *cmd, int no_response) {
    char outstr[1024];
    printf("CMD: %s\n", cmd);
    snprintf(outstr, sizeof (outstr), "%s%s\0", cmd, OBDCMD_NEWLINE);
    //appendseriallog(outstr, SERIAL_OUT);
    write(fd, outstr, strlen(outstr));
    if (0 != no_response) {
        sleep(1);
        readtonextprompt(fd);
    }
}

static long attempt_upgradebaudrate(int fd, long rate, long previousrate) {
    char brd_cmd[64];

    int timeout = 500; // this many ms
    int brt_val = timeout / 5; // ATBRT wants it in increments of five ms
    char brt_cmd[64];
    snprintf(brt_cmd, sizeof (brt_cmd), "ATBRT%02X", brt_val);

    blindcmd(fd, brt_cmd, 1);
    // printf("%s\n", brt_cmd);

    int brd_val = 4000000l / rate;
    snprintf(brd_cmd, sizeof (brd_cmd), "ATBRD%02X" OBDCMD_NEWLINE, brd_val);
    printf("%li [%02X]:", rate, brd_val);

    int nbytes = write(fd, brd_cmd, strlen(brd_cmd));
    if (-1 == nbytes) {
        printf("\n");
        perror("Error writing to serial port upgrading baudrate");
        return -1;
    }

    char elm_response[64];
    memset(elm_response, '\0', sizeof (elm_response));
    int readcount = 0;
    while (NULL == strstr(elm_response, "?") && NULL == strstr(elm_response, "OK")) {
        nbytes = read(fd, elm_response + readcount, sizeof (elm_response) - readcount);
        // printf("Read \"%s\", %i\n", elm_response, readcount);
        if (-1 == nbytes && errno != EAGAIN) {
            printf("\n");
            perror("Error reading[1] from serial port upgrading baudrate");
            return -1;
        }
        if (-1 != nbytes) {
            readcount += nbytes;
        }
    }

    // printf("\nreadcount: %i: \"%s\"\n", readcount, elm_response);
    if (NULL != strstr(elm_response, "OK")) {
        // printf("got OK");
        if (-1 == modifybaud(fd, rate)) {
            printf("Error modifying baudrate to %li\n", rate);
            return -1;
        }

        char elm_response2[256];
        memset(elm_response2, '\0', sizeof (elm_response2));

        //		usleep(timeout * 1000); // Give it a chance to upgrade

        struct timeval start, end;
        gettimeofday(&start, NULL);

        int readcount2 = 0;
        while (readcount2 < 5 && NULL == strstr(elm_response2, ">")) {
            nbytes = read(fd, elm_response2 + readcount2, sizeof (elm_response2) - readcount2);

            if (-1 == nbytes && EAGAIN != errno) {
                perror("Error reading[2] from serial port upgrading baudrate");
                modifybaud(fd, previousrate);
                return -1;
            }

            gettimeofday(&end, NULL);

            long timediff = (end.tv_sec - start.tv_sec) * 1000000l + (end.tv_usec - start.tv_usec);
            if (2000l * timeout < timediff) {
                printf("timed out\n");
                break;
            }
            if (-1 != nbytes) {
                readcount2 += nbytes;
            }
        }
        // printf("\n\"%s\"\n", elm_response2);
        if (NULL != strstr(elm_response2, "ELM")) {
            write(fd, OBDCMD_NEWLINE, strlen(OBDCMD_NEWLINE));
            printf("success, ");
            return 0;
        } else {
            printf("fail [no ELM], ");
            modifybaud(fd, previousrate);
            return -1;
        }
    } else {
        printf("fail [no OK], ");
        modifybaud(fd, previousrate);
        return -1;
    }
}

long upgradebaudrate(int fd, long baudrate_target, long current_baudrate) {
    // AT BRD is discussed on pages 9-10,48-49 of the ELM327 datasheet

    // Temporarily make sure this is nonblocking
    int old_flags = fcntl(fd, F_GETFL);
    if (-1 == fcntl(fd, F_SETFL, O_NONBLOCK)) {
        perror("fcntl");
    }

    // Try these speeds
    long speeds[] = {38400, 57600, 115200, 230400, 460800, 500000, 576000};

    int i;

    if (-1 == baudrate_target) {
        return 0;
    }

    long current_best = -1;

    printf("Baudrate upgrading: ");
    if (0 < baudrate_target) {
        int retval = (0 == attempt_upgradebaudrate(fd, baudrate_target, current_baudrate) ? baudrate_target : -1);
        fcntl(fd, F_SETFL, old_flags);
        printf("\n");
        return retval;
    }

    for (i = 0; i < sizeof (speeds) / sizeof (speeds[0]); i++) {
        if (0 == attempt_upgradebaudrate(fd, speeds[i], current_best)) {
            current_best = speeds[i];
        }
    }

    printf("\n");
    fcntl(fd, F_SETFL, old_flags);
    return current_best;
}

static enum obd_serial_status parseobdline(const char *line, unsigned int mode, unsigned int cmd, unsigned int *retvals,
        unsigned int retvals_size, unsigned int *vals_read, int quiet) {

    unsigned int response; // Response. Should always be 0x40 + mode
    unsigned int cmdret; // Mode returned [should be the same as cmd]

    int count; // number of retvals successfully sscanf'd

    unsigned int currbytes[20]; // Might be a long line if there's a bunch of errors

    int have_cmd; // Set if we expect a cmd thing returned
    if (0x03 == mode || 0x04 == mode) {
        // Don't look for the second "cmd" item in some modes.
        have_cmd = 0;
    } else {
        have_cmd = 1;
    }

    // Number of items to subtract from count to figure out
    //   how many items in curr_bytes we found
    int count_sub;
    if (have_cmd) {
        count_sub = 2;
        count =
                sscanf(line, "%2x %2x "
                "%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x "
                "%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x", &response, &cmdret, currbytes, currbytes + 1, currbytes + 2, currbytes
                + 3, currbytes + 4, currbytes + 5, currbytes + 6, currbytes + 7, currbytes + 8, currbytes + 9, currbytes + 10, currbytes
                + 11, currbytes + 12, currbytes + 13, currbytes + 14, currbytes + 15, currbytes + 16, currbytes + 17, currbytes
                + 18, currbytes + 19);
    } else {
        count_sub = 1;
        count =
                sscanf(line, "%2x "
                "%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x "
                "%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x", &response, currbytes, currbytes + 1, currbytes + 2, currbytes + 3, currbytes
                + 4, currbytes + 5, currbytes + 6, currbytes + 7, currbytes + 8, currbytes + 9, currbytes + 10, currbytes + 11, currbytes
                + 12, currbytes + 13, currbytes + 14, currbytes + 15, currbytes + 16, currbytes + 17, currbytes + 18, currbytes
                + 19);
    }

    if (count <= 2) {
        if (!quiet)
            fprintf(stderr, "Couldn't parse line for %02X %02X: %s\n", mode, cmd, line);
        return OBD_UNPARSABLE;
    }

    if (response != 0x40 + mode) {
        if (!quiet)
            fprintf(stderr, "Unsuccessful mode response for %02X %02X: %s\n", mode, cmd, line);
        return OBD_INVALID_RESPONSE;
    }

    if (have_cmd && cmdret != cmd) {
        if (!quiet)
            fprintf(stderr, "Unsuccessful cmd response for %02X %02X: %s\n", mode, cmd, line);
        return OBD_INVALID_MODE;
    }

    int i;
    for (i = 0; i < count - count_sub && i < retvals_size; i++) {
        retvals[i] = currbytes[i];
    }

    *vals_read = count - count_sub;

    return OBD_SUCCESS;
}

enum obd_serial_status getobdbytes(int fd, unsigned int mode, unsigned int cmd, int numbytes_expected, unsigned int *retvals,
        unsigned int retvals_size, int *numbytes_returned, int quiet) {

    char sendbuf[20]; // Command to send
    int sendbuflen; // Number of bytes in the send buffer

    char retbuf[4096]; // Buffer to store returned stuff

    int nbytes; // Number of bytes read

    if (mode == 0x03 || mode == 0x04) {
        sendbuflen = snprintf(sendbuf, sizeof (sendbuf), "%02X" OBDCMD_NEWLINE, mode);
    } else {
        if (0 == numbytes_expected) {
            sendbuflen = snprintf(sendbuf, sizeof (sendbuf), "%02X%02X" OBDCMD_NEWLINE, mode, cmd);
        } else {
            sendbuflen = snprintf(sendbuf, sizeof (sendbuf), "%02X%02X%01X" OBDCMD_NEWLINE, mode, cmd, numbytes_expected);
        }
    }

    appendseriallog(sendbuf, SERIAL_OUT);
    if (write(fd, sendbuf, sendbuflen) < sendbuflen) {
        return OBD_ERROR;
    }

    nbytes = readserialdata(fd, retbuf, sizeof (retbuf));
    if (0 == nbytes) {
        if (!quiet)
            fprintf(stderr, "No data at all returned from serial port\n");
        return OBD_ERROR;
    } else if (-1 == nbytes) {
        if (!quiet)
            fprintf(stderr, "Error reading from serial port\n");
        return OBD_ERROR;
    }

    // First some sanity checks on the data

    if (NULL != strstr(retbuf, "NO DATA")) {
        if (!quiet)
            fprintf(stderr, "OBD reported NO DATA for %02X %02X: %s\n", mode, cmd, retbuf);
        return OBD_NO_DATA;
    }

    if (0 != strstr(retbuf, "?")) {
        if (!quiet)
            fprintf(stderr, "OBD reported ? for %02X %02X: %s\n", mode, cmd, retbuf);
        return OBD_NO_DATA;
    }

    if (NULL != strstr(retbuf, "UNABLE TO CONNECT")) {
        if (!quiet)
            fprintf(stderr, "OBD reported UNABLE TO CONNECT for %02X %02X: %s\n", mode, cmd, retbuf);
        return OBD_UNABLE_TO_CONNECT;
    }

    /*
     Good grief, this is ugly.
     1) We look go through the output line by line [strtokking]
     2) For each line, if it's a regular line, parse it
     3) If it has a colon near the start, it means it's a multi-line response
     4) So go into crazy C string handling mode.
     */

    char *line = strtok(retbuf, "\r\n>");

    int values_returned = 0;
    enum obd_serial_status ret = OBD_ERROR;
    while (NULL != line) {
        char *colon;
        int joined_lines = 0; // Set if we joined some lines together
        char longline[1024] = "\0"; // Catenate other lines into this

        char *parseline = line; // The line to actually parse.

        while (NULL != line && NULL != (colon = strstr(line, ":"))) {
            // printf("Colon line: %s\n", line);
            strncat(longline, colon + 1, sizeof (longline) - strlen(longline) - 1);
            parseline = longline;
            joined_lines = 1;
            line = strtok(NULL, "\r\n>");
        }
        // We gracefully handle these lines without
        //   needing to actually parse them
        if (3 >= strlen(parseline))
            continue;

        // printf("parseline: %s\n", parseline);

        unsigned int local_rets[20];
        unsigned int vals_read;

        ret = parseobdline(parseline, mode, cmd, local_rets, sizeof (local_rets) / sizeof (local_rets[0]), &vals_read, quiet);

        if (OBD_SUCCESS == ret) {
            int i;
            for (i = 0; i < vals_read; i++, values_returned++) {
                retvals[values_returned] = local_rets[i];
            }
            break;
        }

        if (0 == joined_lines) {
            // If we joined some lines together, this strtok was already done
            line = strtok(NULL, "\r\n>");
        }
    }
    *numbytes_returned = values_returned;
    if (0 == values_returned)
        return ret;
    return OBD_SUCCESS;
}

enum obd_serial_status getobdvalue(int fd, unsigned int cmd, float *ret, int numbytes, OBDConvFunc conv) {
    int numbytes_returned;
    unsigned int obdbytes[4];

    enum obd_serial_status ret_status = getobdbytes(fd, 0x01, cmd, numbytes, obdbytes, sizeof (obdbytes) / sizeof (obdbytes[0]), &numbytes_returned, 0);
    if (OBD_SUCCESS != ret_status)
        return ret_status;

    if (NULL == conv) {
        int i;
        *ret = 0;
        for (i = 0; i < numbytes_returned; i++) {
            *ret = *ret * 256;
            *ret = *ret + obdbytes[i];
        }
    } else {
        *ret = conv(obdbytes[0], obdbytes[1], obdbytes[2], obdbytes[3]);
    }
    return OBD_SUCCESS;
}

int init_OBD(char *serial) {
    struct termios options;

    long baudrate = 9600; // 0 - AUTO
    long baudrate_target = -1;

    int fd = open(serial, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        printf("\serial-dev: %s nicht gefunden\n", serial);
        return -1;
    } else {
        printf("\rserial-dev: %s geöffnet\n", serial);
    }
    tcgetattr(fd, &options);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= !(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= !(OPOST);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 100;

    tcsetattr(fd, TCSANOW, &options);

    if (0 != modifybaud(fd, baudrate)) {
        fprintf(stderr, "Error modifying baudrate.\n");
        close(fd);
        return -1;
    }

    // Reset the device.
    blindcmd(fd, "ATZ", 1);

    if (0 > upgradebaudrate(fd, baudrate_target, baudrate)) {
        fprintf(stderr, "Error upgrading baudrate. Continuing, but may suffer issues\n");
    }
   
    blindcmd(fd, "0100", 1);
    
     
    // Disable command echo [elm327]
    blindcmd(fd, "ATE0", 1);
    // Disable linefeeds [an extra byte of speed can't hurt]
    blindcmd(fd, "ATL0", 1);
    // Don't insert spaces [readability is for ugly bags of mostly water]
    blindcmd(fd, "ATS0", 1);
    // Then do it again to make sure the command really worked
    //blindcmd(fd, "0100", 1);
    
       
    blindcmd(fd, "ATTP5", 1);
    blindcmd(fd, "0100", 1);

     return fd;
}