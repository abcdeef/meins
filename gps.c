#include "gps.h"
#include <syslog.h>

/**
 * Rescale a fixed-point value to a different scale. Rounds towards zero.
 */
static inline int_least32_t minmea_rescale(struct minmea_float *f, int_least32_t new_scale) {
    if (f->scale == 0)
        return 0;
    if (f->scale == new_scale)
        return f->value;
    if (f->scale > new_scale)
        return (f->value + ((f->value > 0) - (f->value < 0)) * f->scale / new_scale / 2) / (f->scale / new_scale);
    else
        return f->value * (new_scale / f->scale);
}

/**
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tofloat(struct minmea_float *f) {
    if (f->scale == 0)
        return NAN;
    return (float) f->value / (float) f->scale;
}

/**
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tocoord(struct minmea_float *f) {
    if (f->scale == 0)
        return NAN;
    int degrees = f->value / (f->scale * 100);
    int minutes = f->value % (f->scale * 100);
    return (float) degrees + (float) minutes / (60 * f->scale);
}

static int hex2int(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

uint8_t minmea_checksum(const char *sentence) {
    // Support senteces with or without the starting dollar sign.
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

bool minmea_check(const char *sentence, bool strict) {
    uint8_t checksum = 0x00;

    // Sequence length is limited.
    if (strlen(sentence) > MINMEA_MAX_LENGTH + 3)
        return false;

    // A valid sentence starts with "$".
    if (*sentence++ != '$')
        return false;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        checksum ^= *sentence++;

    // If checksum is present...
    if (*sentence == '*') {
        // Extract checksum.
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        // Check for checksum mismatch.
        if (checksum != expected)
            return false;
    } else if (strict) {
        // Discard non-checksummed frames in strict mode.
        return false;
    }

    // The only stuff allowed at this point is a newline.
    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}

static inline bool minmea_isfield(char c) {
    return isprint((unsigned char) c) && c != ',' && c != '*';
}

bool minmea_scan(const char *sentence, const char *format, ...) {
    bool result = false;
    bool optional = false;
    va_list ap;
    va_start(ap, format);

    const char *field = sentence;
#define next_field() \
    do { \
        /* Progress to the next field. */ \
        while (minmea_isfield(*sentence)) \
            sentence++; \
        /* Make sure there is a field there. */ \
        if (*sentence == ',') { \
            sentence++; \
            field = sentence; \
        } else { \
            field = NULL; \
        } \
    } while (0)

    while (*format) {
        char type = *format++;

        if (type == ';') {
            // All further fields are optional.
            optional = true;
            continue;
        }

        if (!field && !optional) {
            // Field requested but we ran out if input. Bail out.
            goto parse_error;
        }

        switch (type) {
            case 'c':
            { // Single character field (char).
                char value = '\0';

                if (field && minmea_isfield(*field))
                    value = *field;

                *va_arg(ap, char *) = value;
            }
                break;

            case 'd':
            { // Single character direction field (int).
                int value = 0;

                if (field && minmea_isfield(*field)) {
                    switch (*field) {
                        case 'N':
                        case 'E':
                            value = 1;
                            break;
                        case 'S':
                        case 'W':
                            value = -1;
                            break;
                        default:
                            goto parse_error;
                    }
                }

                *va_arg(ap, int *) = value;
            }
                break;

            case 'f':
            { // Fractional value with scale (struct minmea_float).
                int sign = 0;
                int_least32_t value = -1;
                int_least32_t scale = 0;

                if (field) {
                    while (minmea_isfield(*field)) {
                        if (*field == '+' && !sign && value == -1) {
                            sign = 1;
                        } else if (*field == '-' && !sign && value == -1) {
                            sign = -1;
                        } else if (isdigit((unsigned char) *field)) {
                            int digit = *field - '0';
                            if (value == -1)
                                value = 0;
                            if (value > (INT_LEAST32_MAX - digit) / 10) {
                                /* we ran out of bits, what do we do? */
                                if (scale) {
                                    /* truncate extra precision */
                                    break;
                                } else {
                                    /* integer overflow. bail out. */
                                    goto parse_error;
                                }
                            }
                            value = (10 * value) + digit;
                            if (scale)
                                scale *= 10;
                        } else if (*field == '.' && scale == 0) {
                            scale = 1;
                        } else if (*field == ' ') {
                            /* Allow spaces at the start of the field. Not NMEA
                             * conformant, but some modules do this. */
                            if (sign != 0 || value != -1 || scale != 0)
                                goto parse_error;
                        } else {
                            goto parse_error;
                        }
                        field++;
                    }
                }

                if ((sign || scale) && value == -1)
                    goto parse_error;

                if (value == -1) {
                    /* No digits were scanned. */
                    value = 0;
                    scale = 0;
                } else if (scale == 0) {
                    /* No decimal point. */
                    scale = 1;
                }
                if (sign)
                    value *= sign;

                *va_arg(ap, struct minmea_float *) = (struct minmea_float){value, scale};
            }
                break;

            case 'i':
            { // Integer value, default 0 (int).
                int value = 0;

                if (field) {
                    char *endptr;
                    value = strtol(field, &endptr, 10);
                    if (minmea_isfield(*endptr))
                        goto parse_error;
                }

                *va_arg(ap, int *) = value;
            }
                break;

            case 's':
            { // String value (char *).
                char *buf = va_arg(ap, char *);

                if (field) {
                    while (minmea_isfield(*field))
                        *buf++ = *field++;
                }

                *buf = '\0';
            }
                break;

            case 't':
            { // NMEA talker+sentence identifier (char *).
                // This field is always mandatory.
                if (!field)
                    goto parse_error;

                if (field[0] != '$')
                    goto parse_error;
                for (int f = 0; f < 5; f++)
                    if (!minmea_isfield(field[1 + f]))
                        goto parse_error;

                char *buf = va_arg(ap, char *);
                memcpy(buf, field + 1, 5);
                buf[5] = '\0';
            }
                break;

            case 'D':
            { // Date (int, int, int), -1 if empty.
                struct minmea_date *date = va_arg(ap, struct minmea_date *);

                int d = -1, m = -1, y = -1;

                if (field && minmea_isfield(*field)) {
                    // Always six digits.
                    for (int f = 0; f < 6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char dArr[] = {field[0], field[1], '\0'};
                    char mArr[] = {field[2], field[3], '\0'};
                    char yArr[] = {field[4], field[5], '\0'};
                    d = strtol(dArr, NULL, 10);
                    m = strtol(mArr, NULL, 10);
                    y = strtol(yArr, NULL, 10);
                }

                date->day = d;
                date->month = m;
                date->year = y;
            }
                break;

            case 'T':
            { // Time (int, int, int, int), -1 if empty.
                struct minmea_time *time_ = va_arg(ap, struct minmea_time *);

                int h = -1, i = -1, s = -1, u = -1;

                if (field && minmea_isfield(*field)) {
                    // Minimum required: integer time.
                    for (int f = 0; f < 6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char hArr[] = {field[0], field[1], '\0'};
                    char iArr[] = {field[2], field[3], '\0'};
                    char sArr[] = {field[4], field[5], '\0'};
                    h = strtol(hArr, NULL, 10);
                    i = strtol(iArr, NULL, 10);
                    s = strtol(sArr, NULL, 10);
                    field += 6;

                    // Extra: fractional time. Saved as microseconds.
                    if (*field++ == '.') {
                        int value = 0;
                        int scale = 1000000;
                        while (isdigit((unsigned char) *field) && scale > 1) {
                            value = (value * 10) + (*field++ -'0');
                            scale /= 10;
                        }
                        u = value * scale;
                    } else {
                        u = 0;
                    }
                }

                time_->hours = h;
                time_->minutes = i;
                time_->seconds = s;
                time_->microseconds = u;
            }
                break;

            case '_':
            { // Ignore the field.
            }
                break;

            default:
            { // Unknown.
                goto parse_error;
            }
                break;
        }

        next_field();
    }

    result = true;

parse_error:
    va_end(ap);
    return result;
}

bool minmea_talker_id(char talker[3], const char *sentence) {
    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return false;

    talker[0] = type[0];
    talker[1] = type[1];
    talker[2] = '\0';

    return true;
}

enum minmea_sentence_id minmea_sentence_id(const char *sentence, bool strict) {
    if (!minmea_check(sentence, strict))
        return MINMEA_INVALID;

    char type[6];
    if (!minmea_scan(sentence, "t", type))
        return MINMEA_INVALID;

    if (!strcmp(type + 2, "RMC"))
        return MINMEA_SENTENCE_RMC;
    if (!strcmp(type + 2, "GGA"))
        return MINMEA_SENTENCE_GGA;
    if (!strcmp(type + 2, "GSA"))
        return MINMEA_SENTENCE_GSA;
    if (!strcmp(type + 2, "GLL"))
        return MINMEA_SENTENCE_GLL;
    if (!strcmp(type + 2, "GST"))
        return MINMEA_SENTENCE_GST;
    if (!strcmp(type + 2, "GSV"))
        return MINMEA_SENTENCE_GSV;
    if (!strcmp(type + 2, "VTG"))
        return MINMEA_SENTENCE_VTG;

    return MINMEA_UNKNOWN;
}

_Bool minmea_parse_rmc(struct minmea_sentence_rmc *frame, const char *sentence) {
    // $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
    char type[6];
    char validity;
    int latitude_direction;
    int longitude_direction;
    int variation_direction;
    if (!minmea_scan(sentence, "tTcfdfdffDfd",
            type,
            &frame->time,
            &validity,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->speed,
            &frame->course,
            &frame->date,
            &frame->variation, &variation_direction))
        return false;
    if (strcmp(type + 2, "RMC"))
        return false;

    frame->valid = (validity == 'A');
    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;
    frame->variation.value *= variation_direction;

    return true;
}

_Bool minmea_parse_gga(struct minmea_sentence_gga *frame, const char *sentence) {
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!minmea_scan(sentence, "tTfdfdiiffcfci_",
            type,
            &frame->time,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->fix_quality,
            &frame->satellites_tracked,
            &frame->hdop,
            &frame->altitude, &frame->altitude_units,
            &frame->height, &frame->height_units,
            &frame->dgps_age))
        return false;
    if (strcmp(type + 2, "GGA"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

_Bool minmea_parse_gsa(struct minmea_sentence_gsa *frame, const char *sentence) {
    // $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    char type[6];

    if (!minmea_scan(sentence, "tciiiiiiiiiiiiifff",
            type,
            &frame->mode,
            &frame->fix_type,
            &frame->sats[0],
            &frame->sats[1],
            &frame->sats[2],
            &frame->sats[3],
            &frame->sats[4],
            &frame->sats[5],
            &frame->sats[6],
            &frame->sats[7],
            &frame->sats[8],
            &frame->sats[9],
            &frame->sats[10],
            &frame->sats[11],
            &frame->pdop,
            &frame->hdop,
            &frame->vdop))
        return false;
    if (strcmp(type + 2, "GSA"))
        return false;

    return true;
}

_Bool minmea_parse_gll(struct minmea_sentence_gll *frame, const char *sentence) {
    // $GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41$;
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!minmea_scan(sentence, "tfdfdTc;c",
            type,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->time,
            &frame->status,
            &frame->mode))
        return false;
    if (strcmp(type + 2, "GLL"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

_Bool minmea_parse_gst(struct minmea_sentence_gst *frame, const char *sentence) {
    // $GPGST,024603.00,3.2,6.6,4.7,47.3,5.8,5.6,22.0*58
    char type[6];

    if (!minmea_scan(sentence, "tTfffffff",
            type,
            &frame->time,
            &frame->rms_deviation,
            &frame->semi_major_deviation,
            &frame->semi_minor_deviation,
            &frame->semi_major_orientation,
            &frame->latitude_error_deviation,
            &frame->longitude_error_deviation,
            &frame->altitude_error_deviation))
        return false;
    if (strcmp(type + 2, "GST"))
        return false;

    return true;
}

_Bool minmea_parse_gsv(struct minmea_sentence_gsv *frame, const char *sentence) {
    // $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    // $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
    // $GPGSV,4,2,11,08,51,203,30,09,45,215,28*75
    // $GPGSV,4,4,13,39,31,170,27*40
    // $GPGSV,4,4,13*7B
    char type[6];

    if (!minmea_scan(sentence, "tiii;iiiiiiiiiiiiiiii",
            type,
            &frame->total_msgs,
            &frame->msg_nr,
            &frame->total_sats,
            &frame->sats[0].nr,
            &frame->sats[0].elevation,
            &frame->sats[0].azimuth,
            &frame->sats[0].snr,
            &frame->sats[1].nr,
            &frame->sats[1].elevation,
            &frame->sats[1].azimuth,
            &frame->sats[1].snr,
            &frame->sats[2].nr,
            &frame->sats[2].elevation,
            &frame->sats[2].azimuth,
            &frame->sats[2].snr,
            &frame->sats[3].nr,
            &frame->sats[3].elevation,
            &frame->sats[3].azimuth,
            &frame->sats[3].snr
            )) {
        return false;
    }
    if (strcmp(type + 2, "GSV"))
        return false;

    return true;
}

_Bool minmea_parse_vtg(struct minmea_sentence_vtg *frame, const char *sentence) {
    // $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    // $GPVTG,156.1,T,140.9,M,0.0,N,0.0,K*41
    // $GPVTG,096.5,T,083.5,M,0.0,N,0.0,K,D*22
    // $GPVTG,188.36,T,,M,0.820,N,1.519,K,A*3F
    char type[6];
    char c_true, c_magnetic, c_knots, c_kph, c_faa_mode;

    if (!minmea_scan(sentence, "tfcfcfcfc;c",
            type,
            &frame->true_track_degrees,
            &c_true,
            &frame->magnetic_track_degrees,
            &c_magnetic,
            &frame->speed_knots,
            &c_knots,
            &frame->speed_kph,
            &c_kph,
            &c_faa_mode))
        return false;
    if (strcmp(type + 2, "VTG"))
        return false;
    // check chars
    if (c_true != 'T' ||
            c_magnetic != 'M' ||
            c_knots != 'N' ||
            c_kph != 'K')
        return false;
    frame->faa_mode = c_faa_mode;

    return true;
}

/*
int minmea_gettime(struct timespec *ts, const struct minmea_date *date, const struct minmea_time *time_) {
    if (date->year == -1 || time_->hours == -1)
        return -1;

    struct tm tm;
    memset(&tm, 0, sizeof (tm));
    tm.tm_year = 2000 + date->year - 1900;
    tm.tm_mon = date->month - 1;
    tm.tm_mday = date->day;
    tm.tm_hour = time_->hours;
    tm.tm_min = time_->minutes;
    tm.tm_sec = time_->seconds;

    time_t timestamp = timegm(&tm); 
    if (timestamp != -1) {
        ts->tv_sec = timestamp;
        ts->tv_nsec = time_->microseconds * 1000;
        return 0;
    } else {
        return -1;
    }
}
 */
void nmea(char *line, GPS_T *data) {
    //printf("\r%s\n", line);
    enum minmea_sentence_id nmea_id = minmea_sentence_id(line, true);

    switch (nmea_id) {
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                char buffer[80];
                struct tm tmp;

                tmp.tm_year = frame.date.year + 100;
                tmp.tm_mon = frame.date.month - 1;
                tmp.tm_mday = frame.date.day;
                tmp.tm_hour = frame.time.hours;
                tmp.tm_min = frame.time.minutes;
                tmp.tm_sec = frame.time.seconds;

#ifdef __RASPI__
                putenv("TZ=UTC");
                data->stamp = mktime(&tmp);
                putenv("TZ=Europe/Berlin");

                if (abs(data->stamp - time(NULL)) > 60) {
                    stime(&data->stamp);
                    strftime(buffer, 80, "%d.%m.%Y %H:%M", localtime(&data->stamp));
                    //printf("%s\n", buffer);
                }
#endif
                /*
                struct tm tm2 = *localtime(&data->stamp);

                strftime(buffer, 80, "%d.%m.%Y %H:%M", &tm2);
                printf(INDENT_SPACES "date/time: %s\n", buffer);
                 */
                /*printf(INDENT_SPACES "raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                        frame.latitude.value, frame.latitude.scale,
                        frame.longitude.value, frame.longitude.scale,
                        frame.speed.value, frame.speed.scale);
                printf(INDENT_SPACES "fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000),
                        minmea_rescale(&frame.speed, 1000));
                printf(INDENT_SPACES "floating point degree coordinates and speed: (%f,%f) %f\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude),
                        minmea_tofloat(&frame.speed));
                 */
            } else {
                printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
            }
        }
            break;
        case MINMEA_SENTENCE_GSA:
        {
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, line)) {
                data->fix_type = frame.fix_type;
                data->PDOP = minmea_tofloat(&frame.pdop);
                data->HDOP = minmea_tofloat(&frame.hdop);

            } else {
                printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
            }
        }
            break;
        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                data->fix_quality = frame.fix_quality;
                data->latitude = minmea_tocoord(&frame.latitude);
                data->longitude = minmea_tocoord(&frame.longitude);
                data->satellites_tracked = frame.satellites_tracked;
            } else {
                printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
            }
        }
            break;
        case MINMEA_SENTENCE_VTG:
        {
            struct minmea_sentence_vtg frame;
            if (minmea_parse_vtg(&frame, line)) {
                /*printf(INDENT_SPACES "true track degrees = %f\n",
                        minmea_tofloat(&frame.true_track_degrees));
                printf(INDENT_SPACES "magnetic track degrees = %f\n",
                        minmea_tofloat(&frame.magnetic_track_degrees));
                printf(INDENT_SPACES "speed knots = %f\n",
                        minmea_tofloat(&frame.speed_knots));
                printf(INDENT_SPACES "speed kph = %f\n",
                        minmea_tofloat(&frame.speed_kph));*/
                data->angle = minmea_tofloat(&frame.true_track_degrees);

            } else {
                printf(INDENT_SPACES "sentence is not parsed\n");
            }
        }
            break;
        case MINMEA_SENTENCE_GSV:
        {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
                data->total_sats = frame.total_sats;
                //printf(INDENT_SPACES "message %d of %d\n", frame.msg_nr, frame.total_msgs);
                /*printf(INDENT_SPACES "sattelites in view: %d\n", frame.total_sats);
                for (int i = 0; i < 4; i++)
                    printf(INDENT_SPACES "sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                        frame.sats[i].nr,
                        frame.sats[i].elevation,
                        frame.sats[i].azimuth,
                        frame.sats[i].snr);*/
            } else {
                printf(INDENT_SPACES "sentence is not parsed\n");
            }
        }
            break;
        case MINMEA_INVALID:
            break;
        default:
            break;
    }
}

char line[MINMEA_MAX_LENGTH];
unsigned char rx_buffer[BUFFER_LEN] = {0};
int rx_asd = 0;

void gps_open(int *uart0_filestream, char *uart_dev) {
    *uart0_filestream = open(uart_dev, O_RDONLY | O_NOCTTY | O_NDELAY);

    if (*uart0_filestream == -1) {
        printf("Error - Unable to open UART.\n");
        return;
    }

    struct termios options;
    tcgetattr(*uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(*uart0_filestream, TCIFLUSH);
    tcsetattr(*uart0_filestream, TCSANOW, &options);

    rx_buffer[BUFFER_LEN - 1] = '\0';
}

void gps_read(int *uart0_filestream, GPS_T *data) {
    int rx_length = read(*uart0_filestream, (void*) &rx_buffer[rx_asd], BUFFER_LEN - 1);

    if ((rx_asd + rx_length) > BUFFER_LEN) {
        rx_asd = 0;
    } else if (rx_length > 0) {
        rx_asd += rx_length;

        char *e = &rx_buffer[0];
        char *s = &rx_buffer[0];

        while (e != NULL) {
            e = strchr(e, 13);
            if (e != NULL) {
                *e = '\0';
                s = strchr(s, 36);
                if (s != NULL) {
                    strncpy(line, s, MINMEA_MAX_LENGTH);
                    rx_asd = 0;
                    nmea(line, data);
                }
                *e = 13;
                e++;
                s = e;
            }
        }
    }
}

