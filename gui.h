#ifndef GUI_H_
#define GUI_H_

#define ROT 63489
#define TRA 65534
#define SWZ 1
#define GRY 33825

#define K 0.06f // einheitliche Icongrösse
#define B_X 0.93f // Abstand Mitte -> horizontaler Rand
#define B_Y 0.53f // Abstand Mitte -> vertikaler Rand

#define GPS_I_X 0.93f
#define GPS_I_Y 0.54f
#define GPS_I_R 0.1f

float *buttons2;

float buttons[] = {
    /* x, y, t_x, t_y*/
    /* 0 Dreieck */
    -P, -P, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f,
    P, -P, 1.0f, 0.0f,
    /* 3 plus */
    -K + B_X, K - B_Y, 0.0f, 0.0f,
    -K + B_X, -K - B_Y, 0.0f, 1.0f,
    K + B_X, K - B_Y, 1.0f, 0.0f,
    K + B_X, -K - B_Y, 1.0f, 1.0f,
    /* 7 minus */
    -K + B_X - 0.13f, K - B_Y, 0.0f, 0.0f,
    -K + B_X - 0.13f, -K - B_Y, 0.0f, 1.0f,
    K + B_X - 0.13f, K - B_Y, 1.0f, 0.0f,
    K + B_X - 0.13f, -K - B_Y, 1.0f, 1.0f,
    /* 11 menu */
    -K - B_X, K - B_Y, 0.0f, 0.0f,
    -K - B_X, -K - B_Y, 0.0f, 1.0f,
    K - B_X, K - B_Y, 1.0f, 0.0f,
    K - B_X, -K - B_Y, 1.0f, 1.0f,
    /* 15 GPS_I */
    GPS_I_X - GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 0.0, 1.0,
    GPS_I_X - GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 0.0, 0.0,
    GPS_I_X + GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 1.0, 1.0,
    GPS_I_X + GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 1.0, 0.0,
    /* 19 Dreieck2 */
    -0.02f, -0.03f, 0.0f, 0.0f,
    0.0f, -0.01f, 0.0f, 1.0f,
    0.02f, -0.03f, 1.0f, 0.0f,
    -GPS_I_R / 4, +GPS_I_R / 4, 0.0f, 1.0f,
    -GPS_I_R / 4, -GPS_I_R / 4, 0.0f, 0.0f,
    +GPS_I_R / 4, +GPS_I_R / 4, 1.0f, 1.0f,
    +GPS_I_R / 4, -GPS_I_R / 4, 1.0f, 0.0f,
    /* 26 night */
    -K + B_X - 0.13f, K - B_Y, 0.0f, 0.0f,
    -K + B_X - 0.13f, -K - B_Y, 0.0f, 1.0f,
    K + B_X - 0.13f, K - B_Y, 1.0f, 0.0f,
    K + B_X - 0.13f, -K - B_Y, 1.0f, 1.0f,
    /* 30 neue Aufzeichnung */
    -K - B_X, K - B_Y + 1.06f, 0.0f, 0.0f,
    -K - B_X, -K - B_Y + 1.06f, 0.0f, 1.0f,
    K - B_X, K - B_Y + 1.06f, 1.0f, 0.0f,
    K - B_X, -K - B_Y + 1.06f, 1.0f, 1.0f,
    /* 34 FBO Tex GUI */
    -1.0f, 0.6f, 0.0f, 1.0f,
    -1.0f, -0.6f, 0.0f, 0.0f,
    1.0f, 0.6f, 1.0f, 1.0f,
    1.0f, -0.6f, 1.0f, 0.0f,
};

unsigned short auto_tex = ROT;
unsigned int gui_tex_index[10], gui_tex_zahlen[47];

T_TEXT plus_tex, minus_tex, menu_tex, night_tex, tex_zahlen, new_tex;
//unsigned short *plus_tex, *minus_tex, *menu_tex, *tex_zahlen;

#endif /* GUI_H_ */