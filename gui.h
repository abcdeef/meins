#ifndef GUI_H_
#define GUI_H_

#define ROT 63489
#define TRA 65534
#define SWZ 1
#define GRY 33825

#define K 0.05f // einheitliche Icongrösse
#define B_X 0.93f // Abstand Mitte -> horizontaler Rand
#define B_Y 0.53f // Abstand Mitte -> vertikaler Rand

#define GPS_I_X 0.93f
#define GPS_I_Y 0.54f
#define GPS_I_R 0.1f

float buttons[] = {
    /* x, y, t_x, t_y*/
    /* Dreieck */
      -P, -P - 0.3f, 0.0f, 0.0f,
    0.0f,     -0.3f, 0.0f, 1.0f,
       P, -P - 0.3f, 1.0f, 0.0f,
    /* plus */
    -K + B_X,  K - B_Y, 0.0f, 0.0f,
    -K + B_X, -K - B_Y, 0.0f, 1.0f,
     K + B_X,  K - B_Y, 1.0f, 0.0f,
     K + B_X, -K - B_Y, 1.0f, 1.0f,
    /* minus */
    -K + B_X - 0.13f,  K - B_Y, 0.0f, 0.0f,
    -K + B_X - 0.13f, -K - B_Y, 0.0f, 1.0f,
     K + B_X - 0.13f,  K - B_Y, 1.0f, 0.0f,
     K + B_X - 0.13f, -K - B_Y, 1.0f, 1.0f,
    /* menu */
    -K - B_X,  K - B_Y, 0.0f, 0.0f,
    -K - B_X, -K - B_Y, 0.0f, 1.0f,
     K - B_X,  K - B_Y, 1.0f, 0.0f,
     K - B_X, -K - B_Y, 1.0f, 1.0f,
    /* GPS_I */
    GPS_I_X - GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 0.0, 1.0,
    GPS_I_X - GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 0.0, 0.0,
    GPS_I_X + GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 1.0, 1.0,
    GPS_I_X + GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 1.0, 0.0
};

unsigned short auto_tex = ROT;
unsigned int gui_tex_index[4], gui_tex_zahlen[47];

unsigned short plus_tex[144], minus_tex[144], menu_tex[144];

unsigned short tex_zahlen[200 * 47];

#endif /* GUI_H_ */