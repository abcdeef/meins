#ifndef GUI_H_
#define GUI_H_

#define ROT 63489
#define TRA 65534
#define SWZ 1
#define GRY 33825

#define K 0.05f // einheitliche Icongrösse
#define B_X 0.93f // Abstand Mitte -> horizontaler Rand
#define B_Y 0.53f // Abstand Mitte -> vertikaler Rand

float center[] = { -P, -P - 0.3, 0.0f, 0.0f, 0.0f - 0.3f, 0.0f, P, -P - 0.3f, 0.0f };

// Labels
//#define L_W 27
//#define L_H 26
//#define GL_L_W 0.2f

/*float label[] = { 0.0f, GL_L_W, 0.0f, 0.0f, 0.0f, 0.0f,
 GL_L_W * L_W / L_H * 1, GL_L_W, 0.0f,
 GL_L_W * L_W / L_H * 1, 0.0f, 0.0f };*/

float buttons[] = {
		/* plus */
		-K + B_X, K - B_Y, 0.0f,
		-K + B_X, -K - B_Y, 0.0f,
		K + B_X, K - B_Y, 0.0f,
		K + B_X, -K - B_Y, 0.0f,
		/* minus */
		-K + B_X - 0.13f, K - B_Y, 0.0f,
		-K + B_X - 0.13f, -K - B_Y, 0.0f,
		K + B_X - 0.13f, K - B_Y, 0.0f,
		K + B_X - 0.13f, -K - B_Y, 0.0f,
		/* menu */
		-K - B_X, K - B_Y, 0.0f,
		-K - B_X, -K - B_Y, 0.0f,
		K - B_X, K - B_Y, 0.0f,
		K - B_X, -K - B_Y, 0.0f
};

unsigned short auto_tex = ROT;
unsigned int *gui_tex_index, gui_tex_zahlen[47];

#define GPS_I_X 0.93f
#define GPS_I_Y 0.54f
#define GPS_I_R 0.1f

float gps_i[] = {
GPS_I_X - GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 0.0f, 0.0, 1.0,
GPS_I_X - GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 0.0f, 0.0, 0.0,
GPS_I_X + GPS_I_R / 2, GPS_I_Y + GPS_I_R / 2, 0.0f, 1.0, 1.0,
GPS_I_X + GPS_I_R / 2, GPS_I_Y - GPS_I_R / 2, 0.0f, 1.0, 0.0 };

unsigned short plus_tex[144], minus_tex[144], menu_tex[144];

unsigned short tex_zahlen[200 * 47];

#endif /* GUI_H_ */