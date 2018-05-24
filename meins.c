#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <math.h>
#include <locale.h>
#include <sqlite3.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <linux/reboot.h>
#include <linux/input.h>
#include <sys/reboot.h>
#include <limits.h>
#include <inttypes.h>
#include "util.h"
#include "gui.h"
#include "obd.h"
#include "gps.h"
#include "gpio.h"
#include "sensor.h"

#ifdef __SSE__
#include <xmmintrin.h>
#endif

#ifdef __RASPI__
#include "esUtil_raspi.h"
//#define OBD_SERIAL "/dev/rfcomm1"
#define OBD_SERIAL "/dev/ttyUSB0"
#define RSIZE 5
#define HOME "/home/pi/"
#else
#include "esUtil.h"
#define OBD_SERIAL "/dev/pts/18"
#define RSIZE 3
#define HOME "/home/florian/"
#endif

FILE *sout;

#define BUFFER_OFFSET(i) ((void *) NULL + (2*(i)))
#define ROUND(x) roundf(x*10)/10
//#define MEM(x) (int)(x * 256 )
#define PRINTF(...) fprintf(sout,__VA_ARGS__);fflush(sout);fprintf(stdout,__VA_ARGS__)
#define PRINTGLERROR printOglError(__FILE__, __LINE__);
#ifdef __TM__
#define TM(START,END,...) clock_gettime(CLOCK_MONOTONIC, &END);printf("%lims\n",(END.tv_sec - START.tv_sec) * 1000 + (END.tv_nsec - START.tv_nsec) / 1000000);
#else
#define TM(START, END,...) 
#endif
#define FRACTPART 10000000

#define BUFF_INDEX_GUI 0
#define BUFF_INDEX_TXT 1
#define BUFFER_VERTS_REIS 2
#define BUFFER_INDEX_REIS 3
#define BUFFER_VERTS_YAMA 4
#define BUFFER_INDEX_YAMA 5
#define BUFFER_VERTS_WERDER 6
#define BUFFER_INDEX_WERDER 7
#define BUFFER_GITTER 10
#define BUFFER_DREIECK 14
#define BUFFER_VERTS_AUTO 15
#define BUFFER_INDEX_AUTO 16


//#define FBO_WIDTH 1024
//#define FBO_HEIGHT 1024

FILE *gps_out, *gps_in, *sensor_out, *sensor_in, *eigenschaften;

unsigned char tmp_gl_mode[] = {0, GL_TRIANGLES, GL_TRIANGLES, GL_LINES};

ESContext esContext;

float trans_x = 0.0f, trans_y = 0.0f, trans_z = 0.0f, orto1 = 0.0f, orto2 = 0.0f, orto3 = 0.0f, orto4 = 0.0f;

//char line1_str[21];
//char line2_str[21];

sqlite3 *db_1, *db_2, *db_3;

enum GUI_MODE {
    _2D_, _3D_, _GUI_
};
enum GUI_MODE gui_mode = _2D_;

typedef struct {
    void *val;
    size_t len;
} T_IO;

typedef struct {
    int id;
    double g_x;
    double g_y;
} T_ROUTE;

typedef struct {
    int id;
    int index;
    GLsizei len;
    GLfloat r, g, b;
    int osm_id;
    unsigned char gl_mode;
    float glLineWidth;
    unsigned char typ;
} T_V_E;

#define V_Z_DEFAULT 1.2f

#ifdef __RASPI__
float BR = 4.0f;
#else
float BR = 6.0f;
#endif

float T_Y = V_Z_DEFAULT;
float ORTHO = 2.0f;

#define Y_LEN 10000
#define V_LEN 65536
#define F_LEN 20
//static GLfloat verts[3 * V_LEN] = {0.0f};
//static float verts_F[5 * V_LEN] = {0.0f};
static float verts_F_yama[5 * V_LEN] = {0.0f}, verts_F_werder[5 * V_LEN] = {0.0f}, verts_F_reis[5 * V_LEN] = {0.0f};
static float gitter[320] = {0.0f};

//static int verts_L[3 * V_LEN] = {0};
uint16_t verts_len = 0;
////////////////////////////////
//static T_V_E yama[Y_LEN];
static uint16_t index_yama[V_LEN], index_werder[V_LEN], index_reis[V_LEN];
uint16_t index_yama_len = 0, index_werder_len = 0, index_reis_len = 0;
/////////////////////////
//static T_V_E werder[Y_LEN];
//unsigned short werder_len = 0;
/////////////////////////
//static GLfloat tmp_verts[V_LEN] = {0.0f};
//static T_V_E tmp_yama[Y_LEN];
//unsigned short tmp_vert_len, tmp_yama_len, tmp_werder_len, tmp_reis_len;
static uint16_t tmp_index_yama[V_LEN], tmp_index_werder[V_LEN], tmp_index_reis[V_LEN];
uint16_t tmp_index_yama_len = 0, tmp_index_reis_len = 0, tmp_index_werder_len = 0;

T_ROUTE route[V_LEN], tmp_route[V_LEN];
uint16_t route_len = 0, tmp_route_len = 0;
///////////////////////////////
T_V_E farbe_yama[F_LEN], farbe_werder[F_LEN], farbe_reis[F_LEN], tmp_farbe_reis[F_LEN], tmp_farbe_yama[F_LEN], tmp_farbe_werder[F_LEN], markierung, tmp_osm_werder[F_LEN], osm_werder[F_LEN];
uint16_t farbe_yama_len = 0, farbe_reis_len = 0, farbe_werder_len = 0, osm_werder_len = 0, tmp_osm_werder_len = 0;
uint16_t tmp_farbe_werder_len = 0, tmp_farbe_reis_len = 0, tmp_farbe_yama_len = 0;


uint16_t verts_yama_len = 0, verts_werder_len = 0, verts_reis_len = 0;
////////////////////
//static T_V_E reis[Y_LEN];
//static unsigned short reis_index[V_LEN];
//unsigned short reis_len = 0, reis_index_len = 0;

int sqc_tmp;

//void setPOS(ESContext *esContext);
static int sqc_vertex_lvl1(void *a, int argc, char **argv, char **azColName);
static int sqc_vertex_lvl2(void *a, int argc, char **argv, char **azColName);
static int sqc_vertex_lvl3(void *a, int argc, char **argv, char **azColName);
static int sqc_route_lvl3(void *a, int argc, char **argv, char **azColName);
//static int sqc_vertex3(void *a, int argc, char **argv, char **azColName);

void run_deta_lvl3(double x, double y);
void printM4(ESMatrix *wg);
void ShutDown(int signum);

//#define V_KORR 1.7f

double mf;
double pow2Z;
float rad2deg;
//GLubyte ZOOM_S[] = {18, 17, 16, 15, 14, 12, 10, 8};
//GLubyte *ZOOM;

//int dx = 0, dy = 0, m_n, m_m, p_n, p_m;
GLuint *tmp;
//static struct fixsource_t source;
ESMatrix perspective, modelview, mvpMatrix, mvpDreieck;

//unsigned int stride;
float gps_status[3] = {1.0f, 0.0f, 0.0f};

int_fast8_t new_tex_lvl1 = 0, new_tex_lvl2 = 0, new_tex_lvl3 = 0;
pthread_mutex_t m_pinto = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_deto_lvl1 = PTHREAD_MUTEX_INITIALIZER, mutex_deto_lvl2 = PTHREAD_MUTEX_INITIALIZER, mutex_deto_lvl3 = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    GLuint programMap;
    GLuint programWERDER;
    GLuint programGUI;
    GLuint programTest;
    GLuint programKreis;
    GLuint programDreieck;
    GLuint positionLocWERDERH;
    GLuint positionLocWERDERL;
    GLuint colorLocWERDER;
    GLuint mvpLocWERDER;
    GLuint offsetHLocWERDER;
    GLuint offsetLLocWERDER;
    GLuint MapoffsetLocWERDER;

    // Attribute locations
    GLint positionLocGui;
    GLint positionLocKreis;
    GLint texCoordLocKreis;
    GLint positionLocMap;
    GLint positionLocMapH;
    GLint positionLocMapL;

    GLint offsetHLocMap;
    GLint offsetLLocTex;
    GLint offsetLLocMap;
    GLint offsetMLocMap2;
    GLint positionKreis;
    GLint offsetMLocTex;
    GLint MapoffsetLocDreieck;
    GLint colorLocDreieck;
    GLint MapoffsetLocMap;
    GLint texCoordLocGui;
    GLint colorLocMap;
    GLint texLocGui;
    GLint samplerLocGui;
    GLint samplerLocTex;
    GLint positionLocDreieck;
    GLint texCoordLocTex;

    // Uniform locations
    GLint mvpLocMap;
    GLint colorLocKreis;
    GLint AngleLocDreieck;

    // Vertex data
    GLfloat *vertices;
    GLuint *indices;
    GLuint numIndices;

    // Auflösung
    GLushort r_width;
    GLushort r_height;
    GLuint width;
    GLuint height;
    GLuint tiles;
    GLuint image_sz;
    GLuint imgrow_sz;
    GLubyte element4Pixel;

    // Texture
    GLuint *texindex;
    GLuint *texMap;
    GLuint *texfree;
    GLuint *texreload;
    TEX_BUFFER_FORMAT *tex_buffer;
    GLuint textureId;
    //
#ifdef __FBO__
    GLuint FramebufferName;
    GLuint renderedTexture;
    GLuint depthrenderbuffer;
    GLuint fbo_texture;
#endif
    //Buffer
    GLuint buffindex[20];
    GLuint vao[10];

    //
    uint16_t auto_len;
} UserData;

typedef struct {
    uint16_t gps_file;
    GLuint gpu_t_x;
    GLuint gpu_t_y;
    GLuint *tiles_x;
    GLuint *tiles_y;
    GLuint *gpu_tiles_x;
    GLuint *gpu_tiles_y;
    float v_x;
    float v_y;
    float v;
    float o_x;
    float o_y;
    float angle;
    double gps_longitude;
    double gps_latitude;
    double gps_altitude;
    int gps_mode;
    int gps_sat;

    double osmS;
    float osmS2;
    double g_x;
    double g_y;
    int m_x;
    int m_y;
    int t_x;
    int t_y;
    short i2c_x;
    short i2c_y;
    short i2c_z;

    uint8_t obd_volt;
    float obd_coolant;
    float obd_speed;
    float obd_rpm;
    float obd_maf;
    float obd_o21;
    float obd_o22;
    float obd_iat;
    float obd_tia;
    float obd_stft;
    float obd_ltft;
    float obd_throttle;
    unsigned int obd_dct_count;
    char obf_dtc1[4];
    char obf_dtc2[4];
    char obf_dtc3[4];

    uint_fast8_t * display;

    float i2c_hum;
    float i2c_temp;
    float i2c_angle;

    // Festkomma g_x/g_y Ersatz
    int gi_x[2];
    int gi_y[2];
} POS_T;

void DoubletoFix(double *a, int *ret) {
    double intpart = 0.0;

    ret[1] = (int) (modf(*a, &intpart) * FRACTPART);
    ret[0] = (int) intpart;
}

int printOglError(char *file, int line) {
    //
    // Returns 1 if an OpenGL error occurred, 0 otherwise.
    //
    GLenum glErr;
    int retCode = 0;

    glErr = glGetError();
    while (glErr != GL_NO_ERROR) {
        printf("\rglError in file %s @ line %i - ", file, line);

        switch (glErr) {
            case GL_INVALID_OPERATION: printf("INVALID_OPERATION");
                break;
            case GL_INVALID_ENUM: printf("INVALID_ENUM");
                break;
            case GL_INVALID_VALUE: printf("INVALID_VALUE");
                break;
            case GL_OUT_OF_MEMORY: printf("OUT_OF_MEMORY");
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION: printf("INVALID_FRAMEBUFFER_OPERATION");
                break;
            default: printf("else");
        }
        printf("\n");
        retCode = 1;
        glErr = glGetError();
    }
    return retCode;
}

int Init(ESContext *esContext) {
    UserData *userData = esContext->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;

    PRINTF("\r%s\n\r%s\n\r%s\n\r%s\n", glGetString(GL_VERSION), glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_SHADING_LANGUAGE_VERSION));

    char *f1, *f4, *v7, *f2, *f3, *v1, *v2, *v4, *f5, *v5;
    LoadGLSL(&f2, HOME "GLSL/f2.glsl");
    LoadGLSL(&v2, HOME "GLSL/v2.glsl");
    LoadGLSL(&f3, HOME "GLSL/f3.glsl");
    LoadGLSL(&f4, HOME "GLSL/f4.glsl");
    LoadGLSL(&v4, HOME "GLSL/v4.glsl");
    LoadGLSL(&v5, HOME "GLSL/v5.glsl");
    LoadGLSL(&f5, HOME "GLSL/f5.glsl");
    LoadGLSL(&v7, HOME "GLSL/v7.glsl");
    //LoadGLSL(&v1, HOME "GLSL/v1.glsl");
    //LoadGLSL(&f1, HOME "GLSL/f1.glsl");

    userData->programGUI = esLoadProgram(v2, f2);
    userData->programKreis = esLoadProgram(v2, f3);
    userData->programMap = esLoadProgram(v4, f4);
    userData->programWERDER = esLoadProgram(v5, f5);
    userData->programDreieck = esLoadProgram(v7, f4);
    userData->programTest = esLoadProgram(v4, f4);

    free(f2);
    free(v2);
    free(f3);
    free(f4);
    free(v4);
    free(f5);
    free(v5);
    free(v7);
    //free(v1);
    //free(f1);

    glGenBuffers(20, &userData->buffindex[0]);

    userData->positionLocDreieck = glGetAttribLocation(userData->programDreieck, "a_position");
    userData->AngleLocDreieck = glGetUniformLocation(userData->programDreieck, "u_angle");
    userData->MapoffsetLocDreieck = glGetUniformLocation(userData->programDreieck, "u_mapOffset");
    userData->colorLocDreieck = glGetUniformLocation(userData->programDreieck, "u_color");
    //glEnableVertexAttribArray(userData->positionLocDreieck);

    userData->positionLocGui = glGetAttribLocation(userData->programGUI, "a_position");
    userData->texCoordLocGui = glGetAttribLocation(userData->programGUI, "a_texCoord");
    userData->texLocGui = glGetUniformLocation(userData->programGUI, "s_baseMap");
    userData->samplerLocGui = glGetUniformLocation(userData->programGUI, "s_texture");
    glEnableVertexAttribArray(userData->positionLocGui);
    glEnableVertexAttribArray(userData->texCoordLocGui);

    userData->positionLocKreis = glGetAttribLocation(userData->programKreis, "a_position");
    userData->texCoordLocKreis = glGetAttribLocation(userData->programKreis, "a_texCoord");
    userData->colorLocKreis = glGetUniformLocation(userData->programKreis, "u_f");
    //userData->positionKreis = glGetUniformLocation(userData->programKreis, "u_position");
    glEnableVertexAttribArray(userData->positionLocKreis);
    glEnableVertexAttribArray(userData->texCoordLocKreis);

    userData->positionLocMapH = glGetAttribLocation(userData->programMap, "a_posH");
    userData->positionLocMapL = glGetAttribLocation(userData->programMap, "a_posL");
    userData->colorLocMap = glGetUniformLocation(userData->programMap, "u_color");
    userData->mvpLocMap = glGetUniformLocation(userData->programMap, "u_mvpMatrix");
    userData->offsetHLocMap = glGetUniformLocation(userData->programMap, "u_offsetH");
    userData->offsetLLocMap = glGetUniformLocation(userData->programMap, "u_offsetL");
    //userData->MapoffsetLocMap = glGetUniformLocation(userData->programMap, "u_mapOffset");
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_WERDER]);
    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));
    glEnableVertexAttribArray(userData->positionLocMapH);
    glEnableVertexAttribArray(userData->positionLocMapL);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    userData->positionLocWERDERH = glGetAttribLocation(userData->programWERDER, "a_posH");
    userData->positionLocWERDERL = glGetAttribLocation(userData->programWERDER, "a_posL");
    userData->colorLocWERDER = glGetUniformLocation(userData->programWERDER, "u_color");
    userData->mvpLocWERDER = glGetUniformLocation(userData->programWERDER, "u_mvpMatrix");
    userData->offsetHLocWERDER = glGetUniformLocation(userData->programWERDER, "u_offsetH");
    userData->offsetLLocWERDER = glGetUniformLocation(userData->programWERDER, "u_offsetL");
    userData->MapoffsetLocWERDER = glGetUniformLocation(userData->programWERDER, "u_mapOffset");
    glEnableVertexAttribArray(userData->positionLocWERDERH);
    glEnableVertexAttribArray(userData->positionLocWERDERL);


    // NICHT LÖSCHEN! der raspi  brauch das
    for (int fg = 0; fg < 20; fg++)
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[fg]);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glBufferData(GL_ARRAY_BUFFER, sizeof (buttons), buttons, GL_STATIC_DRAW);
    glVertexAttribPointer(userData->positionLocDreieck, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glEnableVertexAttribArray(userData->positionLocDreieck);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_GITTER]);
    glBufferData(GL_ARRAY_BUFFER, sizeof (gitter), gitter, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_AUTO]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_AUTO]);
    float auto_koord[] = {
        0.0f, 0.0f, 0.0f, -0.1f, 0.0f,
        0.0f, 0.0f, 0.0f, -0.05f, 0.09f,
        0.0f, 0.0f, 0.0f, 0.05f, 0.09f,
        0.0f, 0.0f, 0.0f, 0.1f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.05f, -0.09f,
        0.0f, 0.0f, 0.0f, -0.05f, -0.09f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof (auto_koord), auto_koord, GL_STATIC_DRAW);
    uint16_t auto_index[] = {
        0, 6, 1,
        1, 6, 2,
        2, 6, 3,
        3, 6, 4,
        4, 6, 5,
        5, 6, 0,
    };
    userData->auto_len = sizeof (auto_index) / sizeof (auto_index[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (auto_index), auto_index, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glGenTextures(sizeof (gui_tex_index) / sizeof (unsigned int), &gui_tex_index[0]);
    glGenTextures(sizeof (gui_tex_zahlen) / sizeof (unsigned int), &gui_tex_zahlen[0]);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 12, 12, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, plus_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 12, 12, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, minus_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[3]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 12, 12, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, menu_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[2]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, &auto_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    for (int p = 0; p < 47; p++) {
        glBindTexture(GL_TEXTURE_2D, gui_tex_zahlen[p]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 10, 20, 0, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1, &tex_zahlen[p * 200]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    glViewport(0, -160, 800, 800);
    //glViewport(0, 0, 800, 480);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    esMatrixLoadIdentity(&perspective);
    esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);

    T_Y = 0.0f;
    gui_mode = _2D_;
    posData->o_x = 0.0f;
    posData->o_y = 0.0f;


    PRINTGLERROR
    return GL_TRUE;
}

void printM4(ESMatrix *wg) {
    PRINTF("%f %f %f %f\n", wg->m[0][0], wg->m[0][1], wg->m[0][2], wg->m[0][3]);
    PRINTF("%f %f %f %f\n", wg->m[1][0], wg->m[1][1], wg->m[1][2], wg->m[1][3]);
    PRINTF("%f %f %f %f\n", wg->m[2][0], wg->m[2][1], wg->m[2][2], wg->m[2][3]);
    PRINTF("%f %f %f %f\n\n", wg->m[3][0], wg->m[3][1], wg->m[3][2], wg->m[3][3]);
}

void renderText(UserData *userData, char *str, GLfloat p_x, GLfloat p_y) {
    int len = strlen(str);

    GLfloat fgh[] = {
        -0.025f + p_x, 0.05f + p_y, 0.0f, 0.0f,
        -0.025f + p_x, -0.05f + p_y, 0.0f, 1.0f,
        0.025f + p_x, 0.05f + p_y, 1.0f, 0.0f,
        0.025f + p_x, -0.05f + p_y, 1.0f, 1.0f
    };

#define A_O 0.06f

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_TXT]);

    for (int p = 0; p < len; p++) {
        fgh[0] += A_O;
        fgh[4] += A_O;
        fgh[8] += A_O;
        fgh[12] += A_O;

        glBufferData(GL_ARRAY_BUFFER, sizeof (fgh), fgh, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(userData->positionLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
        glVertexAttribPointer(userData->texCoordLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));

        glBindTexture(GL_TEXTURE_2D, gui_tex_zahlen[str[p] - 43]);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
}

void renderFloat(UserData *userData, float zahl, GLfloat p_x, GLfloat p_y) {
    char str[16];
    snprintf(str, 16, "%f", zahl);
    renderText(userData, str, p_x, p_y);
}

void renderInt(UserData *userData, int zahl, GLfloat p_x, GLfloat p_y) {
    char str[16];
    snprintf(str, 16, "%d", zahl);
    renderText(userData, str, p_x, p_y);
}

void intro(ESContext *esContext) {
    UserData *userData = esContext->userData;

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(userData->programGUI);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(userData->samplerLocGui, 0);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    renderText(userData, "+,-./0123456789:;<=>?@ABCDE", -1.0f, 0.5f);
    renderText(userData, "FGHIJKLMNOPQRSTUVWXYZ", -1.0f, 0.32f);

    eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);
    usleep(800000);
}

void Update(ESContext *esContext, float deltaTime) {
    UserData *userData = ((ESContext*) esContext)->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;

    float dAngle = 0.0f;
    float mAngle = posData->angle * rad2deg;

    esMatrixLoadIdentity(&modelview);
    if (gui_mode == _3D_) {
        esRotate(&modelview, mAngle, 0.0, 0.0, 1.0);
    } else if (gui_mode == _2D_) {
        dAngle = mAngle;
    }
    esTranslate(&modelview, 0.0f, T_Y, 0.0f);


#ifdef __SSE__
    __m128 row1 = _mm_load_ps(&perspective.m[0][0]);
    __m128 row2 = _mm_load_ps(&perspective.m[1][0]);
    __m128 row3 = _mm_load_ps(&perspective.m[2][0]);
    __m128 row4 = _mm_load_ps(&perspective.m[3][0]);
    for (uint_fast8_t m_m = 0; m_m < 4; m_m++) {
        __m128 brod1 = _mm_set1_ps(modelview.m[m_m][0]);
        __m128 brod2 = _mm_set1_ps(modelview.m[m_m][1]);
        __m128 brod3 = _mm_set1_ps(modelview.m[m_m][2]);
        __m128 brod4 = _mm_set1_ps(modelview.m[m_m][3]);
        __m128 row =
                _mm_add_ps(_mm_add_ps(_mm_mul_ps(brod1, row1), _mm_mul_ps(brod2, row2)), _mm_add_ps(_mm_mul_ps(brod3, row3), _mm_mul_ps(brod4, row4)));

        _mm_store_ps(&mvpMatrix.m[m_m][0], row);
    }
#else
    esMatrixMultiply(&mvpMatrix, &modelview, &perspective);
#endif

    //printf("%f\n", posData->v);
    //posData->v_x = sinf(mAngle) * posData->v;
    //posData->v_y = cosf(mAngle) * posData->v;

    posData->g_x += deltaTime * posData->v_x / 3600.0f / posData->osmS;
    posData->g_y -= deltaTime * posData->v_y / 3600.0f / posData->osmS;

    double g_x = posData->g_x + posData->o_x;
    double g_y = posData->g_y - posData->o_y;

    DoubletoFix(&g_x, posData->gi_x);
    DoubletoFix(&g_y, posData->gi_y);

    // DRAW
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(userData->programMap);
    glEnableVertexAttribArray(userData->positionLocMapH);
    glEnableVertexAttribArray(userData->positionLocMapL);

    glUniformMatrix4fv(userData->mvpLocMap, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);
    glUniform2i(userData->offsetHLocMap, posData->gi_x[0], posData->gi_y[0]);
    glUniform2f(userData->offsetLLocMap, fmod(g_x, 1.0f), fmod(g_y, 1.0f));
    PRINTGLERROR

    // Gitter
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_GITTER]);
    pthread_mutex_lock(&mutex_deto_lvl1);
    if (new_tex_lvl1 == 1) {
        glBufferData(GL_ARRAY_BUFFER, sizeof (gitter), gitter, GL_STATIC_DRAW);
        new_tex_lvl1 = 0;
        PRINTGLERROR
    }
    pthread_mutex_unlock(&mutex_deto_lvl1);

    glVertexAttribPointer(userData->positionLocMapH, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));

    glUniform3f(userData->colorLocMap, 0.0f, 0.0f, 0.0f);
    for (uint8_t d = 0; d < 16; d++) {
        glDrawArrays(GL_LINES, 2 * d, 2);
    }
    for (uint8_t d = 16; d < 40; d++) {
        glDrawArrays(GL_LINES, 2 * d, 2);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    PRINTGLERROR
    // #################


    // REIS
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_REIS]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_REIS]);

    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));


    for (uint16_t m_n = 0; m_n < farbe_reis_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe_yama[m_n].r, farbe_yama[m_n].g, farbe_yama[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe_yama[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe_yama[m_n].index));
    }
    PRINTGLERROR

    // YAMA
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_YAMA]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_YAMA]);

    pthread_mutex_lock(&mutex_deto_lvl2);
    if (new_tex_lvl2 == 1) {
        glBufferData(GL_ARRAY_BUFFER, 20 * verts_yama_len, verts_F_yama, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_yama_len * sizeof (short), index_yama, GL_DYNAMIC_DRAW);
        new_tex_lvl2 = 0;
        PRINTGLERROR
    }
    pthread_mutex_unlock(&mutex_deto_lvl2);

    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

    for (uint16_t m_n = 0; m_n < farbe_yama_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe_yama[m_n].r, farbe_yama[m_n].g, farbe_yama[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe_yama[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe_yama[m_n].index));
    }
    PRINTGLERROR


    // WERDER
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_WERDER]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_WERDER]);

    pthread_mutex_lock(&mutex_deto_lvl3);
    if (new_tex_lvl3 == 1) {
        glBufferData(GL_ARRAY_BUFFER, 20 * verts_werder_len, verts_F_werder, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_werder_len * sizeof (short), index_werder, GL_DYNAMIC_DRAW);
        new_tex_lvl3 = 0;
        PRINTGLERROR
    }
    pthread_mutex_unlock(&mutex_deto_lvl3);

    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

    for (uint16_t m_n = 0; m_n < farbe_werder_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe_werder[m_n].r, farbe_werder[m_n].g, farbe_werder[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe_werder[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe_werder[m_n].index));
    }

    ///////////////// DEBUG
    float color[] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
        1.0f, 0.0f, 1.0f,
        0.0f, 1.0f, 1.0f
    };

    for (uint16_t m = 0; m < farbe_werder_len; m++) {
        for (uint16_t n = 0; n < (farbe_werder[m].len / 3); n++) {
            int c = (n % (sizeof (color) / sizeof (color[0]) / 3))*3;

            glUniform3f(userData->colorLocMap, color[c], color[c + 1], color[c + 2]);
            glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(3 * n));

        }
    }
    PRINTGLERROR

    // Auto
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_AUTO]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_AUTO]);

    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

    g_x = posData->o_x;
    g_y = -posData->o_y;

    DoubletoFix(&g_x, posData->gi_x);
    DoubletoFix(&g_y, posData->gi_y);

    glUniform2i(userData->offsetHLocMap, posData->gi_x[0], posData->gi_y[0]);
    glUniform2f(userData->offsetLLocMap, fmod(g_x, 1.0f), fmod(g_y, 1.0f));

    glUniform3f(userData->colorLocMap, 1.0f, 0.0f, 0.0f);
    glDrawElements(GL_TRIANGLES, userData->auto_len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


#ifdef __RASPI__
    // GUI
    glUseProgram(userData->programGUI);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(userData->samplerLocGui, 0);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glVertexAttribPointer(userData->positionLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->texCoordLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));
    // Buttons
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[0]);
    glDrawArrays(GL_TRIANGLE_STRIP, 3, 4);
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[1]);
    glDrawArrays(GL_TRIANGLE_STRIP, 7, 4);
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[3]);
    glDrawArrays(GL_TRIANGLE_STRIP, 11, 4);

    // Text
    //renderInt(userData, posData->obd_speed, -1.0f, 0.54f);
    //renderFloat(userData, posData->g_x, -1.0f, 0.54f);
    //renderFloat(userData, posData->g_y, -1.0f, 0.4f);

    // GPS_I
    glUseProgram(userData->programKreis);
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glVertexAttribPointer(userData->positionLocKreis, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->texCoordLocKreis, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));
    glUniform3f(userData->colorLocKreis, gps_status[0], gps_status[1], gps_status[2]);
    glDrawArrays(GL_TRIANGLE_STRIP, 15, 4);
    PRINTGLERROR
#endif
}

void Button(ESContext *esContext, int x, int y) {
    POS_T *posData = esContext->posData;

}

void Key(ESContext *esContext, unsigned char a, int b, int c) {
    POS_T *posData = esContext->posData;
    GPS_T *gpsData = esContext->gpsData;
    uint l = 0;

    //printf("%i\n", a);
    if (a == 86) {
        a = 35;
    }
    if (a == 82) {
        a = 61;
    }
    if (a == 80) {
        a = 111;
    }
    if (a == 83) {
        a = 113;
    }
    if (a == 88) {
        a = 116;
    }
    if (a == 85) {
        a = 114;
    }
    switch (a) {
        case 84:
            if (gui_mode == _2D_) {
                posData->o_x = 0.0f;
                posData->o_y = 0.0f;
            }
            break;
        case 111:
            if (gui_mode == _2D_) {
                posData->o_y += 0.1f;
            }
            break;
        case 116:
            if (gui_mode == _2D_) {
                posData->o_y -= 0.1f;
            }
            break;
        case 113:
            if (gui_mode == _2D_) {
                posData->o_x -= 0.1f;
            }
            break;
        case 114:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
            }
            break;
        case 79:
            if (gui_mode == _2D_) {
                posData->o_x -= 0.1f;
                posData->o_y += 0.1f;
            }
            break;
        case 81:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
                posData->o_y += 0.1f;
            }
            break;
        case 89:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
                posData->o_y -= 0.1f;
            }
            break;
        case 87:
            if (gui_mode == _2D_) {
                posData->o_x -= 0.1f;
                posData->o_y -= 0.1f;
            }
            break;
        case 119:
            posData->v += 30.0f;
            PRINTF("v: %f\n", posData->v);
            break;
        case 115:
            posData->v -= 30.0f;
            PRINTF("v: %f\n", posData->v);
            break;
        case 97:
            posData->angle -= 3.0f;
            if (posData->angle < 0.0f) {
                posData->angle += 360.0f;
            }
            PRINTF("angle: %f\n", posData->angle);
            break;
        case 100:
            posData->angle += 3.0f;
            if (posData->angle >= 360.0f) {
                posData->angle -= 360.0f;
            }
            PRINTF("angle: %f\n", posData->angle);
            break;
        case 61:
            if (ORTHO < 10.0) {
                ORTHO += 1.0f;
                esMatrixLoadIdentity(&perspective);
                esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
            }
            break;
        case 35:
            if (ORTHO > 1.0) {
                ORTHO -= 1.0f;
                esMatrixLoadIdentity(&perspective);
                esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
            }
            break;
        case 104:
            if (gui_mode == _2D_) {
                T_Y = V_Z_DEFAULT;
                gui_mode = _3D_;
                posData->o_y = 0.3f;
            } else if (gui_mode == _3D_) {

                T_Y = 0.0f;
                gui_mode = _2D_;
                posData->o_x = 0.0f;
                posData->o_y = 0.0f;
            }
            break;
    }

}

int linaere_suche_TVE(T_V_E *M, int count, int id) {
    for (int a = 0; a < count; a++) {
        if (M[a].id == id) {

            return a;
        }
    }
    return -1;
}

unsigned short linaere_suche_farbe(T_V_E *M, unsigned short count, float r, float g, float b) {
    for (unsigned short a = 0; a < count; a++) {
        if ((M[a].r == r) &&(M[a].g == g)&&(M[a].b == b)) {

            return a;
        }
    }
    return __SHRT_MAX__;
}

unsigned short linaere_suche_VERTS_F(float *M1, unsigned short count, float *M2) {
    for (unsigned short n = 0; n < count; n++) {
        if (memcmp(&M1[5 * n], M2, 20) == 0) {

            return n;
        }

        /*if ((M[5 * n] == x1) && (M[5 * n + 1] == y1) && (M[5 * n + 3] == x2)&& (M[5 * n + 4] == y2)) {
            return n;
        }*/
    }
    return USHRT_MAX;
}

unsigned short linaere_suche_VERTS(float *M, unsigned short count, float x, float y) {
    for (unsigned short n = 0; n < count; n++) {
        if ((M[3 * n] == x) && (M[3 * n + 1] == y)) {

            return n;
        }
    }
    return USHRT_MAX;
}

unsigned short binaere_suche(T_V_E *M, unsigned short count, int id) {
    unsigned short mitte;
    unsigned short links = 0;

    if (count == 0) return __SHRT_MAX__;

    unsigned short rechts = count - 1;

    while (links <= rechts) {
        mitte = links + ((rechts - links) / 2);

        if (M[mitte].id == id) {
            return mitte;
        } else {
            if (M[mitte].id > id) {
                rechts = mitte - 1;
            } else {

                links = mitte + 1;
            }
        }
    }
    return __SHRT_MAX__;
}

int cmpfunc_farbe(const void * a, const void * b) {
    float r_a = ((T_V_E *) a)->r;
    float g_a = ((T_V_E *) a)->g;
    float b_a = ((T_V_E *) a)->b;

    float r_b = ((T_V_E *) b)->r;
    float g_b = ((T_V_E *) b)->g;
    float b_b = ((T_V_E *) b)->b;

    if (r_a < r_b)
        return -1;
    else if (r_a > r_b)
        return 1;

    if (g_a < g_b)
        return -1;
    else if (g_a > g_b)
        return 1;

    if (b_a < b_b)
        return -1;
    else if (b_a > b_b)

        return 1;

    return 0;
}

int cmpfunc_id(const void * a, const void * b) {
    return (((T_V_E*) a)->id - ((T_V_E*) b)->id);
}

T_IO io_eigenschaften[5];

void initPOS(ESContext * esContext) {
    POS_T *posData = esContext->posData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    io_eigenschaften[0].val = &posData->g_x;
    io_eigenschaften[0].len = sizeof (double);

    io_eigenschaften[1].val = &posData->g_y;
    io_eigenschaften[1].len = sizeof (double);

    io_eigenschaften[2].val = &posData->gps_altitude;
    io_eigenschaften[2].len = sizeof (double);

    io_eigenschaften[3].val = &posData->angle;
    io_eigenschaften[3].len = sizeof (float);

    io_eigenschaften[4].val = &posData->gps_file;
    io_eigenschaften[4].len = sizeof (uint16_t);

    eigenschaften = fopen(HOME "sit2d_pos", "rb+");
    if (eigenschaften == NULL) {
        eigenschaften = fopen(HOME "sit2d_pos", "wb");

        printf("\rinitPOS: sit2d_pos leer.\n");
        posData->gps_latitude = 52.131538;
        posData->gps_longitude = 11.653050;
        posData->gps_altitude = 0.0;
        posData->angle = 0.0f;

        posData->gps_file = 1;

        posData->g_x = (posData->gps_longitude + 180.0) / 360.0 * pow2Z;
        posData->g_y = (1.0 - log(tan(posData->gps_latitude * rad2deg) + 1.0 / cos(posData->gps_latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;

        for (int a = 0; a< sizeof (io_eigenschaften) / sizeof (T_IO); a++) {
            fwrite(io_eigenschaften[a].val, io_eigenschaften[a].len, 1, eigenschaften);
        }
        fflush(eigenschaften);
    } else {
        for (int a = 0; a< sizeof (io_eigenschaften) / sizeof (T_IO); a++) {
            fread(io_eigenschaften[a].val, io_eigenschaften[a].len, 1, eigenschaften);
        }
        /*posData->gps_file++;
        fseek(eigenschaften, 28, SEEK_SET);
        fwrite(io_eigenschaften[4].val, io_eigenschaften[4].len, 1, eigenschaften);
        fflush(eigenschaften);*/
    }

    DoubletoFix(&posData->g_x, posData->gi_x);
    DoubletoFix(&posData->g_y, posData->gi_y);

    posData->osmS = 40075017.0f * cos(posData->gps_latitude * M_PI / 180.0) / pow(2.0, 18.0 + 8.0) * IMAGE_SIZE;

    printf("\rinitPOS: g: %f %f a: %f\n", posData->g_x, posData->g_y, posData->angle);
}

void * thread_foss(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    struct timespec spec0, spec1;
    uint_fast8_t display0[] = {0, 2, 3}, display1[] = {1, 4, 5};
    uint_fast8_t display0_c = 0, display1_c = 0;
    posData->display = &display0[display0_c];
    //posData->display = &display1[2];

    PRINTF("\rFOSS gestartet\n");

    time_t t;
    struct tm tm;
    uint_fast16_t f = 0, d_f[4], d_button = 0;
    memset(&d_f[0], 128, sizeof (d_f));
    uint32_t button, button_old;

    //volatile unsigned int *gpio;
    gpio_init();
    gpio_lcd_init();

    // Intro laden
    gpio_lcd_send_byte(0x40, GPIO_LOW);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);

    gpio_lcd_send_byte(0x58, GPIO_LOW);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);

    gpio_lcd_send_byte(0x48, GPIO_LOW);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);

    gpio_lcd_send_byte(0x60, GPIO_LOW);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);

    gpio_lcd_send_byte(0x50, GPIO_LOW);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);

    gpio_lcd_send_byte(0x68, GPIO_LOW);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);

    // Intro abspielen
    gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(32, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 2, GPIO_LOW);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 2, GPIO_LOW);
    gpio_lcd_send_byte(3, GPIO_HIGH);
    gpio_lcd_send_byte(3, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 4, GPIO_LOW);
    gpio_lcd_send_byte(1, GPIO_HIGH);
    gpio_lcd_send_byte(1, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 4, GPIO_LOW);
    gpio_lcd_send_byte(4, GPIO_HIGH);
    gpio_lcd_send_byte(4, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 6, GPIO_LOW);
    gpio_lcd_send_byte(2, GPIO_HIGH);
    gpio_lcd_send_byte(2, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 6, GPIO_LOW);
    gpio_lcd_send_byte(5, GPIO_HIGH);
    gpio_lcd_send_byte(5, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 8, GPIO_LOW);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 8, GPIO_LOW);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);
    gpio_lcd_send_byte(255, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 12, GPIO_LOW);
    gpio_lcd_send_byte(2, GPIO_HIGH);
    gpio_lcd_send_byte(2, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 12, GPIO_LOW);
    gpio_lcd_send_byte(5, GPIO_HIGH);
    gpio_lcd_send_byte(5, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 14, GPIO_LOW);
    gpio_lcd_send_byte(1, GPIO_HIGH);
    gpio_lcd_send_byte(1, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 14, GPIO_LOW);
    gpio_lcd_send_byte(4, GPIO_HIGH);
    gpio_lcd_send_byte(4, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 16, GPIO_LOW);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(0, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 16, GPIO_LOW);
    gpio_lcd_send_byte(3, GPIO_HIGH);
    gpio_lcd_send_byte(3, GPIO_HIGH);

    gpio_lcd_send_byte(LCD_LINE_1 + 18, GPIO_LOW);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(LCD_LINE_2 + 18, GPIO_LOW);
    gpio_lcd_send_byte(32, GPIO_HIGH);
    gpio_lcd_send_byte(32, GPIO_HIGH);


    for (uint_fast8_t f = 0; f < 20; f++) {
        gpio_lcd_send_byte(LCD_LINE_1 + f, GPIO_LOW);
        gpio_lcd_send_byte(32, GPIO_HIGH);
        gpio_lcd_send_byte(LCD_LINE_2 + f, GPIO_LOW);
        gpio_lcd_send_byte(32, GPIO_HIGH);
    }

    gpio_set_lcd_maske(*posData->display, &posData->obd_speed);

    char wert1[20], wert2[20], wert3[20], wert4[20];
    char wert1_old[20], wert2_old[20], wert3_old[20], wert4_old[20];
    uint_fast8_t old_rpm = 0;
    float old_speed = 0.0f;

    clock_gettime(CLOCK_MONOTONIC, &spec0);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &spec1);
        f = (spec1.tv_sec - spec0.tv_sec) * 1000 + (spec1.tv_nsec - spec0.tv_nsec) / 1000000;
        //printf("\r%ums\n", f);
        spec0 = spec1;

        gpio_get_button(&button);

        if ((button & (1 << 17)) >> 17 == 0 && (button_old & (1 << 17)) >> 17 == 1) {
            display1_c = sizeof (display1) / sizeof (display1[0]) - 1;
            display0_c++;
            display0_c %= sizeof (display0) / sizeof (display0[0]);
            posData->display = &display0[display0_c];
            memset(&d_f[0], 128, sizeof (d_f));
            gpio_set_lcd_maske(*posData->display, &posData->obd_speed);
            memset(&wert3_old[0], 0, sizeof (wert3_old));
        }
        if ((button & (1 << 18)) >> 18 == 0 && (button_old & (1 << 18)) >> 18 == 1) {
            display0_c = sizeof (display0) / sizeof (display0[0]) - 1;
            display1_c++;
            display1_c %= sizeof (display1) / sizeof (display1[0]);
            posData->display = &display1[display1_c];
            memset(&d_f[0], 128, sizeof (d_f));
            gpio_set_lcd_maske(*posData->display, &posData->obd_speed);
        }
        button_old = button;

        if (*posData->display == 0) {
            t = time(NULL);
            tm = *localtime(&t);

            float lps = 0.0f;
            if (posData->obd_speed != 0.0f) {
                lps = posData->obd_speed / (posData->obd_maf * 0.326530612 * (1.0f + posData->obd_stft / 100.0f));
            } else {
                lps = posData->obd_maf * 0.326530612 * (1.0f + posData->obd_stft / 100.0f);
            }
            if (old_speed == 0.0f && posData->obd_speed > 0.0f) {
                gpio_lcd_send_byte(LCD_LINE_1 + 10, GPIO_LOW);
                gpio_lcd_send_byte('K', GPIO_HIGH);
                gpio_lcd_send_byte('M', GPIO_HIGH);
                gpio_lcd_send_byte('/', GPIO_HIGH);
                gpio_lcd_send_byte('L', GPIO_HIGH);
                gpio_lcd_send_byte(':', GPIO_HIGH);
            } else if (old_speed > 0.0f && posData->obd_speed == 0.0f) {
                gpio_lcd_send_byte(LCD_LINE_1 + 10, GPIO_LOW);
                gpio_lcd_send_byte(32, GPIO_HIGH);
                gpio_lcd_send_byte('L', GPIO_HIGH);
                gpio_lcd_send_byte('/', GPIO_HIGH);
                gpio_lcd_send_byte('H', GPIO_HIGH);
                gpio_lcd_send_byte(':', GPIO_HIGH);
            }
            old_speed = posData->obd_speed;

            strftime(wert1, 6, "%R", &tm);
            snprintf(wert2, 6, "%4.1f", lps);
            snprintf(wert3, 6, "%4.1f", ((float) posData->obd_volt) / 10.0f);
            snprintf(wert4, 6, "%3.0f", posData->i2c_temp);

            d_f[0] += f;
            if (d_f[0] > 2000) {
                gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }

                gpio_lcd_send_byte(LCD_LINE_2 + 17, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 3; a++) {
                    gpio_lcd_send_byte(wert4[a], GPIO_HIGH);
                }
                d_f[0] = 0;
            }

            d_f[2] += f;
            if (d_f[2] > 500) {
                int ret = memcmp(wert3, wert3_old, 2);
                if (ret == 0) {
                    gpio_lcd_send_byte(LCD_LINE_2 + 6, GPIO_LOW);
                    gpio_lcd_send_byte(wert3[3], GPIO_HIGH);
                } else {
                    gpio_lcd_send_byte(LCD_LINE_2 + 3, GPIO_LOW);
                    for (uint_fast8_t a = 0; a < 4; a++) {
                        gpio_lcd_send_byte(wert3[a], GPIO_HIGH);
                    }
                }
                memcpy(&wert3_old[0], &wert3[0], 4);

                gpio_lcd_send_byte(LCD_LINE_1 + 16, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert2[a], GPIO_HIGH);
                }
                d_f[2] = 0;
            }

        } else if (*posData->display == 1) {

            snprintf(wert1, 8, "%1.1f/%1.1f", posData->obd_o21, posData->obd_o22);
            snprintf(wert2, 7, "%2.0f", posData->obd_iat);
            snprintf(wert3, 7, "%5.1f", posData->obd_tia);
            snprintf(wert4, 7, "%4.1f", posData->obd_maf);

            d_f[0] += f;
            if (d_f[0] > 500) {
                gpio_lcd_send_byte(LCD_LINE_1 + 4, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 7; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }
                gpio_lcd_send_byte(LCD_LINE_1 + 18, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 2; a++) {
                    gpio_lcd_send_byte(wert2[a], GPIO_HIGH);
                }
                gpio_lcd_send_byte(LCD_LINE_2 + 5, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert3[a], GPIO_HIGH);
                }
                gpio_lcd_send_byte(LCD_LINE_2 + 16, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 4; a++) {
                    gpio_lcd_send_byte(wert4[a], GPIO_HIGH);
                }

                d_f[0] = 0;
            }
        } else if (*posData->display == 2) {
            if (gpsData->fix_type == 2) {
                snprintf(wert1, 3, "%s", "2D");
            } else if (gpsData->fix_type == 3) {
                snprintf(wert1, 3, "%s", "3D");
            } else {
                snprintf(wert1, 3, "%s", "NO");
            }
            snprintf(wert2, 6, "%2.0i/%2.0i", gpsData->satellites_tracked, gpsData->total_sats);
            snprintf(wert3, 5, "%4.1f", gpsData->HDOP);

            d_f[0] += f;
            if (d_f[0] > 1000) {
                gpio_lcd_send_byte(LCD_LINE_1 + 5, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 2; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }
                gpio_lcd_send_byte(LCD_LINE_1 + 15, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert2[a], GPIO_HIGH);
                }

                int ret = memcmp(wert3, wert3_old, 2);
                if (ret == 0) {
                    gpio_lcd_send_byte(LCD_LINE_2 + 8, GPIO_LOW);
                    gpio_lcd_send_byte(wert3[3], GPIO_HIGH);
                } else {
                    gpio_lcd_send_byte(LCD_LINE_2 + 5, GPIO_LOW);
                    for (uint_fast8_t a = 0; a < 4; a++) {
                        gpio_lcd_send_byte(wert3[a], GPIO_HIGH);
                    }
                }
                memcpy(&wert3_old[0], &wert3[0], 4);

                d_f[0] = 0;
            }
        } else if (*posData->display == 3) {

            snprintf(wert1, 6, "%4.1f", posData->i2c_angle);
            printf("%s\n", wert1);
            d_f[0] += f;
            if (d_f[0] > 1000) {
                gpio_lcd_send_byte(LCD_LINE_1 + 2, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 6; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }
                d_f[0] = 0;
            }

            /*char s_rpm[20];

            uint_fast8_t rpm = (uint_fast8_t) (posData->obd_rpm / 5000.0f * 20.0f) - 1;

            if (rpm != old_rpm) {
                if (rpm < old_rpm) {
                    gpio_lcd_send_byte(LCD_LINE_1 + rpm + 1, GPIO_LOW);
                    for (uint_fast8_t a = 0; a < old_rpm - rpm; a++) {
                        gpio_lcd_send_byte(" ", GPIO_HIGH);
                    }
                } else {
                    gpio_lcd_send_byte(LCD_LINE_1 + old_rpm + 1, GPIO_LOW);
                    for (uint_fast8_t a = 0; a < rpm - old_rpm; a++) {
                        gpio_lcd_send_byte("#", GPIO_HIGH);
                    }
                }
                old_rpm = rpm;
            }

            snprintf(wert2, 5, "%3.0f", posData->obd_speed);
            if (d_f[0] > 500) {
                gpio_lcd_send_byte(LCD_LINE_2 + 9, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 3; a++) {
                    gpio_lcd_send_byte(wert2[a], GPIO_HIGH);
                }
                d_f[0] = 0;
            }
             */
        } else if (*posData->display == 4) {
            snprintf(wert1, 7, "%5.1f", posData->obd_stft);
            snprintf(wert2, 7, "%4.0f", posData->obd_coolant);
            snprintf(wert3, 7, "%5.1f", posData->obd_ltft);

            d_f[0] += f;
            if (d_f[0] > 500) {
                gpio_lcd_send_byte(LCD_LINE_1 + 5, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }
                gpio_lcd_send_byte(LCD_LINE_2 + 5, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert3[a], GPIO_HIGH);
                }

                d_f[0] = 0;
            }

            d_f[1] += f;
            if (d_f[1] > 1000) {
                gpio_lcd_send_byte(LCD_LINE_1 + 16, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 5; a++) {
                    gpio_lcd_send_byte(wert2[a], GPIO_HIGH);
                }
                d_f[1] = 0;
            }

        } else if (*posData->display == 5) {
            snprintf(wert1, 7, "%2i", posData->obd_dct_count);
            d_f[0] += f;
            if (d_f[0] > 1000) {
                gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
                for (uint_fast8_t a = 0; a < 2; a++) {
                    gpio_lcd_send_byte(wert1[a], GPIO_HIGH);
                }

                if (posData->obd_dct_count > 0) {
                    gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
                    for (uint_fast8_t a = 0; a < 4; a++) {
                        gpio_lcd_send_byte(posData->obf_dtc1[a], GPIO_HIGH);
                    }
                    if (strncmp(posData->obf_dtc2, "0000", 4) != 0) {
                        gpio_lcd_send_byte(',', GPIO_HIGH);
                        for (uint_fast8_t a = 0; a < 4; a++) {
                            gpio_lcd_send_byte(posData->obf_dtc2[a], GPIO_HIGH);
                        }
                    }
                    if (strncmp(posData->obf_dtc3, "0000", 4) != 0) {
                        gpio_lcd_send_byte(',', GPIO_HIGH);
                        for (uint_fast8_t a = 0; a < 4; a++) {
                            gpio_lcd_send_byte(posData->obf_dtc3[a], GPIO_HIGH);
                        }
                    }
                }
                d_f[0] = 0;
            }

        }
        //printf("|%s|%s|%s|%s|\n", wert1, wert2, wert3, wert4);
        usleep(100000);

    }
    return NULL;
}

void * thread_ubongo(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    struct timespec t1, t2;
    int gpsd_init = -1;
    double old_g_x, old_g_y, dist;
    size_t max_len = 0;
    float deg;
    int64_t gi_x, gi_y;
    uint32_t stamp = 0, deltatime = 0, sumtime = 0, counter = 0;

    //struct timespec spec0, spec1;

#ifdef __SENSOR_HUM__
    float i2c_hum, i2c_temp, i2c_angle;
    int i2c_init = -1;
    enum e_i2c_status i2c_status;
#endif

    T_IO io_gio[] = {
        {&deltatime, sizeof (int)},
        {&stamp, sizeof (stamp)},
        {&gpsData->fix_type, sizeof (gpsData->fix_type)},
        {&gpsData->g_x, sizeof (double)},
        {&gpsData->g_y, sizeof (double)},
        {&gi_x, sizeof (int64_t)},
        {&gi_y, sizeof (int64_t)},
        {&dist, sizeof (double)},
        {&posData->obd_speed, sizeof (float)},
        {&posData->obd_volt, sizeof (posData->obd_volt)},
        {&deg, sizeof (float)},
        {&posData->i2c_angle, sizeof (posData->i2c_angle)},
    };

    PRINTF("\rUBONGO gestartet\n");

    markierung.r = 0.0f;
    markierung.g = 0.0f;
    markierung.b = 1.0f;

    printf("io_gps:");
    for (int a = 0; a<sizeof (io_gio) / sizeof (T_IO); a++) {
        max_len += io_gio[a].len;
        printf(" %zu", io_gio[a].len);
    }
    printf(" = %zibyte\n", max_len);
    printf("\n");

    sleep(2);
    clock_gettime(CLOCK_MONOTONIC, &t1);

    while (1) {
        //printf("asd\n");
        old_g_x = gpsData->g_x;
        old_g_y = gpsData->g_y;

#ifdef __SENSOR_HUM__
        if (i2c_init != -1) {
            i2c_status = senseHat_read(&i2c_init, &i2c_hum, &i2c_temp, &i2c_angle);
            posData->i2c_hum = i2c_hum;
            posData->i2c_temp = i2c_temp;
            posData->i2c_angle = i2c_angle;
            //close(i2c_init);
            //i2c_init = -1;
        } else {
            i2c_status = senseHat_init(&i2c_init, "/home/pi/uds_socket");
        }
#endif
#ifdef __GPS__
        if (gpsd_init != -1) {
            //printf("####21 %i\n", gpsd_init);
            gps_read(&gpsd_init, gpsData);
            //printf("####22\n");
            if (gpsData->fix_type > 1 && gpsData->PDOP < 6) {
                gps_status[0] = 0.0f;
                gps_status[2] = 1.0f;

                //printf("%f %f\n", gpsData->longitude, gpsData->latitude);

                gpsData->g_x = (gpsData->longitude + 180.0) / 360.0 * pow2Z;
                gpsData->g_y = (1.0 - log(tan(gpsData->latitude * rad2deg) + 1.0 / cos(gpsData->latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;

                dist = sqrt(pow(old_g_x - gpsData->g_x, 2) + pow(old_g_y - gpsData->g_y, 2)) * posData->osmS;
            } else {
                gps_status[0] = 1.0f;
                gps_status[2] = 0.0f;
            }
        } else {
            //printf("####gpsd_init: %i\n", gpsd_init);
            gps_open(&gpsd_init, "/dev/ttyACM0");
            //printf("####gpsd_init: %i\n", gpsd_init);
        }
#endif
        clock_gettime(CLOCK_MONOTONIC, &t2);
        deltatime = (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_nsec - t1.tv_nsec) / 1000000;
        t1 = t2;
        if (gps_in != NULL) {
            counter++;
            for (int a = 0; a<sizeof (io_gio) / sizeof (T_IO); a++) {
                fread(io_gio[a].val, io_gio[a].len, 1, gps_in);
            }
            gpsData->stamp = stamp;
            gps_status[0] = 0.0f;
            gps_status[2] = 1.0f;
            //printf("%i: %f\n", counter, deg);

            //if (counter != 98) {
            //    continue;
            //}

            usleep(deltatime * 1000);
        } else {
            stamp = gpsData->stamp;
            gi_x = fp64(gpsData->g_x, 32);
            gi_y = fp64(gpsData->g_y, 32);
            for (int a = 0; a<sizeof (io_gio) / sizeof (T_IO); a++) {
                fwrite(io_gio[a].val, io_gio[a].len, 1, gps_out);
            }
            fflush(gps_out);

            usleep(300000); // 300ms
        }
        fseek(eigenschaften, 0, SEEK_SET);
        fwrite(io_eigenschaften[0].val, io_eigenschaften[0].len, 1, eigenschaften);
        fwrite(io_eigenschaften[1].val, io_eigenschaften[1].len, 1, eigenschaften);
        fwrite(io_eigenschaften[2].val, io_eigenschaften[2].len, 1, eigenschaften);
        fwrite(io_eigenschaften[3].val, io_eigenschaften[3].len, 1, eigenschaften);
        fflush(eigenschaften);


        if (gpsData->g_x == 0.0) {
            continue;
        }
        double V[2] = {gpsData->g_x - old_g_x, gpsData->g_y - old_g_y};

        //printf("------------------------\n");
        double old_hc = 10.0;
        int32_t ret = -1;
        for (uint16_t n = 1; n < route_len; n++) {
            double A[2] = {route[n - 1].g_x, route[n - 1].g_y};
            double B[2] = {route[n].g_x, route[n].g_y};

            if (route[n - 1].id != route[n].id) {
                continue;
            }

            double a = pow(B[0] - gpsData->g_x, 2.0) + pow(B[1] - gpsData->g_y, 2.0);
            double b = pow(A[0] - gpsData->g_x, 2.0) + pow(A[1] - gpsData->g_y, 2.0);
            double c = pow(A[0] - B[0], 2.0) + pow(A[1] - B[1], 2.0);

            double c1 = (c + b - a) / (2 * sqrt(c));
            double c2 = sqrt(c) - c1;

            if (c1 < 0 || c2 < 0) {
                continue;
            }

            double hc = sqrt(b - pow(c1, 2.0));
            //printf("%i: %i %f\n", n, route[n].id, hc);
            if (old_hc > hc) {
                ret = n;
                old_hc = hc;
            }
        }
        if (ret != -1) {
            double A[2] = {route[ret - 1].g_x, route[ret - 1].g_y};
            double B[2] = {route[ret].g_x, route[ret].g_y};

            double AB[2] = {B[0] - A[0], B[1] - A[1]};
            double AC[2] = {gpsData->g_x - A[0], gpsData->g_y - A[1]};

            double c = pow(AB[0], 2.0) + pow(AB[1], 2.0);
            double tmp = (AB[0] * AC[0] + AB[1] * AC[1]) / c;

            if (sumtime > 2000) {
                posData->g_x = AB[0] * tmp + A[0];
                posData->g_y = AB[1] * tmp + A[1];
                sumtime = 0;
            } else {
                sumtime += deltatime;
            }

            double alpha = acos((AB[0] * V[0] + AB[1] * V[1]) / (sqrt(c) * sqrt(pow(V[0], 2.0) + pow(V[1], 2.0)))) * 180.0f / M_PI;


            V[0] = (alpha > 90.0f) ? -1.0 * AB[0] : AB[0];
            V[1] = (alpha > 90.0f) ? -1.0 * AB[1] : AB[1];

            posData->angle = acos((0.0f * V[0] + 1.0f * V[1]) / (1.0f * sqrt(pow(V[0], 2.0) + pow(V[1], 2.0)))) * 180.0f / M_PI;
            //printf("[%f, %f] [%f, %f] %f %f\n", AB[0], AB[1], V[0], V[1], posData->angle, posData->obd_speed);

            double a = sqrt(pow(posData->obd_speed, 2.0) / (pow(V[0], 2.0) + pow(V[1], 2.0)));

            posData->v_x = a * V[0];
            posData->v_y = -a * V[1];
        } else {
            posData->g_x = gpsData->g_x;
            posData->g_y = gpsData->g_y;
            //printf("%f %f\n", posData->g_x, posData->g_y);
        }

        //printf("%f %f\n", posData->g_x, posData->g_y);
        //printf("############################\n");
        //printf("####7\n");
    }

    return NULL;
}

#define SHIFT 4
#define DB_LVL1 "sit2d_6_lvl1.db"
#define DB_LVL2 "sit2d_6_lvl2.db"
#define DB_LVL3 "sit2d_6_lvl3.db"

#define SQL_ABFRAGE "SELECT 2 as l_lvl,l_cf_r,l_cf_g,l_cf_b,t.L_ID,V_LON,V_LAT\
  FROM (SELECT DISTINCT L_ID\
             FROM V\
            WHERE V_T_LON BETWEEN %i AND %i AND\
                  V_T_LAT BETWEEN %i AND %i) t\
       JOIN V g ON t.L_ID = g.L_ID\
       JOIN L ON g.L_Id = L.L_ID\
       ORDER BY l_lvl,l_cf_r,l_cf_g,l_cf_b,t.L_ID,I_SEQ"

#define SQL_ABFRAGE2 "SELECT distinct l_id,V_LAT,V_LON\
                        FROM R\
                       WHERE T_LON BETWEEN %i AND %i AND\
                             T_LAT BETWEEN %i AND %i\
                    ORDER BY l_id,w_seq"
//L_ID = 27811900

void * thread_deta_lvl1(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;
    PRINTF("\rdeta_lvl1 gestartet\n");

    struct timespec spec0, spec1;

    char sql[700];
    char *zErrMsg = 0;
    int lon1, lat1, d1;

    sleep(1);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &spec0);
        int c = 0;
        for (int16_t d = -8; d < 8; d++) {
            gitter[c] = floor(gpsData->g_x) - 12;
            gitter[c + 1] = floor(gpsData->g_y) - (float) d;
            c += 4;

            gitter[c] = floor(gpsData->g_x) + 12;
            gitter[c + 1] = floor(gpsData->g_y) - (float) d;
            c += 4;
        }
        for (int16_t d = -12; d < 12; d++) {
            gitter[c] = floor(gpsData->g_x) + (float) d;
            gitter[c + 1] = floor(gpsData->g_y) - 12;
            c += 4;

            gitter[c] = floor(gpsData->g_x) + (float) d;
            gitter[c + 1] = floor(gpsData->g_y) + 12;
            c += 4;
        }

        lon1 = (int) floor(posData->g_x);
        lat1 = (int) floor(posData->g_y);
        d1 = (int) floor(BR);

        verts_reis_len = 0;
        tmp_index_reis_len = 0;
        tmp_farbe_reis_len = 0;

        snprintf(sql, 700, SQL_ABFRAGE, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
        //printf("\r%s\n", sql);
        sqlite3_exec(db_1, sql, sqc_vertex_lvl1, 0, &zErrMsg);
        if (zErrMsg != NULL) {
            printf("\n|%s|\n", zErrMsg);
        }

        pthread_mutex_lock(&mutex_deto_lvl1);
        farbe_reis_len = tmp_farbe_reis_len;
        index_reis_len = tmp_index_reis_len;
        memcpy(farbe_reis, tmp_farbe_reis, tmp_farbe_reis_len * sizeof (T_V_E));
        memcpy(index_reis, tmp_index_reis, tmp_index_reis_len * sizeof (unsigned short));

        new_tex_lvl1 = 1;
        pthread_mutex_unlock(&mutex_deto_lvl1);


        printf("deta_lvl1: ");
        TM(spec0, spec1)
        usleep(10000000);
    }

    return NULL;
}

void run_deta_lvl2(double x, double y) {
    char sql[700];
    char *zErrMsg = 0;
    int lon1, lat1, d1;

    lon1 = (int) floor(x);
    lat1 = (int) floor(y);
    d1 = (int) floor(BR);

    verts_yama_len = 0;
    tmp_index_yama_len = 0;
    tmp_farbe_yama_len = 0;

    snprintf(sql, 700, SQL_ABFRAGE, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
    //printf("\r%s\n", sql);
    sqlite3_exec(db_2, sql, sqc_vertex_lvl2, 0, &zErrMsg);
    if (zErrMsg != NULL) {
        printf("\n|%s|\n", zErrMsg);
    }

    pthread_mutex_lock(&mutex_deto_lvl2);
    farbe_yama_len = tmp_farbe_yama_len;
    index_yama_len = tmp_index_yama_len;
    memcpy(farbe_yama, tmp_farbe_yama, tmp_farbe_yama_len * sizeof (T_V_E));
    memcpy(index_yama, tmp_index_yama, tmp_index_yama_len * sizeof (unsigned short));
    new_tex_lvl2 = 1;
    pthread_mutex_unlock(&mutex_deto_lvl2);
}

void * thread_deta_lvl2(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;

    sleep(1);
    PRINTF("\rdeta_lvl2 gestartet\n");

#ifdef __DEBUG__
    struct timespec spec0, spec1;
#endif

    while (1) {
#ifdef __DEBUG__
        clock_gettime(CLOCK_MONOTONIC, &spec0);
#endif
        run_deta_lvl2(posData->g_x + posData->o_x, posData->g_y - posData->o_y);

#ifdef __DEBUG__
        printf("deta_lvl2: ");
        TM(spec0, spec1)
#endif

#ifdef __RASPI__
                usleep(10000000);
#else
                usleep(1000000);
#endif
    }
    return NULL;
}

void run_deta_lvl3(double x, double y) {
    char *zErrMsg;
    char sql[700];

    int lon1 = (int) floor(x);
    int lat1 = (int) floor(y);
    int d1 = (int) floor(BR);
    //printf("%i %i\n", lon1, lat1);
    verts_werder_len = 0;
    tmp_index_werder_len = 0;
    tmp_farbe_werder_len = 0;

    snprintf(sql, 700, SQL_ABFRAGE, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
    //printf("%s\n", sql);
    sqlite3_exec(db_3, sql, sqc_vertex_lvl3, 0, &zErrMsg);
    if (zErrMsg != NULL) {
        printf("\n|%s|\n", zErrMsg);
    }
    pthread_mutex_lock(&mutex_deto_lvl3);
    farbe_werder_len = tmp_farbe_werder_len;
    index_werder_len = tmp_index_werder_len;
    osm_werder_len = tmp_osm_werder_len;
    memcpy(farbe_werder, tmp_farbe_werder, tmp_farbe_werder_len * sizeof (T_V_E));
    memcpy(osm_werder, tmp_osm_werder, tmp_osm_werder_len * sizeof (T_V_E));
    memcpy(index_werder, tmp_index_werder, tmp_index_werder_len * sizeof (unsigned short));
    new_tex_lvl3 = 1;
    pthread_mutex_unlock(&mutex_deto_lvl3);

    route_len = 0;
    tmp_route_len = 0;
    snprintf(sql, 700, SQL_ABFRAGE2, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
    //printf("%s\n", sql);
    sqlite3_exec(db_3, sql, sqc_route_lvl3, 0, &zErrMsg);
    if (zErrMsg != NULL) {
        printf("\n|%s|\n", zErrMsg);
    }
    route_len = tmp_route_len;
    memcpy(route, tmp_route, tmp_route_len * sizeof (T_ROUTE));
}

void * thread_deta_lvl3(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;

    sleep(1);
    PRINTF("\rdeta_lvl3 gestartet\n");

#ifdef __DEBUG__
    struct timespec spec0, spec1;
#endif


    while (1) {
#ifdef __DEBUG__
        clock_gettime(CLOCK_MONOTONIC, &spec0);
#endif
        run_deta_lvl3(posData->g_x + posData->o_x, posData->g_y - posData->o_y);

#ifdef __DEBUG__
        printf("deta_lvl3: ");
        TM(spec0, spec1)
#endif

#ifdef __RASPI__
                usleep(3000000);
#else
                usleep(1000000);
#endif
    }

    return NULL;
}

void * thread_jolla(void *esContext) {
    //POS_T *posData = ((ESContext*) esContext)->posData;
    int fd;
    struct input_event ev[64];
    int i, rb, rawX, rawY;
    char finish = 0;
    //int lk = 0;

    fd = open("/dev/input/event0", O_RDONLY);
    while (fd != -1) {
        rb = read(fd, ev, sizeof (struct input_event) * 64);

        for (i = 0; i < (rb / sizeof (struct input_event)); i++) {
            if (ev[i].type == EV_KEY && ev[i].code == 330 && ev[i].value == 1) {
                finish = 0;
            } else if (ev[i].type == EV_KEY && ev[i].code == 330 && ev[i].value == 0) {
                finish = 1;
            } else if (ev[i].type == EV_ABS && ev[i].code == 0 && ev[i].value > 0) {
                rawX = ev[i].value;
            } else if (ev[i].type == EV_ABS && ev[i].code == 1 && ev[i].value > 0) {
                rawY = ev[i].value;
            }
        }

        if (finish == 1) {
            if (rawX > 710 && rawX < 750 && rawY > 430 && rawY < 480) {
                /*if (*ZOOM > 8) {
                    ZOOM++;
                    PRINTF("ZOOM: %i\n", *ZOOM);
                    setPOS(esContext);
                    PRINTF("jolla: %i/%i %i/%i\n", posData->t_x, posData->gpu_t_x, posData->t_y, posData->gpu_t_y);
                }*/
            } else if (rawX > 760 && rawX < 800 && rawY > 430 && rawY < 480) {
                /*if (*ZOOM < 18) {
                    ZOOM--;
                    PRINTF("ZOOM: %i\n", *ZOOM);
                    setPOS(esContext);
                    PRINTF("jolla: %i/%i %i/%i\n", posData->t_x, posData->gpu_t_x, posData->t_y, posData->gpu_t_y);
                }*/
            }
        }
    }

    close(fd);

    return NULL;
}

double distance_on_geoid(double lat1, double lon1, double lat2, double lon2) {

    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;

    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    double r = 6378137;

    double rho1 = r * cos(lat1);
    double z1 = r * sin(lat1);
    double x1 = rho1 * cos(lon1);
    double y1 = rho1 * sin(lon1);

    double rho2 = r * cos(lat2);
    double z2 = r * sin(lat2);
    double x2 = rho2 * cos(lon2);
    double y2 = rho2 * sin(lon2);

    double dot = (x1 * x2 + y1 * y2 + z1 * z2);

    double cos_theta = dot / (r * r);

    double theta = acos(cos_theta > 1.0f ? 1.0f : 1.0f);

    return r * theta;
}

double getDegrees(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double lam1 = lon1 * M_PI / 180.0;
    double lam2 = lon2 * M_PI / 180.0;

    return atan2(sin(lam2 - lam1) * cos(phi2), cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(lam2 - lam1)) * 180 / M_PI;
}

void * thread_pinto(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    //GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    PRINTF("\rPINTO gestartet\n");

    //int counter = 0;

    //float old_speed = 0.0f; //, deltatime = 0.0f;
    //int deltatime = 0;
    //float deg, old_deg;
    //struct timeval t_start, tv_tmp;
    struct timespec /*to1, to2,*/ t1, t2;
    //int lk = 0, status;
    //double g_x, g_y, old_lat, old_lon, old_g_x, old_g_y, dist;

    //int gpsd_init = -1;

    //int i2c_init = -1;
    //enum e_i2c_status i2c_status;

#ifdef __OBD__    
    // für obd_volt
    char outstr[1024];
    char retbuf[4096];
    int nbytes;
    //     

    int obd_serial = -1;
    enum obd_serial_status obdstatus;
    float tmp_val;

    obdcmds_mode1[0x0D].target = &posData->obd_speed;
    obdcmds_mode1[0x10].target = &posData->obd_maf;
    obdcmds_mode1[0x06].target = &posData->obd_stft;
    obdcmds_mode1[0x11].target = &posData->obd_throttle;
    obdcmds_mode1[0x14].target = &posData->obd_o21;
    obdcmds_mode1[0x15].target = &posData->obd_o22;
    obdcmds_mode1[0x0F].target = &posData->obd_iat;
    obdcmds_mode1[0x0E].target = &posData->obd_tia;
    obdcmds_mode1[0x06].target = &posData->obd_stft;
    obdcmds_mode1[0x07].target = &posData->obd_ltft;
    obdcmds_mode1[0x05].target = &posData->obd_coolant;

    unsigned int display0[] = {100000, 0x06, 0x10, 0x0D, 0x11};
    unsigned int display1[] = {50000, 0x14, 0x15, 0x10, 0x0F, 0x0E};
    unsigned int display4[] = {100000, 0x05, 0x06, 0x07};
#endif

#ifdef __I2C__
    short i2c_x, i2c_y, i2c_z;
#endif

    //int64_t gi_x, gi_y;
    //uint32_t stamp;
    uint32_t deltatime;
    //int32_t tv_sec, tv_usec;
    uint_fast8_t even = 0;

    T_IO io_sensor[] = {
        {&deltatime, sizeof (int)},
        {&posData->obd_speed, sizeof (float)},
        {&posData->obd_volt, sizeof (posData->obd_volt)}
    };
    printf("io_sensor:");
    for (int a = 0; a<sizeof (io_sensor) / sizeof (T_IO); a++) {
        printf(" %zu", io_sensor[a].len);
    }
    printf("\n");

    clock_gettime(CLOCK_MONOTONIC, &t1);
    while (1) {
#ifdef __OBD__     
        //clock_gettime(CLOCK_MONOTONIC, &to1);
        if (obd_serial != -1) {

            if (*posData->display == 0) {
                snprintf(outstr, sizeof (outstr), "%s%s", "ATRV", "\r");
                write(obd_serial, outstr, strlen(outstr));

                nbytes = readserialdata(obd_serial, retbuf, sizeof (retbuf));
                retbuf[4] = '\0';
                sscanf(retbuf, "%f", &tmp_val);
                posData->obd_volt = (uint8_t) (tmp_val * 10.0f);

                for (uint_fast8_t a = 1; a< sizeof (display0) / sizeof (display0[0]); a++) {
                    obdstatus = getobdvalue(obd_serial, display0[a], &tmp_val, obdcmds_mode1[display0[a]].bytes_returned, obdcmds_mode1[display0[a]].conv);
                    if (OBD_SUCCESS == obdstatus) {
                        *((float*) obdcmds_mode1[display0[a]].target) = tmp_val;
                    } else {
                        PRINTF("OBD %x fehler\n", display0[a]);
                        *((float*) obdcmds_mode1[display0[a]].target) = 0.0f;
                        obd_serial = -1;
                    }
                }
                usleep(display0[0]);

            } else if (*posData->display == 1) {
                for (uint_fast8_t a = 1; a< sizeof (display1) / sizeof (display1[0]); a++) {
                    obdstatus = getobdvalue(obd_serial, display1[a], &tmp_val, obdcmds_mode1[display1[a]].bytes_returned, obdcmds_mode1[display1[a]].conv);
                    if (OBD_SUCCESS == obdstatus) {
                        *((float*) obdcmds_mode1[display1[a]].target) = tmp_val;
                    } else {
                        PRINTF("OBD %x fehler\n", display1[a]);
                        *((float*) obdcmds_mode1[display1[a]].target) = 0.0f;
                        obd_serial = -1;
                    }
                }
                usleep(display1[0]);
            } else if (*posData->display == 4) {
                for (uint_fast8_t a = 1; a< sizeof (display4) / sizeof (display4[0]); a++) {
                    obdstatus = getobdvalue(obd_serial, display4[a], &tmp_val, obdcmds_mode1[display4[a]].bytes_returned, obdcmds_mode1[display4[a]].conv);
                    if (OBD_SUCCESS == obdstatus) {
                        *((float*) obdcmds_mode1[display4[a]].target) = tmp_val;
                    } else {
                        PRINTF("OBD %x fehler\n", display4[a]);
                        *((float*) obdcmds_mode1[display4[a]].target) = 0.0f;
                        obd_serial = -1;
                    }
                }
                usleep(display4[0]);
            } else if (*posData->display == 5) {

                /*snprintf(outstr, sizeof (outstr), "%s%s", "0101", "\r");
                write(obd_serial, outstr, strlen(outstr));
                nbytes = readserialdata(obd_serial, retbuf, sizeof (retbuf));
                printf("%s\n", retbuf);
                retbuf[6] = '\0';
                sscanf(&retbuf[4], "%x", &posData->obd_dct_count);
                posData->obd_dct_count -= 0x80;
                 */
                //if (posData->obd_dct_count > 0) {
                snprintf(outstr, sizeof (outstr), "%s%s%c", "03", "\r", '\0');
                write(obd_serial, outstr, strlen(outstr));
                nbytes = readserialdata(obd_serial, retbuf, sizeof (retbuf));

                //printf("%s\n", retbuf);

                if (strncmp(retbuf, "BUS INIT: ERROR", 15) != 0) {
                    memcpy(posData->obf_dtc1, &retbuf[2], 4);
                    memcpy(posData->obf_dtc2, &retbuf[6], 4);
                    memcpy(posData->obf_dtc3, &retbuf[10], 4);
                } else {
                    memset(posData->obf_dtc1, 0, 4);
                    memset(posData->obf_dtc2, 0, 4);
                    memset(posData->obf_dtc3, 0, 4);
                }

                unsigned int asd = 0;
                if (strncmp(posData->obf_dtc1, "0000", 4) == 0) {
                    asd++;
                    if (strncmp(posData->obf_dtc2, "0000", 4) == 0) {
                        asd++;
                        if (strncmp(posData->obf_dtc3, "0000", 4) == 0) {
                            asd++;
                        }
                    }
                }
                posData->obd_dct_count = asd;

                usleep(2000000);
            } else {
                usleep(500000);
            }
        } else {
            if (obd_serial != -1) {
                close(obd_serial);
            }
            obd_serial = init_OBD(OBD_SERIAL);
        }
        //clock_gettime(CLOCK_MONOTONIC, &to2);
        //printf("obd: %ims\n", (to2.tv_sec - to1.tv_sec) * 1000 + (to2.tv_nsec - to1.tv_nsec) / 1000000);
#endif
#ifdef __I2C__
        if (i2c_init != -1) {
            i2c_status = read_i2c(&i2c_init, &i2c_x, &i2c_y, &i2c_z);
            lk = pthread_mutex_lock(&m_pinto);
            posData->i2c_x = i2c_x;
            posData->i2c_y = i2c_y;
            posData->i2c_z = i2c_z;
            pthread_mutex_unlock(&m_pinto);
        } else {
            i2c_status = init_i2c(&i2c_init, HMC5883L_I2C_ADDR);
        }
#endif

        clock_gettime(CLOCK_MONOTONIC, &t2);
        deltatime = (t2.tv_sec - t1.tv_sec) * 1000 + (t2.tv_nsec - t1.tv_nsec) / 1000000;
        t1 = t2;

        if (sensor_in != NULL) {
            for (int a = 0; a<sizeof (io_sensor) / sizeof (T_IO); a++) {
                fread(io_sensor[a].val, io_sensor[a].len, 1, gps_in);
            }
            usleep(deltatime * 1000);
        } else {
            for (int a = 0; a<sizeof (io_sensor) / sizeof (T_IO); a++) {
                fwrite(io_sensor[a].val, io_sensor[a].len, 1, sensor_out);
            }
            fflush(sensor_out);
        }
        even++;
    }

    return NULL;
}

void thread_render(ESContext * esContext) {
    struct timeval t1, t2;
    struct timezone tz;
    float deltatime;
    float totaltime = 0.0f;
    uint_fast16_t frames = 0;
    uint_fast32_t us = 10000;

    gettimeofday(&t1, &tz);
#ifdef __TM__
    struct timespec spec1, spec2;
    uint_fast16_t ms_min = UINT16_MAX, ms_max = 0, ms, ms_avg = 0;
#endif

    while (userInterrupt(esContext) == GL_FALSE) {
        //while (1) {
        gettimeofday(&t2, &tz);
        deltatime = (float) (t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6) * 1000;
        t1 = t2;
#ifdef __TM__
        clock_gettime(CLOCK_MONOTONIC, &spec1);
#endif   
        Update(esContext, deltatime);
        eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);

#ifdef __TM__
        clock_gettime(CLOCK_MONOTONIC, &spec2);

        ms = (spec2.tv_sec - spec1.tv_sec) * 1000 + (spec2.tv_nsec - spec1.tv_nsec) * 1e-6;

        ms_avg += ms;
        if (ms < ms_min)
            ms_min = ms;
        if (ms > ms_max)
            ms_max = ms;

        totaltime += deltatime / 1000.0f;
        frames++;

        if (totaltime > 4.0f) {

            //PRINTF("\rFPS: %3.1f FRAMETIME: %i %4.1f %i  us: %u\n", frames / totaltime, ms_min, (float) ms_avg / frames, ms_max, us);

            if ((frames / totaltime) > 30.0) {
                us += 1000;
            } else {

                us = us > 1000 ? us - 1000 : us;
            }

            totaltime -= 4.0f;
            frames = 0;

            ms_avg = 0;
            ms_min = 4294967295;
            ms_max = 0;

        }
#endif
        usleep(us);
    }
}

static int sqc_vertex_lvl1(void *a, int argc, char **argv, char **azColName) {
    unsigned short gef = USHRT_MAX;
    float tmp_f[] = {atof(argv[1]), atof(argv[2]), atof(argv[3])};
    //unsigned char typ = atoi(argv[0]);
    float tmp_glLineWidth = 5.0f;
    float tmp_verts_F[5] = {0.0f};

    tmp_verts_F[3] = atof(argv[5] + 6);
    tmp_verts_F[4] = atof(argv[6] + 5);
    *(argv[5] + 6) = '\0';
    *(argv[6] + 5) = '\0';
    tmp_verts_F[0] = atof(argv[5]);
    tmp_verts_F[1] = atof(argv[6]);

    memcpy(&verts_F_reis[5 * verts_reis_len], tmp_verts_F, sizeof (tmp_verts_F));
    gef = verts_reis_len;
    verts_reis_len++;
    tmp_index_yama[tmp_index_reis_len] = gef;

    tmp_index_reis_len++;

    if ((tmp_farbe_reis_len == 0) ||
            (tmp_farbe_reis[tmp_farbe_reis_len - 1].r != tmp_f[0] || tmp_farbe_reis[tmp_farbe_reis_len - 1].g != tmp_f[1] || tmp_farbe_reis[tmp_farbe_yama_len - 1].b != tmp_f[2])) {
        tmp_farbe_reis[tmp_farbe_reis_len].r = tmp_f[0];
        tmp_farbe_reis[tmp_farbe_reis_len].g = tmp_f[1];
        tmp_farbe_reis[tmp_farbe_reis_len].b = tmp_f[2];

        //tmp_farbe_yama[tmp_farbe_yama_len].gl_mode = tmp_gl_mode[typ];
        tmp_farbe_reis[tmp_farbe_reis_len].glLineWidth = tmp_glLineWidth;
        //tmp_farbe_yama[tmp_farbe_yama_len].typ = typ;

        tmp_farbe_reis[tmp_farbe_reis_len].index = tmp_index_reis_len - 1;

        tmp_farbe_reis[tmp_farbe_reis_len].len = 1;

        tmp_farbe_reis_len++;
    } else {
        tmp_farbe_reis[tmp_farbe_reis_len - 1].len++;
    }

    //PRINTF("asd\n");
    return 0;
}

static int sqc_vertex_lvl2(void *a, int argc, char **argv, char **azColName) {
    unsigned short gef = USHRT_MAX;
    float tmp_f[] = {atof(argv[1]), atof(argv[2]), atof(argv[3])};
    //unsigned char typ = atoi(argv[0]);
    float tmp_glLineWidth = 5.0f;
    float tmp_verts_F[5] = {0.0f};

    tmp_verts_F[3] = atof(argv[5] + 6);
    tmp_verts_F[4] = atof(argv[6] + 5);
    *(argv[5] + 6) = '\0';
    *(argv[6] + 5) = '\0';
    tmp_verts_F[0] = atof(argv[5]);
    tmp_verts_F[1] = atof(argv[6]);

    memcpy(&verts_F_yama[5 * verts_yama_len], tmp_verts_F, sizeof (tmp_verts_F));
    gef = verts_yama_len;
    verts_yama_len++;
    tmp_index_yama[tmp_index_yama_len] = gef;

    tmp_index_yama_len++;

    if ((tmp_farbe_yama_len == 0) ||
            (tmp_farbe_yama[tmp_farbe_yama_len - 1].r != tmp_f[0] || tmp_farbe_yama[tmp_farbe_yama_len - 1].g != tmp_f[1] || tmp_farbe_yama[tmp_farbe_yama_len - 1].b != tmp_f[2])) {
        tmp_farbe_yama[tmp_farbe_yama_len].r = tmp_f[0];
        tmp_farbe_yama[tmp_farbe_yama_len].g = tmp_f[1];
        tmp_farbe_yama[tmp_farbe_yama_len].b = tmp_f[2];

        //tmp_farbe_yama[tmp_farbe_yama_len].gl_mode = tmp_gl_mode[typ];
        tmp_farbe_yama[tmp_farbe_yama_len].glLineWidth = tmp_glLineWidth;
        //tmp_farbe_yama[tmp_farbe_yama_len].typ = typ;

        tmp_farbe_yama[tmp_farbe_yama_len].index = tmp_index_yama_len - 1;

        tmp_farbe_yama[tmp_farbe_yama_len].len = 1;

        tmp_farbe_yama_len++;
    } else {
        tmp_farbe_yama[tmp_farbe_yama_len - 1].len++;
    }

    //PRINTF("asd\n");
    return 0;
}

static int sqc_vertex_lvl3(void *a, int argc, char **argv, char **azColName) {
    //printf("%s %s\n",argv[5],argv[6]);
    unsigned short gef = USHRT_MAX;
    float tmp_f[] = {atof(argv[1]), atof(argv[2]), atof(argv[3])};
    //int tmp_l_id = atoi(argv[4]);
    float tmp_glLineWidth = 5.0f;
    float tmp_verts_F[5] = {0.0f};

    tmp_verts_F[3] = atof(argv[5] + 6);
    tmp_verts_F[4] = atof(argv[6] + 5);
    *(argv[5] + 6) = '\0';
    *(argv[6] + 5) = '\0';
    tmp_verts_F[0] = atof(argv[5]);
    tmp_verts_F[1] = atof(argv[6]);

    memcpy(&verts_F_werder[5 * verts_werder_len], tmp_verts_F, sizeof (tmp_verts_F));
    gef = verts_werder_len;
    verts_werder_len++;
    tmp_index_werder[tmp_index_werder_len] = gef;

    tmp_index_werder_len++;

    if ((tmp_farbe_werder_len == 0) ||
            (tmp_farbe_werder[tmp_farbe_werder_len - 1].r != tmp_f[0] || tmp_farbe_werder[tmp_farbe_werder_len - 1].g != tmp_f[1] || tmp_farbe_werder[tmp_farbe_werder_len - 1].b != tmp_f[2])) {
        tmp_farbe_werder[tmp_farbe_werder_len].r = tmp_f[0];
        tmp_farbe_werder[tmp_farbe_werder_len].g = tmp_f[1];
        tmp_farbe_werder[tmp_farbe_werder_len].b = tmp_f[2];

        //tmp_farbe_werder[tmp_farbe_werder_len].gl_mode = tmp_gl_mode[typ];
        tmp_farbe_werder[tmp_farbe_werder_len].glLineWidth = tmp_glLineWidth;
        //tmp_farbe_werder[tmp_farbe_werder_len].typ = typ;

        tmp_farbe_werder[tmp_farbe_werder_len].index = tmp_index_werder_len - 1;

        tmp_farbe_werder[tmp_farbe_werder_len].len = 1;

        tmp_farbe_werder_len++;
    } else {
        tmp_farbe_werder[tmp_farbe_werder_len - 1].len++;
    }

    //PRINTF("asd\n");
    return 0;
}

static int sqc_route_lvl3(void *a, int argc, char **argv, char **azColName) {
    //printf("%i %f %f\n", atoi(argv[0]), atof(argv[1]), atof(argv[2]));
    tmp_route[tmp_route_len].id = atoi(argv[0]);
    tmp_route[tmp_route_len].g_y = atof(argv[1]);
    tmp_route[tmp_route_len].g_x = atof(argv[2]);

    tmp_route_len++;

    return 0;
}

void ShutDown(int signum) {
    //void sigfunc(int sig) 
    gpio_lcd_shutdown();
    //gpio_button_led(5, 0);
    UserData *userData = esContext.userData;
    //POS_T *posData = esContext.posData;

    PRINTF("\rthe end is near!\n");
    //fclose(gps_out);
    //fclose(sout);
#ifdef __FIN__
    fclose(gps_in);
#endif

    sqlite3_close(db_1);
    sqlite3_close(db_2);
    sqlite3_close(db_3);

#ifdef __RASPI__
    //gpio_unexport(LCD_BUS, 6);
    //bl_write(BL_OFF);
#endif

    glDeleteProgram(userData->programGUI);
    glDeleteProgram(userData->programKreis);
    glDeleteProgram(userData->programMap);

    /*sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
     */
    exit(EXIT_SUCCESS);
}

void test(ESContext * esContext) {
    UserData *userData = ((ESContext*) esContext)->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;

    uint32_t even = 0;

    /*farbe_werder_len = 1;
    index_werder_len = 3;
    verts_werder_len = 3;

    
    farbe_werder[0].len = 3;
     */
    /*index_werder[0] = 0;
    index_werder[0] = 1;
    index_werder[0] = 2;*/

    //printf("%f %f %f %f %f\n", verts_F_werder[0], verts_F_werder[1], verts_F_werder[2], verts_F_werder[3], verts_F_werder[4]);
    //printf("%f %f %f %f %f\n", verts_F_werder[5], verts_F_werder[6], verts_F_werder[7], verts_F_werder[8], verts_F_werder[9]);
    //printf("%f %f %f %f %f\n", verts_F_werder[10], verts_F_werder[11], verts_F_werder[12], verts_F_werder[13], verts_F_werder[14]);
    /*verts_F_werder[0] = 139560.0f;
    verts_F_werder[1] = 86540.0f;
    verts_F_werder[2] = 0.0f;
    verts_F_werder[3] = -0.5f;
    verts_F_werder[4] = -0.5f;

    verts_F_werder[5] = 139560.0f;
    verts_F_werder[6] = 86540.0f;
    verts_F_werder[7] = 0.0f;
    verts_F_werder[8] = 0.5f;
    verts_F_werder[9] = -0.5f;

    verts_F_werder[10] = 139560.0f;
    verts_F_werder[11] = 86540.0f;
    verts_F_werder[12] = 0.0f;
    verts_F_werder[13] = 0.0f;
    verts_F_werder[14] = 0.5f;*/

    posData->g_x = 139555.26674;
    posData->g_y = 86529.64129;

    DoubletoFix(&posData->g_x, posData->gi_x);
    DoubletoFix(&posData->g_y, posData->gi_y);

    new_tex_lvl3 = 1;

    GLint posH = glGetAttribLocation(userData->programTest, "a_posH");
    GLint posL = glGetAttribLocation(userData->programTest, "a_posL");
    GLint offsetHLocMap = glGetUniformLocation(userData->programTest, "u_offsetH");
    GLint offsetLLocMap = glGetUniformLocation(userData->programTest, "u_offsetL");
    GLint mvpLocMap = glGetUniformLocation(userData->programTest, "u_mvpMatrix");
    GLint colorLocMap = glGetUniformLocation(userData->programTest, "u_color");

    GLuint vbo, vbo2;
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glViewport(0, 0, 800, 480);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &vbo2);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);


    glVertexAttribPointer(posH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(posL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

    glEnableVertexAttribArray(posH);
    glEnableVertexAttribArray(posL);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    while (1) {
        esMatrixLoadIdentity(&modelview);
        esTranslate(&modelview, 0.0f, T_Y, 0.0f);

#ifdef __SSE__
        __m128 row1 = _mm_load_ps(&perspective.m[0][0]);
        __m128 row2 = _mm_load_ps(&perspective.m[1][0]);
        __m128 row3 = _mm_load_ps(&perspective.m[2][0]);
        __m128 row4 = _mm_load_ps(&perspective.m[3][0]);
        for (uint_fast8_t m_m = 0; m_m < 4; m_m++) {
            __m128 brod1 = _mm_set1_ps(modelview.m[m_m][0]);
            __m128 brod2 = _mm_set1_ps(modelview.m[m_m][1]);
            __m128 brod3 = _mm_set1_ps(modelview.m[m_m][2]);
            __m128 brod4 = _mm_set1_ps(modelview.m[m_m][3]);
            __m128 row =
                    _mm_add_ps(_mm_add_ps(_mm_mul_ps(brod1, row1), _mm_mul_ps(brod2, row2)), _mm_add_ps(_mm_mul_ps(brod3, row3), _mm_mul_ps(brod4, row4)));

            _mm_store_ps(&mvpMatrix.m[m_m][0], row);
        }
#else
        esMatrixMultiply(&mvpMatrix, &modelview, &perspective);
#endif

        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(userData->programTest);

        glUniform2i(offsetHLocMap, posData->gi_x[0], posData->gi_y[0]);
        glUniform2f(offsetLLocMap, fmod(posData->g_x, 1.0f), fmod(posData->g_y, 1.0f));
        glUniformMatrix4fv(mvpLocMap, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo2);

        if (even & 1) {
            glBufferData(GL_ARRAY_BUFFER, 20 * verts_werder_len, verts_F_werder, GL_DYNAMIC_DRAW);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_werder_len * sizeof (short), index_werder, GL_DYNAMIC_DRAW);
        }
        glUniform3f(colorLocMap, farbe_werder[0].r, farbe_werder[0].g, farbe_werder[0].b);
        glDrawElements(GL_TRIANGLES, farbe_werder[0].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe_werder[0].index));

        eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);
        even++;
    }
}

int main(int argc, char *argv[]) {
    setlocale(LC_NUMERIC, "en_US.UTF-8");

    UserData userData;
    POS_T posData;
    GPS_T gpsData;
    memset(&posData, 0, sizeof (POS_T));
    memset(&gpsData, 0, sizeof (GPS_T));

    markierung.len = -1;

    char filename[100]/*, buffer[80]*/;

    pthread_t thread_id1, thread_id2, thread_id3, thread_id4, thread_id5, thread_id6;
    pthread_mutex_init(&m_pinto, NULL);
    pthread_mutex_init(&mutex_deto_lvl1, NULL);
    pthread_mutex_init(&mutex_deto_lvl2, NULL);
    pthread_mutex_init(&mutex_deto_lvl3, NULL);

    signal(SIGINT, ShutDown);
    setvbuf(stdout, (char *) NULL, _IONBF, 0);

    snprintf(filename, sizeof (filename) / sizeof (filename[0]), "%s/sit2d.debug", HOME);
    sout = fopen(filename, "wb");
    assert(sout != NULL);

    userData.width = 800;
    userData.height = 480;

#ifdef __RASPI__
    //bl_write(BL_ON);
#endif

    esInitContext(&esContext);
    esContext.userData = &userData;
    esContext.posData = &posData;
    esContext.gpsData = &gpsData;

    rad2deg = M_PI / 180.0f;
    pow2Z = pow(2.0, 18.0);

    initPOS(&esContext);

    if (argc == 2) {
        char *tmp = strchr(argv[1], '.');
        *tmp = '\0';

        snprintf(filename, sizeof (filename) / sizeof (filename[0]), "%s.gps", argv[1]);
        gps_in = fopen(filename, "rb");
        assert(gps_in != NULL);
        PRINTF("%s geladen.\n", filename);

        snprintf(filename, sizeof (filename) / sizeof (filename[0]), "%s.obd", argv[1]);
        sensor_in = fopen(filename, "rb");
        if (sensor_in != NULL) {
            PRINTF("%s geladen.\n", filename);
        } else {
            PRINTF("%s nicht gefunden.\n", filename);
        }
    } else {
        snprintf(filename, sizeof (filename) / sizeof (filename[0]), "%s/sit2d_%i.gps", HOME, posData.gps_file);
        gps_out = fopen(filename, "wb");
        assert(gps_out != NULL);
        snprintf(filename, sizeof (filename) / sizeof (filename[0]), "%s/sit2d_%i.obd", HOME, posData.gps_file);
        sensor_out = fopen(filename, "wb");
        assert(gps_out != NULL);
    }
    assert(esCreateWindow(&esContext, "SiT2D", userData.width, userData.height, ES_WINDOW_RGB) == GL_TRUE);

#define SZ 4096
#define PAGES 8192
    double *sqlite_cache_2 = (double*) malloc(SZ * PAGES);
    assert(sqlite3_config(SQLITE_CONFIG_PAGECACHE, &sqlite_cache_2[0], SZ, PAGES) == SQLITE_OK);

    sqlite3_initialize();
    assert(sqlite3_open_v2(HOME DB_LVL1, &db_1, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);
    assert(sqlite3_open_v2(HOME DB_LVL2, &db_2, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);
    assert(sqlite3_open_v2(HOME DB_LVL3, &db_3, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);

    verts_len = 0;

    load_TGA(minus_tex, HOME "RES/minus.tga");
    load_TGA(plus_tex, HOME "RES/plus.tga");
    load_TGA(menu_tex, HOME "RES/menu.tga");
    load_TGA(tex_zahlen, HOME "RES/ascii.tga");

    assert(Init(&esContext));

    //intro(&esContext);


#ifndef __RASPI__
    esContext.keyFunc = Key;
    esContext.buttonFunc = Button;
#else
    pthread_create(&thread_id1, NULL, &thread_foss, (void*) &esContext);
    pthread_create(&thread_id2, NULL, &thread_pinto, (void*) &esContext);
    pthread_create(&thread_id3, NULL, &thread_jolla, (void*) &esContext);
#endif

    //pthread_create(&thread_id3, NULL, &thread_deta_lvl1, (void*) &esContext);
    pthread_create(&thread_id4, NULL, &thread_deta_lvl2, (void*) &esContext);
    pthread_create(&thread_id5, NULL, &thread_deta_lvl3, (void*) &esContext);
    pthread_create(&thread_id6, NULL, &thread_ubongo, (void*) &esContext);

    thread_render(&esContext);

    pthread_join(thread_id1, NULL);
    pthread_join(thread_id2, NULL);
    pthread_join(thread_id3, NULL);
    pthread_join(thread_id4, NULL);
    pthread_join(thread_id5, NULL);
    pthread_join(thread_id6, NULL);

    //ShutDown(&esContext);

    return (EXIT_SUCCESS);
}