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
#define OBD_SERIAL "/dev/rfcomm0"
#define RSIZE 5
#define HOME "/home/pi/"
#else
#include "esUtil.h"
#define OBD_SERIAL "/dev/pts/18"
#define RSIZE 3
#define HOME "/home/florian/"
#endif

#define BUFFER_OFFSET(i) ((void *) NULL + (2*(i)))
#define ROUND(x) roundf(x*10)/10
//#define MEM(x) (int)(x * 256 )
#define PRINTF(...) fprintf(sout,__VA_ARGS__);fprintf(stdout,__VA_ARGS__)
#define PRINTGLERROR printOglError(__FILE__, __LINE__);
#ifdef __TM__
#define TM(START,END,...) clock_gettime(CLOCK_MONOTONIC, &END);printf("\r%lims\n",(END.tv_sec - START.tv_sec) * 1000 + (END.tv_nsec - START.tv_nsec) / 1000000);
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
#define BUFFER_DREIECK 14

//#define FBO_WIDTH 1024
//#define FBO_HEIGHT 1024

FILE *gps_out, *gps_in, *sout, *pos;

unsigned char tmp_gl_mode[] = {0, GL_TRIANGLES, GL_TRIANGLES, GL_LINES};

ESContext esContext;

float trans_x = 0.0f, trans_y = 0.0f, trans_z = 0.0f, orto1 = 0.0f, orto2 = 0.0f, orto3 = 0.0f, orto4 = 0.0f;

char line1_str[21];
char line2_str[21];

sqlite3 *db_2, *db_3;

enum GUI_MODE {
    _2D_, _3D_, _GUI_
};
enum GUI_MODE gui_mode = _3D_;

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
float BR = 4.0f;
float T_Y = V_Z_DEFAULT;
float ORTHO = 2.0f;

#define Y_LEN 10000
#define V_LEN 65536
#define F_LEN 20
//static GLfloat verts[3 * V_LEN] = {0.0f};
//static float verts_F[5 * V_LEN] = {0.0f};
static float verts_F_yama[5 * V_LEN] = {0.0f}, verts_F_werder[5 * V_LEN] = {0.0f}, verts_F_reis[5 * V_LEN] = {0.0f};
//static int verts_L[3 * V_LEN] = {0};
unsigned short verts_len = 0;
////////////////////////////////
//static T_V_E yama[Y_LEN];
static unsigned short index_yama[V_LEN], index_werder[V_LEN], index_reis[V_LEN];
unsigned short index_yama_len = 0, index_werder_len = 0, index_reis_len = 0;
/////////////////////////
//static T_V_E werder[Y_LEN];
//unsigned short werder_len = 0;
/////////////////////////
//static GLfloat tmp_verts[V_LEN] = {0.0f};
//static T_V_E tmp_yama[Y_LEN];
//unsigned short tmp_vert_len, tmp_yama_len, tmp_werder_len, tmp_reis_len;
static unsigned short tmp_index_yama[V_LEN], tmp_index_werder[V_LEN], tmp_index_reis[V_LEN];
unsigned short tmp_index_yama_len = 0, tmp_index_reis_len = 0, tmp_index_werder_len = 0;

///////////////////////////////
T_V_E farbe_yama[F_LEN], farbe_werder[F_LEN], farbe_reis[F_LEN], tmp_farbe_reis[F_LEN], tmp_farbe_yama[F_LEN], tmp_farbe_werder[F_LEN], markierung;
unsigned short farbe_yama_len = 0, farbe_reis_len = 0, farbe_werder_len = 0;
unsigned short tmp_farbe_werder_len = 0, tmp_farbe_reis_len = 0, tmp_farbe_yama_len = 0;


unsigned short verts_yama_len = 0, verts_werder_len = 0, verts_reis_len = 0;
////////////////////
//static T_V_E reis[Y_LEN];
//static unsigned short reis_index[V_LEN];
//unsigned short reis_len = 0, reis_index_len = 0;

int sqc_tmp;

//void setPOS(ESContext *esContext);
static int sqc_vertex_lvl2(void *a, int argc, char **argv, char **azColName);
static int sqc_vertex_lvl3(void *a, int argc, char **argv, char **azColName);
static int sqc_vertex3(void *a, int argc, char **argv, char **azColName);

void printM4(ESMatrix *wg);
void ShutDown(int signum);

#define V_KORR 1.7f

double mf;
double pow2Z;
float rad2deg;
//GLubyte ZOOM_S[] = {18, 17, 16, 15, 14, 12, 10, 8};
//GLubyte *ZOOM;

int dx = 0, dy = 0, m_n, m_m, p_n, p_m;
GLuint *tmp;
//static struct fixsource_t source;
ESMatrix perspective, modelview, mvpMatrix, mvpDreieck;

unsigned int stride;
float gps_status[3] = {1.0f, 0.0f, 0.0f};

unsigned short new_tex_lvl1 = 0, new_tex_lvl2 = 0, new_tex_lvl3 = 0;
pthread_mutex_t m_pinto = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_deto_lvl2 = PTHREAD_MUTEX_INITIALIZER, mutex_deto_lvl3 = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    GLuint programMap;
    GLuint programWERDER;
    GLuint programGUI;
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
    GLint colorLocGPS_I;
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

} UserData;

typedef struct {
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
    float obd_speed;
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

    float obd_volt;
    float i2c_hum;
    float i2c_temp;

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

GLboolean GenMipMap2D(GLubyte *src, GLubyte **dst, int srcWidth, int srcHeight, int *dstWidth, int *dstHeight) {
    int x, y;
    int texelSize = 3;

    *dstWidth = srcWidth / 2;
    if (*dstWidth <= 0)
        *dstWidth = 1;

    *dstHeight = srcHeight / 2;
    if (*dstHeight <= 0)
        *dstHeight = 1;

    *dst = malloc(sizeof (GLubyte) * texelSize * (*dstWidth) * (*dstHeight));
    if (*dst == NULL)
        exit(0);

    for (y = 0; y < *dstHeight; y++) {
        for (x = 0; x < *dstWidth; x++) {
            int srcIndex[4];
            float r = 0.0f, g = 0.0f, b = 0.0f;
            int sample;

            // Compute the offsets for 2x2 grid of pixels in previous
            // image to perform box filter
            srcIndex[0] = (((y * 2) * srcWidth) + (x * 2)) * texelSize;
            srcIndex[1] = (((y * 2) * srcWidth) + (x * 2 + 1)) * texelSize;
            srcIndex[2] = ((((y * 2) + 1) * srcWidth) + (x * 2)) * texelSize;
            srcIndex[3] = ((((y * 2) + 1) * srcWidth) + (x * 2 + 1)) * texelSize;

            // Sum all pixels
            for (sample = 0; sample < 4; sample++) {
                r += src[srcIndex[sample]];
                g += src[srcIndex[sample] + 1];
                b += src[srcIndex[sample] + 2];
            }

            // Average results
            r /= 4.0;
            g /= 4.0;
            b /= 4.0;

            // Store resulting pixels
            (*dst)[(y * (*dstWidth) + x) * texelSize] = (GLubyte) (r);
            (*dst)[(y * (*dstWidth) + x) * texelSize + 1] = (GLubyte) (g);
            (*dst)[(y * (*dstWidth) + x) * texelSize + 2] = (GLubyte) (b);
        }
    }

    return GL_TRUE;
}

/*
int get_tex_images(unsigned int tx, unsigned int ty, TEX_BUFFER_FORMAT *dst) {
    char buf[50];
    FILE *tex_file = NULL;
    int ret;
    //unsigned short df = 0;

    sprintf(buf, "%s/map/%hho/%u/%u.png", homedir, *ZOOM, tx, ty);
    PRINTF("'%s'\n", buf);
    tex_file = fopen(buf, "rb");
    if (tex_file == NULL) {
        //p_m = curl(*ZOOM, tx, ty);
        return -1;
    }
    //ret = load_png(tex_file, dst, stride, IMAGE_SIZE);
    fclose(tex_file);

    if (ret == -1) {
        remove(buf);
    }

    return 1;
}*/

void upload_tex(ESContext *esContext) {
    UserData *userData = esContext->userData;
    POS_T *posData = esContext->posData;

    //PRINTF("upload_tex\n");
#ifdef __NEW_MESH__
    glBindTexture(GL_TEXTURE_2D, userData->texindex[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
            IMAGE_SIZE * userData->r_height,
            IMAGE_SIZE * userData->r_width, 0,
            GL_RGB, GL_TEX_FORMAT, userData->tex_buffer);
    memcpy(&posData->gpu_tiles_y[0], &posData->tiles_y[0], sizeof (GLuint) * userData->r_height);
    memcpy(&posData->gpu_tiles_x[0], &posData->tiles_x[0], sizeof (GLuint) * userData->r_width);
#else
    unsigned short n = 0, m = 0;
    unsigned short im = 0;
    for (n = 0; n < userData->r_height; n++) {
        for (m = 0; m < userData->r_width; m++) {
            glBindTexture(GL_TEXTURE_2D, userData->texindex[userData->texMap[im]]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, IMAGE_SIZE, IMAGE_SIZE, 0,
                    GL_RGB, GL_TEX_FORMAT, userData->tex_buffer + im * userData->image_sz);
            posData->gpu_tiles_x[m] = posData->tiles_x[m];
            im++;
        }
        posData->gpu_tiles_y[n] = posData->tiles_y[n];
    }
#endif
    posData->gpu_t_x = posData->t_x;
    posData->gpu_t_y = posData->t_y;
    PRINTF("upload_tex:  %i %i  %f %f\n", posData->t_x, posData->t_y, posData->g_x, posData->g_y);

    new_tex_lvl2 = 0;
}

/*
int load_tex_images(ESContext *esContext) {
    UserData *userData = esContext->userData;
    POS_T *posData = esContext->posData;

    unsigned short n = 0, m = 0;
    unsigned short im = 0;

    for (n = 0; n < userData->r_width; n++)
        posData->tiles_x[n] = posData->t_x + n - (int) floor(userData->r_width / 2);
    for (n = 0; n < userData->r_height; n++)
        posData->tiles_y[n] = posData->t_y + n - (int) floor(userData->r_height / 2);

    memset(userData->tex_buffer, 0, userData->image_sz * userData->tiles * sizeof (TEX_BUFFER_FORMAT));
    for (n = 0; n < userData->r_height; n++) {
        for (m = 0; m < userData->r_width; m++) {
            userData->texMap[im] = im;
#ifdef __NEW_MESH__
            get_tex_images(posData->tiles_x[m], posData->tiles_y[n], userData->tex_buffer
                    + (m * userData->imgrow_sz + n * userData->image_sz * userData->r_width));
#else
            get_tex_images(posData->tiles_x[m], posData->tiles_y[n], userData->tex_buffer + im * userData->image_sz);
#endif
            im++;
        }
    }
    return im;
}*/

int Init(ESContext *esContext) {
    UserData *userData = esContext->userData;

    PRINTF("\r%s\n\r%s\n\r%s\n\r%s\n", glGetString(GL_VERSION), glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_SHADING_LANGUAGE_VERSION));

    char *f4, *v7, *f2, *f3, *v2, *v4, *f5, *v5;
    LoadGLSL(&f2, HOME "GLSL/f2.glsl");
    LoadGLSL(&v2, HOME "GLSL/v2.glsl");
    LoadGLSL(&f3, HOME "GLSL/f3.glsl");
    LoadGLSL(&f4, HOME "GLSL/f4.glsl");
    LoadGLSL(&v4, HOME "GLSL/v4.glsl");
    LoadGLSL(&v5, HOME "GLSL/v5.glsl");
    LoadGLSL(&f5, HOME "GLSL/f5.glsl");
    LoadGLSL(&v7, HOME "GLSL/v7.glsl");

    userData->programGUI = esLoadProgram(v2, f2);
    userData->programKreis = esLoadProgram(v2, f3);
    userData->programMap = esLoadProgram(v4, f4);
    userData->programWERDER = esLoadProgram(v5, f5);
    userData->programDreieck = esLoadProgram(v7, f4);

    free(f2);
    free(v2);
    free(f3);
    free(f4);
    free(v4);
    free(f5);
    free(v5);
    free(v7);

    userData->positionLocDreieck = glGetAttribLocation(userData->programDreieck, "a_position");
    userData->AngleLocDreieck = glGetUniformLocation(userData->programDreieck, "u_angle");
    userData->MapoffsetLocDreieck = glGetUniformLocation(userData->programDreieck, "u_mapOffset");
    userData->colorLocDreieck = glGetUniformLocation(userData->programDreieck, "u_color");
    glEnableVertexAttribArray(userData->positionLocDreieck);

    userData->positionLocGui = glGetAttribLocation(userData->programGUI, "a_position");
    userData->texCoordLocGui = glGetAttribLocation(userData->programGUI, "a_texCoord");
    userData->texLocGui = glGetUniformLocation(userData->programGUI, "s_baseMap");
    userData->samplerLocGui = glGetUniformLocation(userData->programGUI, "s_texture");
    glEnableVertexAttribArray(userData->positionLocGui);
    glEnableVertexAttribArray(userData->texCoordLocGui);

    userData->positionLocKreis = glGetAttribLocation(userData->programKreis, "a_position");
    userData->texCoordLocKreis = glGetAttribLocation(userData->programKreis, "a_texCoord");
    userData->colorLocGPS_I = glGetUniformLocation(userData->programKreis, "u_f");
    glEnableVertexAttribArray(userData->positionLocKreis);
    glEnableVertexAttribArray(userData->texCoordLocKreis);

    userData->positionLocMapH = glGetAttribLocation(userData->programMap, "a_posH");
    userData->positionLocMapL = glGetAttribLocation(userData->programMap, "a_posL");
    userData->colorLocMap = glGetUniformLocation(userData->programMap, "u_color");
    userData->mvpLocMap = glGetUniformLocation(userData->programMap, "u_mvpMatrix");
    userData->offsetHLocMap = glGetUniformLocation(userData->programMap, "u_offsetH");
    userData->offsetLLocMap = glGetUniformLocation(userData->programMap, "u_offsetL");
    userData->MapoffsetLocMap = glGetUniformLocation(userData->programMap, "u_mapOffset");
    glEnableVertexAttribArray(userData->positionLocMapH);
    glEnableVertexAttribArray(userData->positionLocMapL);

    userData->positionLocWERDERH = glGetAttribLocation(userData->programWERDER, "a_posH");
    userData->positionLocWERDERL = glGetAttribLocation(userData->programWERDER, "a_posL");
    userData->colorLocWERDER = glGetUniformLocation(userData->programWERDER, "u_color");
    userData->mvpLocWERDER = glGetUniformLocation(userData->programWERDER, "u_mvpMatrix");
    userData->offsetHLocWERDER = glGetUniformLocation(userData->programWERDER, "u_offsetH");
    userData->offsetLLocWERDER = glGetUniformLocation(userData->programWERDER, "u_offsetL");
    userData->MapoffsetLocWERDER = glGetUniformLocation(userData->programWERDER, "u_mapOffset");
    glEnableVertexAttribArray(userData->positionLocWERDERH);
    glEnableVertexAttribArray(userData->positionLocWERDERL);

    glGenBuffers(20, &userData->buffindex[0]);
    // NICHT LÖSCHEN! der raspi  brauch das
    for (int fg = 0; fg < 20; fg++)
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[fg]);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glBufferData(GL_ARRAY_BUFFER, sizeof (buttons), buttons, GL_STATIC_DRAW);

    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_REIS]);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


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

void Update2(ESContext *esContext, float deltaTime) {
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
    for (m_m = 0; m_m < 4; m_m++) {
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

    /*posData->v_x = sinf(mAngle) * posData->v;
    posData->v_y = cosf(mAngle) * posData->v;

    posData->g_x += deltaTime * posData->v_x / 3600.0f / posData->osmS;
    posData->g_y -= deltaTime * posData->v_y / 3600.0f / posData->osmS;
     */
    // DRAW
    /* glClear(GL_COLOR_BUFFER_BIT);

     glUseProgram(userData->programMap);
     glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS]);
     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX]);

     pthread_mutex_lock(&m_deto);

     if (new_tex == 1) {
         glBufferData(GL_ARRAY_BUFFER, 20 * verts_len, verts_F, GL_DYNAMIC_DRAW);
         glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_yama_len * sizeof (short), yama_index, GL_DYNAMIC_DRAW);
         new_tex = 0;
     }

     glUniformMatrix4fv(userData->mvpLocMap, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);
     glUniform2f(userData->offsetHLocMap, floor(posData->g_x), floor(posData->g_y));
     glUniform2f(userData->offsetLLocMap, 0.0f, 0.0f);
     glUniform2f(userData->MapoffsetLocMap, posData->o_x, posData->o_y);
     glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
     glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

     float color[] = {1.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 1.0f,
         1.0f, 0.0f, 1.0f,
         0.0f, 1.0f, 1.0f};

     //if (farbe_werder_len > 0) {
     for (int m = 0; m < farbe_werder_len; m++) {
         for (int n = 0; n < (farbe[m].len / 3); n++) {
             int c = (n % (sizeof (color) / sizeof (float) / 3))*3;

             glUniform3f(userData->colorLocMap, color[c], color[c + 1], color[c + 2]);
             glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(3 * n));


             //glLineWidth(2.0f);

             //glUniform3f(userData->colorLocMap, 0.0f, 0.0f, 0.0f);
             //glDrawElements(GL_LINE_LOOP, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(3 * n));


         }
         //glUniform3f(userData->colorLocMap, 0.0f, 0.0f, 1.0f);
         //glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(12));

     }
     pthread_mutex_unlock(&m_deto);
     PRINTGLERROR*/

    /*
        // Markierungen
        glLineWidth(5.0f);
        glUniform3f(userData->colorLocMap, markierung.r, markierung.g, markierung.b);
        glDrawElements(GL_LINES, markierung.len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(markierung.index));
        PRINTGLERROR

        // Dreieck
        glUseProgram(userData->programDreieck);
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
        glUniform1f(userData->AngleLocDreieck, dAngle);
        glUniform2f(userData->MapoffsetLocDreieck, posData->o_x, posData->o_y);
        glUniform3f(userData->colorLocDreieck, 1.0f, 0.0f, 0.0f);
        glVertexAttribPointer(userData->positionLocDreieck, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        PRINTGLERROR

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
     */
    // Text
    //renderInt(userData, posData->obd_speed, -1.0f, 0.54f);
    //renderFloat(userData, posData->g_x, -1.0f, 0.54f);
    //renderFloat(userData, posData->g_y, -1.0f, 0.4f);
    /*
        // GPS_I
        glUseProgram(userData->programKreis);
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
        glVertexAttribPointer(userData->positionLocKreis, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
        glVertexAttribPointer(userData->texCoordLocKreis, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));
        glUniform3f(userData->colorLocGPS_I, gps_status[0], gps_status[1], gps_status[2]);
        glDrawArrays(GL_TRIANGLE_STRIP, 15, 4);
        PRINTGLERROR
     */
}

void Update(ESContext *esContext, float deltaTime) {
    UserData *userData = ((ESContext*) esContext)->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;
    float dAngle = 0.0f;
    float mAngle = posData->angle * rad2deg;

    esMatrixLoadIdentity(&modelview);
    if (gui_mode == _3D_) {
        //printf("%f\n",mAngle);
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
    for (m_m = 0; m_m < 4; m_m++) {
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

#ifndef __FIN__
    posData->v_x = sinf(mAngle) * posData->v;
    posData->v_y = cosf(mAngle) * posData->v;

    posData->g_x += deltaTime * posData->v_x / 3600.0f / posData->osmS;
    posData->g_y -= deltaTime * posData->v_y / 3600.0f / posData->osmS;
#endif

    DoubletoFix(&posData->g_x, posData->gi_x);
    DoubletoFix(&posData->g_y, posData->gi_y);

    // DRAW
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(userData->programMap);

    glUniformMatrix4fv(userData->mvpLocMap, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);
    //glUniform2i(userData->offsetHLocMap, (int) floor(posData->g_x), (int) floor(posData->g_y));
    glUniform2i(userData->offsetHLocMap, posData->gi_x[0], posData->gi_y[0]);
    glUniform2f(userData->offsetLLocMap, fmod(posData->g_x, 1.0f), fmod(posData->g_y, 1.0f));
    //glUniform2f(userData->offsetLLocMap, (float) (posData->gi_x[1] / FRACTPART), (float) (posData->gi_y[1] / FRACTPART));
    glUniform2f(userData->MapoffsetLocMap, posData->o_x, posData->o_y);
    PRINTGLERROR
    /*
    pthread_mutex_lock(&mutex_deto_lvl2);
    if (new_tex_lvl2 == 1) {
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_REIS]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_REIS]);
        glBufferData(GL_ARRAY_BUFFER, 20 * verts_reis_len, verts_F_reis, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_reis_len * sizeof (short), index_reis, GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_YAMA]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_YAMA]);
        glBufferData(GL_ARRAY_BUFFER, 20 * verts_yama_len, verts_F_yama, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_yama_len * sizeof (short), index_yama, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_WERDER]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_WERDER]);
        glBufferData(GL_ARRAY_BUFFER, 20 * verts_werder_len, verts_F_werder, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_werder_len * sizeof (short), index_werder, GL_DYNAMIC_DRAW);
        
        new_tex_lvl2 = 0;
        PRINTGLERROR
    }
    pthread_mutex_unlock(&mutex_deto_lvl2);
     */
    // REIS
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS_REIS]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX_REIS]);

    glVertexAttribPointer(userData->positionLocMapH, 3, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->positionLocMapL, 2, GL_FLOAT, GL_FALSE, 5 * sizeof (GL_FLOAT), (GLvoid*) (3 * sizeof (GL_FLOAT)));

    for (unsigned short m_n = 0; m_n < farbe_reis_len; m_n++) {
        //glLineWidth(farbe[m_n].glLineWidth);
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

    for (unsigned short m_n = 0; m_n < farbe_yama_len; m_n++) {
        //glLineWidth(farbe[m_n].glLineWidth);
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

    for (unsigned short m_n = 0; m_n < farbe_werder_len; m_n++) {
        //glLineWidth(farbe[m_n].glLineWidth);
        glUniform3f(userData->colorLocMap, farbe_werder[m_n].r, farbe_werder[m_n].g, farbe_werder[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe_werder[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe_werder[m_n].index));
    }
    PRINTGLERROR

    // Markierungen
    //glLineWidth(5.0f);
    //glUniform3f(userData->colorLocMap, markierung.r, markierung.g, markierung.b);
    //glDrawElements(GL_LINES, markierung.len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(markierung.index));
    PRINTGLERROR

    // Dreieck
    glUseProgram(userData->programDreieck);
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glUniform1f(userData->AngleLocDreieck, dAngle);
    glUniform2f(userData->MapoffsetLocDreieck, posData->o_x, posData->o_y);
    glUniform3f(userData->colorLocDreieck, 0.0f, 0.0f, 0.0f);
    glVertexAttribPointer(userData->positionLocDreieck, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glUniform3f(userData->colorLocDreieck, 1.0f, 0.0f, 0.0f);
    glDrawArrays(GL_TRIANGLES, 19, 3);
    PRINTGLERROR


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
    glUniform3f(userData->colorLocGPS_I, gps_status[0], gps_status[1], gps_status[2]);
    glDrawArrays(GL_TRIANGLE_STRIP, 15, 4);
    PRINTGLERROR

}

void Key(ESContext *esContext, unsigned char a, int b, int c) {
    POS_T *posData = esContext->posData;
    uint l = 0;

    //printf("%i\n", a);
    switch (a) {
        case 114:
            verts_len = 0;
            //loadVert(posData->g_x, posData->g_y, BR);


            break;
        case 53:
            if (gui_mode == _2D_) {
                posData->o_x = 0.0f;
                posData->o_y = 0.0f;
            }
            break;
        case 56:
            if (gui_mode == _2D_) {
                posData->o_y += 0.1f;
            }
            break;
        case 50:
            if (gui_mode == _2D_) {
                posData->o_y -= 0.1f;
            }
            break;
        case 52:
            if (gui_mode == _2D_) {
                posData->o_x -= 0.1f;
            }
            break;
        case 54:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
            }
            break;
        case 55:
            if (gui_mode == _2D_) {
                posData->o_x -= 0.1f;
                posData->o_y += 0.1f;
            }
            break;
        case 57:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
                posData->o_y += 0.1f;
            }
            break;
        case 51:
            if (gui_mode == _2D_) {
                posData->o_x += 0.1f;
                posData->o_y -= 0.1f;
            }
            break;
        case 49:
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
        case 45:
            if (ORTHO < 10.0) {
                ORTHO += 1.0f;
                esMatrixLoadIdentity(&perspective);
                esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
            }
            break;
        case 43:
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

void initPOS(ESContext * esContext) {
    POS_T *posData = esContext->posData;

    pos = fopen(HOME "sit2d_pos", "rb+");
    if (pos == NULL)
        pos = fopen(HOME "sit2d_pos", "wb");

    if (fread(&posData->g_x, sizeof (double), 1, pos) == 1) {
        fread(&posData->g_y, sizeof (double), 1, pos);
        fread(&posData->gps_altitude, sizeof (double), 1, pos);
        fread(&posData->angle, sizeof (float), 1, pos);
    } else {
        PRINTF("\rinitPOS: sit2d_pos leer.\n");
        posData->gps_latitude = 52.1317029;
        posData->gps_longitude = 11.6530974;
        posData->gps_altitude = 0.0;
        posData->angle = 0.0f;

        posData->g_x = (posData->gps_longitude + 180.0) / 360.0 * pow2Z;
        posData->g_y = (1.0 - log(tan(posData->gps_latitude * rad2deg) + 1.0 / cos(posData->gps_latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;

        fwrite(&posData->g_x, sizeof (double), 1, pos);
        fwrite(&posData->g_y, sizeof (double), 1, pos);
        fwrite(&posData->gps_latitude, sizeof (double), 1, pos);
        fwrite(&posData->angle, sizeof (float), 1, pos);
        fseek(pos, 0, SEEK_SET);
    }

    DoubletoFix(&posData->g_x, posData->gi_x);
    DoubletoFix(&posData->g_y, posData->gi_y);

    posData->osmS = 40075017 * cos(posData->gps_latitude * M_PI / 180.0) / pow(2.0, 18.0 + 8.0) * IMAGE_SIZE;

    PRINTF("\rinitPOS: g: %f %f a: %f\n", posData->g_x, posData->g_y, posData->angle);
}

void * foss(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    //GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    PRINTF("\rFOSS gestartet\n");
    time_t t;
    struct tm tm;

#ifdef __RASPI__
    int gpio = -1;
    gpio_lcd_init();
    //bl_write(BL_ON);
#endif

    memset(line1_str, 32, 20);
    memset(line2_str, 32, 20);

    for (int f = 0; f < 20; f++) {
        line1_str[f] = 255;
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        for (int a = 0; a < 20; a++) {
            gpio_lcd_send_byte(line1_str[a], GPIO_HIGH);
        }
    }

    while (1) {
        memset(line1_str, 32, 20);
        memset(line2_str, 32, 20);

        t = time(NULL);
        tm = *localtime(&t);
        strftime(line1_str, 21, "%R     %e.%m.%Y", &tm);

        snprintf(line2_str, 21, "U: %.1fV    V: %.0f", posData->obd_volt, posData->v);

#ifdef __RASPI__
        //gpio = gpio_read(GPIO_PIN);

        /*if (gpio == GPIO_HIGH) {
            bl_write(BL_OFF);
            ShutDown(1);
        } else
            bl_write(BL_ON);
         */
#ifdef __LCD__
        gpio_lcd_send_byte(LCD_LINE_1, GPIO_LOW);
        for (int a = 0; a < 20; a++) {
            gpio_lcd_send_byte(line1_str[a], GPIO_HIGH);
        }
        gpio_lcd_send_byte(LCD_LINE_2, GPIO_LOW);
        for (int a = 0; a < 20; a++) {
            if (line2_str[a] == 0)

                break;
            gpio_lcd_send_byte(line2_str[a], GPIO_HIGH);
        }
#endif
#endif
        //printf("\rhum: %.2f  temp: %.2f\n", posData->i2c_hum, posData->i2c_temp);
        usleep(1000000);

    }
    return NULL;
}

/*unsigned short aufLinie(unsigned short *p, int count, double gx, double gy) {
    //#define OIHANFOIA(Z) floorf(Z * 10)
    double Ax, Ay, Bx, By, X2;
    int i1 = 0, i2 = 0;

    for (int n = 0; n < count; n += 2) {
        i1 = p[n] * 5;
        i2 = p[n + 1]*5;

        Ax = verts_F[ i1 ] + verts_F[i1 + 3];
        Ay = verts_F[i1 + 1] + verts_F[i1 + 4];
        Bx = verts_F[i2 ] + verts_F[i2 + 3];
        By = verts_F[i2 + 1] + verts_F[i2 + 4];

        X2 = Ax + ((gy - Ay) / (By - Ay)) * (Bx - Ax);

        //printf("%f %f   %f %f\n", gx, X2, OIHANFOIA(gx), OIHANFOIA(X2));
        if (fabs(gx - X2) < 0.1) {
            //ret[asd++] = n;

            return n;
        }
    }

    return USHRT_MAX;
}*/

void * ubongo(void *esContext) {
    UserData *userData = ((ESContext*) esContext)->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;
    unsigned short ret;
    sleep(3);
    while (1) {
        int asd = 0;
        for (unsigned short a = farbe_yama_len; a < farbe_werder_len; a++) {
            asd += farbe_yama[a].len;
        }
        //printf("%i %i %i %f %f\n", farbe_yama_len, farbe[farbe_yama_len].index, asd, posData->g_x, posData->g_y);
        //ret = aufLinie(&index_yama[farbe[farbe_yama_len].index], asd, posData->g_x, posData->g_y);
        //ret=166;
        printf("\raufLinie: %i\n", ret);
        if (ret != USHRT_MAX) {
            markierung.r = 0.0f;
            markierung.g = 0.0f;
            markierung.b = 1.0f;

            markierung.gl_mode = tmp_gl_mode[3];
            markierung.glLineWidth = 5.0f;
            markierung.typ = 3;

            markierung.index = farbe_yama[farbe_yama_len].index + ret;

            markierung.len = 2;
        } else {

            memset(&markierung, 0, sizeof (T_V_E));
        }
        sleep(1);
    }
}

#define SHIFT 4
#define DB_LVL2 "sit2d_6_lvl2.db"
#define DB_LVL3 "sit2d_6_lvl3.db"

#define SQL_ABFRAGE "SELECT 2 as l_lvl,l_cf_r,l_cf_g,l_cf_b,t.L_ID,V_LON,V_LAT\
  FROM (SELECT DISTINCT L_ID\
             FROM vertex\
            WHERE V_T_LON BETWEEN %i AND %i AND\
                  V_T_LAT BETWEEN %i AND %i) t\
       JOIN vertex g ON t.L_ID = g.L_ID\
       JOIN eigenschaften ON g.L_Id = eigenschaften.L_ID\
       ORDER BY l_lvl,l_cf_r,l_cf_g,l_cf_b,t.L_ID,I_SEQ"

void * deta_lvl2(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    struct timespec spec0, spec1;

    int lon1 = (int) floor(posData->g_x);
    int lat1 = (int) floor(posData->g_y);

    int d1 = (int) floor(BR);

    char sql[700];

    char *zErrMsg = 0;

    verts_yama_len = 0;
    tmp_index_yama_len = 0;
    tmp_farbe_yama_len = 0;

    clock_gettime(CLOCK_MONOTONIC, &spec0);

    snprintf(sql, 700, SQL_ABFRAGE, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
    //printf("\r%s\n", sql);
    sqlite3_exec(db_2, sql, sqc_vertex_lvl2, 0, &zErrMsg);

    pthread_mutex_lock(&mutex_deto_lvl2);
    farbe_yama_len = tmp_farbe_yama_len;
    index_yama_len = tmp_index_yama_len;
    memcpy(farbe_yama, tmp_farbe_yama, tmp_farbe_yama_len * sizeof (T_V_E));
    memcpy(index_yama, tmp_index_yama, tmp_index_yama_len * sizeof (unsigned short));
    new_tex_lvl2 = 1;
    pthread_mutex_unlock(&mutex_deto_lvl2);

    TM(spec0, spec1)
}

void * deta_lvl3(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    struct timespec spec0, spec1;

    int lon1 = (int) floor(posData->g_x);
    int lat1 = (int) floor(posData->g_y);

    int d1 = (int) floor(BR);

    char sql[700];

    char *zErrMsg = 0;

    verts_werder_len = 0;
    tmp_index_werder_len = 0;
    tmp_farbe_werder_len = 0;

    clock_gettime(CLOCK_MONOTONIC, &spec0);

    snprintf(sql, 700, SQL_ABFRAGE, (lon1 - d1) >> SHIFT, (lon1 + d1) >> SHIFT, (lat1 - d1) >> SHIFT, (lat1 + d1) >> SHIFT);
    //printf("\r%s\n", sql);
    sqlite3_exec(db_3, sql, sqc_vertex_lvl3, 0, &zErrMsg);

    pthread_mutex_lock(&mutex_deto_lvl3);
    farbe_werder_len = tmp_farbe_werder_len;
    index_werder_len = tmp_index_werder_len;
    memcpy(farbe_werder, tmp_farbe_werder, tmp_farbe_werder_len * sizeof (T_V_E));
    memcpy(index_werder, tmp_index_werder, tmp_index_werder_len * sizeof (unsigned short));
    new_tex_lvl3 = 1;
    pthread_mutex_unlock(&mutex_deto_lvl3);

    TM(spec0, spec1, '')
}

void * jolla(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
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

void * pinto(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;

    //long int elapsed;

    int counter = 0;

    float speed = 0.0f, old_speed = 0.0f, deltatime = 0.0f;
    float deg, old_deg;
    struct timeval t1, t2, t_start;
    int lk = 0, status;
    double g_x, g_y, old_lat, old_lon, old_g_x, old_g_y, dist;

#ifdef __OBD__
    int obd_serial = -1;
    int obd_pid = 5; //Engine Coolant Temperature

    enum obd_serial_status obdstatus;
    float tmp_val;
    unsigned int cmdid = obdcmds_mode1[obd_pid].cmdid;
    int numbytes = obdcmds_mode1[obd_pid].bytes_returned;
    OBDConvFunc conv = obdcmds_mode1[obd_pid].conv;
#endif

    int gpsd_init = -1;
    //struct GPS_T gpsData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    int i2c_init = -1;
    enum e_i2c_status i2c_status;

#ifdef __I2C__
    short i2c_x, i2c_y, i2c_z;
#endif
#ifdef __SENSOR_HUM__
    float i2c_hum, i2c_temp;
#endif
    gettimeofday(&t_start, NULL);
    t1 = t_start;

    while (1) {
        delay(500L);
        old_lat = posData->gps_latitude;
        old_lon = posData->gps_longitude;

        old_g_x = g_x;
        old_g_y = g_y;

        old_speed = speed;
        old_deg = deg;

        gettimeofday(&t2, NULL);
        deltatime = (float) (t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6) * 1000;

        //elapsed = (t2.tv_sec - t_start.tv_sec)*1000 + (t2.tv_usec - t_start.tv_usec) / 1000;
        t1 = t2;

#ifdef __OBD__
        if (obd_serial != -1) {
            //Engine Coolant Temperature
            obdstatus = getobdvalue(obd_serial, 5, &tmp_val, numbytes, conv);
            if (OBD_SUCCESS == obdstatus) {
                posData->obd_speed = tmp_val;
            } else {
                printf("OBD: fehler: \n");
            }
            obdstatus = getobdvalue(obd_serial, 0x42, &tmp_val, numbytes, conv);
            if (OBD_SUCCESS == obdstatus) {
                posData->obd_volt = tmp_val;
            } else {
                printf("OBD: fehler: \n");
            }
        } else {
            obd_serial = init_OBD(OBD_SERIAL);
        }
#endif
#ifdef __SENSOR_HUM__
        if (i2c_init != -1) {
            i2c_status = sensor_hum_read(&i2c_init, &i2c_hum, &i2c_temp);
            lk = pthread_mutex_lock(&m_pinto);
            posData->i2c_hum = i2c_hum;
            posData->i2c_temp = i2c_temp;
            pthread_mutex_unlock(&m_pinto);

        } else {
            i2c_status = sensor_hum_init(&i2c_init);
        }
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

#ifdef __GPS__
        if (gpsd_init != -1) {
            gps_read(&gpsd_init, gpsData);
            //PRINTF("\rgps: type=%i pdop=%f\n", gpsData->fix_type, gpsData->PDOP);
            if (gpsData->fix_type > 1 && gpsData->PDOP < 6) {
                gps_status[0] = 0.0f;
                gps_status[2] = 1.0f;

                g_x = (gpsData->longitude + 180.0) / 360.0 * pow2Z;
                g_y = (1.0 - log(tan(gpsData->latitude * rad2deg) + 1.0 / cos(gpsData->latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;
                deg = gpsData->angle;
#ifdef __OBD__
                if (obd_serial == -1) {
                    dist = sqrt(pow(old_g_x - g_x, 2) + pow(old_g_y - g_y, 2)) * posData->osmS;
                    speed = dist / deltatime * 3600.0f;
                    if ((speed - old_speed) / deltatime > 10.0f /* km/h/s */) {
                        speed = 10.0f - deltatime + old_speed;
                    }
                    lk = pthread_mutex_lock(&m_pinto);
                    posData->v = speed;
                    pthread_mutex_unlock(&m_pinto);
                }
#endif
#ifdef __I2C__
                if (i2c_init == -1) {
                    deg = getDegrees(old_lat, old_lon, gpsData->fix.latitude, gpsData->fix.longitude);
                    deg = deg < 0.0f ? deg + 360.0f : deg;
                    lk = pthread_mutex_lock(&m_pinto);
                    pthread_mutex_unlock(&m_pinto);
                }
#endif
                lk = pthread_mutex_lock(&m_pinto);
                posData->g_x = g_x;
                posData->g_y = g_y;
                posData->angle = deg;
                pthread_mutex_unlock(&m_pinto);
            } else {
                gps_status[0] = 1.0f;
                gps_status[2] = 0.0f;

            }
        } else {
            gps_open(&gpsd_init, "/dev/ttyAMA0");
        }
#endif

#ifdef __FIN__
        struct timeval tv_tmp;

        //if (fread(&elapsed, sizeof (long), 1, gps_in) == 1) {
        if (fread(&deltatime, sizeof (float), 1, gps_in) == 1) {
            fread(&g_x, sizeof (double), 1, gps_in);
            fread(&g_y, sizeof (double), 1, gps_in);
            fread(&dist, sizeof (double), 1, gps_in);
            fread(&speed, sizeof (float), 1, gps_in);
            fread(&deg, sizeof (float), 1, gps_in);
            fread(&tv_tmp.tv_sec, sizeof (tv_tmp.tv_sec), 1, gps_in);
            fread(&tv_tmp.tv_usec, sizeof (tv_tmp.tv_usec), 1, gps_in);

            //printf("asd: %f %f\n", posData->g_x, posData->g_y);
            PRINTF("\rgps_out %i: %i,%i: %f %f %f %f %f %f \n", counter, tv_tmp.tv_sec, tv_tmp.tv_usec, deltatime, g_x, g_y, dist, speed, deg);
            counter++;

            lk = pthread_mutex_lock(&m_pinto);
            posData->g_x = g_x;
            posData->g_y = g_y;

            //posData->angle = deg;
            pthread_mutex_unlock(&m_pinto);

            gps_status[0] = 0.0f;
            gps_status[2] = 1.0f;
        } else {
            fclose(gps_in);
        }

#else

        int64_t tv_sec = (int64_t) t2.tv_sec;
        int64_t tv_usec = (int64_t) t2.tv_usec;

        fwrite(&deltatime, sizeof (float), 1, gps_out);
        fwrite(&posData->g_x, sizeof (double), 1, gps_out);
        fwrite(&posData->g_y, sizeof (double), 1, gps_out);
        fwrite(&dist, sizeof (double), 1, gps_out);
        fwrite(&speed, sizeof (float), 1, gps_out);
        fwrite(&deg, sizeof (float), 1, gps_out);
        fwrite(&tv_sec, sizeof (tv_sec), 1, gps_out);
        fwrite(&tv_usec, sizeof (tv_usec), 1, gps_out);
        fflush(gps_out);

        fwrite(&posData->g_x, sizeof (double), 1, pos);
        fwrite(&posData->g_y, sizeof (double), 1, pos);
        fwrite(&posData->gps_latitude, sizeof (double), 1, pos);
        fwrite(&posData->angle, sizeof (float), 1, pos);
        fseek(pos, 0, SEEK_SET);
        fflush(pos);
#endif
    }


    return NULL;
}

void esMainLoop(ESContext * esContext) {
    struct timeval t1, t2;
    struct timezone tz;
    float deltatime;
    float totaltime = 0.0f;
    unsigned int frames = 0;
    unsigned int us = 10000;

    gettimeofday(&t1, &tz);
#ifdef __TM__
    struct timespec spec1, spec2;
    unsigned int ms_min = 4294967295, ms_max = 0, ms, ms_avg = 0;
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
        //Update2(esContext, deltatime);
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

            PRINTF("\rFPS: %3.1f FRAMETIME: %i %4.1f %i  us: %u\n", frames / totaltime, ms_min, (float) ms_avg / frames, ms_max, us);

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

void ShutDown(int signum) {
    //void sigfunc(int sig) 

    UserData *userData = esContext.userData;
    POS_T *posData = esContext.posData;

    PRINTF("\rthe end is near!\n");
    //fclose(gps_out);
    //fclose(sout);
#ifdef __FIN__
    fclose(gps_in);
#endif

    sqlite3_close(db_2);

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

int main(int argc, char *argv[]) {
    setlocale(LC_NUMERIC, "en_US.UTF-8");

    UserData userData;
    POS_T posData;
    GPS_T gpsData;
    memset(&posData, 0, sizeof (POS_T));
    memset(&gpsData, 0, sizeof (GPS_T));

    pthread_t thread_id1, thread_id2, thread_id3, thread_id4, thread_id5;
    pthread_mutex_init(&m_pinto, NULL);
    pthread_mutex_init(&mutex_deto_lvl2, NULL);
    pthread_mutex_init(&mutex_deto_lvl3, NULL);

    signal(SIGTERM, ShutDown);
    //signal(SIGINT, ShutDown);
    setvbuf(stdout, (char *) NULL, _IONBF, 0);

    userData.width = 800;
    userData.height = 480;

#ifdef __RASPI__
    //bl_write(BL_ON);
#endif

    char filename[100], buffer[80];

    sout = fopen(HOME "/sit2d_debug.txt", "w");
    assert(sout != NULL);

#ifdef __FIN__
    gps_in = fopen(HOME "sit2d.gps", "rb");
    assert(gps_in != NULL);
    PRINTF("%s geladen.\n", HOME "sit2d.gps");
#else
    strncpy(filename, HOME, 100);

    time_t rawtime;
    struct tm *info;
    time(&rawtime);
    info = localtime(&rawtime);

    strftime(buffer, 80, "%Y%m%d%H%M_sit2d.gps", info);

    /*struct timeval tv_tmp;
    gettimeofday(&tv_tmp, NULL);

    sprintf(str, "Value of Pi = %f", M_PI);
    //printf("%i\n", tv_tmp.tv_sec);
     */
    gps_out = fopen(strncat(filename, buffer, 80), "wb");
    assert(gps_out != NULL);
#endif

    esInitContext(&esContext);
    esContext.userData = &userData;
    esContext.posData = &posData;
    esContext.gpsData = &gpsData;

    esCreateWindow(&esContext, "SiT2D", userData.width, userData.height, ES_WINDOW_RGB);

    rad2deg = M_PI / 180.0f;
    pow2Z = pow(2.0, 18.0);

    initPOS(&esContext);

#define SZ 4096
#define PAGES 8192
    double *sqlite_cache_2 = (double*) malloc(SZ * PAGES);
    assert(sqlite3_config(SQLITE_CONFIG_PAGECACHE, &sqlite_cache_2[0], SZ, PAGES) == SQLITE_OK);

    sqlite3_initialize();
    assert(sqlite3_open_v2(HOME DB_LVL2, &db_2, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);
    assert(sqlite3_open_v2(HOME DB_LVL3, &db_3, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);

    verts_len = 0;

    esMatrixLoadIdentity(&perspective);
    esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);

    load_TGA(minus_tex, HOME "RES/minus.tga");
    load_TGA(plus_tex, HOME "RES/plus.tga");
    load_TGA(menu_tex, HOME "RES/menu.tga");
    load_TGA(tex_zahlen, HOME "RES/ascii.tga");

    assert(Init(&esContext));

#ifndef __RASPI__
    esRegisterKeyFunc(&esContext, Key);
#endif
    PRINTF("\rinit abgeschlossen\n");

    //intro(&esContext);
    glClearColor(1.0f, 1.0f, 0.0f, 1.0f);

    pthread_create(&thread_id1, NULL, &foss, (void*) &esContext);
    pthread_create(&thread_id2, NULL, &pinto, (void*) &esContext);
    //pthread_create(&thread_id3, NULL, &jolla, (void*) &esContext);
    pthread_create(&thread_id4, NULL, &deta_lvl2, (void*) &esContext);
    pthread_create(&thread_id5, NULL, &deta_lvl3, (void*) &esContext);
    //pthread_create(&thread_id5, NULL, &ubongo, (void*) &esContext);

    esMainLoop(&esContext);

    pthread_join(thread_id1, NULL);
    pthread_join(thread_id2, NULL);
    pthread_join(thread_id3, NULL);
    pthread_join(thread_id4, NULL);
    pthread_join(thread_id5, NULL);

    //ShutDown(&esContext);

    return (EXIT_SUCCESS);
}