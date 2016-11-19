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
#include "util.h"
#include "gui.h"
#include "obd.h"
#include "i2c.h"
#include "gps.h"
#include "gpio.h"

#ifdef __SSE__
#include <xmmintrin.h>
#endif

#ifdef __RASPI__
#include "esUtil_raspi.h"
#define OBD_SERIAL "/dev/rfcomm0"
#define RSIZE 5
#else
#include "esUtil.h"
#define OBD_SERIAL "/dev/pts/18"
#define RSIZE 3
#endif

#define BUFFER_OFFSET(i) ((void *) NULL + (2*i))
#define ROUND(x) roundf(x*10)/10
#define MEM(x) (int)(x * 256 )
#define PRINTF(...) fprintf(sout,__VA_ARGS__);fprintf(stdout,__VA_ARGS__)
#define PRINTGLERROR printOglError(__FILE__, __LINE__);
#ifdef __TM__
#define TM(START,END,...) clock_gettime(CLOCK_MONOTONIC, &END);ms = (END.tv_sec - START.tv_sec) * 1000 + (END.tv_nsec - START.tv_nsec) * 1e-6;printf("\r%ims\n",ms);
#else
#define TM(START, END,...) 
#endif

#define BUFF_INDEX_GUI 0
#define BUFF_INDEX_TXT 1
#define BUFFER_VERTS 2
#define BUFFER_INDEX 3

#define FBO_WIDTH 1024
#define FBO_HEIGHT 1024

FILE *gps_out, *gps_in;
FILE *sout;

float trans_x = 0.0f, trans_y = 0.0f, trans_z = 0.0f, orto1 = 0.0f, orto2 = 0.0f, orto3 = 0.0f, orto4 = 0.0f;

char line1_str[21];
char line2_str[21];

sqlite3 *db;

typedef struct {
    int id;
    int index;
    GLsizei len;
    GLfloat r, g, b;
    int osm_id;
} T_V_E;

typedef struct {
    GLsizei len;
    GLfloat r, g, b;
    unsigned short *index;
} T_V_F;

#define BR 8.0
//float BR = 0.5f;
float ORTHO = 2.0f;

#define Y_LEN 10000
#define V_LEN 65536
static GLfloat verts[3 * V_LEN] = {0.0f};
unsigned short verts_len = 0;
////////////////////////////////
static T_V_E yama[Y_LEN];
static unsigned short yama_index[V_LEN];
unsigned short yama_len = 0, yama_index_len = 0;
/////////////////////////
static T_V_E werder[Y_LEN];
unsigned short werder_len = 0;
/////////////////////////
static GLfloat tmp_verts[V_LEN] = {0.0f};
static T_V_E tmp_yama[Y_LEN];
unsigned short tmp_vert_len, tmp_yama_len, tmp_werder_len, tmp_reis_len;
///////////////////////////////
T_V_E farbe[20];
unsigned short farbe_yama_len = 0, farbe_reis_len = 0, farbe_werder_len = 0;
;
////////////////////
static T_V_E reis[Y_LEN];
static unsigned short reis_index[V_LEN];
unsigned short reis_len = 0, reis_index_len = 0;

int sqc_tmp;

void setPOS(ESContext *esContext);
static int sqc_yama(void *a, int argc, char **argv, char **azColName);
static int sqc_yama2(void *a, int argc, char **argv, char **azColName);
static int sqc_vertex(void *NotUsed, int argc, char **argv, char **azColName);
static int sqc_count(void *out_var, int argc, char **argv, char **azColName);
static int sqc_farbe(void *a, int argc, char **argv, char **azColName);

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
ESMatrix perspective, modelview, mvpMatrix, mvpTex;

unsigned int stride;
float gps_status[3] = {1.0f, 0.0f, 0.0f};

unsigned short new_tex = 0;
pthread_mutex_t m_pinto = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t m_deto = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    GLuint programMap;
    GLuint programFBO;
    GLuint programGUI;
    GLuint programKreis;
    GLuint programTex;

    // Attribute locations
    GLint positionLocGui;
    GLint positionLocKreis;
    GLint texCoordLocKreis;
    GLint positionLocMap;
    GLint positionLocFBO;
    GLint offsetLLocMap;
    GLint offsetLLocFBO;
    GLint offsetLLocTex;
    GLint offsetMLocMap;
    GLint offsetMLocMap2;
    GLint offsetMLocTex;
    GLint texCoordLocGui;
    GLint colorLocMap;
    GLint colorLocFBO;
    GLint texLocGui;
    GLint samplerLocGui;
    GLint samplerLocTex;
    GLint positionLocTex;
    GLint texCoordLocTex;

    // Uniform locations
    GLint mvpLocMap;
    GLint mvpLocFBO;
    GLint colorLocGPS_I;
    GLint mvpLocTex;

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
} POS_T;

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

    new_tex = 0;
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

    PRINTF("\r%s\n", glGetString(GL_VERSION));
    PRINTF("\r%s\n", glGetString(GL_VENDOR));
    PRINTF("\r%s\n", glGetString(GL_RENDERER));
    PRINTF("\r%s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

    const char *v2 = "attribute vec4 a_position; \n"
            "attribute vec2 a_texCoord;          \n"
            "varying vec2 v_texCoord;            \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;        \n"
            "   v_texCoord = a_texCoord;         \n"
            "}                                   \n";

    const char *f2 = "precision mediump float;                   \n"
            "varying vec2 v_texCoord;                            \n"
            "uniform sampler2D s_texture;                        \n"
            "void main()                                         \n"
            "{                                                   \n"
            "  gl_FragColor = texture2D( s_texture, v_texCoord );\n"
            "}                                                   \n";

    const char *v3 = "attribute vec4 a_position;     \n"
            "attribute vec2 a_texCoord;          \n"
            "varying vec2 v_texCoord;            \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;        \n"
            "   v_texCoord = a_texCoord;         \n"
            "}                                   \n";

    const char *f3 = "precision mediump float;                      \n"
            "varying vec2 v_texCoord;                               \n"
            "uniform vec3 u_f;                                      \n"
            "void main()                                            \n"
            "{                                                      \n"
            "  float dis = distance(v_texCoord, vec2(0.5, 0.5));                                  \n"
            "  gl_FragColor = dis > 0.5 ? vec4(0.0,0.0,0.0,0.0) : vec4(u_f.x,u_f.y,u_f.z,1.0);    \n"
            "}                                                      \n";
    const char *v4 = "#version 100               \n"
            "attribute vec4 a_position;          \n"
            "attribute vec3 a_color;             \n"
            "uniform mat4 u_mvpMatrix;           \n"
            "uniform ivec2 u_offsetL;            \n"
            "uniform ivec2 u_offsetM;            \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;                             \n"
            "   gl_Position.x -= float(u_offsetL.x);                  \n"
            "   gl_Position.x -= float(u_offsetM.x)/float(1000000);   \n"
            "   gl_Position.y = float(u_offsetL.y) - gl_Position.y;   \n"
            "   gl_Position.y += (float(u_offsetM.y)/float(1000000));   \n"
            "   gl_Position  *= u_mvpMatrix;     \n"
            "   gl_Position.y -= 0.3;            \n"
            "}                                   \n";
    const char *f4 = "#version 100\n"
            "precision mediump float;                      \n"
            "uniform vec3 u_color;                                  \n"
            "void main()                                            \n"
            "{                                                      \n"
            "  gl_FragColor = vec4(u_color.x,u_color.y,u_color.z,1.0);                \n"
            "}                                                      \n";

    const char *v5 = "#version 100               \n"
            "attribute vec4 a_position;          \n"
            "uniform mat4 u_mvpMatrix;           \n"
            "uniform ivec2 u_offsetL;            \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;        \n"
            "   gl_Position.x = gl_Position.x - float(u_offsetL.x);      \n"
            "   gl_Position.y -= float(u_offsetL.y);    \n"
            "   gl_Position  *= u_mvpMatrix;     \n"
            //"   gl_Position.z -= 1.0;"
            "}                                   \n";
    const char *f5 = "#version 100              \n"
            "precision mediump float;            \n"
            "uniform vec3 u_color;               \n"
            "void main()                         \n"
            "{                                   \n"
            "  gl_FragColor = vec4(u_color.x,u_color.y,u_color.z,1.0);                \n"
            "}                                                      \n";

    const char *v6 = "#version 100               \n"
            "attribute vec4 a_position;          \n"
            "attribute vec2 a_texCoord;          \n"
            "varying vec2 v_texCoord;            \n"
            "uniform mat4 u_mvpMatrix;           \n"
            "uniform ivec2 u_offsetL;            \n"
            "uniform ivec2 u_offsetM;            \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;        \n"
            "   gl_Position.x = gl_Position.x - float(u_offsetL.x);      \n"
            //"   gl_Position.x -= float(u_offsetM.x)/float(1000000);   \n"
            "   gl_Position.y -= float(u_offsetL.y);    \n"
            //"   gl_Position.y += (float(u_offsetM.y)/float(1000000));   \n"
            "   gl_Position  *= u_mvpMatrix;     \n"
            //"   gl_Position.y -= 0.3;            \n"
            "   v_texCoord = a_texCoord;         \n"
            "}                                   \n";
    const char *f6 = "precision mediump float;                   \n"
            "varying vec2 v_texCoord;                            \n"
            "uniform sampler2D s_texture;                        \n"
            "void main()                                         \n"
            "{                                                   \n"
            "  gl_FragColor = texture2D( s_texture, v_texCoord );\n"
            "}                                                   \n";

    // Load the shaders and get a linked program object
    userData->programGUI = esLoadProgram(v2, f2);
    userData->programKreis = esLoadProgram(v3, f3);
    userData->programMap = esLoadProgram(v4, f4);
    userData->programFBO = esLoadProgram(v5, f5);
    userData->programTex = esLoadProgram(v6, f6);

    userData->positionLocTex = glGetAttribLocation(userData->programTex, "a_position");
    userData->texCoordLocTex = glGetAttribLocation(userData->programTex, "a_texCoord");
    userData->mvpLocTex = glGetUniformLocation(userData->programTex, "u_mvpMatrix");
    userData->samplerLocTex = glGetUniformLocation(userData->programTex, "s_texture");
    userData->offsetLLocTex = glGetUniformLocation(userData->programTex, "u_offsetL");
    userData->offsetMLocTex = glGetUniformLocation(userData->programTex, "u_offsetM");
    glEnableVertexAttribArray(userData->positionLocTex);
    glEnableVertexAttribArray(userData->texCoordLocTex);

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

    userData->positionLocMap = glGetAttribLocation(userData->programMap, "a_position");
    userData->colorLocMap = glGetUniformLocation(userData->programMap, "u_color");
    userData->mvpLocMap = glGetUniformLocation(userData->programMap, "u_mvpMatrix");
    userData->offsetLLocMap = glGetUniformLocation(userData->programMap, "u_offsetL");
    userData->offsetMLocMap = glGetUniformLocation(userData->programMap, "u_offsetM");
    glEnableVertexAttribArray(userData->positionLocMap);

    userData->positionLocFBO = glGetAttribLocation(userData->programFBO, "a_position");
    userData->colorLocFBO = glGetUniformLocation(userData->programFBO, "u_color");
    userData->mvpLocFBO = glGetUniformLocation(userData->programFBO, "u_mvpMatrix");
    userData->offsetLLocFBO = glGetUniformLocation(userData->programFBO, "u_offsetL");
    glEnableVertexAttribArray(userData->positionLocFBO);

    glGenBuffers(20, &userData->buffindex[0]);
    // NICHT LÖSCHEN! der raspi  brauch das
    for (int fg = 0; fg < 20; fg++)
        glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[fg]);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glBufferData(GL_ARRAY_BUFFER, sizeof (buttons), buttons, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // Generate the vertex data
#ifdef __NEW_MESH__
    userData->numIndices =
            esGenMesh2(SCALE, userData->r_width, userData->r_height, &userData->vertices, &userData->texCoord, &userData->indices);
    glGenBuffers(1, &userData->buffindex[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, userData->numIndices * sizeof (unsigned int), &userData->indices[0],
            GL_STATIC_DRAW);
    //glBufferData(GL_ARRAY_BUFFER, (userData->r_width + 1) * (userData->r_height + 1) * 3 * sizeof(GLfloat), &userData->vertices[0], GL_STATIC_DRAW);
    glGenTextures(1, &userData->texindex[0]);
    glBindTexture(GL_TEXTURE_2D, userData->texindex[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
#else
    /*
     userData->numIndices = esGenMesh(SCALE, userData->r_width, userData->r_height, &userData->vertices, &userData->texCoord, NULL);
     glGenTextures(userData->tiles, &userData->texindex[0]);
     unsigned short n = 0;
     for (n = 0; n < userData->tiles; n++) {
     glBindTexture( GL_TEXTURE_2D, userData->texindex[userData->texMap[n]]);
     glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
     glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
     glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
     glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
     }
     */
#endif
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

#ifdef __FBO__
    GLint maxRenderbufferSize;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maxRenderbufferSize);
    PRINTF("\rGL_MAX_RENDERBUFFER_SIZE: %ix%i\n", maxRenderbufferSize, maxRenderbufferSize);
    if ((maxRenderbufferSize <= FBO_WIDTH) || (maxRenderbufferSize <= FBO_HEIGHT)) {
        printf("Cannot use framebuffer objects!\n");
        exit(EXIT_FAILURE);
    }

    glGenFramebuffers(1, &userData->FramebufferName);
    glGenTextures(1, &userData->renderedTexture);
    glGenRenderbuffers(1, &userData->depthrenderbuffer);

    glBindTexture(GL_TEXTURE_2D, userData->renderedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, FBO_WIDTH, FBO_HEIGHT,
            0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glBindRenderbuffer(GL_RENDERBUFFER, userData->depthrenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, FBO_WIDTH, FBO_HEIGHT);

    glBindFramebuffer(GL_FRAMEBUFFER, userData->FramebufferName);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, userData->renderedTexture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, userData->depthrenderbuffer);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        printf("Framebuffer object is not complete!\n");
        exit(EXIT_FAILURE);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif

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

void map(UserData *userData) {
    // Residential
    for (unsigned short m_n = 0; m_n < farbe_reis_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe[m_n].r, farbe[m_n].g, farbe[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe[m_n].index));
    }
    // Flächen
    for (unsigned short m_n = farbe_reis_len; m_n < farbe_yama_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe[m_n].r, farbe[m_n].g, farbe[m_n].b);
        glDrawElements(GL_TRIANGLES, farbe[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe[m_n].index));
    }
    // Wege
    glLineWidth(5.0f);
    for (unsigned short m_n = farbe_yama_len; m_n < farbe_werder_len; m_n++) {
        glUniform3f(userData->colorLocMap, farbe[m_n].r, farbe[m_n].g, farbe[m_n].b);
        glDrawElements(GL_LINES, farbe[m_n].len, GL_UNSIGNED_SHORT, BUFFER_OFFSET(farbe[m_n].index));
    }
    PRINTGLERROR
}

void Update(ESContext *esContext, float deltaTime) {
    UserData *userData = ((ESContext*) esContext)->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;

    esMatrixLoadIdentity(&modelview);
    esRotate(&modelview, posData->angle, 0.0, 0.0, 1.0);
    esTranslate(&modelview, 0.0f, V_Z, 0.0f);

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

    posData->v_x = sinf(posData->angle * rad2deg) * posData->v;
    posData->v_y = cosf(posData->angle * rad2deg) * posData->v;

    posData->g_x += deltaTime * posData->v_x / 3600.0f / posData->osmS;
    posData->g_y -= deltaTime * posData->v_y / 3600.0f / posData->osmS;

    posData->t_x = (int) floor(posData->g_x);
    posData->t_y = (int) floor(posData->g_y);

    posData->m_x = (int) floor(fmod(posData->g_x, 1.0f)*1000000.0);
    posData->m_y = (int) floor(fmod(posData->g_y, 1.0f)*1000000.0);


    // DRAW
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, -160, 800, 800);

    glUseProgram(userData->programMap);
    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFFER_VERTS]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->buffindex[BUFFER_INDEX]);

    pthread_mutex_lock(&m_deto);

    if (new_tex == 1) {
        glBufferData(GL_ARRAY_BUFFER, 12 * verts_len, verts, GL_DYNAMIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, yama_index_len * sizeof (short), yama_index, GL_DYNAMIC_DRAW);
        new_tex = 0;
    }

    glUniformMatrix4fv(userData->mvpLocMap, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);
    glUniform2i(userData->offsetLLocMap, posData->t_x, posData->t_y);
    glUniform2i(userData->offsetMLocMap, posData->m_x, posData->m_y);
    glVertexAttribPointer(userData->positionLocMap, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*) 0);

    map(userData);
    pthread_mutex_unlock(&m_deto);


    // GUI
    glUseProgram(userData->programGUI);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(userData->samplerLocGui, 0);

    glBindBuffer(GL_ARRAY_BUFFER, userData->buffindex[BUFF_INDEX_GUI]);
    glVertexAttribPointer(userData->positionLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) 0);
    glVertexAttribPointer(userData->texCoordLocGui, 2, GL_FLOAT, GL_FALSE, 4 * sizeof (GL_FLOAT), (GLvoid*) (2 * sizeof (GL_FLOAT)));
    // Dreieck
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[2]);
    glDrawArrays(GL_TRIANGLES, 0, 3);
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
            /*
            case 52:
                trans_x = 0.0f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 53:
                trans_y = 0.0f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 54:
                trans_z = 0.0f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 55:
                trans_x += 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 56:
                trans_y += 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 57:
                trans_z += 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 49:
                trans_x -= 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 50:
                trans_y -= 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 51:
                trans_z -= 0.1f;
                printf("%f %f %f\n", trans_x, trans_y, trans_z);
                break;
            case 47:
                orto1 -= 0.1f;
                printf("%f/%f\n", orto1, orto2);
                break;
            case 42:
                orto1 += 0.1f;
                printf("%f/%f\n", orto1, orto2);
                break;
            case 45:
                orto2 -= 0.1f;
                printf("%f/%f\n", orto1, orto2);
                break;
            case 43:
                orto2 += 0.1f;
                printf("%f/%f\n", orto1, orto2);
                break;


            case 105:
                orto3 -= 0.1f;
                printf("%f/%f\n", orto3, orto4);
                break;
            case 107:
                orto3 += 0.1f;
                printf("%f/%f\n", orto3, orto4);
                break;
            case 111:
                orto4 -= 0.1f;
                printf("%f/%f\n", orto3, orto4);
                break;
            case 108:
                orto4 += 0.1f;
                printf("%f/%f\n", orto3, orto4);
                break;
             */
        case 119:
            posData->v += 30.0f;
            PRINTF("v: %f\n", posData->v)
                    ;
            break;
        case 115:
            posData->v -= 30.0f;
            PRINTF("v: %f\n", posData->v)
                    ;
            break;
        case 97:
            posData->angle -= 3.0f;
            if (posData->angle < 0.0f) {
                posData->angle += 360.0f;
            }
            PRINTF("angle: %f\n", posData->angle)
                    ;
            break;
        case 100:
            posData->angle += 3.0f;
            if (posData->angle >= 360.0f) {
                posData->angle -= 360.0f;
            }
            PRINTF("angle: %f\n", posData->angle)
                    ;
            break;
        case 45:
            if (ORTHO < 10.0) {
                ORTHO += 1.0f;
                esMatrixLoadIdentity(&perspective);
                esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
            }
            break;
        case 43:
            if (ORTHO > 2.0) {
                ORTHO -= 1.0f;
                esMatrixLoadIdentity(&perspective);
                esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
            }
            break;
    }

}
//static bool intrack = false;
//static time_t timeout = 5; /* seconds */
//static double minmove = 0; /* meters */

/*
void conditionally_log_fix(struct gps_data_t gpsdata) {
    double int_time, old_int_time = 0.0f;
    double old_lat = 0.0f, old_lon = 0.0f;
    bool first = true;

    int_time = gpsdata.fix.time;
    if ((int_time == old_int_time) || gpsdata.fix.mode < MODE_2D)
        return;

    if (minmove > 0 && !first && earth_distance(gpsdata.fix.latitude, gpsdata.fix.longitude, old_lat, old_lon) < minmove)
        return;

    if (fabs(int_time - old_int_time) > timeout && !first) {
        intrack = false;
    }

    if (!intrack) {
        intrack = true;
        if (first)
            first = false;
    }

    old_int_time = int_time;
    if (minmove > 0) {
        old_lat = gpsdata.fix.latitude;
        old_lon = gpsdata.fix.longitude;
    }
    PRINTF("lat=\"%f\" lon=\"%f\"\n", gpsdata.fix.latitude, gpsdata.fix.longitude);
}*/
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

unsigned short linaere_suche_VERTS(float *M, unsigned short count, float x, float y) {
    for (unsigned short n = 0; n < count; n++) {
        if ((M[3 * n] == x) && (M[3 * n + 1] == y)) {
            return n;
        }
    }
    return __SHRT_MAX__;
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

void insertVert_index(T_V_E *src, unsigned short len, unsigned short *index, unsigned short *index_len, T_V_E *fa, unsigned short *farbe_len) {
    unsigned short gef2 = 0;
    float r = -1.0, g = -1.0, b = -1.0;

    for (unsigned short a = 0; a < len; a++) {
        if ((src[a].r != r) || (src[a].g != g) || (src[a].b != b)) {
            fa[*farbe_len].r = src[a].r;
            fa[*farbe_len].g = src[a].g;
            fa[*farbe_len].b = src[a].b;
            r = src[a].r;
            g = src[a].g;
            b = src[a].b;

            fa[*farbe_len].index = *index_len;
            fa[*farbe_len].len = src[a].len;

            (*farbe_len)++;
        } else {
            fa[*farbe_len - 1].len += src[a].len;
        }

        for (unsigned short n = 0; n < src[a].len; n++) {
            gef2 = linaere_suche_VERTS(verts, verts_len, tmp_verts[ 3 * (src[a].index + n)], tmp_verts[ 3 * (src[a].index + n) + 1]);
            index[*index_len] = gef2;
            (*index_len)++;
        }
    }
}

void insertT_V_E(T_V_E *dst, unsigned short *dst_len, T_V_E *src, unsigned short len) {
    unsigned short gef = 0;
    for (unsigned short n = 0; n < tmp_vert_len; n++) {
        gef = linaere_suche_VERTS(verts, verts_len, tmp_verts[ 3 * n], tmp_verts[ 3 * n + 1]);

        if (gef == __SHRT_MAX__) {

            memcpy(&verts[3 * verts_len], &tmp_verts[3 * n], 3 * sizeof (GLfloat));
            verts_len++;
        }
    }

    memcpy(dst, src, len * sizeof (T_V_E));
    *dst_len = len;
}

void insertT_V_E_old(T_V_E *dst, unsigned short *dst_len, T_V_E *src, unsigned short len) {
    unsigned short gef = 0, gef2 = 0;
    for (int a = 0; a < len; a++) {
        gef = binaere_suche(dst, *dst_len, src[a].id);

        if (gef == __SHRT_MAX__) {
            for (unsigned short n = 0; n < src[a].len; n++) {
                gef2 = linaere_suche_VERTS(verts, verts_len, tmp_verts[ 3 * (src[a].index + n)], tmp_verts[ 3 * (src[a].index + n) + 1]);

                if (gef2 == __SHRT_MAX__) {

                    memcpy(&verts[3 * verts_len], &tmp_verts[3 * (src[a].index + n)], 3 * sizeof (GLfloat));
                    verts_len++;
                }
            }
            //memcpy(&verts[3 * verts_len], &tmp_verts[3 * src[a].index], 3 * src[a].len * sizeof (GLfloat));
            //src[a].index = verts_len;
            //verts_len += src[a].len;
            memcpy(&dst[(*dst_len)++], &src[a], sizeof (T_V_E));
        }
    }
}

void loadVert(double lon, double lat, double d) {

    struct timespec spec0, spec1, spec2;
    unsigned int ms;

    char where[110];
    char sql_vertex[200];
    char sql_polygon[200];
    char sql_reis[200];
    char sql_verts[200];
    char sql[500];

    setlocale(LC_NUMERIC, "en_US.UTF-8");

    snprintf(where, 110, " v_lon between %f and  %f and v_lat between %f and %f", lon - d, lon + d, lat - d, lat + d);
    snprintf(sql_vertex, 200, "(SELECT DISTINCT v_l_id FROM vertex WHERE %s)", where);
    snprintf(sql_polygon, 200, "(SELECT DISTINCT v_l_id FROM polygon WHERE %s)", where);
    snprintf(sql_reis, 200, "(SELECT DISTINCT v_l_id FROM residential WHERE %s)", where);

    snprintf(sql_verts, 200, "(SELECT DISTINCT v_l_id FROM v_verts WHERE %s)", where);

    char *zErrMsg = 0;

    tmp_vert_len = 0;
    tmp_yama_len = 0;
    tmp_werder_len = 0;
    tmp_reis_len = 0;
    sqc_tmp = 0;

    //snprintf(sql, 1000, "SELECT v_lon,v_lat FROM polygon t JOIN %s g ON t.v_l_id = g.v_l_id UNION SELECT v_lon,v_lat FROM vertex t JOIN %s  g ON t.v_l_id = g.v_l_id UNION SELECT v_lon,v_lat FROM residential t JOIN %s g ON t.v_l_id = g.v_l_id", sql_polygon, sql_vertex, sql_reis);

    clock_gettime(CLOCK_MONOTONIC, &spec0);
    clock_gettime(CLOCK_MONOTONIC, &spec1);

    // Flächen laden
    snprintf(sql, 500, "SELECT v_lon,v_lat FROM polygon t JOIN %s g ON t.v_l_id=g.v_l_id ORDER BY t.v_l_id ASC,t.v_seq ASC", sql_polygon);
    sqlite3_exec(db, sql, sqc_vertex, 0, &zErrMsg);
    //printf("%s\n", sql);
    TM(spec1, spec2, 'Flächen laden')

    // Wege laden
    snprintf(sql, 500, "SELECT v_lon,v_lat FROM vertex t JOIN %s g ON t.v_l_id=g.v_l_id ORDER BY t.v_l_id ASC,t.v_seq ASC", sql_vertex);
    sqlite3_exec(db, sql, sqc_vertex, 0, &zErrMsg);
    //printf("%s\n", sql);
    TM(spec2, spec1, 'Wege laden')

    // Residental laden
    snprintf(sql, 500, "SELECT v_lon,v_lat FROM residential t JOIN %s g ON t.v_l_id=g.v_l_id ORDER BY t.v_l_id ASC,t.v_seq ASC", sql_reis);
    sqlite3_exec(db, sql, sqc_vertex, 0, &zErrMsg);
    //printf("%s\n", sql);
    TM(spec1, spec2, 'Residental laden')

    // Anzahl der der Flächenfarben
    //snprintf(sql, 500, "SELECT count(*) FROM (SELECT distinct R,G,B FROM eigenschaften JOIN %s ON l_id=v_l_id JOIN config_farbe ON CF_ID=L_TYP)", sql_polygon);
    //sqlite3_exec(db, sql, sqc_farbe, &farbe_len, &zErrMsg);

    // Eigenschaften Fläche
    snprintf(sql, 500, "SELECT L_ID,L_COUNT,R,G,B,OSM_ID FROM eigenschaften JOIN %s ON l_id=v_l_id JOIN config_farbe ON CF_ID=L_TYP ORDER BY l_id ASC", sql_polygon);
    sqlite3_exec(db, sql, sqc_yama, &tmp_reis_len, &zErrMsg);
    tmp_yama_len = tmp_reis_len;
    //printf("%s\n",sql);
    TM(spec2, spec1, 'Eigenschaften Fläche')

    // Eigenschaften Weg
    snprintf(sql, 500, "SELECT L_ID,L_COUNT,R,G,B,OSM_ID FROM eigenschaften JOIN %s ON l_id=v_l_id JOIN config_farbe ON CF_ID=L_TYP ORDER BY l_id ASC", sql_vertex);
    sqlite3_exec(db, sql, sqc_yama, &tmp_reis_len, &zErrMsg);
    tmp_werder_len = tmp_reis_len;
    //printf("%s\n", sql);
    TM(spec1, spec2, 'Eigenschaften Weg')

    // Eigenschaften Residental
    snprintf(sql, 500, "SELECT L_ID,L_COUNT,R,G,B,OSM_ID FROM eigenschaften JOIN %s ON l_id=v_l_id JOIN config_farbe ON CF_ID=L_TYP ORDER BY l_id ASC", sql_reis);
    sqlite3_exec(db, sql, sqc_yama, &tmp_reis_len, &zErrMsg);
    //printf("%s\n", sql);
    TM(spec2, spec1, 'Eigenschaften Residental')

    //qsort(yama, yama_len, sizeof (T_V_E), cmpfunc_id);
    //qsort(werder, werder_len, sizeof (T_V_E), cmpfunc_id);
    //qsort(reis, reis_len, sizeof (T_V_E), cmpfunc_id);
    //TM(spec1, spec2, 'qsort')

    // neue Flächen hinzufügen
    insertT_V_E(yama, &yama_len, &tmp_yama[0], tmp_yama_len);
    insertT_V_E(werder, &werder_len, &tmp_yama[tmp_yama_len], tmp_werder_len - tmp_yama_len);
    insertT_V_E(reis, &reis_len, &tmp_yama[tmp_werder_len], tmp_reis_len - tmp_werder_len);
    TM(spec2, spec1, 'insertT_V_E')

    qsort(yama, yama_len, sizeof (T_V_E), cmpfunc_farbe);
    qsort(reis, reis_len, sizeof (T_V_E), cmpfunc_farbe);
    qsort(werder, werder_len, sizeof (T_V_E), cmpfunc_farbe);
    TM(spec1, spec2, 'qsort')


    ///////////////////
    T_V_E tmp_farbe[20];
    static unsigned short tmp_yama_index[V_LEN];
    unsigned short tmp_farbe_werder_len = 0, tmp_farbe_reis_len = 0, tmp_farbe_yama_len = 0;
    unsigned short tmp_yama_index_len = 0;

    insertVert_index(reis, reis_len, tmp_yama_index, &tmp_yama_index_len, tmp_farbe, &tmp_farbe_werder_len);
    tmp_farbe_reis_len = tmp_farbe_yama_len;
    insertVert_index(yama, yama_len, tmp_yama_index, &tmp_yama_index_len, tmp_farbe, &tmp_farbe_werder_len);
    tmp_farbe_yama_len = tmp_farbe_werder_len;
    insertVert_index(werder, werder_len, tmp_yama_index, &tmp_yama_index_len, tmp_farbe, &tmp_farbe_werder_len);


    pthread_mutex_lock(&m_deto);
    farbe_werder_len = tmp_farbe_werder_len;
    farbe_reis_len = tmp_farbe_reis_len;
    farbe_yama_len = tmp_farbe_yama_len;
    yama_index_len = tmp_yama_index_len;
    memcpy(farbe, tmp_farbe, 20 * sizeof (T_V_E));
    memcpy(yama_index, tmp_yama_index, tmp_yama_index_len * sizeof (unsigned short));
    new_tex = 1;
    pthread_mutex_unlock(&m_deto);




    TM(spec2, spec1, 'insertVert_index')

    printf("\rloadVert   %f, %f   %.1f\n", lon, lat, BR);
    printf("\rloadVert   verts: %i  index: %i  yama: %i  werder: %i resi: %i\n", verts_len, yama_index_len, yama_len, werder_len, reis_len);
    printf("\r###############################################################\n");
    /*
     vbox: alt: ca. 800ms
     */
    TM(spec0, spec2, 'gesamt')
}

void setPOS(ESContext * esContext) {

    POS_T *posData = esContext->posData;

    PRINTF("\rsetPOS lon/lat: %f %f\n", posData->gps_longitude, posData->gps_latitude);

    posData->g_x = (posData->gps_longitude + 180.0) / 360.0 * pow2Z;
    posData->g_y = (1.0 - log(tan(posData->gps_latitude * rad2deg) + 1.0 / cos(posData->gps_latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;
    PRINTF("\rsetPOS g: %f %f\n", posData->g_x, posData->g_y);

    posData->t_x = (int) floor(posData->g_x);
    posData->t_y = (int) floor(posData->g_y);
    PRINTF("\rsetPOS t: %i %i\n", posData->t_x, posData->t_y);

    posData->m_x = (int) floor(fmod(posData->g_x, 1.0f)*1000000.0);
    posData->m_y = (int) floor(fmod(posData->g_y, 1.0f)*1000000.0);
    PRINTF("\rsetPOS m: %i %i\n", posData->m_x, posData->m_y);

    posData->osmS = 40075017 * cos(posData->gps_latitude * M_PI / 180.0) / pow(2.0, 18.0 + 8.0) * IMAGE_SIZE;
}

void * foss(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    //GPS_T *gpsData = ((ESContext*) esContext)->gpsData;

    time_t t;
    struct tm tm;

#ifdef __RASPI__
    int gpio = -1;
    gpio_lcd_init();
    bl_write(BL_ON);
#endif

    while (1) {
        memset(line1_str, 32, 20);
        memset(line2_str, 32, 20);

        t = time(NULL);
        tm = *localtime(&t);
        strftime(line1_str, 21, "%R     %e.%m.%Y", &tm);

        snprintf(line2_str, 21, "U: %.2fV", posData->obd_volt);

#ifdef __RASPI__
        //gpio = gpio_read(GPIO_PIN);

        /*if (gpio == GPIO_HIGH) {
            bl_write(BL_OFF);
            ShutDown(1);
        } else
            bl_write(BL_ON);
         */
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
        usleep(100000);

    }
    return NULL;
}

void * deta(void *esContext) {
    POS_T *posData = ((ESContext*) esContext)->posData;
    UserData *userData = ((ESContext*) esContext)->userData;

    while (1) {

        loadVert(posData->g_x, posData->g_y, BR);
        //new_tex = 1;
        sleep(5);
        //posData->v = 60.0f;
    }
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

    float speed = 0.0f, old_speed = 0.0f, deltatime = 0.0f;
    float deg, old_deg;
    struct timeval t1, t2;
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

#ifdef __GPS__
    int gpsd_init = -1;
    //struct GPS_T gpsData;
    GPS_T *gpsData = ((ESContext*) esContext)->gpsData;
#endif

#ifdef __I2C__
    int i2c_init = -1;
    short i2c_x, i2c_y, i2c_z;
    enum e_i2c_status i2c_status;
#endif

    gettimeofday(&t1, NULL);

    while (1) {
        usleep(500000);

        //posData->v = 30.0f;

        old_lat = posData->gps_latitude;
        old_lon = posData->gps_longitude;

        old_g_x = g_x;
        old_g_y = g_y;

        old_speed = speed;
        old_deg = deg;

        gettimeofday(&t2, NULL);
        deltatime = (float) (t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6) * 1000;
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
            gps_open(&gpsd_init, "/dev/ttyAMA02");
        }
#endif
        //printf("%f\n",posData->g_y);
        /*if (fabs(posData->g_y - posData->gpu_t_y) > 1.0f || fabs(posData->g_x - posData->gpu_t_x) > 1.0f) {
            posData->gpu_t_x = (GLuint) floor(posData->g_x);
            posData->gpu_t_y = (GLuint) floor(posData->g_y);
            loadVert(posData->g_x, posData->g_y, BR);
        }*/

        fwrite(&posData->gps_latitude, sizeof (double), 1, gps_out);
        fwrite(&posData->gps_longitude, sizeof (double), 1, gps_out);
        fwrite(&dist, sizeof (double), 1, gps_out);
        fwrite(&deltatime, sizeof (float), 1, gps_out);
        fwrite(&speed, sizeof (float), 1, gps_out);
        fwrite(&deg, sizeof (float), 1, gps_out);
    }

#ifdef __FIN__
    if (fread(&gpsData->fix.latitude, sizeof (double), 1, gps_in) == 1) {
        fread(&gpsData->fix.longitude, sizeof (double), 1, gps_in);
        fread(&dist, sizeof (double), 1, gps_in);
        fread(&deltatime, sizeof (float), 1, gps_in);
        fread(&speed, sizeof (float), 1, gps_in);
        fread(&deg, sizeof (float), 1, gps_in);
    } else {
        fclose(gps_in);
    }
    gpsData->status = 1;
#endif

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

            /*if ((frames / totaltime) > 30.0) {
                us += 1000;
            } else {
                us = us > 1000 ? us - 1000 : us;
            }*/

            totaltime -= 4.0f;
            frames = 0;

            ms_avg = 0;
            ms_min = 4294967295;
            ms_max = 0;

        }
#endif
        //usleep(us);
    }
}

static int sqc_vertex(void *NotUsed, int argc, char **argv, char **azColName) {
    tmp_verts[3 * tmp_vert_len] = atof(argv[0]);
    tmp_verts[3 * tmp_vert_len + 1] = atof(argv[1]);
    tmp_verts[3 * tmp_vert_len + 2] = 0.0f;

    tmp_vert_len++;

    return 0;
}

static int sqc_farbe(void *a, int argc, char **argv, char **azColName) {
    *((unsigned short*) a) = atoi(argv[0]);

    return 0;
}

static int sqc_yama(void *a, int argc, char **argv, char **azColName) {
    tmp_yama[*((int*) a)].id = atoi(argv[0]);
    tmp_yama[*((int*) a)].len = atoi(argv[1]);
    tmp_yama[*((int*) a)].r = atof(argv[2]);
    tmp_yama[*((int*) a)].g = atof(argv[3]);
    tmp_yama[*((int*) a)].b = atof(argv[4]);
    tmp_yama[*((int*) a)].osm_id = atoi(argv[5]);

    tmp_yama[*((int*) a)].index = sqc_tmp;
    sqc_tmp += tmp_yama[*((int*) a)].len;

    (*((int*) a))++;

    return 0;
}

static int sqc_yama2(void *a, int argc, char **argv, char **azColName) {
    tmp_yama[*((int*) a)].id = atoi(argv[0]);
    tmp_yama[*((int*) a)].len = atoi(argv[1]);
    tmp_yama[*((int*) a)].r = atof(argv[2]);
    tmp_yama[*((int*) a)].g = atof(argv[3]);
    tmp_yama[*((int*) a)].b = atof(argv[4]);

    tmp_yama[*((int*) a)].index = sqc_tmp;
    sqc_tmp += tmp_yama[*((int*) a)].len;

    (*((int*) a))++;

    return 0;
}

static int sqc_count(void *out_var, int argc, char **argv, char **azColName) {
    *((int*) out_var) = atoi(argv[0]);

    return 0;
}
ESContext esContext;

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

    sqlite3_close(db);

#ifdef __RASPI__
    //gpio_unexport(LCD_BUS, 6);
    bl_write(BL_OFF);
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
    UserData userData;
    POS_T posData;
    GPS_T gpsData;

    pthread_t thread_id1, thread_id2, thread_id3, thread_id4;
    pthread_mutex_init(&m_pinto, NULL);
    pthread_mutex_init(&m_deto, NULL);

    signal(SIGTERM, ShutDown);
    //signal(SIGINT, ShutDown);
    setvbuf(stdout, (char *) NULL, _IONBF, 0);

    userData.width = 800;
    userData.height = 480;

    userData.r_height = RSIZE;
    userData.r_width = RSIZE;

#ifdef __RASPI__
    const char *homedir = "/home/pi";
    bl_write(BL_ON);
#else
    const char *homedir = getenv("HOME");
#endif

    char filename[50];
    strncpy(filename, homedir, 50);
    sout = fopen(strncat(filename, "/sit2d_debug.txt", 16), "w");
    strncpy(filename, homedir, 50);
    gps_out = fopen(strncat(filename, "/sit2d.gps", 10), "w");

#ifdef __FIN__
    strcpy(filename, homedir);
    gps_in = fopen(strcat(filename, "/asd.gps"), "r");
    assert(gps_in != NULL);
    PRINTF("%s geladen.\n", filename);
#endif

    userData.element4Pixel = 1;
    userData.imgrow_sz = IMAGE_SIZE * userData.element4Pixel;
    userData.image_sz = IMAGE_SIZE * userData.imgrow_sz;
    userData.tiles = userData.r_height * userData.r_width;

#ifdef __NEW_MESH__
    stride = userData.imgrow_sz * userData.r_width;
#else
    stride = userData.imgrow_sz;
#endif
    //userData.tex_buffer = malloc(userData.image_sz * userData.tiles * sizeof(TEX_BUFFER_FORMAT));
    //assert(userData.tex_buffer != NULL);
    //userData.texindex = calloc(sizeof(GLuint), userData.tiles);
    //userData.texMap = calloc(sizeof(GLuint), userData.tiles);
    //userData.texfree = calloc(sizeof(GLuint), userData.tiles);
    //userData.texreload = calloc(sizeof(GLuint), userData.tiles);
    //tmp = calloc(sizeof(GLuint), userData.tiles);
    //posData.tiles_x = calloc(sizeof(GLuint), userData.r_height);
    //posData.tiles_y = calloc(sizeof(GLuint), userData.r_width);
    //posData.gpu_tiles_x = calloc(sizeof(GLuint), userData.r_height);
    //posData.gpu_tiles_y = calloc(sizeof(GLuint), userData.r_width);
    //userData.font_buffer = calloc(sizeof(char), 512 * 256 * 4);
    //label_tex = calloc(sizeof(GLuint), 10);

    esInitContext(&esContext);
    esContext.userData = &userData;
    esContext.posData = &posData;
    esContext.gpsData = &gpsData;

    esCreateWindow(&esContext, "SiT2D", userData.width, userData.height, ES_WINDOW_RGB);

    rad2deg = M_PI / 180.0f;
    pow2Z = pow(2.0, 18.0);
    posData.gps_latitude = 52.1340037;
    posData.gps_longitude = 11.6547915;
    //posData.gps_latitude = 52.0370333;
    //posData.gps_longitude = 11.6658727;
    posData.gps_altitude = 0.0;
    posData.obd_speed = 0.0;
    posData.obd_volt = 12.4;
    posData.angle = 0.0f;
    posData.v = 0.0;

    setPOS(&esContext);

    posData.gpu_t_x = (GLuint) floor(posData.g_x);
    posData.gpu_t_y = (GLuint) floor(posData.g_y);

#define SZ 4096
#define PAGES 8192
    strncpy(filename, homedir, 50);
    char *db_file = strncat(filename, "/sit2d_4.db", 50);
    double *sqlite_cache = (double*) malloc(SZ * PAGES);
    assert(sqlite3_config(SQLITE_CONFIG_PAGECACHE, &sqlite_cache[0], SZ, PAGES) == SQLITE_OK);
    sqlite3_initialize();
    assert(sqlite3_open_v2(db_file, &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);

    verts_len = 0;
    yama_len = 0;
    werder_len = 0;

    loadVert(posData.g_x, posData.g_y, BR);

    //exit(0);

    esMatrixLoadIdentity(&perspective);
    esMatrixLoadIdentity(&mvpTex);
    esOrtho(&perspective, -ORTHO, ORTHO, -ORTHO, ORTHO, -50, 50);
    esOrtho(&mvpTex, -BR, BR, -BR, BR, -50, 50);

    char buf[50];

    snprintf(buf, 50, "%s/RES/minus.tga", homedir);
    load_TGA(minus_tex, buf);
    snprintf(buf, 50, "%s/RES/plus.tga", homedir);
    load_TGA(plus_tex, buf);
    snprintf(buf, 50, "%s/RES/menu.tga", homedir);
    load_TGA(menu_tex, buf);
    snprintf(buf, 50, "%s/RES/ascii.tga", homedir);
    load_TGA(tex_zahlen, buf);

    //load_tex_images(&esContext);
    assert(Init(&esContext));
    //upload_tex(&esContext);

    //loadVert(posData.g_x, posData.g_y, BR);
    //glBindBuffer(GL_ARRAY_BUFFER, userData.buffindex[BUFF_INDEX_MAP]);
    //glBufferData(GL_ARRAY_BUFFER, 12 * vert_len, verts, GL_DYNAMIC_DRAW);
#ifndef __RASPI__
    esRegisterKeyFunc(&esContext, Key);
#endif
    PRINTF("\rinit abgeschlossen\n");

    //intro(&esContext);
    glClearColor(1.0f, 1.0f, 0.0f, 1.0f);

    //pthread_create(&thread_id1, NULL, &foss, (void*) &esContext);
    //pthread_create(&thread_id2, NULL, &pinto, (void*) &esContext);
    //pthread_create(&thread_id3, NULL, &jolla, (void*) &esContext);
    pthread_create(&thread_id4, NULL, &deta, (void*) &esContext);

    esMainLoop(&esContext);

    pthread_join(thread_id1, NULL);
    pthread_join(thread_id2, NULL);
    pthread_join(thread_id3, NULL);
    pthread_join(thread_id4, NULL);

    //ShutDown(&esContext);

    return (EXIT_SUCCESS);
}