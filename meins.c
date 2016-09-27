#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <math.h>
#include <sqlite3.h>
#include <stdbool.h>
#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>
#include "util.h"
#include "gui.h"
#include "obd.h"
#include "i2c.h"
#include "gps.h"

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

#define BUFFER_OFFSET(i) ((char *) NULL + (4*i))
#define ROUND(x) roundf(x*10)/10
#define MEM(x) (int)(x * 256 )
#define PRINTF(...) fprintf(sout,__VA_ARGS__);fprintf(stdout,__VA_ARGS__)

FILE *gps_out, *gps_in;
FILE *sout;

sqlite3 *db;

typedef struct {
    GLsizei len;
    GLfloat r, g, b;

} T_V_EIGENSCHAFTEN;

static T_V_EIGENSCHAFTEN yama[2000];
static T_V_EIGENSCHAFTEN tmp_yama[400];

unsigned int Indices_len;
GLuint *Indices;

int vert_len, tmp_vert_len, yama_len, tmp_yama_len, werder_len, tmp_werder_len;

#define V_LEN 20000
static GLfloat verts[V_LEN] = {0.0f};
static GLfloat tmp_verts[V_LEN] = {0.0f};
static GLfloat verts2[V_LEN] = {0.0f};

void setPOS(ESContext *esContext);
static int sqc_yama(void *NotUsed, int argc, char **argv, char **azColName);
static int sqc_vertex(void *NotUsed, int argc, char **argv, char **azColName);
static int sqc_count(void *out_var, int argc, char **argv, char **azColName);

void printM4(ESMatrix *wg);

#define V_KORR 1.7f

GLuint *label_tex;

const char *homedir;

double mf;
double pow2Z;
float rad2deg;
GLubyte ZOOM_S[] = {18, 17, 16, 15, 14, 12, 10, 8};
GLubyte *ZOOM;

int dx = 0, dy = 0, m_n, m_m, p_n, p_m;
GLuint *tmp;
//static struct fixsource_t source;
ESMatrix perspective, modelview, mvpMatrix;

unsigned int stride;
float gps_status[3] = {1.0f, 0.0f, 0.0f};

unsigned short new_tex = 0;
pthread_mutex_t m_pinto = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    // Handle to a program object
    GLuint programObject2;
    GLuint programGUI;
    GLuint programKreis;

    // Attribute locations
    GLint positionLoc;
    GLint positionLoc2;
    GLint positionLoc3;
    GLint positionLoc4;
    GLint positionLoc5;
    GLint offsetLoc;
    GLint offsetLoc2;
    GLint offsetLoc3;
    GLint texCoordLoc;
    GLint texCoordLoc2;
    GLint colorLoc;
    GLint texLoc;
    GLint texLoc2;
    GLint samplerLoc;
    GLint samplerLoc2;
    // Uniform locations
    GLint mvpLoc;
    GLint mvpLoc2;
    GLint mvpLoc3;
    GLint gps_i_f;

    // Vertex data
    GLfloat *vertices;
    GLfloat *texCoord;
    GLuint *indices;
    GLuint numIndices;

    // MVP matrix
    //ESMatrix mvpMatrix;

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
    char *font_buffer;
    GLuint textureId;

    //Buffer
    GLuint bufindex[2];
} UserData;

typedef struct {
    GLuint gpu_t_x;
    GLuint gpu_t_y;
    GLuint t_x;
    GLuint t_y;
    GLuint *tiles_x;
    GLuint *tiles_y;
    GLuint *gpu_tiles_x;
    GLuint *gpu_tiles_y;
    double mx;
    double my;
    float v_x;
    float v_y;
    GLfloat v;
    GLfloat angle;
    double gps_longitude;
    double gps_latitude;
    double gps_altitude;
    int gps_mode;
    int gps_sat;
    float obd_speed;
    double osmS;
    double g_x;
    double g_y;
    short i2c_x;
    short i2c_y;
    short i2c_z;
} POS_T;

typedef struct {
    float lat;
    float lon;
    char name[100];
} LABEL_T;

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

    const char *v4 = "#version 100\n"
            "attribute vec4 a_position; \n"
            "attribute vec3 a_color;             \n"
            "uniform mat4 u_mvpMatrix;           \n"
            "uniform vec2 u_offset;              \n"
            "void main()                         \n"
            "{                                   \n"
            "   gl_Position = a_position;        \n"
            //"   gl_Position.x -= u_offset.x;     \n"
            //"   gl_Position.y = u_offset.y - gl_Position.y;     \n"
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

    // Load the shaders and get a linked program object
    userData->programGUI = esLoadProgram(v2, f2);
    userData->programKreis = esLoadProgram(v3, f3);
    userData->programObject2 = esLoadProgram(v4, f4);

    userData->positionLoc2 = glGetAttribLocation(userData->programGUI, "a_position");
    userData->texCoordLoc2 = glGetAttribLocation(userData->programGUI, "a_texCoord");
    userData->texLoc2 = glGetUniformLocation(userData->programGUI, "s_baseMap");
    userData->samplerLoc2 = glGetUniformLocation(userData->programGUI, "s_texture");

    userData->positionLoc3 = glGetAttribLocation(userData->programKreis, "a_position");
    userData->positionLoc4 = glGetAttribLocation(userData->programKreis, "a_texCoord");
    userData->gps_i_f = glGetUniformLocation(userData->programKreis, "u_f");

    userData->positionLoc5 = glGetAttribLocation(userData->programObject2, "a_position");
    userData->colorLoc = glGetUniformLocation(userData->programObject2, "u_color");
    userData->mvpLoc3 = glGetUniformLocation(userData->programObject2, "u_mvpMatrix");
    userData->offsetLoc3 = glGetUniformLocation(userData->programObject2, "u_offset");

    glEnableVertexAttribArray(userData->positionLoc);
    glEnableVertexAttribArray(userData->texCoordLoc);

    glEnableVertexAttribArray(userData->positionLoc2);
    glEnableVertexAttribArray(userData->texCoordLoc2);

    glEnableVertexAttribArray(userData->positionLoc3);
    glEnableVertexAttribArray(userData->positionLoc4);

    glEnableVertexAttribArray(userData->positionLoc5);

    // Generate the vertex data
#ifdef __NEW_MESH__
    userData->numIndices =
            esGenMesh2(SCALE, userData->r_width, userData->r_height, &userData->vertices, &userData->texCoord, &userData->indices);
    glGenBuffers(1, &userData->bufindex[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->bufindex[0]);
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
    gui_tex_index = calloc(sizeof (unsigned int), 4);
    glGenTextures(4, &gui_tex_index[0]);
    glGenTextures(47, &gui_tex_zahlen[0]);

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
    /*
     glGenBuffers(1, &userData->bufindex[0]);
     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, userData->bufindex[0]);
     glBufferData(GL_ELEMENT_ARRAY_BUFFER, Indices_len * sizeof(GLuint), &Indices[0], GL_STATIC_DRAW);

     glGenBuffers(1, &userData->bufindex[0]);
     glBindBuffer(GL_ARRAY_BUFFER, userData->bufindex[0]);
     glBufferData(GL_ARRAY_BUFFER, 3 * vert_len * sizeof(GLfloat), verts, GL_STATIC_DRAW);

     glGenBuffers(2, &userData->bufindex[0]);
     glBindBuffer(GL_ARRAY_BUFFER, userData->bufindex[0]);
     glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_data), vertex_data,	GL_STATIC_DRAW);
     glVertexAttribPointer(userData->positionLoc, 3, GL_FLOAT, GL_FALSE, 0, userData->vertices);
     //glVertexAttribPointer(userData->texCoordLoc, 2, GL_FLOAT, GL_FALSE, 0, userData->texCoord);
     */
    userData->texCoord = calloc(3 * 8, sizeof (GLfloat));
    for (int a = 0; a < 3; a++) {
        userData->texCoord[3 + a * 8] = 1.0f;
        userData->texCoord[4 + a * 8] = 1.0f;
        userData->texCoord[6 + a * 8] = 1.0f;
        userData->texCoord[7 + a * 8] = 1.0f;
    }
    /*
     for (int fg; fg < 16; fg += 8) {
     printf("%f %f\n", userData->texCoord[fg], userData->texCoord[fg + 1]);
     printf("%f %f\n", userData->texCoord[fg + 2], userData->texCoord[fg + 3]);
     printf("%f %f\n", userData->texCoord[fg + 4], userData->texCoord[fg + 5]);
     printf("%f %f\n", userData->texCoord[fg + 6], userData->texCoord[fg + 7]);
     printf("#######################\n");
     }*/

    /*
     userData->texCoord[0] = 0.f;
     userData->texCoord[1] = 0.f;
     userData->texCoord[2] = 0.f;
     userData->texCoord[3] = 1.f;
     userData->texCoord[4] = 1.f;
     userData->texCoord[5] = 0.f;
     userData->texCoord[6] = 1.f;
     userData->texCoord[7] = 1.f;

     userData->texCoord[8] = 0.f;
     userData->texCoord[9] = 0.f;
     userData->texCoord[10] = 0.f;
     userData->texCoord[11] = 1.f;
     userData->texCoord[12] = 1.f;
     userData->texCoord[13] = 0.f;
     userData->texCoord[14] = 1.f;
     userData->texCoord[15] = 1.f;
     */

    glViewport(0, -160, 800, 800);
 
    return GL_TRUE;
}

void printM4(ESMatrix *wg) {
    PRINTF("%f %f %f %f\n", wg->m[0][0], wg->m[0][1], wg->m[0][2], wg->m[0][3]);
    PRINTF("%f %f %f %f\n", wg->m[1][0], wg->m[1][1], wg->m[1][2], wg->m[1][3]);
    PRINTF("%f %f %f %f\n", wg->m[2][0], wg->m[2][1], wg->m[2][2], wg->m[2][3]);
    PRINTF("%f %f %f %f\n\n", wg->m[3][0], wg->m[3][1], wg->m[3][2], wg->m[3][3]);
}

void debug(ESContext *esContext) {
    UserData *userData = esContext->userData;
    POS_T *posData = esContext->posData;
    PRINTF("----------------\n");
    int n, m;
    for (n = 0; n < userData->r_height; n++) {
        for (m = 0; m < userData->r_width; m++) {
            PRINTF("%i ", userData->texMap[n * userData->r_width + m]);
        }
        PRINTF("\n");
    }
    PRINTF("gpu_tiles_x: %i:  ", posData->gpu_t_x);
    for (n = 0; n < userData->r_width; n++) {
        PRINTF("%i ", posData->gpu_tiles_x[n]);
    }
    PRINTF("\ntiles_x:     %i:  ", posData->t_x);
    for (n = 0; n < userData->r_width; n++) {
        PRINTF("%i ", posData->tiles_x[n]);
    }
    PRINTF("\ngpu_tiles_y: %i:  ", posData->gpu_t_y);
    for (n = 0; n < userData->r_height; n++) {
        PRINTF("%i ", posData->gpu_tiles_y[n]);
    }
    PRINTF("\ntiles_y:     %i:  ", posData->t_y);
    for (n = 0; n < userData->r_height; n++) {
        PRINTF("%i ", posData->tiles_y[n]);
    }
    PRINTF("\n################\n");
}

void renderText(GLint *positionLoc, char *str, GLfloat p_x, GLfloat p_y) {
    int len = strlen(str);

    GLfloat fgh[] = {-0.025f + p_x, 0.05f + p_y, 0.0f, -0.025f + p_x, -0.05f + p_y, 0.0f, 0.025f + p_x, 0.05f + p_y, 0.0f, 0.025f + p_x,
        -0.05f + p_y, 0.0f};

    glVertexAttribPointer(*positionLoc, 3, GL_FLOAT, GL_FALSE, 0, fgh);

#define A_O 0.06f

    for (int p = 0; p < len; p++) {
        fgh[0] += A_O;
        fgh[3] += A_O;
        fgh[6] += A_O;
        fgh[9] += A_O;

        glBindTexture(GL_TEXTURE_2D, gui_tex_zahlen[str[p] - 43]);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
}

void renderFloat(GLint *positionLoc, float zahl, GLfloat p_x, GLfloat p_y) {
    char str[16];
    snprintf(str, 16, "%f", zahl);
    renderText(positionLoc, str, p_x, p_y);
}

void renderInt(GLint *positionLoc, int zahl, GLfloat p_x, GLfloat p_y) {
    char str[16];
    snprintf(str, 16, "%d", zahl);
    renderText(positionLoc, str, p_x, p_y);
}

void intro(ESContext *esContext) {
    UserData *userData = esContext->userData;

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(userData->programGUI);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(userData->samplerLoc2, 0);

    glVertexAttribPointer(userData->texCoordLoc2, 2, GL_FLOAT, GL_FALSE, 0, userData->texCoord);

    //while (userInterrupt(esContext) == GL_FALSE) {
    //for (int p = 0; p < 47; p++) {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    //renderZahlen(&userData->positionLoc2, p, -0.9f, 0.3f);
    renderText(&userData->positionLoc2, "+,-./0123456789:;<=>?@ABCDE", -1.0f, 0.5f);
    renderText(&userData->positionLoc2, "FGHIJKLMNOPQRSTUVWXYZ", -1.0f, 0.32f);

    eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);
    usleep(500000);
    //}
}

void Update2(ESContext *esContext, float deltaTime) {
    UserData *userData = esContext->userData;
    POS_T *posData = ((ESContext*) esContext)->posData;
    //struct gps_data_t *gpsData = ((ESContext*) esContext)->gpsData;

    esMatrixLoadIdentity(&modelview);
    esRotate(&modelview, posData->angle, 0.0, 0.0, 1.0);
    esTranslate(&modelview, 0.0, V_Z, 0.0);

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
    posData->g_y -= deltaTime * posData->v_y / 3600.0 / posData->osmS;

    // DRAW

    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(userData->programObject2);


    for (m_n = 0; m_n < 3 * vert_len; m_n += 3) {
        //verts2[m_n] = verts[m_n] - posData->g_x;
        verts2[m_n] = 0.0f;
        //verts2[m_n + 1] = posData->g_y - verts[m_n + 1];
        verts2[m_n + 1] = 0.0f;
        verts2[m_n + 2] = 0.0f;
    }

    glUniformMatrix4fv(userData->mvpLoc3, 1, GL_FALSE, (GLfloat*) mvpMatrix.m);
    //glUniform2f(userData->offsetLoc3, (float) posData->g_x, (float) posData->g_y);
    glUniform2f(userData->offsetLoc3, 0.0f, 0.0f);
    glVertexAttribPointer(userData->positionLoc5, 3, GL_FLOAT, GL_FALSE, 0, verts2);

    GLint summe = 0;
    // Flächen
    for (m_n = 0; m_n < yama_len; m_n++) {
        glUniform3f(userData->colorLoc, yama[m_n].r, yama[m_n].g, yama[m_n].b);
        //printf("\r%i %i %i %d\n", m_n, summe, yama[m_n].len, yama_len);
        if (m_n == 2)
            //glDrawArrays(GL_TRIANGLE_FAN, summe, yama[m_n].len);
            glDrawArrays(GL_POINTS, summe, yama[m_n].len);
        summe += yama[m_n].len;
        
    }
/*
    // Wege
    glLineWidth(5.0f);
    for (m_n = yama_len; m_n < (werder_len + yama_len); m_n++) {
        glUniform3f(userData->colorLoc, yama[m_n].r, yama[m_n].g, yama[m_n].b);
        //printf("%i %i %i %d\n", m_n, summe, yama[m_n].len, yama_len);
        glDrawArrays(GL_LINE_STRIP, summe, yama[m_n].len);
        summe += yama[m_n].len;
        break;
    }
*/


    // GUI
    glUseProgram(userData->programGUI);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(userData->samplerLoc2, 0);

    glVertexAttribPointer(userData->positionLoc2, 3, GL_FLOAT, GL_FALSE, 0, center);
    glVertexAttribPointer(userData->texCoordLoc2, 2, GL_FLOAT, GL_FALSE, 0, userData->texCoord);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[2]);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glVertexAttribPointer(userData->positionLoc2, 3, GL_FLOAT, GL_FALSE, 0, buttons);

    glBindTexture(GL_TEXTURE_2D, gui_tex_index[0]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[1]);
    glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
    glBindTexture(GL_TEXTURE_2D, gui_tex_index[3]);
    glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);

    renderInt(&userData->positionLoc2, posData->obd_speed, -1.0f, 0.54f);
    renderFloat(&userData->positionLoc2, posData->g_x, -1.0f, 0.4f);
    renderFloat(&userData->positionLoc2, posData->g_y, -1.0f, 0.26f);

    // GPS_I
    glUseProgram(userData->programKreis);

    glVertexAttribPointer(userData->positionLoc3, 3, GL_FLOAT, GL_FALSE, 20, gps_i);
    glVertexAttribPointer(userData->positionLoc4, 2, GL_FLOAT, GL_FALSE, 20, &gps_i[3]);
    glUniform3f(userData->gps_i_f, gps_status[0], gps_status[1], gps_status[2]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

}

void Key(ESContext *esContext, unsigned char a, int b, int c) {
    POS_T *posData = esContext->posData;
    uint l = 0;

    switch (a) {
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
        case 43:
            if (*ZOOM < 18) {
                GLubyte z_old = *ZOOM;
                ZOOM--;
                setPOS(esContext);
                for (l = 0; l < 3 * vert_len; l += 3) {
                    verts[l] *= 2.0f * (*ZOOM - z_old);
                    verts[l + 1] *= 2.0f * (*ZOOM - z_old);
                }
            }
            break;
        case 45:
            if (*ZOOM > 8) {
                GLubyte z_old = *ZOOM;
                ZOOM++;
                setPOS(esContext);
                for (l = 0; l < 3 * vert_len; l += 3) {
                    verts[l] /= 2.0f * (z_old - *ZOOM);
                    verts[l + 1] /= 2.0f * (z_old - *ZOOM);
                }
            }
            break;
    }

    esMatrixLoadIdentity(&modelview);
    esRotate(&modelview, posData->angle, 0.0, 0.0, 1.0);
    //printM4(&modelview);
    esTranslate(&modelview, 0.0, V_Z, 0.0);

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

}

void getLabel(ESContext *esContext) {

}

void ShutDown(ESContext *esContext) {
    //void sigfunc(int sig) 
    UserData *userData = esContext->userData;
    POS_T *posData = esContext->posData;

#ifdef __FOUT__
    fclose(sout);
#endif
    fclose(gps_out);

    sqlite3_close(db);
    //free(userData->vertices);
    //free(userData->indices);

    // Delete program object
    glDeleteProgram(userData->programGUI);
    glDeleteProgram(userData->programKreis);
    glDeleteProgram(userData->programObject2);

    free(tmp);
    //free(userData->tex_buffer);
    //free(userData->font_buffer);
    free(posData->tiles_x);
    //free(posData->tiles_y);

    //free(posData);
    //free(userData);
    free(gui_tex_index);
    free(Indices);

    //free(verts_org);
    //free(verts);
    //free(werder);

    PRINTF("\rthe end is near!\n");
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

void loadVert(double lon, double lat) {
    char where[500];
    char where2[500];
    char sql[1000];

    PRINTF("\rloadVert: %f, %f\n", lon, lat);

    //tmp_verts = calloc(10000, sizeof (GLfloat));
    //tmp_yama = calloc(400, sizeof (T_V_EIGENSCHAFTEN));

#define BR 20.0
    snprintf(where, 500, "where v_lon between %f and  %f and v_lat between %f and %f", lon - BR, lon + BR, lat - BR, lat + BR);
    //sprintf(where, " where v_l_id=4784 ");

    char *zErrMsg = 0;

    tmp_vert_len = 0;

    // Flächen laden
    snprintf(sql, 1000, "select v_lon, v_lat from polygon %s order by v_l_id asc, v_lvl asc, v_seq asc", where);
    sqlite3_exec(db, sql, sqc_vertex, 0, &zErrMsg);

    // Wege laden
    snprintf(sql, 1000, "select v_lon, v_lat from vertex %s order by v_l_id asc, v_seq asc", where);
    sqlite3_exec(db, sql, sqc_vertex, 0, &zErrMsg);

    // Eigenschaften Fläche
    tmp_yama_len = 0;
    snprintf(sql, 1000, "select count(*),avg(l_f_r),avg(l_f_g),avg(l_f_b) from polygon left join eigenschaften on l_id=v_l_id %s group by v_l_id order by v_l_id asc", where);
    sqlite3_exec(db, sql, sqc_yama, 0, &zErrMsg);

    // Eigenschaften Weg
    snprintf(sql, 1000, "select count(distinct v_l_id) from vertex %s", where);
    sqlite3_exec(db, sql, sqc_count, &tmp_werder_len, &zErrMsg);

    snprintf(sql, 1000, "select count(*),avg(l_f_r),avg(l_f_g),avg(l_f_b) from vertex left join eigenschaften on v_l_id=l_id %s group by v_l_id order by v_l_id asc", where);
    sqlite3_exec(db, sql, sqc_yama, 0, &zErrMsg);
    tmp_yama_len -= tmp_werder_len;
    //////////////////////

    int lk = pthread_mutex_lock(&m_pinto);
    vert_len = tmp_vert_len;
    yama_len = tmp_yama_len;
    werder_len = tmp_werder_len;
    memcpy(verts, tmp_verts, 3 * vert_len * sizeof (GLfloat));
    memcpy(yama, tmp_yama, (yama_len + werder_len) * sizeof (T_V_EIGENSCHAFTEN));
    pthread_mutex_unlock(&m_pinto);

    PRINTF("\rvert_len: %i  yama_len: %i  werder_len: %i\n", vert_len, yama_len, werder_len);
    //free(tmp_verts);
    //free(tmp_yama);
}

void setPOS(ESContext *esContext) {
    POS_T *posData = esContext->posData;

    pow2Z = pow(2.0, *ZOOM);
    PRINTF("\rsetPOS lon/lat: %f %f\n", posData->gps_longitude, posData->gps_latitude);

    double g_x = (posData->gps_longitude + 180.0) / 360.0 * pow2Z;
    double g_y = (1.0 - log(tan(posData->gps_latitude * rad2deg) + 1.0 / cos(posData->gps_latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;
    posData->g_x = g_x;
    posData->g_y = g_y;
    PRINTF("\rsetPOS g_x: %f %f\n", g_x, g_y);

    posData->t_x = (int) floor(g_x);
    posData->t_y = (int) (floor(g_y));
    PRINTF("\rsetPOS t_x: %i %i\n", posData->t_x, posData->t_y);

    posData->mx = -SCALE / 2.0 + modf(g_x, &mf) * SCALE;
    posData->my = SCALE / 2.0 - modf(g_y, &mf) * SCALE;
    PRINTF("\rsetPOS m_x: %f %f\n", posData->mx, posData->my);

    posData->osmS = 40075017 * cos(posData->gps_latitude * M_PI / 180.0) / pow(2.0, *ZOOM + 8) * IMAGE_SIZE;
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
                if (*ZOOM > 8) {
                    ZOOM++;
                    PRINTF("ZOOM: %i\n", *ZOOM);
                    setPOS(esContext);
                    PRINTF("jolla: %i/%i %i/%i\n", posData->t_x, posData->gpu_t_x, posData->t_y, posData->gpu_t_y);
                }
            } else if (rawX > 760 && rawX < 800 && rawY > 430 && rawY < 480) {
                if (*ZOOM < 18) {
                    ZOOM--;
                    PRINTF("ZOOM: %i\n", *ZOOM);
                    setPOS(esContext);
                    PRINTF("jolla: %i/%i %i/%i\n", posData->t_x, posData->gpu_t_x, posData->t_y, posData->gpu_t_y);
                }
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
    struct gps_t gpsData;
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
            obdstatus = getobdvalue(obd_serial, cmdid, &tmp_val, numbytes, conv);
            if (OBD_SUCCESS == obdstatus) {
                posData->obd_speed = tmp_val;
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
            gps_read(&gpsd_init, &gpsData);

            if (gpsData.fix_type > 1 && gpsData.PDOP < 6) {
                gps_status[0] = 0.0f;
                gps_status[2] = 1.0f;

                g_x = (gpsData.longitude + 180.0) / 360.0 * pow2Z;
                g_y = (1.0 - log(tan(gpsData.latitude * rad2deg) + 1.0 / cos(gpsData.latitude * rad2deg)) / M_PI) / 2.0 * pow2Z;

                lk = pthread_mutex_lock(&m_pinto);
                posData->g_x = g_x;
                posData->g_y = g_y;
                pthread_mutex_unlock(&m_pinto);
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
                    posData->angle = deg;
                    pthread_mutex_unlock(&m_pinto);
                }
#endif
            } else {
                gps_status[0] = 1.0f;
                gps_status[2] = 0.0f;
            }
        } else {
            gps_open(&gpsd_init, "/dev/ttyAMA02");
        }
#endif

        if (fabs(posData->g_y - posData->gpu_t_y) > 1.0f || fabs(posData->g_x - posData->gpu_t_x) > 1.0f) {
            posData->gpu_t_x = (GLuint) floor(posData->g_x);
            posData->gpu_t_y = (GLuint) floor(posData->g_y);
            loadVert(posData->g_x, posData->g_y);
        }

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

void esMainLoop(ESContext *esContext) {
    POS_T *posData = esContext->posData;

    struct timeval t1, t2;
    struct timezone tz;
    float deltatime;
    float totaltime = 0.0f;
    unsigned int frames = 0;
    unsigned int us = 22000;

    gettimeofday(&t1, &tz);
    struct timespec spec1, spec2;
    unsigned int ms_min = 4294967295, ms_max = 0, ms, ms_avg = 0;

    esMatrixLoadIdentity(&modelview);
    esRotate(&modelview, posData->angle, 0.0, 0.0, 1.0);
    esTranslate(&modelview, 0.0, V_Z, 0.0);

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

    while (userInterrupt(esContext) == GL_FALSE) {
        //while (1) {
        gettimeofday(&t2, &tz);
        deltatime = (float) (t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6) * 1000;
        t1 = t2;
        clock_gettime(CLOCK_MONOTONIC, &spec1);
        //esContext->updateFunc(esContext, deltatime);

        Update2(esContext, deltatime);
        eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);
        clock_gettime(CLOCK_MONOTONIC, &spec2);
        totaltime += deltatime / 1000.0;
        frames++;

        ms = (spec2.tv_sec - spec1.tv_sec) * 1000 + (spec2.tv_nsec - spec1.tv_nsec) * 1e-6;

        ms_avg += ms;
        if (ms < ms_min)
            ms_min = ms;
        if (ms > ms_max)
            ms_max = ms;
        //PRINTF("%i\n", ms_max);
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
        usleep(us);
    }
}

static int sqc_vertex(void *NotUsed, int argc, char **argv, char **azColName) {
    tmp_verts[3 * tmp_vert_len] = atof(argv[0]);
    tmp_verts[3 * tmp_vert_len + 1] = atof(argv[1]);
    tmp_verts[3 * tmp_vert_len + 2] = 0.0f;

    tmp_vert_len++;

    return 0;
}

static int sqc_yama(void *NotUsed, int argc, char **argv, char **azColName) {
    tmp_yama[tmp_yama_len].len = atoi(argv[0]);
    tmp_yama[tmp_yama_len].r = atof(argv[1]);
    tmp_yama[tmp_yama_len].g = atof(argv[2]);
    tmp_yama[tmp_yama_len].b = atof(argv[3]);

    tmp_yama_len++;

    return 0;
}

static int sqc_count(void *out_var, int argc, char **argv, char **azColName) {
    *((int*) out_var) = atoi(argv[0]);

    return 0;
}

int main(int argc, char *argv[]) {
    ESContext esContext;
    UserData userData;
    POS_T posData;

    pthread_t thread_id2;
    pthread_mutex_init(&m_pinto, NULL);

    //signal(SIGINT, ShutDown);
    setvbuf(stdout, (char *) NULL, _IONBF, 0);

    userData.width = 800;
    userData.height = 480;

    userData.r_height = RSIZE;
    userData.r_width = RSIZE;

#ifdef __RASPI__
    homedir = "/home/pi";
#define RES ""
#else
    homedir = getenv("HOME");
#define RES "/workspace"
#endif
    char filename[50];
    strncpy(filename, homedir, 50);
    sout = fopen(strcat(filename, "/sit2d_debug.txt"), "w");
    strcpy(filename, homedir);
    gps_out = fopen(strcat(filename, "/sit2d.gps"), "w");

#ifdef __FIN__
    strcpy(filename, homedir);
    gps_in = fopen(strcat(filename, "/asd.gps"), "r");
    assert(gps_in != NULL);
    PRINTF("%s geladen.\n", filename);
#endif

    ZOOM = &ZOOM_S[0];
#ifdef __565__
    userData.element4Pixel = 1;
#else
    userData.element4Pixel = 3;
#endif
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
    //esContext.gpsData = &gpsData;

    esCreateWindow(&esContext, "SiT2D", userData.width, userData.height,
            ES_WINDOW_RGB /*| ES_WINDOW_MULTISAMPLE*/);

    rad2deg = M_PI / 180.0f;

    posData.gps_latitude = 52.0611;
    posData.gps_longitude = 11.7229;
    posData.gps_altitude = 0.0;
    posData.obd_speed = 0.0;
    posData.angle = 0.0f;
    posData.v = 0.0;

    setPOS(&esContext);

    posData.gpu_t_x = (GLuint) floor(posData.g_x);
    posData.gpu_t_y = (GLuint) floor(posData.g_y);

    strncpy(filename, homedir, 50);
    char *db_file = strncat(filename, "/sit2d_2.db", 50);
    assert(sqlite3_open_v2(db_file, &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, NULL) == SQLITE_OK);
    loadVert(posData.g_x, posData.g_y);

    esMatrixLoadIdentity(&perspective);
    //esPerspective( &perspective, 60.0f, (GLfloat) esContext->width / (GLfloat) esContext->height, 1.0f, 100.0f );
    esOrtho(&perspective, -V_Y, V_Y, -V_Y, V_Y, -50, 50);

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
    PRINTF("\r%s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

    esRegisterUpdateFunc(&esContext, Update2);
#ifndef __RASPI__
    esRegisterKeyFunc(&esContext, Key);
#endif
    PRINTF("\rinit abgeschlossen\n");

    //pthread_create(&thread_id1, NULL, &paris, (void*) &esContext);
    //pthread_create(&thread_id2, NULL, &pinto, (void*) &esContext);
    //pthread_create(&thread_id3, NULL, &jolla, (void*) &esContext);

    intro(&esContext);
    glClearColor(1.0f, 1.0f, 0.0f, 1.0f);
    esMainLoop(&esContext);

    //pthread_join(thread_id1, NULL);
    pthread_join(thread_id2, NULL);
    //pthread_join(thread_id3, NULL);

    ShutDown(&esContext);

    return (EXIT_SUCCESS);
}