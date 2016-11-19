#ifndef UTIL_H
#define UTIL_H

#define IMAGE_SIZE 256

#define SCALE 1
//#define ORTHO 1.5f
#define V_Z 1.2f
#define P 0.03f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __565__
#define GL_TEX_FORMAT GL_UNSIGNED_SHORT_5_6_5
#define TEX_BUFFER_FORMAT unsigned short
#else
#define GL_TEX_FORMAT GL_UNSIGNED_BYTE
#define TEX_BUFFER_FORMAT char
#endif

int load_png(FILE *fp, TEX_BUFFER_FORMAT *dst, unsigned int, unsigned short image_size);
void load_TGA(unsigned short * buffer, char *fileName);
int long2tilex(double lon, int z);
int lat2tiley(double lat, int z);
double tilex2long(int x, int z);
double tiley2lat(int y, int z);
double long2offsetX(double lon, int z, int scale);
float lat2offsetY(double lat, int z, int scale);
void printBin(unsigned char *CH);
void write_png_file(char* file_name, int width, int height, char *row_pointers);
void writeBMP(unsigned char *dst, unsigned int width, unsigned height);

#endif /* UTIL_H */

