#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
//#include <png.h>
#include <assert.h>
#include <sys/stat.h>
#include "util.h"

#define EXIST F_OK

unsigned short red_mask = 0xF800;
unsigned short green_mask = 0x7E0;
unsigned short blue_mask = 0x1F;

/*static size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream) {
	size_t written = fwrite(ptr, size, nmemb, (FILE *) stream);
	return written;
}*/
/*
void abort_(const char * s, ...) {
	va_list args;
	va_start(args, s);
	vfprintf(stderr, s, args);
	fprintf(stderr, "\n");
	va_end(args);
	abort();
}*/

void printBin(unsigned char *CH) {
	unsigned char Mask = 0x01;
	unsigned char P2_B2[8];
	int n = 7;
	do {
		P2_B2[n--] = *CH & Mask ? 1 : 0;
	} while (Mask <<= 1);
	for (n = 0; n < 8; n++)
		printf("%i", P2_B2[n]);
	printf("\n");
}
void printBin2(unsigned short *CH) {
	unsigned short Mask = 1;
	unsigned char P2_B2[16];
	int n = 15;
	do {
		P2_B2[n--] = *CH & Mask ? 1 : 0;
	} while (Mask <<= 1);
	for (n = 0; n < 16; n++)
		printf("%i", P2_B2[n]);
	printf("\n");
}
/*
 int curl(int zoom, int x, int y) {
 CURL *curl_handle;
 FILE *pagefile;
 char url[100], dir_zoom[2], dir_x[50], filename[50];
 //char *filename_tmp = "tmp";

 sprintf(dir_zoom, "map/%i", zoom);
 sprintf(dir_x, "map/%i/%i", zoom, x);
 sprintf(filename, "map/%i/%i/%i.png", zoom, x, y);
 //sprintf(url, "http://bardeen:8000/osm_tiles/%i/%i/%i.png", zoom, x, y);
 //sprintf(url, "https://a.tile.thunderforest.com/landscape/%i/%i/%i.png", zoom, x, y);
 sprintf(url, "http://c.tiles.wmflabs.org/osm-no-labels/%i/%i/%i.png", zoom, x, y);
 printf("%s\n", url);
 if (access(dir_zoom, EXIST) == -1) {
 mkdir(dir_zoom, 0771);
 }
 if (access(dir_x, EXIST) == -1) {
 mkdir(dir_x, 0771);
 }
 curl_global_init(CURL_GLOBAL_ALL);
 curl_handle = curl_easy_init();
 curl_easy_setopt(curl_handle, CURLOPT_URL, url);
 curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 1L);
 curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, write_data);

 pagefile = fopen(filename, "wb");
 if (pagefile) {
 fseek(pagefile, 0, SEEK_SET);
 curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, pagefile);
 curl_easy_perform(curl_handle);
 fseek(pagefile, 0, SEEK_END); // seek to end of file
 if (ftell(pagefile) == 0) {
 fclose(pagefile);
 printf("%i\n", remove(filename));
 return 0;
 }
 fclose(pagefile);
 }
 curl_easy_cleanup(curl_handle);

 return 1;
 }
 */
void load_TGA(unsigned short * buffer, char *fileName) {
	FILE *f;
	unsigned char row[8], bbp;
	unsigned short width, height;
	TEX_BUFFER_FORMAT r, g, b, a, rgba;

	f = fopen(fileName, "rb");
	if (f == NULL) {
		printf("load_TGA: Fehler fopen: %s\n", fileName);
                return;
	}
	// HEADER
	fread(row, 8, 1, f);
	fread(row, 4, 1, f);
	fread(&width, 2, 1, f);
	fread(&height, 2, 1, f);
	fread(&bbp, 1, 1, f);
	fread(row, 1, 1, f);
	//printf("%ix%i %i\n", width, height, bbp);

	//BGR_A
	for (int m = height - 1; m > -1; m--) {
		for (int n = 0; n < width; n++) {
			fread(row, 4, 1, f);
			r = row[2] << 8;
			g = row[1] << 3;
			b = row[0] >> 2;
			a = row[3] >> 7;

			rgba = (r & red_mask) | (g & 1984) | (b & 62) | (a & 1);
			//printf("%i %i\n", m, n);
			*(buffer + n + m * width) = rgba;
		}
	}
	fclose(f);
}
/*
int load_png(FILE *fp, TEX_BUFFER_FORMAT *dst, unsigned int stride, unsigned short image_size) {
	unsigned int sig_read = 0, i;
	png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	NULL, NULL);
	assert(png_ptr != NULL);
	png_infop info_ptr = png_create_info_struct(png_ptr);

	//printf("png1\n");
	if (setjmp(png_jmpbuf(png_ptr))) {
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return -1;
	}
	//printf("png2\n");

	png_init_io(png_ptr, fp);
	png_set_sig_bytes(png_ptr, sig_read);

	//png_read_info(png_ptr, info_ptr);
	\*
	 int width = png_get_image_width(png_ptr, info_ptr);
	 int height = png_get_image_height(png_ptr, info_ptr);
	 int color_type = png_get_color_type(png_ptr, info_ptr);
	 int bit_depth = png_get_bit_depth(png_ptr, info_ptr);

	 int number_of_passes = png_set_interlace_handling(png_ptr);
	 *\
	png_read_update_info(png_ptr, info_ptr);
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[read_png_file] Error during read_image");

	png_read_png(png_ptr, info_ptr,
	PNG_TRANSFORM_STRIP_16 | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_EXPAND,
	NULL);
	//printf("png3\n");
	png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);
	//png_read_image(png_ptr, row_pointers);

	TEX_BUFFER_FORMAT *dst_z = dst;
	TEX_BUFFER_FORMAT r, g, b, rgb;
	unsigned short n;


	for (i = 0; i < image_size; i++) {
		for (n = 0; n < image_size; n++) {
			r = *(row_pointers[i] + 3 * n) << 8;
			g = *(row_pointers[i] + 3 * n + 1) << 3;
			b = *(row_pointers[i] + 3 * n + 2) >> 3;
			rgb = (r & red_mask) | (g & green_mask) | (b & blue_mask);
			*dst_z++ = rgb;
		}
	}
	png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

	return 1;
}
*/