#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "main.h"
#include "fonts.h"

#define READ_BUFF_SIZE 768

#define MATRIX_WIDTH  64
#define MATRIX_HEIGHT 32

typedef struct RGB {
	uint8_t R;
	uint8_t G;
	uint8_t B;
}RGB_t;

#pragma pack(2)
typedef struct{
	short bfType; //BMP='B''M'(19778)
	uint32_t bfSize; //size of file 
	short bfReserved1;
	short bfReserved2;
	int bfOffBits;  //size of header  offet of the image data
	
	int biSize; // size of info header 
	int biWidth; 
	int biHeight;
	short biPlanes; //always 1 
	short biBitCount; //bit per pexil:1/2/4/8/16/24/32 
	int biCompression;  //compress or not 
	int biSizeImage;  //size of image
	int biXPelsPerMeter; 
	int biYPelsPerMeter; 
	int biClrUsed;  //color used 
	int biClrImportant;
}BMPHEAD_t;

#pragma pack ()

void MatrixClear(void);
void MatrixWritePixel(int x, int y, RGB_t dot);
void MatrixWritePixel2(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void MatrixDrawImage(void);
void MatrixDrawBMP(char* fileName, int x, int y);
void MatrixSetDirection(uint8_t dir);

char MatrixWriteChar(char ch, FontDef Font, RGB_t dotColor);
char MatrixWriteString(char* str, FontDef Font, int x, int y, RGB_t dotColor);

#endif