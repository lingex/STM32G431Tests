#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "main.h"
#include "fonts.h"

#define READ_BUFF_SIZE 768

#define MATRIX_WIDTH  64
#define MATRIX_HEIGHT 32

#define MATRIX_REFRESH_RATE 120	//Hz
#define MATRIX_SCAN_ROW 16

#define USE_BITBAND 1
#define USE_GAMMA 0

/*
//CHAIN_TYPE=0: spiTxBuff:[scale:0~7][line:0~15(16~31)][colorbit(B1|G1|R1|B2|G2|R2)]
MCU->IN R1
OUT R1 -> IN G1
OUT G1 -> IN B1
OUT B1 -> IN R2
OUT R2 -> IN G2
OUT G2 -> IN B2

//CHAIN_TYPE=1: spiTxBuff:[scale:0~7][line:0~15(16~31)][colorbit(B1|B2|G1|G2|R1|R2)]
MCU->IN R1
OUT R1 -> IN R2
OUT R2 -> IN G1
OUT G1 -> IN G2
OUT G2 -> IN B1
OUT B1 -> IN B2
*/
#define CHAIN_TYPE 0

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


typedef void (*FuncPtr)(int x, int y, uint8_t r, uint8_t g, uint8_t b);


void MatrixClear(void);
void MatrixWritePixel(int x, int y, RGB_t dot);
void MatrixWritePixel2(int x, int y, uint8_t r, uint8_t g, uint8_t b);
void MatrixDrawImage(void);
void MatrixDrawBMP(char* fileName, int x, int y);
void MatrixSetDirection(uint8_t dir);

char MatrixWriteChar(char ch, FontDef Font, RGB_t dotColor);
char MatrixWriteString(char* str, FontDef Font, int x, int y, RGB_t dotColor);

#endif