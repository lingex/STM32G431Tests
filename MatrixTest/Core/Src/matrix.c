#include "matrix.h"
#include "image_bmp.h"
#include "gamma.h"
#include "ff.h"
#include "printf.h"

static int curX = 0;
static int curY = 0;
static uint8_t disp_direction = 0;

extern FATFS FatFs;

extern uint8_t spiTxBuff[8][16][48];

//const uint8_t scaleTap[] = {1,2,4,8,32,64,128};

#if USE_BITBAND
const uint8_t shiftTab[] = 
{
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
	7,6,5,4,3,2,1,0,
};
#else
const uint8_t shiftTab[] = 
{
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1,
	128,64,32,16,8,4,2,1
};
#endif

#if CHAIN_TYPE
const uint8_t colorffsetTab[][3] = 
{
	{0,  16,  32},
	{8,  24,  40}
};
#else
const uint8_t colorffsetTab[][3] = 
{
	{0,  8,  16},
	{24, 32, 40}
};
#endif



char MatrixWriteChar(char ch, FontDef Font, RGB_t dotColor)
{
	uint32_t i, b, j;
	RGB_t offDot = {0,0,0};

	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				MatrixWritePixel(curX + j, (curY + i), dotColor);
			}
			else
			{
				MatrixWritePixel(curX + j, (curY + i), offDot);
			}
		}
	}

	// The current space is now taken
	curX += Font.FontWidth;

	// Return written char for validation
	return ch;
}

char MatrixWriteString(char* str, FontDef Font, int x, int y, RGB_t dotColor)
{
	curX = x;
	curY = y;
	// Write until null-byte
	while (*str)
	{
		if (MatrixWriteChar(*str, Font, dotColor) != *str)
		{
			// Char could not be written
			return *str;
		}

		// Next char
		str++;
	}

	// Everything ok
	return *str;
}

void MatrixWritePixel(int x, int y, RGB_t dot)
{
#if USE_GAMMA
	MatrixWritePixel2(x, y, gammaR[dot.R], gammaG[dot.G], gammaB[dot.B]);
#else
	MatrixWritePixel2(x, y, dot.R, dot.G, dot.B);
#endif

}

void MatrixWritePixel2(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	// if (x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT)
	// {
	// 	return;
	// }

	if (disp_direction == 0)
	{
		x = MATRIX_WIDTH - 1 - x;
	}
	else
	{
		y = MATRIX_HEIGHT - 1 - y;
	}

	uint8_t* pBufB = NULL;
	uint8_t* pBufG = NULL;
	uint8_t* pBufR = NULL;
	uint8_t lineOffset = x>>3;	//div 8
	int colorOffset = 0;

	if (y >= MATRIX_HEIGHT/2)
	{
		y -= 16;
		colorOffset = 1;
	}

#if USE_BITBAND
	uint8_t shiftVal = shiftTab[x];
#else
	uint8_t shiftVal = shiftTab[x];
#endif

	for (int scale = 0; scale < 8; scale++)
	{
		pBufR = &(spiTxBuff[scale][y][lineOffset + colorffsetTab[colorOffset][2]]);
		pBufG = &(spiTxBuff[scale][y][lineOffset + colorffsetTab[colorOffset][1]]);
		pBufB = &(spiTxBuff[scale][y][lineOffset + colorffsetTab[colorOffset][0]]);

#if USE_BITBAND
		BitBand(pBufB, shiftVal) = b >>(7-scale);
		BitBand(pBufG, shiftVal) = g >>(7-scale);
		BitBand(pBufR, shiftVal) = r >>(7-scale);
#else
		//uint8_t scaleVal = scaleTap[scale];	//may slightly slow
		uint8_t scaleVal = 0x80 >> scale;
		if ((b & scaleVal) != 0)
		{
			*pBufB |= shiftVal;
		}
		else
		{
			*pBufB &= ~shiftVal;
		}
		if ((g & scaleVal) != 0)
		{
			*pBufG |= shiftVal;
		}
		else
		{
			*pBufG &= ~shiftVal;
		}
		if ((r & scaleVal) != 0)
		{
			*pBufR |= shiftVal;
		}
		else
		{
			*pBufR &= ~shiftVal;
		}
#endif
	}
}

static inline void MatrixWritePixel2a(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	MatrixWritePixel2(x, MATRIX_HEIGHT - 1 - y, r, g, b);
}
static inline void MatrixWritePixel2b(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
	MatrixWritePixel2(MATRIX_WIDTH - 1 - x, y, r, g, b);
}

void MatrixDrawImage(void)
{
	int x = 0;
	int y = 0;
	for (size_t i = 0; i < sizeof(gImage_image_bmp); i+=3)
	{
		MatrixWritePixel2(x, y, gImage_image_bmp[i+0], gImage_image_bmp[i+1],gImage_image_bmp[i+2]);
		if (x < MATRIX_WIDTH-1)
		{
			x++;
		}
		else
		{
			x=0;
			if (y < MATRIX_HEIGHT-1)
			{
				y++;
			}
			else
			{
				y=0;
			}
		}
	}
}

void MatrixClear(void)
{
	memset(spiTxBuff, 0, sizeof(spiTxBuff));
}

void MatrixDrawBMP(char* fileName, int x, int y)
{
	int xOffset = x;
	int rc = 0;
	FIL fil;
	uint8_t buff[READ_BUFF_SIZE];
	UINT bRead = 0;

#ifdef DEBUG
	uint32_t start = HAL_GetTick();
#endif
	rc = f_open(&fil, fileName, FA_READ);

#ifdef DEBUG
	uint32_t endT = HAL_GetTick();
	printf("file open cost :%lu ms, tick:%lu.\r\n", endT - start, endT);
#endif
	if (rc == FR_OK && f_read(&fil, buff, READ_BUFF_SIZE, &bRead) == FR_OK)
	{
		BMPHEAD_t *pHeader = (BMPHEAD_t *)buff;
#ifdef DEBUG
		endT = HAL_GetTick();
		printf("first buff load cost :%lu ms, buff size: %d, tick:%lu.\r\n", endT - start, READ_BUFF_SIZE, endT);
		printf("file size: %u, image size: %u, %dx%d .\r\n", pHeader->bfSize, pHeader->biSizeImage, pHeader->biWidth, pHeader->biHeight);
#endif
		if (pHeader->bfSize <= 0 || pHeader->bfType != 19778 || pHeader->biBitCount != 24)
		{
			printf("Not supported file: %s, 24bit-BMP only", fileName);
			return;
		}
		const int maxWith = MIN(pHeader->biWidth, MATRIX_WIDTH) - 1;

		//image direction
		const FuncPtr WritePixel = pHeader->biHeight > 0 ? MatrixWritePixel2a : MatrixWritePixel2b;

		//first block
		int count = MIN(READ_BUFF_SIZE, pHeader->biSizeImage);
		for (size_t i = pHeader->bfOffBits; i < count; i += 3)
		{
			WritePixel(x + xOffset, y, buff[i+2], buff[i+1], buff[i+0]);
			if (x < maxWith)
			{
				x++;
			}
			else
			{
				x=0;
				if (y < MATRIX_HEIGHT-1)
				{
					y++;
				}
				else
				{
					break;//out of range
				}
			}
		}
		//left block
		int32_t leftSize = READ_BUFF_SIZE <= pHeader->biSizeImage ? (pHeader->bfSize - READ_BUFF_SIZE) : 0;
		while (leftSize > 0)
		{
			if (f_read(&fil, buff, READ_BUFF_SIZE, &bRead) == FR_OK)
			{
				for (size_t i = 0; i < bRead; i += 3)
				{
					WritePixel(x + xOffset, y, buff[i+2], buff[i+1], buff[i+0]);

					if (x < maxWith)
					{
						x++;
					}
					else
					{
						x=0;
						if (y < MATRIX_HEIGHT-1)
						{
							y++;
						}
						else
						{
							break;//out of range
						}
					}
				}
			}
			leftSize -= bRead;
		}
	}
	f_close(&fil);
}

void MatrixSetDirection(uint8_t dir)
{
	disp_direction = dir;
}