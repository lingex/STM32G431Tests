
#include "main.h"
#include "fonts.h"

/**
 * This Library is written and optimized by Olivier Van den Eede(4ilo) in 2016
 * for Stm32 Uc and HAL-i2c lib's.
 *
 * To use this library with ssd1306 oled display you will need to customize the defines below.
 *
 * This library uses 2 extra files (fonts.c/h).
 * In this files are 3 different fonts you can use:
 * 		- Font_7x10
 * 		- Font_11x18
 * 		- Font_16x26
 *
 */

#ifndef ssd1306
#define ssd1306

// I2c port as defined in main generated by CubeMx
#define SSD1306_I2C_PORT		hi2c1
// I2c address
#define SSD1306_I2C_ADDR        0x78
// SSD1306 width in pixels
#define SSD1306_WIDTH           128
// SSD1306 LCD height in pixels
#define SSD1306_HEIGHT          32

//important: i2c event interrupt must be enable when using DMA
#define USING_DMA 1
#define SSD1306_TASK_SIZE 64

//
//  Enumeration for screen colors
//
typedef enum {
	Black = 0x00, // Black color, no pixel
	White = 0x01  //Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//
//  Struct to store transformations
//
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

typedef struct {
	uint8_t TaskType;	//0=cmd, 1=data
	uint8_t TaskPara;
} SSD1306_TASK;

//	Definition of the i2c port in main
extern I2C_HandleTypeDef SSD1306_I2C_PORT;

//
//  Function definitions
//
uint8_t ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);

void ssd1306_TaskGo(void);

void ssd1306_Power(int sw);

#endif
