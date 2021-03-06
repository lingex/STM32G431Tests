
#include"ssd1306.h"


// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Screen object
static SSD1306_t SSD1306;

#if SSD1306_USING_DMA
static SSD1306_TASK TaskQueue[SSD1306_TASK_SIZE];
static volatile uint8_t taskPushIndex = 0;
static volatile uint8_t taskSendIndex = 0;
#endif

//
//  Send a byte to the command register
//
static void ssd1306_WriteCommand(uint8_t command)
{
#if SSD1306_USING_DMA
	if (taskPushIndex >= SSD1306_TASK_SIZE)
	{
		//abort should not be here
		return;
	}
	TaskQueue[taskPushIndex].TaskType = 1;
	TaskQueue[taskPushIndex].TaskPara = command;
	taskPushIndex++;
#else
	HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR,0x00,1,&command,1,10);
#endif
}

//
//  Write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(void)
{
#if SSD1306_USING_DMA
	while(HAL_I2C_GetState(&SSD1306_I2C_PORT) != HAL_I2C_STATE_READY)
	{

	}
	for (uint8_t i = 0; i < 8; i++)
	{
		if (taskPushIndex + 4 >= SSD1306_TASK_SIZE)
		{
			//abort should not be here
			return;
		}
		//cmd
		TaskQueue[taskPushIndex].TaskType = 1;
		TaskQueue[taskPushIndex].TaskPara = 0xB0 + i;
		taskPushIndex++;
		TaskQueue[taskPushIndex].TaskType = 1;
		TaskQueue[taskPushIndex].TaskPara = 0xB0 + i;
		taskPushIndex++;
		TaskQueue[taskPushIndex].TaskType = 1;
		TaskQueue[taskPushIndex].TaskPara = 0xB0 + i;
		taskPushIndex++;
		//data
		TaskQueue[taskPushIndex].TaskType = 2;
		TaskQueue[taskPushIndex].TaskPara = i;
		taskPushIndex++;
	}
	ssd1306_TaskGo();

#else
	for (uint8_t i = 0; i < 8; i++)
	{
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);

		HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
#endif
}

//
//	Initialize the oled screen
//
uint8_t ssd1306_Init(void)
{
	// Wait for the screen to boot
	HAL_Delay(50);

	/* Init LCD */
	//12864
	// ssd1306_WriteCommand(0xAE); //display off
	// ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
	// ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	// ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	// ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
	// ssd1306_WriteCommand(0x00); //---set low column address
	// ssd1306_WriteCommand(0x10); //---set high column address
	// ssd1306_WriteCommand(0x40); //--set start line address
	// ssd1306_WriteCommand(0x81); //--set contrast control register
	// ssd1306_WriteCommand(0xFF);
	// ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
	// ssd1306_WriteCommand(0xA6); //--set normal display
	// ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
	//ssd1306_WriteCommand(0x3F); //
	// ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	// ssd1306_WriteCommand(0xD3); //-set display offset
	// ssd1306_WriteCommand(0x00); //-not offset
	// ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	// ssd1306_WriteCommand(0xF0); //--set divide ratio
	// ssd1306_WriteCommand(0xD9); //--set pre-charge period
	// ssd1306_WriteCommand(0x22); //
	// ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
	// ssd1306_WriteCommand(0x12);
	// ssd1306_WriteCommand(0xDB); //--set vcomh
	// ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
	// ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	// ssd1306_WriteCommand(0x14); //
	// ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel

	//12832
	ssd1306_WriteCommand(0x40);
	ssd1306_WriteCommand(0xB0);
	ssd1306_WriteCommand(0xc8);
	ssd1306_WriteCommand(0x81);
	ssd1306_WriteCommand(0xff);
	ssd1306_WriteCommand(0xa1);
	ssd1306_WriteCommand(0xa6);
	ssd1306_WriteCommand(0xa8);
	ssd1306_WriteCommand(0x1f);
	ssd1306_WriteCommand(0xd3);
	ssd1306_WriteCommand(0x00);
	ssd1306_WriteCommand(0xd5);
	ssd1306_WriteCommand(0xf0);
	ssd1306_WriteCommand(0xd9);
	ssd1306_WriteCommand(0x22);
	ssd1306_WriteCommand(0xda);
	ssd1306_WriteCommand(0x02);
	ssd1306_WriteCommand(0xdb);
	ssd1306_WriteCommand(0x49);
	ssd1306_WriteCommand(0x8d);
	ssd1306_WriteCommand(0x14);
	ssd1306_WriteCommand(0xaf);

	// Clear screen
	ssd1306_Fill(Black);

	// Flush buffer to screen
	ssd1306_UpdateScreen();

	// Set default values for screen object
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;

	SSD1306.Initialized = 1;

	return 1;
}

//
//  Fill the whole screen with the given color
//
void ssd1306_Fill(SSD1306_COLOR color)
{
	/* Set memory */
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}

//
//	Draw one pixel in the screenbuffer
//	X => X Coordinate
//	Y => Y Coordinate
//	color => Pixel color
//
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
	{
		// Don't write outside the buffer
		return;
	}

	// Check if pixel should be inverted
	if (SSD1306.Inverted)
	{
		color = (SSD1306_COLOR)!color;
	}

	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	}
	else
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

//
//  Draw 1 char to the screen buffer
//	ch 		=> char om weg te schrijven
//	Font 	=> Font waarmee we gaan schrijven
//	color 	=> Black or White
//
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;

	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}

	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			}
			else
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}

	// The current space is now taken
	SSD1306.CurrentX += Font.FontWidth;

	// Return written char for validation
	return ch;
}

//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	// Write until null-byte
	while (*str)
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
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

//
//	Position the cursor
//
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void ssd1306_Power(int sw)
{
	if (sw == 0)
	{
		ssd1306_WriteCommand(0xAE); //display off
	}
	else
	{
		ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel
	}
}

#if SSD1306_USING_DMA
void ssd1306_TaskGo(void)
{
	if (taskSendIndex < taskPushIndex && taskSendIndex < SSD1306_TASK_SIZE)
	{
		SSD1306_TASK* pTask = &(TaskQueue[taskSendIndex]);

		if (pTask->TaskType == 1)
		{
			HAL_I2C_Mem_Write_DMA(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR,0x00,1,&(pTask->TaskPara),1);
		}
		else if (pTask->TaskType == 2)
		{
			HAL_I2C_Mem_Write_DMA(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, &(SSD1306_Buffer[SSD1306_WIDTH * pTask->TaskPara]), SSD1306_WIDTH);
		}
	}
	else
	{
		//all done
		taskSendIndex = 0;
		taskPushIndex = 0;
	}
}
#endif

void ssd1306_IntCallBack(void)
{
	taskSendIndex++;
	ssd1306_TaskGo();
}
