/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "fatfs_sd.h"
#include "printf.h"
#include "player.h"
#include "matrix.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if 1	//PWM on OE
#define DIS_OE_ON(_val)	(__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, _val))
#define DIS_OE_OFF()	(__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0))

#else
#define DIS_OE_ON(_val)	(HAL_GPIO_WritePin(DIS_OE_GPIO_Port, DIS_OE_Pin, GPIO_PIN_RESET))
#define DIS_OE_OFF()	(HAL_GPIO_WritePin(DIS_OE_GPIO_Port, DIS_OE_Pin, GPIO_PIN_SET))
#endif

#define DIS_BRIGHTNESS(_val)	(__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, _val))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
SSD1306_COLOR lcdColor = White;
uint8_t timerFlag = 1;
FATFS FatFs;
static volatile uint8_t playerGo = 0;

uint8_t spiTxBuff[8][16][48] = {0};//R1|R2|G1|G2|B1|B2

const uint8_t dispScaleVal[8] = {128, 64, 32, 16, 8, 4, 2, 1};
//const uint8_t dispScaleVal[8] = {64, 32, 16, 8, 4, 2, 1, 1};
volatile uint8_t dispScalePos = 0;
volatile uint8_t dispRow = 0;
volatile uint8_t dispBrightness = 10;

volatile uint32_t rndValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void SetRowPin(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void _putchar(char character)
{
  // send char to console etc.
  uint8_t s[2] = { 0 };
  s[0] = character;
  HAL_UART_Transmit(&hlpuart1, s, 1, 1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t count = 0;
  char tmpBuf[64] = {0};
  RTC_TimeTypeDef timeOfRtc = {0};
  RTC_DateTypeDef dayOfRtc = {0};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_RTC_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_RNG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  HAL_Delay(50);
  ssd1306_Fill(Black);

  ssd1306_SetCursor(8,8);
  ssd1306_WriteString("Init...",Font_11x18,White);
  ssd1306_UpdateScreen();
  HAL_Delay(200);

  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	FRESULT rc = f_mount(&FatFs, "", 0);
	if (rc != FR_OK)
	{
		return rc;
	}
	PlayerInit(&htim6, &hdac1);

	//PlayerStart("Afire.MP3");
	//PlayerStart("Take my breath away.mp3");
	//PlayerStart("tmba.MP3");
	//PlayerStart("lmoyzly.mp3");

	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
	//MatrixDispTypeDef hDisp;
	//hDisp.pOETimer = &htim3;
	//hDisp.channelOE = TIM_CHANNEL_2;
	//MatrixInit(&hDisp);
	
	//HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	//CS
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	//OE

	//DIS_BRIGHTNESS(dispBrightness);

	//matrix refresh timer
	HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim16);

	uint32_t start = HAL_GetTick();
	//MatrixDrawBMP("CHROME.bmp", 0, 0);
	//MatrixDrawBMP("CHROME.bmp", 32, 0);
	MatrixDrawBMP("HUB.bmp", 0, 0);
	printf("Draw bmp in TF card: %lu ms.\r\n", HAL_GetTick() - start);
	HAL_Delay(2000);
	MatrixClear();
	HAL_Delay(1000);
	start = HAL_GetTick();
	MatrixDrawImage();
	printf("Draw internal image cost: %lu ms.\r\n", HAL_GetTick() - start);
	HAL_Delay(2000);
	MatrixClear();
	//printf("Clear cost: %lu ms.\r\n", HAL_GetTick() - start);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    if (timerFlag != 0)
    {
		timerFlag = 0;
		count++;
		if (count % 60 == 0)
		{
			if (lcdColor == Black)
			{
				lcdColor = White;
				ssd1306_Fill(Black);
			}
			else
			{
				lcdColor = Black;
				ssd1306_Fill(White);
			}
		}
		HAL_RTC_GetTime(&hrtc, &timeOfRtc, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &dayOfRtc, RTC_FORMAT_BIN);

		ssd1306_SetCursor(0,8);
		sprintf(tmpBuf, "%02d:%02d:%02d", timeOfRtc.Hours, timeOfRtc.Minutes, timeOfRtc.Seconds);
		ssd1306_WriteString(tmpBuf,Font_11x18,lcdColor);
		ssd1306_UpdateScreen();

		RGB_t dot0 = {timeOfRtc.Seconds,100,timeOfRtc.Minutes};

		MatrixSetDirection(timeOfRtc.Seconds > 30 ? 1 : 0);
		MatrixClear();
		MatrixWriteString(tmpBuf, Font_7x10, 4, 10, dot0);
	}
	if (playerGo == 2)
	{
		playerGo = 0;
		PlayerStart("tmba.mp3");
	}
	PlayerUpdate();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = ENABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 921600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if (HAL_RTCEx_BKUPRead(&hrtc, 0) == 0xfc)
  {
    goto exit;
  }

  HAL_RTCEx_BKUPWrite(&hrtc, 0, 0xfc);

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x14;
  sTime.Minutes = 0x21;
  sTime.Seconds = 0x12;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x15;
  sDate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  exit:;
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIS_CS_Pin|DIS_B_Pin|DIS_C_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIS_LAT_GPIO_Port, DIS_LAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIS_A_Pin|DIS_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIS_CS_Pin DIS_B_Pin DIS_C_Pin */
  GPIO_InitStruct.Pin = DIS_CS_Pin|DIS_B_Pin|DIS_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIS_LAT_Pin */
  GPIO_InitStruct.Pin = DIS_LAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIS_LAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIS_A_Pin DIS_D_Pin */
  GPIO_InitStruct.Pin = DIS_A_Pin|DIS_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	//plState = PLAYER_STATE_DMA_EMPTY;
	flg_dma_done = 1;
}
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
		PlayerVolumeAdj(-1);
		playerGo = 2;
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  timerFlag = 1;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance)
	{
#ifdef SSD1306_USING_DMA
		ssd1306_IntCallBack();
#endif
	}
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hspi3.Instance)
	{
		//HAL_GPIO_WritePin(DIS_OE_GPIO_Port, DIS_OE_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(DIS_LAT_GPIO_Port, DIS_LAT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hspi3.Instance)
	{
		//HAL_GPIO_WritePin(DIS_OE_GPIO_Port, DIS_OE_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
		SetRowPin();

		HAL_GPIO_WritePin(DIS_LAT_GPIO_Port, DIS_LAT_Pin, GPIO_PIN_SET);

		if (dispRow < 15)
		{
			dispRow++;
		}
		else
		{
			dispRow = 0;
			if (dispScalePos < 7)
			{
				dispScalePos++;
			}
			else
			{
				dispScalePos = 0;
			}
		}
		//DIS_OE_ON(dispScaleVal[dispScalePos]);
		HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if (htim->Instance == htim16.Instance)
	if (htim->Instance == htim3.Instance)
	{
		//HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
		//DIS_OE_OFF();
		DIS_OE_ON(dispScaleVal[dispScalePos]);
		HAL_GPIO_WritePin(DIS_LAT_GPIO_Port, DIS_LAT_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)spiTxBuff[dispScalePos][dispRow], sizeof(spiTxBuff[0][0])/sizeof(spiTxBuff[0][0][0]));
	}
}

void SetRowPin(void)
{
	switch (dispRow)
	{
	case 15:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 14:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 13:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 12:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 11:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 10:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 9:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 8:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_RESET);
		break;
	case 7:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	case 0:
		HAL_GPIO_WritePin(DIS_A_GPIO_Port, DIS_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_B_GPIO_Port, DIS_B_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_C_GPIO_Port, DIS_C_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIS_D_GPIO_Port, DIS_D_Pin, GPIO_PIN_SET);
		break;
	
	default:
		break;
	}
}


void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit)
{
	rndValue = random32bit;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
