#include <string.h>
#include "player.h"
#include "ff.h"
#include "spiritMP3Dec.h"
#include "printf.h"

extern FATFS FatFs;

FIL playingFile;
//uint8_t __attribute__((section(".dtcmram"))) array[1024] __attribute__ (aligned(64)));
//uint8_t __attribute__((section(".sram2"))) array[1024] __attribute__ (aligned(64)));
//uint16_t audioBuffer[BUFFER_SIZE];
uint16_t __attribute__((section(".sram2"))) audioBuffer[BUFFER_SIZE];
//uint16_t audioBuffer[BUFFER_SIZE]__attribute__((section(".ARM.__at_0x20004000")));
uint16_t volume = 80;	//default: 80

TIM_HandleTypeDef *pSampleTimer = NULL;
DAC_HandleTypeDef* pDac = NULL;


#ifdef DEBUG
extern UART_HandleTypeDef hlpuart1;
char tmpBuf[32] = {0};
#endif


static volatile PlayerState_t playerState = PLAYER_STATE_IDLE;
TSpiritMP3Decoder decoder;

static void PlayerSetSampleRate(uint32_t audioFreq);
static void PlayStream(uint16_t* pBuffer, uint32_t Size);
static unsigned int ReadData(void* data, unsigned int size, void* token);
static void VolumeScale(uint16_t* pBuffer, unsigned int samples);
static int PlayFirstFrame(void);



static void PlayerSetSampleRate(uint32_t audioFreq)
{
	uint16_t period = (SystemCoreClock / audioFreq) - 1;

	pSampleTimer->Init.Prescaler = 0;
	pSampleTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
	pSampleTimer->Init.Period = period;
	pSampleTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pSampleTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(pSampleTimer);
}

static void PlayStream(uint16_t* pBuffer, uint32_t Size)
{
	//HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)pBuffer, DMA_MAX(Size/AUDIODATA_SIZE), DAC_ALIGN_12B_R);
	HAL_DACEx_DualStart_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)pBuffer, Size, DAC_ALIGN_12B_L);
}

static int PlayFirstFrame(void)
{
	int rc = -1;
	unsigned int samples = 0;

	TSpiritMP3Info mp3Info;
	samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[0], BUFFER_SIZE/2, &mp3Info);

	if (samples < 1)
	{
		// Error, EOF
		return rc;
	}

	VolumeScale(audioBuffer, samples);	
	printf("Playing sample rate: %dHz, bitrate:%dkbps \r\n", mp3Info.nSampleRateHz, mp3Info.nBitrateKbps);

	//note: no need to consider about those 'Variable Bit Rate (VBR) files', this demo decoder version not supported
	PlayerSetSampleRate(mp3Info.nSampleRateHz);
	PlayStream(audioBuffer, samples);

	return rc;
}

static unsigned int ReadData(void* data, unsigned int size, void* token)
{
	unsigned int read = 0;
	if (f_read(&playingFile, data, size, &read) != FR_OK)
	{
		return 0;
	}
	return read;
}

int PlayerStart(char *fileName)
{
	int rc = 0;

	rc = f_open(&playingFile, fileName, FA_READ);

	if (rc == FR_OK)
	{
		/* Initialize MP3 decoder */
#if 1	//without callback
		SpiritMP3DecoderInit(&decoder, ReadData, NULL, NULL);
#else	//with callback
		SpiritMP3DecoderInit(&decoder, ReadData, DacProcess, NULL);
#endif
		PlayFirstFrame();
	}
	return rc;
}

void PlayerStop(void)
{
	//HAL_DAC_Stop_DMA(pDac,DAC_CHANNEL_1);
	HAL_DACEx_DualStop_DMA(pDac, DAC_CHANNEL_1);
#ifdef DEBUG
	sprintf(tmpBuf, "Tick:%lu.\n\r", HAL_GetTick());
	HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif

	f_close(&playingFile);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
}

void PlayerUpdate(void)
{
	unsigned int samples = 0;

	switch (playerState)
	{
	case PLAYER_STATE_DMA_TX_COMPLETE:
	{
		playerState = PLAYER_STATE_IDLE;
		//point to the head of the audio buffer
		PlayStream(audioBuffer, BUFFER_SIZE/2);

		samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[BUFFER_SIZE/2], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			printf("Stop.\n\r");
			PlayerStop();
			break;
		}
		VolumeScale(&audioBuffer[BUFFER_SIZE/2], samples);
		HAL_GPIO_TogglePin(B1_GPIO_Port, B1_Pin);
	}
		break;
	case PLAYER_STATE_DMA_TX_HALF:
	{
		playerState = PLAYER_STATE_IDLE;

#ifdef DEBUG
			uint32_t start = HAL_GetTick();
#endif
		samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[0], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			break;
		}
#ifdef DEBUG
		//printf("Read:%lu.\n\r", HAL_GetTick() - start);
		uint32_t endT = HAL_GetTick();
		sprintf(tmpBuf, "Re:%lu, tick:%lu.\n\r", endT - start, endT);
		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
		//start = HAL_GetTick();
#endif
		VolumeScale(&audioBuffer[0], samples);

#ifdef DEBUG
		//sprintf(tmpBuf, "Ve:%lu.\n\r", HAL_GetTick() - start);
		//HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
	}
		break;
	default:
		break;
	}
}

void PlayerPause(void)
{
	//HAL_DAC_Stop_DMA(pDac,DAC_CHANNEL_1);
	//HAL_DACEx_DualStop_DMA(pDac);
	HAL_DACEx_DualStop(pDac);
	HAL_TIM_Base_Stop(pSampleTimer);
}

void PlayerResume(void)
{
	HAL_DACEx_DualStart(pDac);
	HAL_TIM_Base_Start(pSampleTimer);
}

void PlayerInit(TIM_HandleTypeDef *sTimer, DAC_HandleTypeDef* sDac)
{
	pSampleTimer = sTimer;
	pDac = sDac;

	HAL_TIM_Base_Start(pSampleTimer);

	if (HAL_DACEx_DualSetValue(pDac, DAC_ALIGN_12B_R, 0, 0) != HAL_OK)
	{
		/* DAC value set error */
		Error_Handler();
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	playerState = PLAYER_STATE_DMA_TX_COMPLETE;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	playerState = PLAYER_STATE_DMA_TX_HALF;
}

static void VolumeScale(uint16_t* pBuffer, unsigned int samples)
{
	for(int i = 0; i < samples * 2; i++)
	{
		int32_t val = (int16_t)*pBuffer;

		*pBuffer = val * volume / 100;
		pBuffer++;
	}
}

void PlayerVolumeAdj(int32_t vol)
{
	if (vol >= 0)
	{
		volume = vol;
	}
	else
	{
		if (volume > 10)
		{
			volume -= 10;
		}
		else
		{
			volume = 100;
		}
	}
}

void DacProcess(void* buff, int isShort, int ch, void* token)
{
	//nothing to do right this moment
}
