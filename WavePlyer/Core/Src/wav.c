#include <string.h>
#include "main.h"
#include "wav.h"
#include "app_fatfs.h"
#include "printf.h"

#ifdef DEBUG
extern UART_HandleTypeDef hlpuart1;
char tmpBuf[64] = {0};
#endif

TIM_HandleTypeDef *pSampleTimer = NULL;
DAC_HandleTypeDef* pDac = NULL;
extern uint8_t playerBusy;

volatile PlayerState plState = PLAYER_STATE_IDLE;
static uint8_t fileBuffer[BUFSIZE];
static uint8_t dmaBuffer[2][BUFSIZE];
static uint8_t dmaBank = 0;
static uint32_t bytes_last = 0;
static uint8_t bytesPerSample = 0;
static uint16_t numSamples = 0;
FuncP PtrDataPrepare;

volatile int32_t volume = 90;

FIL fil;
FRESULT res;
UINT count = 0;
struct Wav_Header header;
FATFS FatFs;
DIR dir;
FILINFO fno;


static void setSampleRate(uint16_t freq)
{
  uint16_t period = (SystemCoreClock / freq) - 1;

  pSampleTimer->Init.Prescaler = 0;
  pSampleTimer->Init.CounterMode = TIM_COUNTERMODE_UP;
  pSampleTimer->Init.Period = period;
  pSampleTimer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  pSampleTimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(pSampleTimer);
}

static inline uint16_t val2Dac8(int32_t v)
{
  uint16_t out = v << 3;
  out = out * volume / 100;
  return out;
}

static inline uint16_t val2Dac16(int32_t v)
{
  v >>= 4;
  v += 2047;
  v = v * volume / 100;
  return v & 0xfff;
}

static void prepareDACBuffer_8Bit(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput)
{
  uint8_t *pInput = (uint8_t *)pIn;

  for (int i=0; i<numSamples; i++) {
    int32_t val = 0;

    for(int j=0; j<channels; j++) {
      val += *pInput++;
    }
    val /= channels;
    *pOutput++ = val2Dac8(val);
  }
}

static void prepareDACBuffer_16Bit(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput)
{
  int16_t *pInput = (int16_t *)pIn;

  for (int i=0; i<numSamples; i++) {
    int32_t val = 0;

    for(int j=0; j<channels; j++) {
      val += *pInput++;
    }
    val /= channels;
    *pOutput++ = val2Dac16(val);
  }
}

static void PlayerWave()
{
	bytesPerSample = header.bitsPerSample / 8 / header.channels;
	PtrDataPrepare = (header.bitsPerSample == 8)? prepareDACBuffer_8Bit : prepareDACBuffer_16Bit;

	bytes_last = header.dataChunkLength;

  if(bytes_last > 0)
  {
	int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);
    UINT bytes_read;
	dmaBank = 0;

    res = f_read(&fil, fileBuffer, blksize, &bytes_read);

    if (res == FR_OK && bytes_read > 0)
    {
		numSamples = bytes_read / bytesPerSample ;
		int16_t *pInput = (int16_t *)fileBuffer;
		uint16_t *pOutput = (uint16_t *)dmaBuffer[dmaBank];

		PtrDataPrepare(header.channels, numSamples, pInput, pOutput);

		HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);

		dmaBank = 1;
		bytes_last -= blksize;
		plState = PLAYER_STATE_PLAYING;

		//prepare next buff
		if (bytes_last > 0)
		{
			blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);
			res = f_read(&fil, fileBuffer, blksize, &bytes_read);
			if (res == FR_OK && bytes_read > 0)
			{
				numSamples = bytes_read / bytesPerSample;
				pOutput = (uint16_t *)dmaBuffer[dmaBank];
				PtrDataPrepare(header.channels, numSamples, pInput, pOutput);
				bytes_last -= blksize;
			}
		}
		else
		{
			numSamples = 0;
		}
	}
	else
	{
		f_close(&fil);
		if(f_mount(NULL, "", 1) != FR_OK)
		{
			Error_Handler();
		}
	}
  }
}

static void PrepareNextBuff(void)
{
	//while (numSamples > 0)
	while (bytes_last > 0)
	{
#ifdef DEBUG
		uint32_t start = HAL_GetTick();
#endif
		plState = PLAYER_STATE_PLAYING;
		HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);

		{
			int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);
			UINT bytes_read;

			res = f_read(&fil, fileBuffer, blksize, &bytes_read);
			if (res == FR_OK && bytes_read > 0)
			{
				numSamples = bytes_read / bytesPerSample;
				int16_t *pInput = (int16_t *)fileBuffer;
				uint16_t *pOutput = (uint16_t *)dmaBuffer[dmaBank];

				PtrDataPrepare(header.channels, numSamples, pInput, pOutput);

				dmaBank = (dmaBank == 0) ? 1 : 0;
				bytes_last -= blksize;
			}
			else
			{
				numSamples = 0;
				f_close(&fil);
				HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
				plState = PLAYER_STATE_IDLE;
				if(f_mount(NULL, "", 1) != FR_OK)
				{
					Error_Handler();
				}
			}
#ifdef DEBUG
			sprintf(tmpBuf, "Read:%u, Cost:%lu.\n\r", bytes_read, HAL_GetTick() - start);
			HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
		}
		//else
		{
			//numSamples = 0;
		}
#ifdef DEBUG
		uint32_t start2 = HAL_GetTick();
#endif

		while (plState == PLAYER_STATE_PLAYING)
		{
			/* code */
		}
#ifdef DEBUG
		sprintf(tmpBuf, "Played:%lu.\n\r", HAL_GetTick() - start2);
		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
		plState = PLAYER_STATE_PLAYING;
	}
	plState = PLAYER_STATE_IDLE;
	return;


	if (numSamples > 0)
	{
		plState = PLAYER_STATE_PLAYING;
		HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);
	}
	else
	{
		f_close(&fil);
		HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
		plState = PLAYER_STATE_IDLE;
		if(f_mount(NULL, "", 1) != FR_OK)
		{
			Error_Handler();
		}
		return;
	}
	if(bytes_last > 0)
	{
		int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);
		UINT bytes_read;

		res = f_read(&fil, fileBuffer, blksize, &bytes_read);
		if (res == FR_OK && bytes_read > 0)
		{
			numSamples = bytes_read / bytesPerSample;
			int16_t *pInput = (int16_t *)fileBuffer;
			uint16_t *pOutput = (uint16_t *)dmaBuffer[dmaBank];

			PtrDataPrepare(header.channels, numSamples, pInput, pOutput);

			dmaBank = (dmaBank == 0) ? 1 : 0;
			bytes_last -= blksize;
		}
		else
		{
			f_close(&fil);
			HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
			plState = PLAYER_STATE_IDLE;
			if(f_mount(NULL, "", 1) != FR_OK)
			{
				Error_Handler();
			}
		}
#ifdef DEBUG
		//sprintf(tmpBuf, "Read:%u, Cost:%lu.\n\r", bytes_read, HAL_GetTick() - start);
		//HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
	}
	else
	{
		numSamples = 0;
	}
}

static uint8_t isSupprtedWavFile(const struct Wav_Header *header)
{
  if (strncmp(header->riff, "RIFF", 4 ) != 0)
    return 0;

  if (header->vfmt != 1)
    return 0;

  if (strncmp(header->dataChunkHeader, "data", 4 ) != 0)
    return 0;

  return 1;
}

static void playWavFile(char *filename)
{
	HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);

    res = f_open(&fil, filename, FA_READ);
    if (res != FR_OK)
    {
      return;
    }

    res = f_read(&fil, &header, sizeof(struct Wav_Header), &count);
    if (res == FR_OK && isSupprtedWavFile(&header))
    {
        setSampleRate(header.sampleFreq);
        PlayerWave();
    }
	else
	{
		res = f_close(&fil);
	}
}

void WavPlayerInit(TIM_HandleTypeDef *sTimer, DAC_HandleTypeDef* sDac)
{
    pSampleTimer = sTimer;
    pDac = sDac;
}

void WavPlayAll(void)
{
    res = f_mount(&FatFs, "", 0);
    if (res != FR_OK)
    {
      //return EXIT_FAILURE;
      return;
    }
    res = f_opendir(&dir, "");
    if (res != FR_OK)
    {
      //return EXIT_FAILURE;
      return;
    }
    while(1)
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }

      char *filename = fno.fname;
      //if (strstr(filename, ".WAV") != NULL)
      //if (wcschr(filename, ".WAV") != 0)
      if (strstr(filename, ".WAV") != NULL)
      {
        playWavFile(filename);
      }
    }
    res = f_closedir(&dir);
    /* Unmount SDCARD */
    if(f_mount(NULL, "", 1) != FR_OK)
    {
      Error_Handler();
    }
}

void WavPlayFile(char* fileName)
{
	res = f_mount(&FatFs, "", 0);
    if (res != FR_OK)
    {
      //return EXIT_FAILURE;
      return;
    }
    res = f_opendir(&dir, "");
    if (res != FR_OK)
    {
      //return EXIT_FAILURE;
      return;
    }
	res = f_readdir(&dir, &fno);
    if (res != FR_OK || fno.fname[0] == 0)
    {
      return;
    }
	playWavFile(fileName);
}

void PlayerUpdate(void)
{
    if (plState == PLAYER_STATE_DMA_EMPTY)
    {
        PrepareNextBuff();
    }
}

void PlayerStop(void)
{
	if (plState != PLAYER_STATE_IDLE)
	{
		HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
		plState = PLAYER_STATE_IDLE;
	}
}
