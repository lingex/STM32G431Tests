#include <string.h>
#include "main.h"
#include "wav.h"
#include "app_fatfs.h"
#include "printf.h"

TIM_HandleTypeDef *pSampleTimer = NULL;
DAC_HandleTypeDef* pDac = NULL;

#ifdef DEBUG
extern UART_HandleTypeDef hlpuart1;
char tmpBuf[64] = {0};
#endif

static uint8_t dmaBuffer[2][BUFSIZE];
static uint8_t fileBuffer[BUFSIZE];
static uint8_t dmaBank = 0;

uint8_t playerBusy;
volatile uint8_t playerBuffEmpty = 0;

static uint32_t bytes_last = 0;
static uint16_t numSamples = 0;
static int32_t volume = 80;

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

static void outputSamples()
{
	const uint8_t channels = header.channels;
	const uint8_t bytesPerSample = header.bitsPerSample / 8;

	FuncP prepareData = (header.bitsPerSample == 8)? prepareDACBuffer_8Bit : prepareDACBuffer_16Bit;

	dmaBank = 0;
	bytes_last = header.dataChunkLength;
	playerBusy = 1;

	//Non-Blocking
	if(bytes_last > 0)
	{
		int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);

		UINT bytes_read;
		FRESULT res;
		//uint8_t fileBuffer[BUFSIZE];

		res = f_read(&fil, fileBuffer, blksize, &bytes_read);
		if (res != FR_OK || bytes_read == 0)
		{
			return;
		}
		numSamples = bytes_read / bytesPerSample / channels;
		int16_t     *pInput = (int16_t *)fileBuffer;
		uint16_t   *pOutput = (uint16_t *)dmaBuffer[dmaBank];

		prepareData(channels, numSamples, pInput, pOutput);

		HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);

		dmaBank = (dmaBank == 0) ? 1 : 0;
		bytes_last -= blksize;

		playerBuffEmpty = 0;

		//next buff
		blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);

		res = f_read(&fil, fileBuffer, blksize, &bytes_read);
		if (res == FR_OK && bytes_read > 0)
		{
			numSamples = bytes_read / bytesPerSample / channels;
			pInput = (int16_t *)fileBuffer;
			pOutput = (uint16_t *)dmaBuffer[dmaBank];
			prepareData(channels, numSamples, pInput, pOutput);

			bytes_last -= blksize;
		}
	}
	else
	{
		HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
		f_close(&fil);
	}
}

static void ContinuePlay(void)
{
	const uint8_t channels = header.channels;
	const uint8_t bytesPerSample = header.bitsPerSample / 8;

	FuncP prepareData = (header.bitsPerSample == 8)? prepareDACBuffer_8Bit : prepareDACBuffer_16Bit;

	playerBuffEmpty = 0;

	if (numSamples > 0)
	{
		HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);

		if (bytes_last > 0)
		{

#ifdef DEBUG
			uint32_t start = HAL_GetTick();
#endif
			//next buff
			int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);
			//uint8_t fileBuffer[BUFSIZE];
			UINT bytes_read;

			res = f_read(&fil, fileBuffer, blksize, &bytes_read);
			if (res == FR_OK && bytes_read > 0)
			{
				dmaBank = (dmaBank == 0) ? 1 : 0;
				numSamples = bytes_read / bytesPerSample / channels;
				int16_t     *pInput = (int16_t *)fileBuffer;
				uint16_t   *pOutput = (uint16_t *)dmaBuffer[dmaBank];
				prepareData(channels, numSamples, pInput, pOutput);

    			bytes_last -= blksize;
			}
#ifdef DEBUG
			sprintf(tmpBuf, "Read:%lu.\n\r", HAL_GetTick() - start);
			HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
		}
		else
		{
			numSamples = 0;
		}
	}
	else
	{
		//end of file
		playerBusy = 0;
		HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
		f_close(&fil);
		if(f_mount(NULL, "", 1) != FR_OK)
		{
			Error_Handler();
		}
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
    res = f_open(&fil, filename, FA_READ);
    if (res != FR_OK)
    {
      return;
    }

    res = f_read(&fil, &header, sizeof(struct Wav_Header), &count);
    if (res == FR_OK && isSupprtedWavFile(&header))
    {
        setSampleRate(header.sampleFreq);
        outputSamples();
    }
	else
	{
    	f_close(&fil);
		f_mount(NULL, "", 1);
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	playerBuffEmpty = 1;
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

void PlayTask(void)
{
	if (playerBuffEmpty != 0)
	{
		ContinuePlay();
	}
}

void VolumeAdj(void)
{
	if (volume >= 10)
	{
		volume -= 10;
	}
	else
	{
		volume = 100;
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
