#include <string.h>
#include "main.h"
#include "wav.h"
#include "app_fatfs.h"

TIM_HandleTypeDef *pSampleTimer = NULL;
DAC_HandleTypeDef* pDac = NULL;
extern uint8_t playerBusy;

volatile uint8_t flg_dma_done;
static uint8_t fileBuffer[BUFSIZE];
static uint8_t dmaBuffer[2][BUFSIZE];
static uint8_t dmaBank = 0;

uint8_t playing = 0;
uint8_t playBreak = 0;

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

  pSampleTimer->Instance = TIM4;
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
  return out;
}

static inline uint16_t val2Dac16(int32_t v)
{
  v >>= 4;
  v += 2047;
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

static void outputSamples(FIL *fil, struct Wav_Header *header)
{
  const uint8_t channels = header->channels;
  const uint8_t bytesPerSample = header->bitsPerSample / 8;

  FuncP prepareData = (header->bitsPerSample == 8)? prepareDACBuffer_8Bit : prepareDACBuffer_16Bit;

  flg_dma_done = 1;
  dmaBank = 0;
  uint32_t bytes_last = header->dataChunkLength;
  playerBusy = 1;
  playBreak = 0;

  while(0 < bytes_last)
  {
    int blksize = (header->bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);

    UINT bytes_read;
    FRESULT res;

    res = f_read(fil, fileBuffer, blksize, &bytes_read);
    if (res != FR_OK || bytes_read == 0)
    {
      break;
    }
    uint16_t numSamples = bytes_read / bytesPerSample / channels;
    int16_t     *pInput = (int16_t *)fileBuffer;
    uint16_t   *pOutput = (uint16_t *)dmaBuffer[dmaBank];

    prepareData(channels, numSamples, pInput, pOutput);

    // wait for DMA complete
    while(flg_dma_done == 0)
    {

    }

    //HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);  //may cause noise
    flg_dma_done = 0;
    HAL_DAC_Start_DMA(pDac, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);

    dmaBank = (dmaBank == 0) ? 1 : 0;
    bytes_last -= blksize;
    if (playBreak != 0)
    {
        break;
    }
  };

  while(flg_dma_done == 0)
  {

  }

  HAL_DAC_Stop_DMA(pDac, DAC_CHANNEL_1);
  playerBusy = 0;
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
    if (res == FR_OK)
    {
        if (isSupprtedWavFile(&header))
        {
            setSampleRate(header.sampleFreq);
            outputSamples(&fil, &header);
        }
    }
    res = f_close(&fil);
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
	flg_dma_done = 1;
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
    if (playing != 0)
    {
        //TODO
    }
}

void PlayNextTrack(void)
{
    playBreak = 1;
}
