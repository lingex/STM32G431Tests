#ifndef __WAV_H__
#define __WAV_H__

#pragma pack(push)
#pragma pack(1)
struct Wav_Header {
  char     riff[4];
  uint32_t fileSize;
  char     fileTypeHeader[4];
  char     formatChunkMarker[4];
  uint32_t formatChunkLength;
  uint16_t vfmt;
  uint16_t channels;
  uint32_t sampleFreq;
  uint32_t sampleBytesPerSecond;
  uint16_t blkSize;
  uint16_t bitsPerSample;
  char     dataChunkHeader[4];
  uint32_t dataChunkLength;
};
#pragma pack(pop)

typedef void (*FuncP)(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput);

#define BUFSIZE 1024

typedef enum
{
	PLAYER_STATE_IDLE = 0,
	PLAYER_STATE_PLAYING = 1,
	PLAYER_STATE_DMA_EMPTY = 2,
}PlayerState;



void WavPlayerInit(TIM_HandleTypeDef *sTimer, DAC_HandleTypeDef* sDac);

void WavPlayAll(void);
void WavPlayFile(char* fileName);

void PlayerUpdate(void);

void PlayerStop(void);


#endif
