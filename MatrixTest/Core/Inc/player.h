
#ifndef __PLAYER_H_
#define __PLAYER_H_

#include "main.h"

#define BUFFER_SIZE		(576 * 4)	//in bytes	default 576*4

typedef enum
{
	PLAYER_STATE_IDLE,
	PLAYER_STATE_DMA_TX_COMPLETE,
	PLAYER_STATE_DMA_TX_HALF,
}PlayerState_t;

int  PlayerStart(char *path);
void PlayerStop(void);
void PlayerPause(void);
void PlayerResume(void);
void PlayerUpdate(void);
void PlayerVolumeAdj(int32_t vol);

void PlayerInit(TIM_HandleTypeDef *sTimer, DAC_HandleTypeDef* sDac);

void DacProcess(void* buff, int isShort, int ch, void* token);


#endif // __PLAYER_H_
