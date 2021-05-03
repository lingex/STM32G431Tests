/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define DIS_OE_Pin GPIO_PIN_7
#define DIS_OE_GPIO_Port GPIOA
#define DIS_CS_Pin GPIO_PIN_0
#define DIS_CS_GPIO_Port GPIOB
#define DIS_B_Pin GPIO_PIN_10
#define DIS_B_GPIO_Port GPIOB
#define DIS_LAT_Pin GPIO_PIN_7
#define DIS_LAT_GPIO_Port GPIOC
#define DIS_A_Pin GPIO_PIN_8
#define DIS_A_GPIO_Port GPIOA
#define DIS_D_Pin GPIO_PIN_9
#define DIS_D_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define DIS_CLK_Pin GPIO_PIN_3
#define DIS_CLK_GPIO_Port GPIOB
#define DIS_C_Pin GPIO_PIN_4
#define DIS_C_GPIO_Port GPIOB
#define DIS_R1_Pin GPIO_PIN_5
#define DIS_R1_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_6
#define SD_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MIN(a,b) (((a)<(b))? (a):(b))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
