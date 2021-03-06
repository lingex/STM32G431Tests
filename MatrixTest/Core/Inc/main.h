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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
void SystemClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define DIS_CS_Pin GPIO_PIN_0
#define DIS_CS_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define DIS_D_Pin GPIO_PIN_11
#define DIS_D_GPIO_Port GPIOB
#define DIS_C_Pin GPIO_PIN_12
#define DIS_C_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_14
#define SD_CS_GPIO_Port GPIOB
#define DIS_B_Pin GPIO_PIN_11
#define DIS_B_GPIO_Port GPIOA
#define DIS_A_Pin GPIO_PIN_12
#define DIS_A_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define DIS_LAT_Pin GPIO_PIN_2
#define DIS_LAT_GPIO_Port GPIOD
#define DIS_SPI3_CLK_Pin GPIO_PIN_3
#define DIS_SPI3_CLK_GPIO_Port GPIOB
#define DIS_SPI3_DAT_Pin GPIO_PIN_5
#define DIS_SPI3_DAT_GPIO_Port GPIOB
#define DIS_OE_Pin GPIO_PIN_7
#define DIS_OE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MIN(a,b) (((a)<(b))? (a):(b))

#define BitBand(Addr, Bit) *((volatile uint32_t*)(((uint32_t)(Addr) & 0xF0000000) + 0x02000000 + (((uint32_t)(Addr) & 0x00FFFFFF) << 5) + ((Bit) << 2)))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
