/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define NomPS 2
#define DataPS500k 10
#define DataPS1M 5
#define NomPS500k 4
#define fsw 3600
#define fsw10kHz 9000
#define DataPS 5
#define fsw40kHz 2250
#define NomPS1M 2
#define fsw50kHz 1800
#define fsw25kHz 3600
#define CSOA_Pin GPIO_PIN_0
#define CSOA_GPIO_Port GPIOA
#define CSOB_Pin GPIO_PIN_1
#define CSOB_GPIO_Port GPIOA
#define CS_CON_Pin GPIO_PIN_4
#define CS_CON_GPIO_Port GPIOA
#define CLK_CON_Pin GPIO_PIN_5
#define CLK_CON_GPIO_Port GPIOA
#define MISO_CON_Pin GPIO_PIN_6
#define MISO_CON_GPIO_Port GPIOA
#define MOSI_CON_Pin GPIO_PIN_7
#define MOSI_CON_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_0
#define ENABLE_GPIO_Port GPIOB
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_10
#define PWMA_GPIO_Port GPIOA
#define ENABLEA15_Pin GPIO_PIN_15
#define ENABLEA15_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
