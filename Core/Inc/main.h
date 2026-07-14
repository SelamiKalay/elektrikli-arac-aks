/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define DORTLU_BUTON_Pin GPIO_PIN_2
#define DORTLU_BUTON_GPIO_Port GPIOE
#define SOL_SIGNAL_KOL_Pin GPIO_PIN_6
#define SOL_SIGNAL_KOL_GPIO_Port GPIOE
#define SAG_SIGNAL_KOL_Pin GPIO_PIN_13
#define SAG_SIGNAL_KOL_GPIO_Port GPIOC
#define SAG_ROLE_Pin GPIO_PIN_0
#define SAG_ROLE_GPIO_Port GPIOC
#define SOL_ROLE_Pin GPIO_PIN_1
#define SOL_ROLE_GPIO_Port GPIOC
#define SILECEK_ROLE_Pin GPIO_PIN_3
#define SILECEK_ROLE_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_6
#define STATUS_LED_GPIO_Port GPIOA
#define CAN_LED_Pin GPIO_PIN_7
#define CAN_LED_GPIO_Port GPIOA
#define SAG_KAPI_Pin GPIO_PIN_7
#define SAG_KAPI_GPIO_Port GPIOE
#define FAR_BUTON_Pin GPIO_PIN_8
#define FAR_BUTON_GPIO_Port GPIOE
#define SOL_KAPI_Pin GPIO_PIN_9
#define SOL_KAPI_GPIO_Port GPIOE
#define FREN_GIRIS_Pin GPIO_PIN_10
#define FREN_GIRIS_GPIO_Port GPIOE
#define SILECEK_BILGISI_Pin GPIO_PIN_8
#define SILECEK_BILGISI_GPIO_Port GPIOA
#define MCP_INT_Pin GPIO_PIN_0
#define MCP_INT_GPIO_Port GPIOD
#define MCP_INT_EXTI_IRQn EXTI0_IRQn
#define MCP_RESET_Pin GPIO_PIN_1
#define MCP_RESET_GPIO_Port GPIOD
#define FAR_ROLE_Pin GPIO_PIN_2
#define FAR_ROLE_GPIO_Port GPIOD
#define UYARI_ROLE_Pin GPIO_PIN_3
#define UYARI_ROLE_GPIO_Port GPIOD
#define STOP_ROLE_Pin GPIO_PIN_4
#define STOP_ROLE_GPIO_Port GPIOD
#define KONTAKTOR_ROLE_Pin GPIO_PIN_5
#define KONTAKTOR_ROLE_GPIO_Port GPIOD
#define MCP_CS_Pin GPIO_PIN_6
#define MCP_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
