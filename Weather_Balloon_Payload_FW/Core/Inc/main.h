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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_PPS_Pin GPIO_PIN_0
#define GPS_PPS_GPIO_Port GPIOA
#define GPS_PPS_EXTI_IRQn EXTI0_IRQn
#define GPS_EXTINT_Pin GPIO_PIN_1
#define GPS_EXTINT_GPIO_Port GPIOA
#define GPS_EXTINT_EXTI_IRQn EXTI1_IRQn
#define TSENS_INT_ALERT_Pin GPIO_PIN_2
#define TSENS_INT_ALERT_GPIO_Port GPIOA
#define TSENS_INT_ALERT_EXTI_IRQn EXTI2_TSC_IRQn
#define TSNS_EXT_ALERT_Pin GPIO_PIN_3
#define TSNS_EXT_ALERT_GPIO_Port GPIOA
#define TSNS_EXT_ALERT_EXTI_IRQn EXTI3_IRQn
#define PLL_LE_Pin GPIO_PIN_4
#define PLL_LE_GPIO_Port GPIOA
#define GPS_RESETB_Pin GPIO_PIN_0
#define GPS_RESETB_GPIO_Port GPIOB
#define PLL_CE_Pin GPIO_PIN_1
#define PLL_CE_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOB
#define LED_0_Pin GPIO_PIN_13
#define LED_0_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_14
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOB
#define PA_EN_Pin GPIO_PIN_3
#define PA_EN_GPIO_Port GPIOB
#define EN_5P0_Pin GPIO_PIN_4
#define EN_5P0_GPIO_Port GPIOB
#define PLL_MUXOUT_Pin GPIO_PIN_5
#define PLL_MUXOUT_GPIO_Port GPIOB
#define SD_DET_Pin GPIO_PIN_8
#define SD_DET_GPIO_Port GPIOB
#define BUZZ_LED_EN_Pin GPIO_PIN_9
#define BUZZ_LED_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SD_SPI_HANDLE hspi1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
