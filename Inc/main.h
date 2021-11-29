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
#include "stm32f0xx_hal.h"

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
#define LED_Red_Pin GPIO_PIN_13
#define LED_Red_GPIO_Port GPIOB

/**********SX1280引脚专用***************/
//SPI的四个引脚不变
#define SX1280_BUSY_Pin GPIO_PIN_13
#define SX1280_BUSY_GPIO_Port GPIOC
#define SX1280_RESET_Pin GPIO_PIN_14
#define SX1280_RESET_GPIO_Port GPIOC
#define SX1280_TX_EN_Pin GPIO_PIN_12
#define SX1280_TX_EN_GPIO_Port GPIOB
#define SX1280_RX_EN_Pin GPIO_PIN_13
#define SX1280_RX_EN_GPIO_Port GPIOA

#define A7106_IRQ_Pin GPIO_PIN_6
#define A7106_IRQ_GPIO_Port GPIOB
#define A7106_IRQ_EXTI_IRQn EXTI4_15_IRQn
/**************************************/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
