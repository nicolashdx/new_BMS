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
#include "stm32h7xx_hal.h"

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
#define PC13_GPIO_Pin GPIO_PIN_13
#define PC13_GPIO_GPIO_Port GPIOC
#define PC15_GPIO_Pin GPIO_PIN_14
#define PC15_GPIO_GPIO_Port GPIOC
#define PC14_GPIO_Pin GPIO_PIN_15
#define PC14_GPIO_GPIO_Port GPIOC
#define Reset_Eletrobuild_Pin GPIO_PIN_7
#define Reset_Eletrobuild_GPIO_Port GPIOE
#define USART1_Tx_Pin GPIO_PIN_9
#define USART1_Tx_GPIO_Port GPIOA
#define USART1_Rx_Pin GPIO_PIN_10
#define USART1_Rx_GPIO_Port GPIOA
#define CAN_Rx_Pin GPIO_PIN_11
#define CAN_Rx_GPIO_Port GPIOA
#define CAN_Tx_Pin GPIO_PIN_12
#define CAN_Tx_GPIO_Port GPIOA
#define LED_DEBUG_Pin GPIO_PIN_7
#define LED_DEBUG_GPIO_Port GPIOD
#define CHARGE_ENABLE_Pin GPIO_PIN_4
#define CHARGE_ENABLE_GPIO_Port GPIOB
#define AIR_AUX_MINUS_Pin GPIO_PIN_5
#define AIR_AUX_MINUS_GPIO_Port GPIOB
#define AIR_AUX_PLUS_Pin GPIO_PIN_6
#define AIR_AUX_PLUS_GPIO_Port GPIOB
#define ERROR_LED_Pin GPIO_PIN_7
#define ERROR_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ISOSPI_CS_GPIO_Port GPIOA
#define ISOSPI_CS_Pin GPIO_PIN_5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
