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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_BTN_Pin GPIO_PIN_13
#define EN_BTN_GPIO_Port GPIOC
#define MCU_IV_ADC__Pin GPIO_PIN_0
#define MCU_IV_ADC__GPIO_Port GPIOC
#define MCU_IV_ADC_C1_Pin GPIO_PIN_1
#define MCU_IV_ADC_C1_GPIO_Port GPIOC
#define MCU_IW_ADC__Pin GPIO_PIN_2
#define MCU_IW_ADC__GPIO_Port GPIOC
#define MCU_IW_ADC_C3_Pin GPIO_PIN_3
#define MCU_IW_ADC_C3_GPIO_Port GPIOC
#define MCU_IU_ADC__Pin GPIO_PIN_1
#define MCU_IU_ADC__GPIO_Port GPIOA
#define MCU_IU_ADC_A2_Pin GPIO_PIN_2
#define MCU_IU_ADC_A2_GPIO_Port GPIOA
#define EN_CLK_Pin GPIO_PIN_3
#define EN_CLK_GPIO_Port GPIOA
#define EN_DT_Pin GPIO_PIN_1
#define EN_DT_GPIO_Port GPIOB
#define MCU_SVPWM_U__Pin GPIO_PIN_13
#define MCU_SVPWM_U__GPIO_Port GPIOB
#define MCU_SVPWM_V__Pin GPIO_PIN_14
#define MCU_SVPWM_V__GPIO_Port GPIOB
#define MCU_SVPWM_W__Pin GPIO_PIN_15
#define MCU_SVPWM_W__GPIO_Port GPIOB
#define BT_RESET_Pin GPIO_PIN_6
#define BT_RESET_GPIO_Port GPIOC
#define BT_IRQ_Pin GPIO_PIN_7
#define BT_IRQ_GPIO_Port GPIOC
#define Q3_CTRL_Pin GPIO_PIN_8
#define Q3_CTRL_GPIO_Port GPIOC
#define MCU_SVPWM_U_A8_Pin GPIO_PIN_8
#define MCU_SVPWM_U_A8_GPIO_Port GPIOA
#define MCU_SVPWM_V_A9_Pin GPIO_PIN_9
#define MCU_SVPWM_V_A9_GPIO_Port GPIOA
#define MCU_SVPWM_W_A10_Pin GPIO_PIN_10
#define MCU_SVPWM_W_A10_GPIO_Port GPIOA
#define MCU_U_DIS_Pin GPIO_PIN_2
#define MCU_U_DIS_GPIO_Port GPIOD
#define Q1_CTRL_Pin GPIO_PIN_4
#define Q1_CTRL_GPIO_Port GPIOB
#define Q2_CTRL_Pin GPIO_PIN_5
#define Q2_CTRL_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_7
#define MCU_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
