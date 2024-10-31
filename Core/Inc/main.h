/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define TEST_LED_Pin GPIO_PIN_13
#define TEST_LED_GPIO_Port GPIOC
#define KEY1_INC_Freq_Pin GPIO_PIN_6
#define KEY1_INC_Freq_GPIO_Port GPIOA
#define KEY2_DEC_Freq_Pin GPIO_PIN_7
#define KEY2_DEC_Freq_GPIO_Port GPIOA
#define KEY3_INC_DT_Pin GPIO_PIN_4
#define KEY3_INC_DT_GPIO_Port GPIOB
#define KEY4_DEC_DT_Pin GPIO_PIN_5
#define KEY4_DEC_DT_GPIO_Port GPIOB
#define KEY5_INC_DUTY_Pin GPIO_PIN_6
#define KEY5_INC_DUTY_GPIO_Port GPIOB
#define KEY6_DEC_DUTY_Pin GPIO_PIN_7
#define KEY6_DEC_DUTY_GPIO_Port GPIOB
#define KEY7_SWITCH_MODE_Pin GPIO_PIN_9
#define KEY7_SWITCH_MODE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADCVin_Pin GPIO_PIN_0
#define ADCVin_GPIO_Port GPIOA
#define ADCIin_Pin GPIO_PIN_1
#define ADCIin_GPIO_Port GPIOA
#define ADCVout_Pin GPIO_PIN_2
#define ADCVout_GPIO_Port GPIOA
#define ADCIout_Pin GPIO_PIN_3
#define ADCIout_GPIO_Port GPIOA
#define ADCVadj_Pin GPIO_PIN_4
#define ADCVadj_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_1
#define LED_Y_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_3
#define KEY_1_GPIO_Port GPIOB
#define KEY_2_Pin GPIO_PIN_4
#define KEY_2_GPIO_Port GPIOB



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
