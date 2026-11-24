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

#include "cmsis_os.h"

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
#define PWM_SERVO_1_Pin GPIO_PIN_5
#define PWM_SERVO_1_GPIO_Port GPIOE
#define PWM_SERVO_2_Pin GPIO_PIN_6
#define PWM_SERVO_2_GPIO_Port GPIOE
#define LED_Red_Pin GPIO_PIN_9
#define LED_Red_GPIO_Port GPIOF
#define LED_IDLE_Pin GPIO_PIN_10
#define LED_IDLE_GPIO_Port GPIOF
#define KEY_UP_Pin GPIO_PIN_0
#define KEY_UP_GPIO_Port GPIOA
#define DIP_SW_Pin GPIO_PIN_1
#define DIP_SW_GPIO_Port GPIOA
#define BATTERY_Pin GPIO_PIN_2
#define BATTERY_GPIO_Port GPIOA
#define VISION_RX_Pin GPIO_PIN_3
#define VISION_RX_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_5
#define LCD_RST_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define SERVO_TX_Pin GPIO_PIN_10
#define SERVO_TX_GPIO_Port GPIOB
#define SERVO_RX_Pin GPIO_PIN_11
#define SERVO_RX_GPIO_Port GPIOB
#define STEP_TX_Pin GPIO_PIN_6
#define STEP_TX_GPIO_Port GPIOC
#define STEP_RX_Pin GPIO_PIN_7
#define STEP_RX_GPIO_Port GPIOC
#define Terminal_TX_Pin GPIO_PIN_9
#define Terminal_TX_GPIO_Port GPIOA
#define Terminal_RX_Pin GPIO_PIN_10
#define Terminal_RX_GPIO_Port GPIOA
#define OPS_TX_Pin GPIO_PIN_10
#define OPS_TX_GPIO_Port GPIOC
#define OPS_RX_Pin GPIO_PIN_11
#define OPS_RX_GPIO_Port GPIOC
#define SCANER_TX_Pin GPIO_PIN_12
#define SCANER_TX_GPIO_Port GPIOC
#define SCANER_RX_Pin GPIO_PIN_2
#define SCANER_RX_GPIO_Port GPIOD
#define VISION_TX_Pin GPIO_PIN_5
#define VISION_TX_GPIO_Port GPIOD
#define START_Pin GPIO_PIN_14
#define START_GPIO_Port GPIOG
#define LCD_MISO_Pin GPIO_PIN_4
#define LCD_MISO_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
