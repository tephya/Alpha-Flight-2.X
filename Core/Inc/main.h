/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBAT_Pin GPIO_PIN_0
#define VBAT_GPIO_Port GPIOC
#define CRT_Pin GPIO_PIN_1
#define CRT_GPIO_Port GPIOC
#define xGPS_Pin GPIO_PIN_0
#define xGPS_GPIO_Port GPIOA
#define rGPS_Pin GPIO_PIN_1
#define rGPS_GPIO_Port GPIOA
#define xELRS_Pin GPIO_PIN_2
#define xELRS_GPIO_Port GPIOA
#define rELRS_Pin GPIO_PIN_3
#define rELRS_GPIO_Port GPIOA
#define IMU1_INT_Pin GPIO_PIN_4
#define IMU1_INT_GPIO_Port GPIOA
#define IMU1_INT_EXTI_IRQn EXTI4_IRQn
#define IMU1_SCK_Pin GPIO_PIN_5
#define IMU1_SCK_GPIO_Port GPIOA
#define IMU1_SDO_Pin GPIO_PIN_6
#define IMU1_SDO_GPIO_Port GPIOA
#define IMU1_SDI_Pin GPIO_PIN_7
#define IMU1_SDI_GPIO_Port GPIOA
#define IMU1_CS_Pin GPIO_PIN_4
#define IMU1_CS_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_13
#define SD_SCK_GPIO_Port GPIOB
#define SD_SDO_Pin GPIO_PIN_14
#define SD_SDO_GPIO_Port GPIOB
#define SD_SDI_Pin GPIO_PIN_15
#define SD_SDI_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_8
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_9
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_10
#define S3_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_11
#define S4_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IMU2_INT_Pin GPIO_PIN_15
#define IMU2_INT_GPIO_Port GPIOA
#define IMU2_INT_EXTI_IRQn EXTI15_10_IRQn
#define IMU2_SCK_Pin GPIO_PIN_10
#define IMU2_SCK_GPIO_Port GPIOC
#define IMU2_SDO_Pin GPIO_PIN_11
#define IMU2_SDO_GPIO_Port GPIOC
#define IMU2_SDI_Pin GPIO_PIN_12
#define IMU2_SDI_GPIO_Port GPIOC
#define Tele_Pin GPIO_PIN_2
#define Tele_GPIO_Port GPIOD
#define IMU2_CS_Pin GPIO_PIN_3
#define IMU2_CS_GPIO_Port GPIOB
#define Buzz_Pin GPIO_PIN_5
#define Buzz_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define State_LED_Pin GPIO_PIN_9
#define State_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
