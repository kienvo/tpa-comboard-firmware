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
#include "stm32f1xx_hal.h"

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
uint8_t digital_read(uint16_t channel, uint8_t mode);
void digital_write(uint16_t channel, uint8_t pinstate);
uint8_t analog_read(uint16_t channel, uint16_t *pvalue, uint8_t mode);
uint8_t analog_write(uint16_t channel, uint16_t volt);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW4_Pin GPIO_PIN_13
#define SW4_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_0
#define LED_ERR_GPIO_Port GPIOC
#define LED_COMM_Pin GPIO_PIN_1
#define LED_COMM_GPIO_Port GPIOC
#define LED_RUN_Pin GPIO_PIN_2
#define LED_RUN_GPIO_Port GPIOC
#define MCU_DI1_Pin GPIO_PIN_6
#define MCU_DI1_GPIO_Port GPIOA
#define MCU_DI2_Pin GPIO_PIN_7
#define MCU_DI2_GPIO_Port GPIOA
#define MCU_DI3_Pin GPIO_PIN_4
#define MCU_DI3_GPIO_Port GPIOC
#define MCU_DI4_Pin GPIO_PIN_5
#define MCU_DI4_GPIO_Port GPIOC
#define MCU_DI5_Pin GPIO_PIN_0
#define MCU_DI5_GPIO_Port GPIOB
#define MCU_DI6_Pin GPIO_PIN_1
#define MCU_DI6_GPIO_Port GPIOB
#define MCU_DI7_Pin GPIO_PIN_2
#define MCU_DI7_GPIO_Port GPIOB
#define MCU_DI8_Pin GPIO_PIN_12
#define MCU_DI8_GPIO_Port GPIOB
#define MCU_DO1_Pin GPIO_PIN_13
#define MCU_DO1_GPIO_Port GPIOB
#define MCU_DO2_Pin GPIO_PIN_14
#define MCU_DO2_GPIO_Port GPIOB
#define MCU_DO3_Pin GPIO_PIN_15
#define MCU_DO3_GPIO_Port GPIOB
#define MCU_DO4_Pin GPIO_PIN_6
#define MCU_DO4_GPIO_Port GPIOC
#define MCU_DO5_Pin GPIO_PIN_7
#define MCU_DO5_GPIO_Port GPIOC
#define MCU_DO6_Pin GPIO_PIN_8
#define MCU_DO6_GPIO_Port GPIOC
#define MCU_DO7_Pin GPIO_PIN_9
#define MCU_DO7_GPIO_Port GPIOC
#define UART1_RE_Pin GPIO_PIN_8
#define UART1_RE_GPIO_Port GPIOA
#define RESET_ENC_Pin GPIO_PIN_12
#define RESET_ENC_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define MCU_DO8_Pin GPIO_PIN_6
#define MCU_DO8_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_8
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_9
#define SW3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
