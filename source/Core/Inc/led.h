/**
  ******************************************************************************
  * @file    led.h
  * @brief   This file contains the headers of the led.h handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 LE VU PHONG.
  * </center></h2>
  *
  *
 ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_LED_H_
#define INC_LED_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define LED_RUN_ON() 	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, 1)
#define LED_RUN_OFF() 	HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, 0)
#define LED_RUN_TOGGLE() 	HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin)

#define LED_COM_ON() 	HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, 1)
#define LED_COM_OFF() 	HAL_GPIO_WritePin(LED_COMM_GPIO_Port, LED_COMM_Pin, 0)
#define IS_LED_COM_ON()	HAL_GPIO_ReadPin(LED_COMM_GPIO_Port, LED_COMM_Pin)


#define LED_ERR_ON() 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, 1)
#define LED_ERR_OFF() 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, 0)
#define LED_ERR_TOGGLE() 	HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin)
/* Exported functions prototypes ---------------------------------------------*/
 void led_init(uint8_t mode);
 void led_irq_handler(void);
#ifdef __cplusplus
}
#endif

#endif /* INC_LED_H_ */

/*****************************END OF FILE**************************************/
