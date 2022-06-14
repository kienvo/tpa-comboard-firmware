/**
  ******************************************************************************
  * @file    led.c
  * @brief   This file provides code for the configuration
  * 	     of the led instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* private macro -------------------------------------------------------------*/

/* private defines -----------------------------------------------------------*/
#define LED_RUN_CYCLE		20 // 40 * 50(timer) = 2000ms = 2s
#define LED_RUN_TOGGLE_CYCLE	1 // 3 * 50 = 150ms
/* private types -------------------------------------------------------------*/

/* private variables----------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;

uint16_t led_timer_count;
uint8_t num_toggle;
uint8_t  toggle_count;
uint16_t led_run_off_cycle;
/* private function prototypes -----------------------------------------------*/
void led_init(uint8_t mode)
{
	if(mode > 3)
	{
		mode = 3;
	}
	num_toggle = mode * 2;
	toggle_count = 0;
	led_timer_count = 0;
	led_run_off_cycle = LED_RUN_CYCLE - (LED_RUN_TOGGLE_CYCLE * num_toggle);
	HAL_TIM_Base_Start_IT(&htim4);
}
void led_irq_handler(void)
{
	if(IS_LED_COM_ON())//if led communication on -> off
	{
		LED_COM_OFF();
	}

	led_timer_count++;
	if((led_timer_count > LED_RUN_TOGGLE_CYCLE) && (toggle_count < num_toggle))
	{
		LED_RUN_TOGGLE();
		led_timer_count = 0;
		toggle_count++;
	}

	if(led_timer_count > led_run_off_cycle)
	{
		led_timer_count = 0;
		toggle_count = 0;
	}
}
/*****************************End Of File**************************************/
