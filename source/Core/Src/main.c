/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "modbus.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AD_MAX_VALUE	4095 //12bit adc
#define VREF			3300 // Vref = 3.3V

#define PROTOCOL_MODBUS_RTU		1
#define PROTOCOL_MODBUS_TCP		2
#define PROTOCOL_TCP_IP			3

#define RI1		1//Kohm
#define RI2		0.47//Kohm
#define RO1		20//Kohm
#define RO2		10//Kohm
#define CALIB_VALUE	1.607
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CONVERT_ANAL_TO_DIGIT(__VAL__)		round(((__VAL__) * AD_MAX_VALUE) / VREF) // convert analog (mV) to digital (12bit value): result u16
#define CONVERT_DIGIT_TO_ANAL(__VAL__)		round(((__VAL__) * VREF) / AD_MAX_VALUE ) // convert digital (12bit value) to analog (mV): result float
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const float AI_SCALE = (float)RI2 / (float)RI1;
const float AO_SCALE = (float)1 + ((float)RO1 / (float)RO2);

static uint8_t protocol;

uint16_t digit_output_pins[8] = {
		MCU_DO1_Pin, MCU_DO2_Pin, MCU_DO3_Pin, MCU_DO4_Pin,
		MCU_DO5_Pin, MCU_DO6_Pin, MCU_DO7_Pin, MCU_DO8_Pin};
GPIO_TypeDef* digit_output_ports[8] = {
		MCU_DO1_GPIO_Port, MCU_DO2_GPIO_Port, MCU_DO3_GPIO_Port, MCU_DO4_GPIO_Port,
		MCU_DO5_GPIO_Port, MCU_DO6_GPIO_Port, MCU_DO7_GPIO_Port, MCU_DO8_GPIO_Port};

uint16_t digit_input_pins[8] = {
		MCU_DI1_Pin, MCU_DI2_Pin, MCU_DI3_Pin, MCU_DI4_Pin,
		MCU_DI5_Pin, MCU_DI6_Pin, MCU_DI7_Pin, MCU_DI8_Pin};
GPIO_TypeDef* digit_input_ports[8] = {
		MCU_DI1_GPIO_Port, MCU_DI2_GPIO_Port, MCU_DI3_GPIO_Port, MCU_DI4_GPIO_Port,
		MCU_DI5_GPIO_Port, MCU_DI6_GPIO_Port, MCU_DI7_GPIO_Port, MCU_DI8_GPIO_Port};

uint32_t analog_output_channels[2] = {DAC_CHANNEL_1, DAC_CHANNEL_2};
uint32_t analog_input_channels[4] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void analog_init(void);
uint8_t selected_protocol(void);
void protocol_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//to use printf function
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart3,(uint8_t *)ptr,len,0xFFFFFFFF);
  return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  protocol_init();
  led_init(protocol);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(protocol == PROTOCOL_MODBUS_RTU)
	  {
		  modbus_checking_request();
	  }
	  HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void protocol_init(void)
{
	protocol = selected_protocol();
	if(protocol == PROTOCOL_MODBUS_RTU)
	{
		//init modbus rtu
		modbus_init(MODBUS_RTU);
	}
	else if(protocol == PROTOCOL_MODBUS_TCP)
	{
		//init modbus tcp

	}
	else if(protocol == PROTOCOL_TCP_IP)
	{
		//inti tcp/ip
	}
}

void analog_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
}

/*mode = 0 read digital input, mode != 0 read digital output*/
uint8_t digital_read(uint16_t channel, uint8_t mode)
{
	if(mode == 0)
	{
		return HAL_GPIO_ReadPin(digit_input_ports[channel], digit_input_pins[channel]);
	}
	return HAL_GPIO_ReadPin(digit_output_ports[channel], digit_output_pins[channel]);
}

void digital_write(uint16_t channel, uint8_t pinstate)
{
	HAL_GPIO_WritePin(digit_output_ports[channel], digit_output_pins[channel], pinstate);
}

/*mode = 0 read analog input, mode != 0 read analog output*/
uint8_t analog_read(uint16_t channel, uint16_t *pvalue, uint8_t mode)
{
	uint32_t values = 0;
	if(mode != 0)//doc gia tri set DAC
	{
		values = HAL_DAC_GetValue(&hdac, analog_output_channels[channel]);
		*pvalue = CONVERT_DIGIT_TO_ANAL(values);
		return HAL_OK;
	}
	// doc gia tri analog vao
	uint8_t i;
	HAL_StatusTypeDef status;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = analog_input_channels[channel];
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if (status != HAL_OK)
	{
		return status;
	}

	for(i = 0; i < 10; i++)
	{
		HAL_ADC_Start(&hadc1);
		status = HAL_ADC_PollForConversion(&hadc1, 10);
		if(status != HAL_OK)
		{
			return status;
		}
		values += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	values = (values / 10) & 0xFFFF;//tinh trung binh
	*pvalue = CONVERT_DIGIT_TO_ANAL(values) * CALIB_VALUE;
	return status;
}

uint8_t analog_write(uint16_t channel, uint16_t volt)
{
	if(volt > VREF)
	{
		volt = VREF;
	}
	uint16_t value = CONVERT_ANAL_TO_DIGIT(volt);
	return HAL_DAC_SetValue(&hdac, analog_output_channels[channel], DAC_ALIGN_12B_R, value);
}
/**
 * sw1		sw2		protocol
 *  0		 x		modbus rtu
 *  1		 0		modbus tcp
 *  1		 1		tcp/ip
 */

uint8_t selected_protocol(void)
{
	uint8_t state;

	state = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
	if(state == 0)// select modbus rtu
	{
		return PROTOCOL_MODBUS_RTU;
	}
	state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
	if(state == 0)// select modbus tcp
	{
		return PROTOCOL_MODBUS_TCP;
	}
	else // select tcp/ip
	{
		return PROTOCOL_TCP_IP;
	}

}

void delay_us(uint32_t delays)
{
	uint8_t i;
	while((delays--) > 0)
	{
		for(i = 0; i < 72; i++);

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
