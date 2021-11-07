/**
  ******************************************************************************
  * @file    modbus.c
  * @brief   This file provides code for the configuration
  * 	     of the modbus instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "modbus.h"
#include "usart.h"
#include "led.h"
#include "stdio.h"
#include "math.h"
/* private defines -----------------------------------------------------------*/
#define MB_huart	huart1

#define MAX_AI_VOLT	10000 // 10V
/* private macro -------------------------------------------------------------*/
#define __RS485_ENABLE_IT()	__HAL_UART_ENABLE_IT(&MB_huart, UART_IT_RXNE)

extern const float AI_SCALE;
extern const float AO_SCALE;
#define CONVERT_AI_TO_AD(__VAL__)	round((__VAL__) * AI_SCALE)
#define CONVERT_AD_TO_AI(__VAL__)	round((__VAL__) / AI_SCALE)

#define CONVERT_AO_TO_AD(__VAL__)	round((__VAL__) / AO_SCALE)
#define CONVERT_AD_TO_AO(__VAL__)	round((__VAL__) * AO_SCALE)
/* private types -------------------------------------------------------------*/

/* private variables----------------------------------------------------------*/
ModbusHandle_TypeDef modbus;
static uint8_t rtu_rx_state;
static uint8_t rtu_rx_count;


/* private function prototypes -----------------------------------------------*/
static uint16_t modbus_CRC16(uint8_t *pBuffer, uint8_t len)
{
	uint16_t crc = 0xFFFF;
	uint8_t pos,i;

	for(pos = 0; pos < len; pos++)
	{
		crc = crc ^ pBuffer[pos]; // XOR byte into least sig. byte of crc
		for(i = 0; i < 8; i++) // Loop over each bit
		{
			if((crc & 0x0001) !=0) // If the LSB is set
			{
				crc = crc >> 1;
				crc = crc ^ 0xA001; // Shift right and XOR 0xA001
			}
			else // Else LSB is not set
			{
				crc = crc >> 1; // Just shift right
			}
		}
	}
	//return ((crc << 8) & 0xFF00) | ((crc >> 8) & 0x00FF);
	return crc;
}
static void modbus_clear_buffer(void)
{
	uint8_t i;
	modbus.ID = 0;
	modbus.Funtion = 0;
	modbus.New = 0;
	modbus.Lenght = 0;
	for(i = 0; i < 50; i++) {
		modbus.Data[i] = 0;
	}

}

void modbus_rtu_receive_irq(uint8_t data)
{
//	printf("%.2X\n", data);
	switch (rtu_rx_state) {
		case MB_RTU_RX_ID:
//			if(data != MB_SLAVE_ID) break;
			modbus.ID = data;
			rtu_rx_state = MB_RTU_RX_FUNC;
			break;
		case MB_RTU_RX_FUNC:
			modbus.Funtion = data;
			rtu_rx_count = 0;
			rtu_rx_state = MB_RTU_RX_ADDR_QUA;
			break;
		case MB_RTU_RX_ADDR_QUA:
			modbus.Data[rtu_rx_count++] = data;
			if(rtu_rx_count >= 4) {
				if(modbus.Funtion < 7) {
					rtu_rx_state = MB_RTU_RX_CRC_HI;
					modbus.Lenght = rtu_rx_count;
					break;
				}
				rtu_rx_state = MB_RTU_RX_BYTE_COUNT;
			}
			break;
		case MB_RTU_RX_BYTE_COUNT:
			modbus.Data[rtu_rx_count++] = data;
			modbus.Lenght = rtu_rx_count + data;
			rtu_rx_state = MB_RTU_RX_VALUE;
			break;
		case MB_RTU_RX_VALUE:
			modbus.Data[rtu_rx_count++] = data;
			if(rtu_rx_count >= modbus.Lenght) {
				rtu_rx_state = MB_RTU_RX_CRC_HI;
			}
			break;
		case MB_RTU_RX_CRC_HI:
//			modbus.Crc = (data << 8) & 0xFF00;
			modbus.Crc = data & 0x00FF;
			rtu_rx_state = MB_RTU_RX_CRC_LO;
			break;
		case MB_RTU_RX_CRC_LO:
			modbus.Crc |= (data << 8);
			modbus.New = 1;
			rtu_rx_state = MB_RTU_RX_ID;
//			printf("RX new frame[%.2X %.2X %.4X]\n", Modbus.ID, Modbus.Funtion, Modbus.Crc);
			break;
		default:
			rtu_rx_state = MB_RTU_RX_ID;
			break;
	}
}

static void modbus_response(ModbusHandle_TypeDef *hmodbus)
{
	uint8_t* ptxdata;
	uint8_t txlen;
//	uint8_t crcs[2];
	if(hmodbus->Protocol == MODBUS_RTU){//RTU
		ptxdata = &hmodbus->ID;
		txlen = hmodbus->Lenght + 2;
		hmodbus->Crc = modbus_CRC16(ptxdata, txlen);
//		crcs[0] = hmodbus->Crc
		__RS485_TX_Mode();
		HAL_UART_Transmit(&MB_huart, ptxdata, txlen, 0xFFFFFFFF);
		HAL_UART_Transmit(&MB_huart, (uint8_t*)&hmodbus->Crc, 2, 0xFFFFFFFF);
		__RS485_RX_Mode();
	}
	else//TCP
	{

	}
}

static void modbus_response_exception(ModbusHandle_TypeDef *hmodbus, uint8_t exceptionCode)
{
	hmodbus->Funtion = hmodbus->Funtion | MB_EXCP_FUNC_CODE;
	hmodbus->Data[0] = exceptionCode;
	hmodbus->Lenght = 1;

	modbus_response(hmodbus);
}

//Function 1 read coils
static void modbus_processing_function_1(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_COILS) {
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if((startAddr + quantity - 1) > MB_STR_ADDR_COILS_VALID) {
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}
	uint8_t i;
	modbus.Data[0] = 1; //byte count
	modbus.Data[1] = 0x00;//status coils
	for(i = 0; i < quantity; i++) {
//		resData[1] |= outputCoils[startAddr + i] << i;
		modbus.Data[1] |= digital_read(startAddr + i, 1) << i;
	}
	modbus.Lenght = 2;
	modbus_response(&modbus);
}

//Function 2: read input
static void modbus_processing_function_2(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_INPUT)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if((startAddr + quantity - 1) > MB_STR_ADDR_INPUT_VALID)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}
	uint8_t i;
	modbus.Data[0] = 1; //byte count
	modbus.Data[1] = 0x00;//status coils
	for(i = 0; i < quantity; i++)
	{
//		resData[1] |= descreteInput[startAddr + i] << i;
		modbus.Data[1] |= digital_read(startAddr + i, 0) << i;
	}
	modbus.Lenght = 2;
	modbus_response(&modbus);
}

//Function 3: read registers
static void modbus_processing_function_3(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_REG)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if((startAddr + quantity - 1) > MB_STR_ADDR_REG_VALID)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}

	uint8_t i;
	uint8_t* dataVal = &modbus.Data[1];
	uint16_t anal_value = 0;
	uint16_t vout = 0;
	modbus.Data[0] = 2 * quantity; //byte count
	for(i = 0; i < quantity; i++) {
		if(analog_read(startAddr + i, &anal_value, 1) != HAL_OK)
		{
			modbus_response_exception(&modbus, MB_EXCP_SERV_DEVI_FAIL);
			return;
		}
		vout = CONVERT_AD_TO_AO(anal_value);
		dataVal[i * 2] = vout >> 8;
		dataVal[i * 2 + 1] = vout & 0xFF;
	}
	modbus.Lenght = modbus.Data[0] + 1;
	modbus_response(&modbus);
}

//Function 4: read input registers
static void modbus_processing_function_4(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_INPUT_REG)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if((startAddr + quantity - 1) > MB_STR_ADDR_INPUT_REG_VALID)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}
	uint8_t i;
	uint8_t* dataVal = &modbus.Data[1];
	uint16_t anal_value = 0;
	uint16_t vout = 0;
	modbus.Data[0] = 2 * quantity; //byte count
	for(i = 0; i < quantity; i++) {
		if(analog_read(startAddr + i, &anal_value, 0) != HAL_OK)
		{
			modbus_response_exception(&modbus, MB_EXCP_SERV_DEVI_FAIL);
			return;
		}
		vout = CONVERT_AD_TO_AI(anal_value);
		dataVal[i * 2] = vout >> 8;
		dataVal[i * 2 + 1] = vout & 0xFF;
	}
	modbus.Lenght = modbus.Data[0] + 1;
	modbus_response(&modbus);
}

//Function 5: write single coil
static void modbus_processing_function_5(void)
{
	uint16_t value = (modbus.Data[2] << 8) | modbus.Data[3];
	if(value != 0 && value != 0xFF00)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if(startAddr > MB_STR_ADDR_COILS_VALID) {
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}

	if(value == 0) {
		digital_write(startAddr, 0);
	}
	else {
		digital_write(startAddr, 1);
	}
	modbus.Lenght = 4;
	modbus_response(&modbus);
}

//Function 6: write single register
static void modbus_processing_function_6(void)
{
	uint16_t value = (modbus.Data[2] << 8) | modbus.Data[3];
	if(value > MAX_AI_VOLT)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if(startAddr > MB_STR_ADDR_REG_VALID)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}


	uint16_t vint = CONVERT_AO_TO_AD(value);
//	printf("set V DAC = %d\n", vint);
	if(analog_write(startAddr, vint) != HAL_OK)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}
	modbus.Lenght = 4;
	modbus_response(&modbus);
}

//Function 15: write multi coils
static void modbus_processing_function_15(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	uint8_t byteCount = modbus.Data[4];
	uint8_t nByte = 1;// quantity / 8;

	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_COILS || byteCount != nByte)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if(startAddr > MB_STR_ADDR_COILS_VALID)
	{
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}

	uint8_t i;
	uint8_t state = modbus.Data[5];
	for(i = 0; i < quantity; i++) {
		digital_write(startAddr + i, (state >> i) & 0x01);
	}

	modbus.Lenght = 4;
	modbus_response(&modbus);
}

//Function 16: write multi registers
static void modbus_processing_function_16(void)
{
	uint16_t quantity = (modbus.Data[2] << 8) | modbus.Data[3];
	uint8_t byteCount = modbus.Data[4];
	uint8_t nByte = quantity * 2;

	if(quantity < MB_MIN_QUANTITY || quantity > MB_MAX_QUANTITY_OF_REG || byteCount != nByte) {
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_VAL);
		return;
	}

	uint16_t startAddr = (modbus.Data[0] << 8) | modbus.Data[1];
	if(startAddr > MB_STR_ADDR_REG_VALID) {
		modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_DATA_ADDR);
		return;
	}

	uint8_t i;
	uint8_t *value = &modbus.Data[5];
	uint16_t val_to_write;
	uint16_t vint;
	for(i = 0; i < quantity; i++) {
		val_to_write = (value[i * 2] << 8) | value[2 * i + 1];
		vint = CONVERT_AO_TO_AD(val_to_write);
		if(analog_write(startAddr + i, vint) != HAL_OK)
		{
			modbus_response_exception(&modbus, MB_EXCP_SERV_DEVI_FAIL);
			return;
		}
	}

	modbus.Lenght = 4;
	modbus_response(&modbus);
}
void modbus_checking_request(void)
{
	if(modbus.New == 0) return;

	if(modbus.Protocol == MODBUS_RTU)
	{
		if(modbus.ID != MB_SLAVE_ID) // frame not addressed
		{
			printf("wrong id\n");
			modbus_clear_buffer();
			return;
		}
		if(modbus_CRC16(&modbus.ID, modbus.Lenght + 2) != modbus.Crc) // error crc
		{
			printf("wrong crc\n");
			modbus_clear_buffer();
			return;
		}
	}
	LED_COM_ON();
	switch (modbus.Funtion) {
		case MB_FUNC1_READ_COILS:
			modbus_processing_function_1();
			break;
		case MB_FUNC2_READ_INPUTS:
			modbus_processing_function_2();
			break;
		case MB_FUNC3_READ_HOLDING_REG:
			modbus_processing_function_3();
			break;
		case MB_FUNC4_READ_INPUT_REG:
			modbus_processing_function_4();
			break;
		case MB_FUNC5_WRITE_SINGLE_COIL:
			modbus_processing_function_5();
			break;
		case MB_FUNC6_WRITE_SINGLE_REG:
			modbus_processing_function_6();
			break;
		case MB_FUNC15_WRITE_MUL_COILS:
			modbus_processing_function_15();
			break;
		case MB_FUNC16_WRITE_MUL_REG:
			modbus_processing_function_16();
			break;
		default: //exception function code not supported
			modbus_response_exception(&modbus, MB_EXCP_ILLEGAL_FUNC);
			break;
	}

	modbus_clear_buffer();
}
void modbus_init(ModbusProtocol_TypeDef protocol)
{
	modbus_clear_buffer();

	if(protocol == MODBUS_RTU)
	{
		rtu_rx_count = 0;
		rtu_rx_state = 0;

		__RS485_ENABLE_IT();
		__RS485_RX_Mode();
	}

}
/*****************************End Of File**************************************/
