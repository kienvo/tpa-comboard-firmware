/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   This file contains the headers of the modbus.h handlers.
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
#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* Exported types ------------------------------------------------------------*/
 typedef enum
 {
 	MBS_OK = 0x00,
 	MBS_ERROR = 0x01,
 	MBS_BUSY = 0x02,
 	MBS_TIMEOUT = 0x03,

 }Modbus_Status_TypeDef;//(MBS)

 typedef enum
 {
 	MODBUS_RTU = 0x00,
 	MODBUS_TCP = 0x01,
 }ModbusProtocol_TypeDef;

 typedef struct
 {
 	uint16_t	TransactionID;
 	uint16_t	ProtocolID;
 	uint16_t	Length;
 	uint8_t		UnitID;
 }MBAP_Header_TypeDef;
 typedef struct
 {
	uint16_t				TransactionID;
	uint16_t				ProtocolID;
	uint16_t				Length;
 	uint8_t					ID; // slave ID
 	uint8_t					Function; // function code
 	uint8_t 				Data[256]; // receive data
 	uint16_t				Crc; //CRC
 	MBAP_Header_TypeDef		MBAP_Header; // MBAP header use for modbus TCP
 	ModbusProtocol_TypeDef 	Protocol; // modbus RTU/TCP
 	volatile uint8_t		New;
 }ModbusHandle_TypeDef;

 typedef enum
 {
 	MB_RTU_RX_ID = 0,
 	MB_RTU_RX_FUNC,
 	MB_RTU_RX_ADDR_QUA,
 	MB_RTU_RX_BYTE_COUNT,
 	MB_RTU_RX_VALUE,
 	MB_RTU_RX_CRC_HI,
 	MB_RTU_RX_CRC_LO,
 }ModbusRTU_RX_State_TypeDef;
 /* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define RS485_RE_Port		GPIOA
#define RS485_RE_Pin		GPIO_PIN_8

#define __RS485_TX_Mode()		HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, GPIO_PIN_SET)
#define __RS485_RX_Mode()		HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, GPIO_PIN_RESET)
/* Exported define -----------------------------------------------------------*/
#define MB_SLAVE_ID	0x01
#define MB_MAX_QUANTITY_OF_COILS			8
#define MB_MAX_QUANTITY_OF_INPUT			8
#define MB_MAX_QUANTITY_OF_REG				2
#define MB_MAX_QUANTITY_OF_INPUT_REG		4
#define MB_MIN_QUANTITY						1

#define MB_STR_ADDR_COILS_VALID				7
#define MB_STR_ADDR_INPUT_VALID				7
#define MB_STR_ADDR_REG_VALID				1
#define MB_STR_ADDR_INPUT_REG_VALID			3

/* define modbus function support code */
#define MB_FUNC1_READ_COILS					0x01
#define MB_FUNC2_READ_INPUTS				0X02
#define MB_FUNC3_READ_HOLDING_REG			0x03
#define MB_FUNC4_READ_INPUT_REG				0x04
#define MB_FUNC5_WRITE_SINGLE_COIL			0x05
#define MB_FUNC6_WRITE_SINGLE_REG			0x06
#define MB_FUNC15_WRITE_MUL_COILS			0x0F
#define MB_FUNC16_WRITE_MUL_REG				0x10

/* define modbus exception response code*/
#define MB_EXCP_FUNC_CODE					0x80
#define MB_EXCP_ILLEGAL_FUNC				0x01 /* Function code nhận được trong truy vấn không được server công nhận hoặc cho phép */
#define MB_EXCP_ILLEGAL_DATA_ADDR			0x02 /* Địa chỉ data nhận được trong truy vấn là địa chỉ không được cho phép trên server. */
#define MB_EXCP_ILLEGAL_DATA_VAL			0x03 /* Giá trị trong trường data của query là giá trị không được server chấp nhận. */
#define MB_EXCP_SERV_DEVI_FAIL				0x04 /* Đã xảy ra lỗi không thể khôi phục trong khi server đang cố gắng thực hiện hành động được yêu cầu */
#define MB_EXCP_ACKNOWLEDGE					0x05 /* Server đã nhận được requets yêu cầu và nhưng cần một khoảng thời gian dài để xử lý request này. Response này được trả lại để ngăn lỗi timeout xảy ra cho client. */
#define MB_EXCP_SERV_DEVI_BUSY				0x06 /* Server đang xử lý một chương trình cần thời gian dài, client cần gởi lại request sau. */

/* Exported functions prototypes ---------------------------------------------*/
void modbus_init(ModbusProtocol_TypeDef protocol);
void modbus_checking_request(void);
void modbus_rtu_receive_irq(uint8_t data);
void modbus_tcp_parse_frame(uint8_t *frame, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif /* INC_MODBUS_H_ */

/*****************************END OF FILE**************************************/
