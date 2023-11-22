/*
 * mcp2515_driver.h
 *
 *  Created on: Nov 21, 2023
 *      Author: yuta
 */

#ifndef USR_MCP2515_DRIVER_H_
#define USR_MCP2515_DRIVER_H_

#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"

#include "MCP2515.h"

#define SPI_TIMEOUT 1000


typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint16_t  	 GPIO_Pin;

	SPI_HandleTypeDef *hspi;
}MCP_HandleTypeDef;

typedef struct
{
	uint8_t mode;
	uint8_t syncJumpWidth;
	uint8_t baudRatePrescaler;
	uint8_t propSegment;
	uint8_t phaseSegment1;
	uint8_t phaseSegment2;
	uint8_t startOfFrame;
	uint8_t wakeUpFilter;
	uint8_t bitTimeLengthMode;
	uint8_t SAM;
	uint16_t filter[6];
	uint16_t mask[2];


}MCP_InitTypeDef;

typedef struct
{
	uint8_t DLC;
	uint16_t SID;
	uint16_t EID;
}MCP_TxHeaderTypeDef;

HAL_StatusTypeDef MCP_SPI_Transmit(MCP_HandleTypeDef *hMCP, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef MCP_SPI_Receive(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t *pTxData, uint16_t Size);

HAL_StatusTypeDef MCP_Write(MCP_HandleTypeDef *hMCP, uint8_t Address, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef MCP_Read(MCP_HandleTypeDef *hMCP, uint8_t addr, uint8_t *pRxData, uint8_t Size);

HAL_StatusTypeDef MCP_Read_RxBuffer(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t Size);
HAL_StatusTypeDef MCP_Write_TxBuffer(MCP_HandleTypeDef *hMCP, MCP_TxHeaderTypeDef *TxHeader, uint8_t *pTxData);

HAL_StatusTypeDef MCP_SetStdFilter(MCP_HandleTypeDef *hMCP, uint8_t FilterN, uint16_t ID);
HAL_StatusTypeDef MCP_SetStdMask(MCP_HandleTypeDef *hMCP, uint8_t   MaskN, uint16_t ID);
HAL_StatusTypeDef MCP_Config(MCP_HandleTypeDef *hMCP, MCP_InitTypeDef *iMCP);
HAL_StatusTypeDef MCP_Reset(MCP_HandleTypeDef *hMCP);

HAL_StatusTypeDef MCP_CAN_Transmit(MCP_HandleTypeDef *hMCP, );
HAL_StatusTypeDef MCP_CAN_Receive (MCP_HandleTypeDef );




#endif /* USR_MCP2515_DRIVER_H_ */
