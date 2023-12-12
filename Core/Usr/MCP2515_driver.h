/*
 * mcp2515_driver.h
 *
 *  Created on: Nov 21, 2023
 *      Author: yuta
 */

#ifndef USR_MCP2515_DRIVER_H_
#define USR_MCP2515_DRIVER_H_

#include "main.h"

#include "MCP2515.h"

#define MCP_SPI_TIMEOUT 1000
#define MCP_TRUE 1
#define MCP_FALSE 0


typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint16_t  	 GPIO_Pin;

	SPI_HandleTypeDef *hspi;
}MCP_HandleTypeDef;

typedef struct
{
	//For CANCTRL Register
	uint8_t mode; //MODE_NORMAL, MODE_SLEEP, MODE_LOOPBACK, MODE_LISTENONLY, MODE_CONFIG or MODE_POWERUP
	uint8_t oneShotMode ; //MODE_ONESHOT or MCP_FALSE
	uint8_t clockPrescaler; //CLKOUT_PS1, CLKOUT_PS2, CLKOUT_PS4 or CLKOUT_PS8(0,1,2,3)
	uint8_t clockEnable; //CLKOUT_ENABLE or CLKOUT_DISABLE
	uint8_t abort; //ABORT_TX or MCP_FALSE


	uint8_t syncJumpWidth;//SJW1, SJW2, SJW3 or SJW4
	uint8_t baudRatePrescaler;
	uint8_t propSegment;//must be 0~7. Length of propSegment1(1Tq~8Tq).
	uint8_t phaseSegment1;//must be 0~7. Length of PhaseSegment1(1Tq~8Tq).
	uint8_t phaseSegment2;//must be 1~7. Length of PhaseSegment2(2Tq~8Tq).
	uint8_t startOfFrame;//SOF_ENABLE or SOF_DISABLE
	uint8_t wakeUpFilter;//WAKFIL_ENABLE or WAKFIL_DISABLE
	uint8_t bitTimeLengthMode;
	uint8_t sample;


	uint16_t SFilter[6];
	uint16_t EFilter[6];
	uint16_t SMask[2];
	uint16_t EMask[2];
	uint8_t enableRX0IT;//MCP_TRUE/FALSE
	uint8_t enableRX1IT;//MCP_TRUE/FALSE
	uint8_t CANITEnable;


}MCP_InitTypeDef;

typedef struct
{
	uint8_t DLC;
	uint16_t SID;
	uint16_t EID;
}MCP_TxHeaderTypeDef;

void __USR_Split_uint16to2uint8_t(uint16_t, uint8_t*, uint8_t*);

HAL_StatusTypeDef MCP_SPI_Transmit(MCP_HandleTypeDef *hMCP, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef MCP_SPI_Receive(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t *pTxData, uint16_t Size);

HAL_StatusTypeDef MCP_Write(MCP_HandleTypeDef *hMCP, uint8_t Address, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef MCP_Read(MCP_HandleTypeDef *hMCP, uint8_t addr, uint8_t *pRxData, uint8_t Size);

HAL_StatusTypeDef MCP_Read_RxBuffer(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t Size);
HAL_StatusTypeDef MCP_Write_TxBuffer(MCP_HandleTypeDef *hMCP, MCP_TxHeaderTypeDef *TxHeader, uint8_t *pTxData);

HAL_StatusTypeDef MCP_Config(MCP_HandleTypeDef *hMCP, MCP_InitTypeDef *iMCP);
HAL_StatusTypeDef MCP_Reset(MCP_HandleTypeDef *hMCP);
HAL_StatusTypeDef MCP_Start(MCP_HandleTypeDef *hMCP, MCP_InitTypeDef *iMCP);



#endif /* USR_MCP2515_DRIVER_H_ */
