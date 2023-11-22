/*
 * mcp2515_driver.c
 *
 *  Created on: Nov 21, 2023
 *      Author: yuta
 */
#include "MCP2515_driver.h"
#include "MCP2515.h"


HAL_StatusTypeDef MCP_Config(MCP_HandleTypeDef *hMCP, MCP_InitTypeDef *iMCP)
{
	
}


HAL_StatusTypeDef MCP_SPI_Transmit(MCP_HandleTypeDef *hMCP, uint8_t *pData, uint16_t Size)
{
	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, RESET)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_SPI_Transmit(hMCP->hspi, pData, Size, SPI_TIMEOUT)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, SET)){
		return HAL_ERROR;
	}
}


HAL_StatusTypeDef MCP_SPI_Receive(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t *pTxData, uint16_t Size)
{
	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, RESET)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_SPI_TransmitReceive(hMCP->hspi, TxData, pRxData, Size, SPI_TIMEOUT)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, SET)){
		return HAL_ERROR;
	}
}


HAL_StatusTypeDef MCP_Write(MCP_HandleTypeDef *hMCP, uint8_t Address, uint8_t *pTxData, uint16_t Size)
{
	uint8_t buff[Size+2] = {MCP_WRITE, Address};

	for(uint32_t i=2; i<Size+2; i++){
		buff[i] = pTxData[i-2];
	}


	return MCP_SPI_Transmit(hMCP, pTxData, Size+2);
}


HAL_StatusTypeDef MCP_Read(MCP_HandleTypeDef *hMCP, uint8_t addr, uint8_t *pRxData, uint8_t Size)
{
	uint8_t TxData[2] = {MCP_READ, addr};

	return MCP_SPI_Receive(hMCP, pRxData, TxData, sizeof(TxData,TxData[0]));
}


HAL_StatusTypeDef MCP_Reset(MCP_HandleTypeDef *hMCP)
{
	uint8_t TxData = MCP_RESET;

	return MCP_SPI_Transmit(hMCP, &TxData, 1);
}


HAL_StatusTypeDef MCP_Read_RxBuffer(MCP_HandleTypeDef *hMCP, uint8_t *pRxData, uint8_t Size)
{
	uint8_t TxData = MCP_READ_RX0;


	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, RESET)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_SPI_TransmitReceive(hMCP->hspi, &TxData, pRxData, Size, SPI_TIMEOUT)){
		return HAL_ERROR;
	}

	TxData = 0;
	return MCP_Write(hMCP, MCP_CANINTF, &TxData, 1);


}


HAL_StatusTypeDef MCP_Write_TxBuffer(MCP_HandleTypeDef *hMCP, MCP_TxHeaderTypeDef *TxHeader, uint8_t *pTxData)
{
	uint8_t a=0;
	uint8_t loadCommand[3] = {MCP_LOAD_TX0, MCP_LOAD_TX1, MCP_LOAD_TX2};
	uint8_t RTSCommand[4] = {MCP_RTS_TX0, MCP_RTS_TX1, MCP_RTS_TX2, MCP_RTS_ALL};


	uint8_t TxBuff[TxHeader->DLC + 6] = {};
	TxBuff[0] = loadCommand[a];
	TxBuff[1] = TxHeader->SID>>8;
	TxBuff[2] = TxHeader->SID & 0xff;
	TxBuff[3] = TxHeader->EID>>8;
	TxBuff[4] = TxHeader->EID & 0xff;
	TxBuff[5] = TxHeader->DLC;

	for(uint8_t i=0; i<TxHeader->DLC; i++){
		TxBuff[i+6] = pTxData[i];
	}

	if(HAL_OK != MCP_SPI_Transmit(hMCP, TxBuff, TxHeader->DLC + 6)){
		return HAL_ERROR;
	}

	return MCP_SPI_Transmit(hMCP, &RTSCommand[a]);
}
