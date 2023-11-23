/*
 * mcp2515_driver.c
 *
 *  Created on: Nov 21, 2023
 *      Author: yuta
 */
#include "MCP2515_driver.h"
#include "MCP2515.h"




HAL_StatusTypeDef MCP_SPI_Transmit(MCP_HandleTypeDef *hMCP, uint8_t *pData, uint16_t Size)
{
	if(HAL_OK != HAL_GPIO_WritePin(hMCP->GPIOx, hMCP->GPIO_Pin, RESET)){
		return HAL_ERROR;
	}
	if(HAL_OK != HAL_SPI_Transmit(hMCP->hspi, pData, Size, MCP_SPI_TIMEOUT)){
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
	if(HAL_OK != HAL_SPI_TransmitReceive(hMCP->hspi, TxData, pRxData, Size, MCP_SPI_TIMEOUT)){
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
	if(HAL_OK != HAL_SPI_TransmitReceive(hMCP->hspi, &TxData, pRxData, Size, MCP_SPI_TIMEOUT)){
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
	Split_uint16to2uint8_t(TxHeader->SID, &TxBuff[0], &TxBuff[1]);
	Split_uint16to2uint8_t(TxHeader->SID, &TxBuff[2], &TxBuff[3]);
	TxBuff[5] = TxHeader->DLC;


	for(uint8_t i=0; i<TxHeader->DLC; i++){
		TxBuff[i+6] = pTxData[i];
	}

	if(HAL_OK != MCP_SPI_Transmit(hMCP, TxBuff, TxHeader->DLC + 6)){
		return HAL_ERROR;
	}

	return MCP_SPI_Transmit(hMCP, &RTSCommand[a]);
}


void Split_uint16To2uint8(uint16_t buff, uint8_t *HByte, uint8_t *LByte){
	*HByte = trgbuff>>8;
	*LByte = trgbuf & 0xff;
}


HAL_StatusTypeDef MCP_Config(MCP_HandleTypeDef *hMCP, MCP_InitTypeDef *iMCP){
	uint8_t TxData[MCP_BFPCTRL - MCP_RXF0SIDH];  //13byte

	if(iMCP->phaseSegment1>=8 || iMCP->phaseSegment2>=8 || iMCP->phaseSegment2 == 0 ||iMCP->PropSegment>=8)return HAL_ERROR;

	//Set filter 0~2. Register 0b00000000~0b00001100
	for(uint8_t i=0; i<3; i++){
		Split_uint16To2uint8(iMCP->SFilter[i], TxData[4*i], TxData[4*i + 1]);
		Split_uint16To2uint8(iMCP->EFilter[i], TxData[4*i + 2], TxData[4*i + 3]);
	}

	//Configure RXnBF Pins setting.
	TxData[MCP_BFPCTRL] = iMCP->enableRX0IT?0x05:0x00|iMCP->enableRX1IT?0x0A:0x00;

	if(HAL_OK != MCP_Write(hMCP, MCP_RXF0SIDH, TxData, MCP_BFPCTRL - MCP_RXF0SIDH)){
		return HAL_ERROR;
	}


	//Set filter 3~5. Register 0b00010000~0b00011011
	for(uint8_t i=0; i<3; i++){
		Split_uint16To2uint8(iMCP->SFilter[i+3], TxData[4*i], TxData[4*i + 1]);
		Split_uint16To2uint8(iMCP->EFilter[i+3], TxData[4*i + 2], TxData[4*i + 3]);
	}

	if(HAL_OK != MCP_Write(hMCP, MCP_RXF0SIDH, TxData, MCP_RXF5EID0 - MCP_RXF3SIDH)){
		return HAL_ERROR;
	}

	// Set mask
	for(uint8_t i=0; i<2; i++){
		Split_uint16To2uint8(iMCP->SMask[i], TxData[4*i], TxData[4*i + 1]);
		Split_uint16To2uint8(iMCP->EMask[i], TxData[4*i + 2], TxData[4*i + 3]);
	}

	TxData[8] = iMCP->startOfFrame|iMCP->wakeUpFilter|iMCP->phaseSegment2;//MCP_CNF3
	TxData[9] = BTLMODE|iMCP->SAM|(iMCP->phaseSegment1<<3)|iMCP->propSegment;//MCP_CNF2
	TxData[10] = iMCP->syncJumpWidth|iMCP->baudRatePrescaler;//MCP_CNF1
	TxData[11] = iMCP->CANITEnable;//MCP_CANINTE

	if(HAL_OK != MCP_Write(hMCP, MCP_RXF0SIDH, TxData, MCP_CANINTE - MCP_RXM0SIDH)){
		return HAL_ERROR;
	}

	return HAL_OK;


}
