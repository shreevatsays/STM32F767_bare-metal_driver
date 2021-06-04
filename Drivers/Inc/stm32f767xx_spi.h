/*
 * stm32f767xx_spi.h
 *
 *  Created on: 04-Feb-2021
 *      Author: shree
 */

#ifndef INC_STM32F767XX_SPI_H_
#define INC_STM32F767XX_SPI_H_

#include "stm32f767xx.h"



#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1


#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

#define SPI_DATA_SIZE_8BITS				7
#define SPI_DATA_SIZE_16BITS			15


#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1


#define SPI_SSM_DIS						0
#define SPI_SSM_EN						1


#define SPI_TXE_FLAG					(1 << SPI_SR_BIT_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_BIT_RXNE)
#define SPI_BSY_FLAG					(1 << SPI_SR_BIT_BSY)

#define SPI_NSSP_EN						ENABLE
#define SPI_NSSP_DIS					DISABLE

#define SPI_FRXTH_HF					0
#define SPI_FRXTH_QF					(1 << SPI_CR2_BIT_FRXTH)


typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DataSize;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

	uint8_t SPI_NSSP;
	uint8_t SPI_FRXTH;

}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);


void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t Len);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);


#endif /* INC_STM32F767XX_SPI_H_ */
