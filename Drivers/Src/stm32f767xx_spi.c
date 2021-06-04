/*
 * stm32f767xx_spi.c
 *
 *  Created on: 04-Feb-2021
 *      Author: shree
 */


#include "stm32f767xx_spi.h"


void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS ==  ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if(pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}
	}
	else if(ENorDIS ==  DISABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DIS();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DIS();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DIS();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DIS();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DIS();
		}
		else if(pSPIx == SPI6)
		{
			SPI6_PCLK_DIS();
		}
	}
}




void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp=0;
	uint32_t frxth;

	//Enable Peripheral clock
	SPI_PCLKControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure Device Mode
	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);

	//2. Configure the BUS Config
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		temp &= ~(1 << 15);

	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		temp |= (1 << 15);
	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		temp &= ~(1 << 15);

		//RXONLY bit must be set
		temp |= (1 << 10);
	}


	//3. Configure SPI Serial CLock
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BIT_BR);

	//4. Configure CPOL
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_BIT_CPOL);

	//5. Configure CPHA
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_BIT_CPHA);

	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_BIT_SSM);


	pSPIHandle->pSPIx->CR1 |= temp;


	temp = 0;
	//6. Configure DataSize
	temp |= (pSPIHandle->SPIConfig.SPI_DataSize << SPI_CR2_BIT_DS);


	  /* Align by default the rs fifo threshold on the data size */
	  if (pSPIHandle->SPIConfig.SPI_DataSize > SPI_DATA_SIZE_8BITS)
	  {
	    frxth = SPI_FRXTH_HF;
	  }
	  else
	  {
	    frxth = SPI_FRXTH_QF;
	  }

	  temp |= frxth;

	  temp |= (pSPIHandle->SPIConfig.SPI_NSSP << SPI_CR2_BIT_NSSP);

	pSPIHandle->pSPIx->CR2 |= temp;


}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuff, uint32_t Len)
{
	//Transmit until len is 0
	while(Len > 0)
	{
		//1. Check TXE is complete (wait until TXE is set)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//16-Bit DataSize Format
//		if(pSPIx->CR2 & ( SPI_DATA_SIZE_8BITS << SPI_CR2_BIT_DS))
//		{
//			//Load the data into DR register
//			pSPIx->pSPIx->DR = *((uint16_t*)pTxBuff);
//			Len -= 2;
//			pTxBuff += 2;
//		}
//		else
		{
			//Load the data into DR register
			pSPIx->DR = *pTxBuff;
			Len--;
			pTxBuff++;

		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuff, uint32_t Len)
{}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS ==  ENABLE)
	{

		pSPIx->CR1 |= (1 << SPI_CR1_BIT_SPE);
	}
	else if(ENorDIS ==  DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_BIT_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS ==  ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_BIT_SSI);
	}
	else if(ENorDIS ==  DISABLE)
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_BIT_SSI);
	}
}

