/*
 * stm32f767xx_usart.c
 *
 *  Created on: 03-Feb-2021
 *      Author: shree
 */


#include "stm32f767xx_usart.h"
#include "stm32f767xx_rcc.h"





void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t cmd)
{
	if(cmd == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE );
	}
}


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else if(pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}
		else if(pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	}
	else
	{

	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->ISR & FlagName)
	{
		return SET;
	}

	return RESET;
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }
  else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }
  else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}


void USART_Init(USART_Handle_t *pUSARTHandle)
{

	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
		tempreg &= ~(1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );
		tempreg &= ~( 1 << USART_CR1_RE);

	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //configure the Word length configuration item
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		tempreg &= ~( 1 << USART_CR1_M0 );
		tempreg &= ~( 1 << USART_CR1_M1 );
	}
	else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		tempreg &= ~( 1 << USART_CR1_M0 );
		tempreg |= ( 1 << USART_CR1_M1 );
	}
	else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
	{
		tempreg |= ( 1 << USART_CR1_M0 );
		tempreg &= ~( 1 << USART_CR1_M1 );
	}


    //Configuration of parity control bit fields
	if( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		tempreg |= ( 0 << USART_CR1_PCE);
	}
	else if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);
		tempreg &= ~( 1 << USART_CR1_PS);

	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE)
	{
		tempreg &= ~( 1 << USART_CR3_CTSE);
		tempreg &= ~( 1 << USART_CR3_RTSE);
	}
	else if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg &= ~( 1 << USART_CR3_RTSE);

	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		tempreg &= ~( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	// configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_BaudRate);

}

void USART_SendData(USART_Handle_t *hUSARTx,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(hUSARTx->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(hUSARTx->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			hUSARTx->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(hUSARTx->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			//hUSARTx->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);
			hUSARTx->pUSARTx->TDR = (uint8_t)(*pTxBuffer & 0xFFU);
			//Implement the code to increment the buffer address
			pTxBuffer++;
		}

	}


	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(hUSARTx->pUSARTx,USART_FLAG_TC));

}
