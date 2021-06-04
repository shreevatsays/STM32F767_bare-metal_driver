/*
 * 004_usart_test.c
 *
 *  Created on: Feb 6, 2021
 *      Author: shree
 */


#include <string.h>

#include "stm32f767xx.h"
#include "stm32f767xx_gpio.h"
#include "stm32f767xx_usart.h"


/**USART6 GPIO Configuration
PB12     ------> USART5_RX
PB13    ------> USART5_TX
*/

USART_Handle_t hUSART5;

char myData[] = "Hello World";

void delay(void)
{
	uint32_t i=0;
	for(i=0; i<500000; i++)
	{}
}


void USART5_GPIOInit(void)
{
	GPIO_Handle_t USART5Pins;

	//USART5 - RX
	USART5Pins.pGPIOx = GPIOB;
	USART5Pins.GPIO_PinConfig.GPIO_PinNumber = 12;
	USART5Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART5Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 8;
	USART5Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART5Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	USART5Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&USART5Pins);

	//USART5 - TX
	USART5Pins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&USART5Pins);

}

void USART5_Init(void)
{

	hUSART5.pUSARTx = UART5;
	hUSART5.USART_Config.USART_Mode = USART_MODE_TXRX;
	hUSART5.USART_Config.USART_BaudRate = USART_STD_BAUD_115200;
	hUSART5.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	hUSART5.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	hUSART5.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	hUSART5.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	hUSART5.USART_Config.USART_OverSampling = USART_OVER_SAMPLING_8;

	USART_Init(&hUSART5);

}


int main()
{

	USART5_GPIOInit();

	USART5_Init();

	//3.Enable the USART6 peripheral
	USART_PeripheralControl(hUSART5.pUSARTx, ENABLE);

	while(1)
	{
		USART_SendData(&hUSART5, (uint8_t*)myData, strlen(myData));
		delay();
	}

	return 0;
}
