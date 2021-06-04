/*
 * 001_LED_Toggle.c
 *
 *  Created on: 01-Feb-2021
 *      Author: shree
 */


#include "stm32f767xx.h"
#include "stm32f767xx_gpio.h"

void delay()
{
	uint32_t i=0;
	for(i=0; i<500000; i++)
	{}
}

int main()
{
	GPIO_Handle_t GpioLED, GpioUserButton;

	GpioLED.pGPIOx = GPIOB;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = 7;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PCLKControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLED);

	while(1)
	{
		GPIO_TogglePin(GPIOB, 7);
		delay();
	}

	return 0;
}
