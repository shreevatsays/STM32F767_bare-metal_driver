/*
 * 003_spi_test.c
 *
 *  Created on: Feb 5, 2021
 *      Author: shree
 */


/*
 * 001_LED_Toggle.c
 *
 *  Created on: 01-Feb-2021
 *      Author: shree
 */

#include <string.h>

#include "stm32f767xx.h"
#include "stm32f767xx_gpio.h"
#include "stm32f767xx_spi.h"

/**SPI1 GPIO Configuration
 *	PA4	------>	SPI1_SS
 *	PA5	------> SPI1_SCK
 *	PA6	------> SPI1_MISO
 *	PA7	------> SPI1_MOSI
 *	ALT Function mode: 5
*/

SPI_Handle_t hSPI1;

void delay()
{
	uint32_t i=0;
	for(i=0; i<500000; i++)
	{}
}

void SPI1_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	//SPI - SCK
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&SPIPins);

	//SPI - MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&SPIPins);

	//SPI - MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&SPIPins);

	//SPI - SS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
	GPIO_Init(&SPIPins);

}

void SPI1_Init(void)
{


	hSPI1.pSPIx = SPI1;
	hSPI1.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	hSPI1.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	hSPI1.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	hSPI1.SPIConfig.SPI_DataSize = SPI_DATA_SIZE_8BITS;
	hSPI1.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	hSPI1.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	hSPI1.SPIConfig.SPI_SSM = SPI_SSM_EN;

	hSPI1.SPIConfig.SPI_NSSP = SPI_NSSP_EN;


	SPI_Init(&hSPI1);
}

int main()
{
	char userData[] = "Hello World!!!";

	//1. Configure GPIO pins as SPI pins
	SPI1_GPIOInit();

	//2. SPI peripheral init
	SPI1_Init();

	// this makes nss signal internally high and avoides MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	//3.Enable the SPI1 peripheral
	SPI_PeripheralControl(hSPI1.pSPIx, ENABLE);



	//SPI_SendData(hSPI1.pSPIx, (uint8_t*)userData, strlen(userData));


	//GPIO_PCLKControl(GPIOB, ENABLE);
	//GPIO_Init(&GpioLED);

	//SS idle state - HIGH
	GPIO_WriteToOutputPin(GPIOA, 4, 1);

	while(1)
	{
		GPIO_WriteToOutputPin(GPIOA, 4, 0);
		SPI_SendData(hSPI1.pSPIx, (uint8_t*)userData, strlen(userData));
		GPIO_WriteToOutputPin(GPIOA, 4, 1);
		delay();

	}

	return 0;
}


