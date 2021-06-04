/*
 * stm32f767xx.h
 *
 *  Created on: Jan 28, 2021
 *      Author: shree
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include "stdint.h"

#define FLASH_BASEADD			0x08000000U
#define SRAM1_BASEADD			0x20020000U


#define PERIPHERAL_BASE			0x40000000U
#define APB1_BASE				PERIPHERAL_BASE
#define APB2_BASE				0x40010000U
#define AHB1_BASE				0x40020000U
#define AHB2_BASE				0x50000000U


#define GPIOA_BASEADD			(AHB1_BASE + 0x0000U)
#define GPIOB_BASEADD			(AHB1_BASE + 0x0400U)
#define GPIOC_BASEADD			(AHB1_BASE + 0x0800U)
#define GPIOD_BASEADD			(AHB1_BASE + 0x0C00U)
#define GPIOE_BASEADD			(AHB1_BASE + 0x1000U)
#define GPIOF_BASEADD			(AHB1_BASE + 0x1400U)
#define GPIOG_BASEADD			(AHB1_BASE + 0x1800U)
#define GPIOH_BASEADD			(AHB1_BASE + 0x1C00U)
#define GPIOI_BASEADD			(AHB1_BASE + 0x2000U)
#define GPIOJ_BASEADD			(AHB1_BASE + 0x2400U)
#define GPIOK_BASEADD			(AHB1_BASE + 0x2800U)

#define RCC_BASEADD				(AHB1_BASE + 0x3800U)

/*
 * SPI
 * */
#define SPI1_BASEADD			(APB2_BASE + 0x3000U)
#define SPI2_BASEADD			(APB1_BASE + 0x3800U)
#define SPI3_BASEADD			(APB1_BASE + 0x3C00U)
#define SPI4_BASEADD			(APB2_BASE + 0x3400U)
#define SPI5_BASEADD			(APB2_BASE + 0x5000U)
#define SPI6_BASEADD			(APB2_BASE + 0x5400U)


/*
 * USART/UART
 * */
#define USART1_BASEADD			(APB2_BASE + 0x1000U)
#define USART6_BASEADD			(APB2_BASE + 0x1400U)
#define USART2_BASEADD			(APB1_BASE + 0x4400U)
#define USART3_BASEADD			(APB1_BASE + 0x4800U)

#define UART4_BASEADD			(APB1_BASE + 0x4C00U)
#define UART5_BASEADD			(APB1_BASE + 0x5000U)
#define UART7_BASEADD			(APB1_BASE + 0x7800U)
#define UART8_BASEADD			(APB1_BASE + 0x7C00U)



//Some Generic macros
#define ENABLE 					1
#define DISABLE 				0

#define SET						ENABLE
#define RESET					DISABLE

#define GPIO_PIN_SET			ENABLE
#define GPIO_PIN_RESET   		DISABLE

#define FLAG_SET				SET
#define FLAG_RESET				RESET

//Bit position SPI peripheral
#define SPI_CR1_BIT_CPHA			0
#define SPI_CR1_BIT_CPOL			1
#define SPI_CR1_BIT_MSTR			2
#define SPI_CR1_BIT_BR				3
#define SPI_CR1_BIT_SPE				6
#define SPI_CR1_BIT_LSBFIRST		7
#define SPI_CR1_BIT_SSI				8
#define SPI_CR1_BIT_SSM				9
#define SPI_CR1_BIT_RXONLY			10
#define SPI_CR1_BIT_CRCL			11
#define SPI_CR1_BIT_CRCNEXT			12
#define SPI_CR1_BIT_CRCEN			13
#define SPI_CR1_BIT_BIDIOE			14
#define SPI_CR1_BIT_BIDIMODE		15

#define SPI_CR2_BIT_RXDMAEN			0
#define SPI_CR2_BIT_TXDMAEN			1
#define SPI_CR2_BIT_SSOE			2
#define SPI_CR2_BIT_NSSP			3
#define SPI_CR2_BIT_FRF				4
#define SPI_CR2_BIT_ERRIE			5
#define SPI_CR2_BIT_RXNEIE			6
#define SPI_CR2_BIT_TXEIE			7
#define SPI_CR2_BIT_DS				8		//DFF
#define SPI_CR2_BIT_FRXTH			12
#define SPI_CR2_BIT_LDMA_RX			13
#define SPI_CR2_BIT_LDMA_TX			14

#define SPI_SR_BIT_RXNE				0
#define SPI_SR_BIT_TXE				1
#define SPI_SR_BIT_BSY				7


/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_UE					0
#define USART_CR1_UESM 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M0 					12
#define USART_CR1_MME 					13
#define USART_CR1_CMIE 					14
#define USART_CR1_OVER8  				15
#define USART_CR1_DEDT  				16
#define USART_CR1_DEAT  				21
#define USART_CR1_RTOIE  				26
#define USART_CR1_EOBIE					27
#define USART_CR1_M1 					28


/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADDM7   				4
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN   				11
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_ISR_PE       				0
#define USART_ISR_FE       				1
#define USART_ISR_NF       				2
#define USART_ISR_ORE      				3
#define USART_ISR_IDLE       			4
#define USART_ISR_RXNE        			5
#define USART_ISR_TC       				6
#define USART_ISR_TXE        			7
#define USART_ISR_LBDF        			8
#define USART_ISR_CTSIF        			10
#define USART_ISR_CTS        			10


#define RCC_GET_SYSCLK_SOURCE() (RCC->CFGR & RCC_CFGR_SWS)

/*
 * Peripheral register definition structure for GPIO
 *
 * */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	volatile uint32_t RCC_AHB3RSTR;
	volatile uint32_t RCC_RESERVED0;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	volatile uint32_t RCC_RESERVED1[2];
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t RCC_AHB3ENR;
	volatile uint32_t RCC_RESERVED2;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	volatile uint32_t RCC_RESERVED3[2];
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	volatile uint32_t RCC_AHB3LPENR;
	volatile uint32_t RCC_RESERVED4;
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	volatile uint32_t RCC_RESERVED5[2];
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	volatile uint32_t RCC_RESERVED6[2];
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
	volatile uint32_t RCC_PLLSAICFGR;
	volatile uint32_t RCC_DCKCFGR1;
	volatile uint32_t RCC_DCKCFGR2;

}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t GTPR;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
} USART_RegDef_t;

typedef struct
{
  uint32_t PLLState;
  uint32_t PLLSource;
  uint32_t PLLM;
  uint32_t PLLN;
  uint32_t PLLP;
  uint32_t PLLQ;
  uint32_t PLLR;

}RCC_PLLRegDef_t;

typedef struct
{
  uint32_t OscillatorType;
  uint32_t HSEState;
  uint32_t LSEState;
  uint32_t HSIState;
  uint32_t HSICalibrationValue;
  uint32_t LSIState;
  RCC_PLLRegDef_t PLL;

}RCC_OscRegDef_t;

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADD)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADD)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADD)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADD)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADD)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADD)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADD)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADD)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASEADD)
#define GPIOJ					((GPIO_RegDef_t*)GPIOJ_BASEADD)
#define GPIOK					((GPIO_RegDef_t*)GPIOK_BASEADD)

#define RCC						((RCC_RegDef_t*)RCC_BASEADD)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADD)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADD)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADD)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADD)
#define SPI5					((SPI_RegDef_t*)SPI5_BASEADD)
#define SPI6					((SPI_RegDef_t*)SPI6_BASEADD)

#define USART1					((USART_RegDef_t*)USART1_BASEADD)
#define USART2					((USART_RegDef_t*)USART2_BASEADD)
#define USART3					((USART_RegDef_t*)USART3_BASEADD)
#define USART6					((USART_RegDef_t*)USART6_BASEADD)
#define UART4					((USART_RegDef_t*)UART4_BASEADD)
#define UART5					((USART_RegDef_t*)UART5_BASEADD)
#define UART7					((USART_RegDef_t*)UART7_BASEADD)
#define UART8					((USART_RegDef_t*)UART8_BASEADD)


//GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*)GPIOA_BASEADD;
//GPIO_RegDef_t *pGPIOA = GPIOA;
//GPIO_RegDef_t *pGPIOB = GPIOB;
//GPIO_RegDef_t *pGPIOC = GPIOC;
//GPIO_RegDef_t *pGPIOD = GPIOD;
//GPIO_RegDef_t *pGPIOE = GPIOE;
//GPIO_RegDef_t *pGPIOF = GPIOF;
//GPIO_RegDef_t *pGPIOG = GPIOG;
//GPIO_RegDef_t *pGPIOH = GPIOH;
//GPIO_RegDef_t *pGPIOI = GPIOI;
//GPIO_RegDef_t *pGPIOJ = GPIOJ;
//GPIO_RegDef_t *pGPIOK = GPIOK;

//RCC_RegDef_t *pRCC = RCC;

#define RCC_PWR_CLK_ENABLE()		(RCC->RCC_APB1ENR |= (1 << 28))

#define RCC_SYSCFG_CLK_ENABLE()		(RCC->RCC_APB2ENR |= (1 << 14))


#define GPIOA_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 10))


#define GPIOA_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DIS()				(RCC->RCC_AHB1ENR &= ~(1 << 10))

#define SPI1_PCLK_EN()					(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()					(RCC->RCC_APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()					(RCC->RCC_APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()					(RCC->RCC_APB2ENR |= (1 << 21))

#define SPI1_PCLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()					(RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DIS()					(RCC->RCC_APB2ENR &= ~(1 << 21))

#define USART1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 31))

#define USART1_PCLK_DIS()				(RCC->RCC_APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DIS()				(RCC->RCC_APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DIS()				(RCC->RCC_APB1ENR &= ~(1 << 31))

#define GPIOA_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 9)); (RCC->RCC_AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET()				do{(RCC->RCC_AHB1RSTR |= (1 << 10)); (RCC->RCC_AHB1RSTR &= ~(1 << 10));}while(0)


#endif /* INC_STM32F767XX_H_ */
