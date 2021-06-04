/*
 * stm32f767xx_rcc.h
 *
 *  Created on: Feb 6, 2021
 *      Author: shree
 */

#ifndef INC_STM32F767XX_RCC_H_
#define INC_STM32F767XX_RCC_H_

#include "stm32f767xx.h"

// returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

// returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F767XX_RCC_H_ */
