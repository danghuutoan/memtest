/*
 * usart.h
 *
 *  Created on: Apr 15, 2015
 *      Author: toandang
 */

#ifndef USART_USART_H_
#define USART_USART_H_
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "fifo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
void USART_Config(void);
//void USARTPritnf(const char* format,...);
void USARTPritnf(const char * format, ...);
void vDebugPrintf(const char *fmt, ...);
#endif /* USART_USART_H_ */
