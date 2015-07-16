/*
 * spi.h
 *
 *  Created on: Mar 31, 2015
 *      Author: toandang
 */

#ifndef __SPI_H__
#define __SPI_H__

/*
 * spi.h
 *
 *  Created on: Mar 29, 2015
 *      Author: toan
 */

#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define SPI_PIN_PORT		GPIOB

#define SPI_CS_PIN			GPIO_Pin_12
#define SPI_CS_PORT			GPIOB
#define CS_PinSource		GPIO_PinSource12

#define SPI_SCK_PIN			GPIO_Pin_13
#define SPI_SCK_PORT		GPIOB
#define SCK_PinSource		GPIO_PinSource13

#define SPI_MISO_PIN		GPIO_Pin_14
#define SPI_MISO_PORT		GPIOB
#define MISO_PinSource		GPIO_PinSource14

#define SPI_MOSI_PIN		GPIO_Pin_15
#define SPI_MOSI_PORT		GPIOB
#define MOSI_PinSource		GPIO_PinSource15

#define SPIx				SPI2
#define SPI_CLK				RCC_APB1Periph_SPI2
#define SPI_PIN_CLK			RCC_AHB1Periph_GPIOB

#define SPIx_IRQn			SPI2_IRQn

#define SPIx_AF				GPIO_AF_SPI2

#define CS_SET();			GPIO_SetBits(SPI_PIN_PORT,SPI_CS_PIN);
#define CS_RESET();			GPIO_ResetBits(SPI_PIN_PORT,SPI_CS_PIN);
#define SPIx_IRQHandler		SPI2_IRQHandler

#define RCC_SPIClockCmd();	RCC_APB2PeriphClockCmd(SPI_CLK, ENABLE);
typedef enum
{
  SPI_TX_MODE,
  SPI_RX_MODE
}SPI_Mode_t;

SPI_Mode_t SPI_MODE;
void SPI_StartReceive(void);
void SPI_Configuration(void);
void SPI_DataSend(uint8_t* data);





#endif /* SPI_SPI_H_ */
