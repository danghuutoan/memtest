/*
 * i2c.h
 *
 *  Created on: Apr 3, 2015
 *      Author: toandang
 */

#ifndef I2C_I2C_H_
#define I2C_I2C_H_
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"
#include "led.h"
#define SLAVE_ADDRESS (0x70<<1)
#define ADDRESS 	  0x31

void I2C_Configuration(void);
#define MAX_LEN			100

typedef struct{
	uint8_t data[MAX_LEN];
	uint8_t len;
	uint8_t mode;
} I2C_buffer;

I2C_buffer * I2C_GetBuffer(void);
void I2C_StopReceived(void);
void I2C_StartRx(uint8_t leng);
FlagStatus I2C_GetTxFlag(void);
FlagStatus I2C_GetRxFlag(void);
void I2C_StartTx(void);
uint8_t I2C_ByteRead(uint8_t addr);
void I2C_ByteWrite(uint8_t addr, uint8_t data);
#endif /* I2C_I2C_H_ */
