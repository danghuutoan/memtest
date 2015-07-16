/*
 * fifo.h
 *
 *  Created on: Apr 15, 2015
 *      Author: toandang
 */

#ifndef FIFO_FIFO_H_
#define FIFO_FIFO_H_
#include "stm32f4xx_usart.h"
/* Pointer implementation of the FIFO */
#define FifoSize 1000 /* Number of 8 bit data in the Fifo */
char *PUTPT;    /* Pointer of where to put next */
char *GETPT;    /* Pointer of where to get next */
/* FIFO is empty if PUTPT=GETPT */
/* FIFO is full if PUTPT+1=GETPT */
char Fifo[FifoSize]; /* The statically allocated fifo data */
void InitFifo(void);
int PutFifo (char data);
int GetFifo (char *datapt);
void SendFifo(char data);


#endif /* FIFO_FIFO_H_ */
