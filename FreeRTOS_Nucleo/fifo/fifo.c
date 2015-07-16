
/*
 * fifo.c
 *
 *  Created on: Apr 15, 2015
 *      Author: toandang
 */
#include "fifo.h"

void InitFifo(void) {
    PUTPT=GETPT=&Fifo[0]; /* Empty when PUTPT=GETPT */
}
int PutFifo (char data) { char *Ppt; /* Temporary put pointer */
    Ppt=PUTPT; /* Copy of put pointer */
    *(Ppt++)=data; /* Try to put data into fifo */
    USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
    if (Ppt == &Fifo[FifoSize]) Ppt = &Fifo[0]; /* Wrap */
    if (Ppt == GETPT ){
        return(0);}   /* Failed, fifo was full */
    else{
        PUTPT=Ppt;
        return(-1);   /* Successful */
    }
}
int GetFifo (char *datapt) {
    if (PUTPT== GETPT){
        return(0);}   /* Empty if PUTPT=GETPT */
    else{
        *datapt=*(GETPT++);
        if (GETPT == &Fifo[FifoSize])
            GETPT = &Fifo[0];
        return(-1);
    }
}

//void SendFifo(char data)
//{
//  int status;
//  status = PutFifo(data);
////        while(status==0)
////          {
////            status = PutFifo(data);
////          }
//}
