/*
 * i2c.c
 *
 *  Created on: Apr 3, 2015
 *      Author: toandang
 */
#include "i2c.h"

volatile uint32_t Event = 0;
uint8_t index;
uint8_t mode;
uint8_t len;
uint8_t byte2Read;
volatile uint8_t MasterTxBuffer[3];
volatile uint8_t RX_Buffer[100];
FlagStatus TX_OK;
FlagStatus RxComplete_f = RESET;
FlagStatus TxComplete_f = RESET;
I2C_buffer buffer;
int n =0;
FlagStatus I2C_GetTxFlag(void)
{
  return TxComplete_f;
}
FlagStatus I2C_GetRxFlag(void)
{
  return RxComplete_f;
}
void I2C_StopReceived(void)
{
	index = 0;
	RxComplete_f = SET;
}
I2C_buffer* I2C_GetBuffer(void)
{
	I2C_buffer* p;
	p = &buffer;
	return p;
}
void I2C_StartTx(void)
{

  TxComplete_f = RESET;
  index =0;
  mode = I2C_Direction_Transmitter;
  I2C_Cmd(I2C1, ENABLE);
}
void I2C_StartRx(uint8_t leng)
{
  RxComplete_f = RESET;
  index =0;
  mode = I2C_Direction_Receiver;
  len = leng;
  I2C_Cmd(I2C1, ENABLE);
}
void I2C_Configuration(void)
{
	GPIO_InitTypeDef GPIO_I2C_InitStruct;
			NVIC_InitTypeDef NVIC_InitStruct;
			I2C_InitTypeDef I2C_InitStruct;


			/* RCC Configuration */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

			/*SDA GPIO clock enable */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

			/*SCL GPIO clock enable */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

			/* Reset I2C3 IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);

			/* Release reset signal of I2Cx IP */
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

			/* GPIO Configuration */
			/*Configure I2C SCL pin */
			/* Set default I2C GPIO	settings */
			GPIO_I2C_InitStruct.GPIO_Pin = GPIO_Pin_8;
			GPIO_I2C_InitStruct.GPIO_Mode = GPIO_Mode_AF;
			GPIO_I2C_InitStruct.GPIO_OType = GPIO_OType_OD;
			GPIO_I2C_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_I2C_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			/* Init pins */
			GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);
			/*Configure I2C SDA pin */
			GPIO_I2C_InitStruct.GPIO_Pin = GPIO_Pin_9;
			/* Init pins */
			GPIO_Init(GPIOB, &GPIO_I2C_InitStruct);

			/* Connect PB6 to I2C_SCL */
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
			/* Connect PB9 to I2C_SDA */
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

			/* NVIC configuration */
			/* Configure the Priority Group to 1 bit */
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

			/* Configure the I2C event priority */
			NVIC_InitStruct.NVIC_IRQChannel = I2C1_EV_IRQn;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStruct);

			/* Configure I2C error interrupt to have the higher priority */
			NVIC_InitStruct.NVIC_IRQChannel = I2C1_ER_IRQn;
			NVIC_Init(&NVIC_InitStruct);

			I2C_DeInit(I2C1);

			/* Initialize I2C peripheral */
			I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
			I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
			I2C_InitStruct.I2C_OwnAddress1 = ADDRESS;
			I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
			I2C_InitStruct.I2C_ClockSpeed = 400000;
			I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

			/* Initialize I2C */
			I2C_Init(I2C1, &I2C_InitStruct);

			/* Enable Error and Buffer Interrupts */
			I2C_ITConfig(I2C1, (I2C_IT_ERR | I2C_IT_EVT | I2C_IT_BUF), ENABLE);

			/* Enable I2C */
			I2C_Cmd(I2C1, ENABLE);
}

void I2C1_ER_IRQHandler(void)
{

  /* Check on I2C2 AF flag and clear it */
  if (I2C_GetITStatus(I2C1, I2C_IT_AF))
  {
    I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
  }
  if ((I2C_ReadRegister(I2C1, I2C_Register_SR1) & 0xFF00) != 0x00)
    	{
    		/* Clears error flags */
    		I2C1->SR1 &= 0x00FF;
    	}
}



void I2C1_EV_IRQHandler(void)
{
	//LED_On();

  /* Get Last I2C Event */
  Event = I2C_GetLastEvent(I2C1);
  switch (Event)
    {

    case I2C_EVENT_MASTER_MODE_SELECT:
      /* Send slave address for writing */
        TxComplete_f = RESET;
        RxComplete_f = RESET;
        index =0;
        I2C_ITConfig(I2C1,I2C_IT_BUF,ENABLE);
        I2C_Send7bitAddress(I2C1, SLAVE_ADDRESS,  mode );
      break;
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
      I2C_SendData(I2C1, buffer.data[index++]);
      break;
    case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
      if(index >= buffer.len)
        {
          I2C_GenerateSTOP(I2C1,ENABLE);
          while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
          TxComplete_f = SET;
          I2C_ITConfig(I2C1,I2C_IT_BUF,DISABLE);
          I2C_Cmd(I2C1, DISABLE);
          break;
        }
      I2C_SendData(I2C1, buffer.data[index++]);
      break;

    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
      //LED_On();
      byte2Read = len;
      if(byte2Read == 1)
        I2C_AcknowledgeConfig(I2C1,DISABLE);
      break;
    case I2C_EVENT_MASTER_BYTE_RECEIVED:
      if(byte2Read ==2)
        I2C_AcknowledgeConfig(I2C1,DISABLE);
      if(byte2Read == 1)
        {
        I2C_GenerateSTOP(I2C1,ENABLE);
        while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
        RxComplete_f = SET;
        I2C_ITConfig(I2C1,I2C_IT_BUF,DISABLE);
        I2C_Cmd(I2C1, DISABLE);
        }
      buffer.data[index++] = I2C_ReceiveData(I2C1);
      byte2Read --;
      break;


    /**************SLAVE RECEIVE EVENT*********************************/
    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED :
    	//read SR1 followed by reading SR2 in order to clear ADDR bit
    	I2C_GetLastEvent(I2C1);

    	index =0;
       	break;
    case I2C_EVENT_SLAVE_BYTE_RECEIVED:
    case I2C_EVENT_SLAVE_BYTE_RECEIVED| I2C_SR1_BTF:
    	buffer.data[index++]=I2C_ReceiveData(I2C1);
       	break;
    case I2C_EVENT_SLAVE_STOP_DETECTED:
    	buffer.len = index;
    	//clear STPOF by reading SR1 register followed by writing to the CR1 register
    	//I2C_GetLastEvent(I2C1);
    	I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF);
    	I2C_Cmd(I2C1, ENABLE);
    	I2C_StopReceived();
    	break;

    /*****************SLAVE TRANSMITT EVENT*******************************/
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
    	//read SR1 followed by reading SR2 in order to clear ADDR bit
    	I2C_GetLastEvent(I2C1);
    	//LED_On();
    	index=0;
    	I2C_SendData(I2C1, buffer.data[index++]);
    	break;
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
    	if(index < buffer.len)
    	I2C_SendData(I2C1, buffer.data[index++]);
    	break;
    case I2C_EVENT_SLAVE_ACK_FAILURE:
    	index = buffer.len;
    	I2C_StopReceived();
        //LED_On();
    	break;
    default:
      break;
    }
}
uint8_t I2C_ByteRead(uint8_t addr)
{
  uint8_t data;
  I2C_buffer * buffer;

  buffer = I2C_GetBuffer();
  buffer->len = 1;
  buffer->data[0]= addr;
  I2C_StartTx();
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetTxFlag());

  I2C_StartRx(1);
  I2C_GenerateSTART(I2C1,ENABLE);

  while(!I2C_GetRxFlag());
  data = buffer->data[0];
  return data;
}


void I2C_ByteWrite(uint8_t addr, uint8_t data)
{
  I2C_buffer * buffer;
 //while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
  buffer = I2C_GetBuffer();
  buffer->len = 2;
  buffer->data[0]= addr;
  buffer->data[1]= data;
  I2C_StartTx();
  I2C_GenerateSTART(I2C1,ENABLE);
  while(!I2C_GetTxFlag());
}
