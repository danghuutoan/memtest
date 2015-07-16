/*
 * spi.c
 *
 *  Created on: Mar 29, 2015
 *      Author: toan
 */
#include "spi.h"
#include "stm32f4xx.h"
#define   		DATAGRAM_MAX_LEN 	3
uint8_t         SPI_Buffer[DATAGRAM_MAX_LEN]={0xff,0xff,0xff};
uint8_t         SPI_index;


void SPI_StartReceive(void)
{
	SPI_MODE = SPI_RX_MODE ;
	SPI_index = 0;
	SPI_Cmd(SPIx, ENABLE);
	SPI_I2S_ReceiveData(SPIx);
	//SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_TXE,DISABLE);
}

void SPI_Configuration(void){

	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = SPIx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	/*Enable SPI2 clock*/
	RCC_APB1PeriphClockCmd(SPI_CLK, ENABLE);

	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize =  SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;

	SPI_Init(SPIx, &SPI_InitStructure);
	/*Enable GPIOA clock*/
	RCC_AHB1PeriphClockCmd(SPI_PIN_CLK, ENABLE);

	/* SPI Pin Configuration */
	GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN | SPI_SCK_PIN | SPI_MISO_PIN|SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(SPI_PIN_PORT, &GPIO_InitStructure);
	/* Alternate function mapping */
	GPIO_PinAFConfig(SPI_PIN_PORT,CS_PinSource,SPIx_AF);
	GPIO_PinAFConfig(SPI_PIN_PORT,SCK_PinSource,SPIx_AF);
	GPIO_PinAFConfig(SPI_PIN_PORT,MISO_PinSource,SPIx_AF);
	GPIO_PinAFConfig(SPI_PIN_PORT,MOSI_PinSource,SPIx_AF);
	/* Enable RX interrupt*/
	SPI_I2S_ITConfig(SPIx,SPI_I2S_IT_RXNE,ENABLE);
	 /* Enable SPI1 */
	SPI_Cmd(SPIx, ENABLE);
}
void SPIx_IRQHandler (void){

	if(SPI_I2S_GetITStatus(SPIx,SPI_I2S_IT_TXE)!=RESET)
		{
#if defined(MASTER)
			CS_SET();
#endif
			if(SPI_index< DATAGRAM_MAX_LEN)
			{
#if defined(MASTER)
				CS_RESET();
#endif
				SPI_I2S_SendData(SPIx, SPI_Buffer[SPI_index++]);
			}
			else
			{
				SPI_index =0;
				SPI_I2S_ITConfig(SPIx,SPI_I2S_IT_TXE,ENABLE);

			}
		}
	else
		{

		}

	if(SPI_I2S_GetITStatus(SPIx,SPI_I2S_IT_RXNE)!=RESET)
		{
			if(SPI_MODE == SPI_TX_MODE)
			{

				/* Receive a dummy byte to clear the DR register*/
				SPI_I2S_ReceiveData(SPIx);
			}
			else
			{
				GPIO_SetBits(GPIOA,GPIO_Pin_5);
				SPI_Buffer[SPI_index++] = (uint8_t)SPI_I2S_ReceiveData(SPIx);
				if(SPI_index >= DATAGRAM_MAX_LEN)
				{
					 /* Disable  SPI */
					SPI_Cmd(SPIx, DISABLE);
					SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);
					SPI_I2S_ReceiveData(SPIx);
					//SPI_index = 0;
				}
				//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
				  // Read dummy byte and empty the register
				  //SPI_I2S_ReceiveData(SPI2);
			}
		}
}

void SPI_DataSend(uint8_t* data)
{

	SPI_MODE = SPI_TX_MODE ;
	SPI_index = 0;
	SPI_Buffer[0] = *data;
	SPI_Buffer[1] = *(data+1);
	SPI_Buffer[2] = *(data+2);

	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	/* Send byte through the PLM SPI peripheral */
	SPI_I2S_SendData(SPIx, SPI_Buffer[SPI_index++]);

	SPI_Cmd(SPIx, ENABLE);

	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE , ENABLE);

}
