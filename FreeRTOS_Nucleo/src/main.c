/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
#include <stdint.h>
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "led.h"
#include "spi.h"
#include "i2c.h"
#include "register_map.h"
#include "usart.h"
#include "fifo.h"
#include <stdio.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "timers.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void vTask1( void * pvParameters );
void vTask2( void * pvParameters );
int vFuncCallMalloc(void)
{
	uint8_t* ptr;
	static uint16_t j =1;
	uint8_t i;
//	ptr = pvPortMalloc(j*sizeof(uint8_t));
//	if(ptr == NULL)
//	{
//		return -1;
//	}
	vDebugPrintf("malloc %d bytes \r\n",j);
	for(i =0;i<j;i++)
		{
			*ptr = i;
			ptr++;
		}
	j++;
	return 1;
	//vPortFree(ptr);
	/*do notthing*/
}
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */




int main(void)
{



    InitFifo();
    USART_Config();
	LED_Init();
	 vDebugPrintf("hello %d %d\n\r", 1000,2000);
	 xTaskCreate(	vTask1,"Task1",200,NULL,1,NULL);
	 xTaskCreate(	vTask2,"Task2",200,NULL,1,NULL);

	 vTaskStartScheduler();
	 for(;;);

}


void vTask1( void * pvParameters )
{

	for(;;)
	{

		LED_On();
		vDebugPrintf("Task1 is created !!\n\r");
		if(vFuncCallMalloc() == -1)
			{
				vDebugPrintf("out of stack \n\r");
			}
		vTaskDelay(1);
	}
}
void vTask2( void * pvParameters )
{
	for(;;)
	{
		LED_On();
		vDebugPrintf("Task2 is created !!\n\r");
		if(vFuncCallMalloc() == -1)
		{
			vDebugPrintf("out of stack \n\r");
		}
		vTaskDelay(1);
	}
}
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
