/**
  ******************************************************************************
  * @file    hal_i2c.c
  * @author  Infonam Embedded Team
  * @version V1.0.0
  * @date    June-10-2015
  * @brief   This file expandes for #I2C driver
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "hal_i2c.h"
/**< #ASIC Hardware target */
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

/* Private typedef -----------------------------------------------------------*/

/* pin hardware configuration */
typedef struct _i2c_hw_pin_config{
    uint32_t               Clock;
    GPIO_TypeDef           *Port;
    uint16_t               PinNum;
    uint8_t                AFPin;
    uint8_t                AFFunc;
}i2c_hw_pin;

/* i2c interrupt */
typedef struct _i2c_irq {
    uint8_t                EventIRQ;
    uint8_t                ErrorIRQ;
}i2c_irq;

/* I2C hw configuration */
typedef struct _I2C_iStruct {
    I2C_TypeDef            *I2CBase;
    uint32_t               ClockMask;
    i2c_irq                IRQ;
    i2c_hw_pin             PIN_SCL;
    i2c_hw_pin             PIN_SDA;
}i2c_hw_config;

/* direction transfer */
typedef enum _i2c_transfer {
    I2C_TX,
    I2C_RX
}i2c_transfer_t;

/* Operation mode handle structure */
typedef struct _i2c_isr_handle
{
    void                             (*Evt)(uint16_t,void*);
    void                             (*Err)(uint16_t,void*);
    hal_i2c_omode_t                  omode;
    hal_i2c_status_t                 sm;
    uint8_t                          rx_dir;
    i2c_transfer_t                   dir;           /* transfer dir */
    volatile hal_i2c_master_buffer_t *t_data;       /* handler of data from upper layer */
    void                             *hdl;
}i2c_isr_handle_t;

/* Private variables ---------------------------------------------------------*/

/* I2C configure */
static const i2c_hw_config I2C_iParam[] =
{
    /* I2C1 */
    {
        .I2CBase     = I2C1,                 /* I2C1 base address */
        .ClockMask   = RCC_APB1Periph_I2C1,  /* I2C1 Clock bit */
        /* Interrupt */
        {
           .EventIRQ = I2C1_EV_IRQn,
           .ErrorIRQ = I2C1_ER_IRQn, 
        },
        /* I2C1_SCL */
        {
            .Clock = RCC_AHB1Periph_GPIOB,
            .Port  = GPIOB,
            .PinNum= GPIO_Pin_8,
            .AFPin = GPIO_PinSource8,
            .AFFunc= GPIO_AF_I2C1,
        },
        /* I2C1_SDA */  
        {
            .Clock = RCC_AHB1Periph_GPIOB,
            .Port  = GPIOB,
            .PinNum= GPIO_Pin_9,
            .AFPin = GPIO_PinSource9,
            .AFFunc= GPIO_AF_I2C1,
        },  
    },
    /* I2C2 */
    {
        .I2CBase     = I2C2,                 /* I2C2 base address */
        .ClockMask   = RCC_APB1Periph_I2C2,  /* I2C2 Clock bit */
        /* Interrupt */
        {
           I2C2_EV_IRQn,
           I2C2_ER_IRQn, 
        },
        /* I2C2_SCL */
        {
            .Clock = RCC_AHB1Periph_GPIOF,
            .Port  = GPIOF,
            .PinNum= GPIO_Pin_1,
            .AFPin = GPIO_PinSource1,
            .AFFunc= GPIO_AF_I2C2,
        },
        /* I2C2_SDA */  
        {
            .Clock = RCC_AHB1Periph_GPIOF,
            .Port  = GPIOF,
            .PinNum= GPIO_Pin_0,
            .AFPin = GPIO_PinSource0,
            .AFFunc= GPIO_AF_I2C2,
        },  
    },
    /* I2C3 */
    {
        .I2CBase     = I2C3,                 /* I2C3 base address */
        .ClockMask   = RCC_APB1Periph_I2C3,  /* I2C3 Clock bit */
        /* Interrupt */
        {
           I2C3_EV_IRQn,
           I2C3_ER_IRQn, 
        },
        /* I2C3_SCL */
        {
            .Clock = RCC_AHB1Periph_GPIOH,
            .Port  = GPIOH,
            .PinNum= GPIO_Pin_7,
            .AFPin = GPIO_PinSource7,
            .AFFunc= GPIO_AF_I2C3,
        },  
        /* I2C3_SDA */
        {
            .Clock = RCC_AHB1Periph_GPIOC,
            .Port  = GPIOC,
            .PinNum= GPIO_Pin_9,
            .AFPin = GPIO_PinSource9,
            .AFFunc= GPIO_AF_I2C3,
        },  
    }
};

/* Interrupt Handler */
static i2c_isr_handle_t irq_hdl[HAL_I2C_MAX_IDX];


/* Private functions -------------------------------------------------------- */
static void hal_i2c_irq_event(uint8_t chid);
static void hal_i2c_irq_error(uint8_t chid);
/* Exported functions ------------------------------------------------------- */

/*
 * hal_i2c_init
 * The function shall initialize independence #I2C channel
 */
int hal_i2c_init( hal_i2c_t * i2c )
{
    int ret;
    GPIO_InitTypeDef  GPIO_InitStruct; 
    I2C_InitTypeDef   I2C_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;       
    hal_i2c_channel_t chid;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == i2c)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= i2c->chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        chid = i2c->chid;        
        /* configure I2C pin */
        /* enable clock pin */
        RCC_AHB1PeriphClockCmd(I2C_iParam[chid].PIN_SCL.Clock,ENABLE); /* SCL pin */
        RCC_AHB1PeriphClockCmd(I2C_iParam[chid].PIN_SDA.Clock,ENABLE); /* SDA pin */
        /* AF configure */
        /* Connect PinSource to I2Cx */
        GPIO_PinAFConfig(
                        I2C_iParam[chid].PIN_SCL.Port,
                        I2C_iParam[chid].PIN_SCL.AFPin, 
                        I2C_iParam[chid].PIN_SCL.AFFunc
                        ); /* SCL */
        GPIO_PinAFConfig(
                        I2C_iParam[chid].PIN_SDA.Port,
                        I2C_iParam[chid].PIN_SDA.AFPin, 
                        I2C_iParam[chid].PIN_SDA.AFFunc
                        ); /* SDA */                    
        /* GPIO mode configure */
        GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;
        GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Speed  = GPIO_Speed_50MHz;
        /* SCL pin init */
        GPIO_InitStruct.GPIO_Pin    = I2C_iParam[chid].PIN_SCL.PinNum;
        GPIO_Init(I2C_iParam[chid].PIN_SCL.Port, &GPIO_InitStruct);
        /* SDA pin init */
        GPIO_InitStruct.GPIO_Pin    = I2C_iParam[chid].PIN_SDA.PinNum;
        GPIO_Init(I2C_iParam[chid].PIN_SDA.Port, &GPIO_InitStruct);

        /* enable clock for I2C module */
        RCC->APB1ENR |= I2C_iParam[chid].ClockMask;
        /* Reset i2c peripheral */
        RCC->APB1RSTR |= I2C_iParam[chid].ClockMask; 

        RCC->APB1RSTR &= ~I2C_iParam[chid].ClockMask;

        I2C_iParam[i2c->chid].I2CBase->CR1 |= 0x8000;
        I2C_iParam[i2c->chid].I2CBase->CR1 &= (uint16_t) ~((uint16_t) 0x8000);

        /* Initialize I2C peripheral */
        I2C_InitStruct.I2C_Mode         = I2C_Mode_I2C;
        I2C_InitStruct.I2C_DutyCycle    = I2C_DutyCycle_2;
        I2C_InitStruct.I2C_Ack          = I2C_Ack_Enable;
        I2C_InitStruct.I2C_OwnAddress1  = i2c->cfg.owner_address;
        I2C_InitStruct.I2C_ClockSpeed   = (uint32_t)i2c->cfg.clock_speed;
        
        /* configure I2C address mode */
        if( I2C_7BIT == i2c->cfg.amode )
        {
            I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;            
        }
        else if( I2C_10BIT == i2c->cfg.amode )
        {
            I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_10bit;
        }
        else
        {
            /* Not supported other address mode yet */
        }
        /* Initialize I2C */
        I2C_Init(I2C_iParam[i2c->chid].I2CBase, &I2C_InitStruct);
#if (HAL_I2C_DEBUG == STD_ON)
        /* check Interrupt configure */
        if((void*)0 == i2c->evt_isr || (void*)0 == i2c->err_isr )
        {
            ret  = (int)I2C_NULL_PTR;
        }
        else
#endif /* HAL_I2C_DEBUG */            
        {
            /* NVIC configuration */
            /* Configure the Priority Group to 4 bit */
            NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);            
            /* Configure the I2C event priority */
            NVIC_InitStruct.NVIC_IRQChannel                   = I2C_iParam[i2c->chid].IRQ.EventIRQ;
            NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = i2c->cfg.priority;
            NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
            NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
            NVIC_Init(&NVIC_InitStruct);
            /* Configure I2C error interrupt to the higher layer */
            NVIC_InitStruct.NVIC_IRQChannel                   = I2C_iParam[i2c->chid].IRQ.ErrorIRQ;
            NVIC_Init(&NVIC_InitStruct);

            /* store interrupt call back function */
            irq_hdl[i2c->chid].Evt    = i2c->evt_isr;
            irq_hdl[i2c->chid].Err    = i2c->err_isr;
            irq_hdl[i2c->chid].omode  = i2c->cfg.omode;
            irq_hdl[i2c->chid].sm     = I2C_READY;
            I2C_Cmd(I2C_iParam[i2c->chid].I2CBase, ENABLE);

            ret = I2C_OK;
        }
    }
    return ret;
}

/*
 * hal_i2c_deinit
 * The function stops operation on independence #I2C channel
 */
int hal_i2c_deinit( hal_i2c_t * i2c )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == i2c)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= i2c->chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        /* disable clock for I2C module */
        RCC->APB1ENR &= ~(I2C_iParam[i2c->chid].ClockMask);
        /* de-init I2C Peripheral for reconfigure */
        I2C_DeInit(I2C_iParam[i2c->chid].I2CBase);
        /* disable clock pin */
        RCC_AHB1PeriphClockCmd(I2C_iParam[i2c->chid].PIN_SCL.Clock,DISABLE); /* SCL pin */
        RCC_AHB1PeriphClockCmd(I2C_iParam[i2c->chid].PIN_SDA.Clock,DISABLE); /* SDA pin */
        ret = I2C_OK;
    }
    return ret;    
}

/*
 * hal_i2c_master_tx
 * The function write data from master to slave
 */
int hal_i2c_master_tx( hal_i2c_channel_t chid, hal_i2c_master_buffer_t * buf )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == buf)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    /* resource busy or ready */
    else if( I2C_READY != irq_hdl[chid].sm )           
    {
        ret = (int)I2C_CHANNEL_BUSY;
    }
    /* check is master mode */    
    else if( I2C_MASTER != irq_hdl[chid].omode )           
    {
        ret = (int)I2C_UNKNOWN_ERROR;
    }        
    else        
#endif /* HAL_I2C_DEBUG */        
    {
        /* While the bus is busy */
        if(I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_BUSY))
        {
            ret = (int)I2C_CHANNEL_BUSY; 
        }
        else
        {
            irq_hdl[chid].t_data     = buf;
            /* set transfer direction is TX */
            irq_hdl[chid].dir        = I2C_TX;
            /* set resource transmitting data */
            irq_hdl[chid].sm         = I2C_TX_TRASMITTING;   
            /* set rx_direction to transmitter first for write slave and iaddr */          
            irq_hdl[chid].rx_dir     = I2C_Direction_Transmitter;         
            /* for Interrupt Hardware handle */
            /************************************************/
            /* Generate the Start condition, 
            All processes TX is progressing in Interrupt handle */
            I2C_GenerateSTART(I2C_iParam[chid].I2CBase, ENABLE);
            /************************************************/
            ret = (int)I2C_OK;            
        }
    }
    return ret;
}

/*
 * hal_i2c_master_rx
 * The function write data from master to slave
 */
int hal_i2c_master_rx( hal_i2c_channel_t chid, hal_i2c_master_buffer_t * buf )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == buf)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    /* resource busy or ready */
    else if( I2C_READY != irq_hdl[chid].sm )           
    {
        ret = (int)I2C_CHANNEL_BUSY;
    }
    /* check is master mode */    
    else if( I2C_MASTER != irq_hdl[chid].omode )           
    {
        ret = (int)I2C_UNKNOWN_ERROR;
    }  
    else if ( buf->rxbuf.len < 1 )
    {
        ret = (int)I2C_INVALID_LENGTH;
    }   
    else     
#endif /* HAL_I2C_DEBUG */ 
    {
        /* While the bus is busy */
        if(I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_BUSY))
        {
            ret = (int)I2C_CHANNEL_BUSY; 
        }
        else
        {
            if(buf->rxbuf.len == 1)
            {
                buf->rxbuf.len += 1;
            }

            irq_hdl[chid].t_data     = buf;
            /* set transfer direction is RX */
            irq_hdl[chid].dir        = I2C_RX;
            /* set resource receiving data */
            irq_hdl[chid].sm         = I2C_RX_RECEIVING;   
            /* set rx_direction to transmitter first for write slave and iaddr */          
            irq_hdl[chid].rx_dir     = I2C_Direction_Transmitter;     ;
            /* for Interrupt Hardware handle */
            /************************************************/
            /* Generate the Start condition, 
            All processes RX is progressing in Interrupt handle */
            I2C_GenerateSTART(I2C_iParam[chid].I2CBase, ENABLE);
            I2C_AcknowledgeConfig(I2C_iParam[chid].I2CBase, ENABLE);
            /************************************************/
            ret = (int)I2C_OK;         
        }
    }
    return ret;    
}

/*
 * hal_i2c_get_status
 * The function gets I2C bus status 
 */
int hal_i2c_get_status( hal_i2c_channel_t chid )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        ret = (int)irq_hdl[chid].sm;  
    }
    return ret;
}

/*
 * hal_i2c_get_bus_status
 * The function gets I2C bus status 
 */
int hal_i2c_get_bus_status( hal_i2c_channel_t chid )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        if(I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_BUSY))
        {
            ret = I2C_BUS_BUSY;
        }
        else
        {
            ret = (int)irq_hdl[chid].sm;
        }        
    }
    return ret;
}

/*
 * hal_i2c_slave_writefifo
 * The function sends one byte to fifo hardware
 */
int hal_i2c_slave_writefifo( hal_i2c_channel_t chid, uint8_t * byte )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == byte)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        I2C_iParam[chid].I2CBase->DR = (uint8_t)(*byte);
        ret = I2C_OK;
    }
    return ret;
}

/*
 * hal_i2c_readbyte
 * The function reads one byte from hardware fifo
 */
int hal_i2c_slave_readfifo( hal_i2c_channel_t chid, uint8_t * byte )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if((void *)0 == byte)
    {
        ret = (int)I2C_NULL_PTR;
    }
    else if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else
#endif /* HAL_I2C_DEBUG */    
    {
        *byte = ( uint8_t )I2C_iParam[chid].I2CBase->DR ;
        ret = I2C_OK;
    }
    return ret;
}

/*
 * hal_i2c_enable_irq
 * The function enables hardware irq
 */
int hal_i2c_enable_irq( hal_i2c_channel_t chid )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else        
#endif /* HAL_I2C_DEBUG */  
    {
        /* Enable Event Error and Buffer Interrupts */
        I2C_ITConfig(I2C_iParam[chid].I2CBase, (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR ), ENABLE);
        ret = (int)I2C_OK;
    }
    return ret;
}

/*
 * hal_i2c_disable_irq
 * The function disables hardware interrupt
 */
int hal_i2c_disable_irq( hal_i2c_channel_t chid )
{
    int ret;
#if (HAL_I2C_DEBUG == STD_ON)
    if(HAL_I2C_MAX_IDX <= chid )
    {
        ret = (int)I2C_INVALID_CHANNEL;
    }
    else        
#endif /* HAL_I2C_DEBUG */  
    {
        /* Disable Event Error and Buffer Interrupts */
        I2C_ITConfig(I2C_iParam[chid].I2CBase, (I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR ), DISABLE);
        ret = (int)I2C_OK;
    }
    return ret;
}

/*
 * hal_i2c_irq_event
 * The local function processes i2c irq event
 */

static void hal_i2c_irq_event(uint8_t chid)
{
    uint32_t evt;
    uint8_t  dump_asm = 12;
/* Operation mode */
    switch (irq_hdl[chid].omode)
    {
        /* In Master mode */
        case I2C_MASTER:
            /* check dir is Tx or RX */
            if(I2C_TX == irq_hdl[chid].dir)
            {
                /* Get Last I2C Event */
                switch (I2C_GetLastEvent(I2C_iParam[chid].I2CBase))
                {
                    /* Check on EV5 */
                    case I2C_EVENT_MASTER_MODE_SELECT :
                        /* Send slave address for write */
                        I2C_Send7bitAddress(I2C_iParam[chid].I2CBase, irq_hdl[chid].t_data->slave_addr, I2C_Direction_Transmitter);
                        break;
                    /* Check on EV6 */
                    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
                        /* send first byte is internal address */
                        I2C_SendData(I2C_iParam[chid].I2CBase, *(irq_hdl[chid].t_data->txbuf.buf++));
                        irq_hdl[chid].t_data->txbuf.len--;
                        break;
                    /* Check on EV8 */
                    case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
                    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
                        if( 0 == irq_hdl[chid].t_data->txbuf.len )
                        {
                            /* Send STOP condition */
                            I2C_GenerateSTOP(I2C_iParam[chid].I2CBase, ENABLE);
                            /* Notificate to upper layer TX is completed */
                            (*(irq_hdl[chid].Evt))((uint16_t)I2C_TX_COMPLETED, irq_hdl[chid].t_data->hdl);
                            while(dump_asm--);
                            /* set local state machine to idle */
                            irq_hdl[chid].sm = I2C_READY;                    
                        }
                        else
                        {
                            /* Send data to slave */
                            I2C_SendData(I2C_iParam[chid].I2CBase, *(irq_hdl[chid].t_data->txbuf.buf++));
                            irq_hdl[chid].t_data->txbuf.len--;
                        }
                        break;
                    default:
                        break;
                }
            }
            /* I2C RX */
            else
            {
                evt = I2C_GetLastEvent(I2C_iParam[chid].I2CBase);
                /* Get Last I2C Event */
                switch (evt)
                {        
                    /* Check on EV5 */
                    case I2C_EVENT_MASTER_MODE_SELECT :
                        /* Send slave address for write */
                        /* in the first time, it will be on transmitter mode */
                        /* in the next time, it will be on receiver mode */
                        I2C_Send7bitAddress(I2C_iParam[chid].I2CBase, irq_hdl[chid].t_data->slave_addr, irq_hdl[chid].rx_dir);
                        break;
                    /* Check on EV6 */
                    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
                        
                        /* send first byte is internal address */
                        I2C_SendData(I2C_iParam[chid].I2CBase, *(irq_hdl[chid].t_data->txbuf.buf++));
                        irq_hdl[chid].t_data->txbuf.len--;

                        break;
                    /* Check on EV8 */
                    case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
                    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
                        if(irq_hdl[chid].t_data->txbuf.len)
                        {
                            /* send first byte is internal address */
                            I2C_SendData(I2C_iParam[chid].I2CBase, *(irq_hdl[chid].t_data->txbuf.buf++));
                            irq_hdl[chid].t_data->txbuf.len--;
                        }
                        else
                        {
                            /* I2C direction */
                            irq_hdl[chid].rx_dir = I2C_Direction_Receiver;
                            /* Send Start condition a second time */
                            I2C_GenerateSTART(I2C_iParam[chid].I2CBase, ENABLE);
                        }
                        break;
                    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
                        /* do nothing */
                        break;
                    //case 0x00030044:
                    case I2C_EVENT_MASTER_BYTE_RECEIVED:
                        /* Test on EV7 and clear it */
                        /* Read a byte from the slave */
                        *(irq_hdl[chid].t_data->rxbuf.buf++) = I2C_ReceiveData(I2C_iParam[chid].I2CBase);
                        /* Decrement the read bytes counter */
                        irq_hdl[chid].t_data->rxbuf.len--;
                        if ( 1 == irq_hdl[chid].t_data->rxbuf.len )
                        {
                            I2C_AcknowledgeConfig(I2C_iParam[chid].I2CBase, DISABLE );
                        }
                        if ( 0 == irq_hdl[chid].t_data->rxbuf.len )
                        {
                            I2C_GenerateSTOP(I2C_iParam[chid].I2CBase,ENABLE);
                            /* Enable Acknowledgement to be ready for another reception */                            
                            /* notif to upper layer */
                            (*(irq_hdl[chid].Evt))((uint16_t)I2C_RX_COMPLETED,irq_hdl[chid].t_data->hdl);
                            /* set local state machine to idle */
                            irq_hdl[chid].sm = I2C_READY;
                        }                    
                        break;
                    default:
                        break;
                }
            }
        break;
        /* In Slave mode */
        case I2C_SLAVE:
            /* check bus event */
            switch(I2C_GetLastEvent(I2C_iParam[chid].I2CBase))
            {
                /* Event transmitter address matched */
                case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
                    /* clear address by read ST2 */
                    I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_ADDR);
                    ((void)(I2C_iParam[chid].I2CBase->SR2));

                    /* notif to upper layer */
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_TRANSMITTER_ADDR_MATCHED,irq_hdl[chid].t_data->hdl);
                    break;
                /* Event slave byte transmitted */                    
                case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
                case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_BYTE_TRANSMITTED,irq_hdl[chid].t_data->hdl);
                    break;
                /* Event slave receiver address matched */
                case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:

                    /* clear address by read ST2 */
                    I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_ADDR);
                    ((void)(I2C_iParam[chid].I2CBase->SR2));

                    /* notif to upper layer */
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_RECEIVER_ADDR_MATCHED,irq_hdl[chid].t_data->hdl);
                    break;
                /* Event slave byte received */
                case I2C_EVENT_SLAVE_BYTE_RECEIVED:
                case I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF:
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_BYTE_RECEIVED,irq_hdl[chid].t_data->hdl);
                    break;
                /* Event slave stop detect */
                case I2C_EVENT_SLAVE_STOP_DETECTED:
                    printf("I2C stop \r\n");
                    /* Clear STOPF by reading SR1, then writing CR1 */
                    I2C_GetFlagStatus(I2C_iParam[chid].I2CBase, I2C_FLAG_STOPF);
                    I2C_Cmd(I2C_iParam[chid].I2CBase, ENABLE);
                    /* notif to upper layer */                
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_STOP,irq_hdl[chid].t_data->hdl);
                    break;
                /* Event slave ack failure */
                case I2C_EVENT_SLAVE_ACK_FAILURE:
                    (*(irq_hdl[chid].Evt))((uint16_t)I2C_SLAVE_EVT_ACK_FAILURE,irq_hdl[chid].t_data->hdl);
                default:
                    /* Read SR1 register to get I2C error */
                    if ((I2C_ReadRegister(I2C_iParam[chid].I2CBase, I2C_Register_SR1) & 0xFF00) != 0x00)
                    {
                        /* Clears error flags */
                        I2C_iParam[chid].I2CBase->SR1 &= 0x00FF;
                    }
                    break;
            }
        break;

        default: break;
    }
}

/**
 * hal_i2c_irq_error
 * The local function processes i2c irq error
 */
static void hal_i2c_irq_error(uint8_t chid)
{
    uint16_t err = I2C_ReadRegister(I2C_iParam[chid].I2CBase, I2C_Register_SR1) & 0xFF00;
    /* Read SR1 register to get I2C error */

    if (err != 0x00)
    {
        /* Clears error flags */
        I2C_iParam[chid].I2CBase->SR1 &= 0x00FF;
        /* notif err to upper layer */
        (*(irq_hdl[chid].Err))((uint16_t)err,irq_hdl[chid].t_data->hdl);
    }
}

/**
 * I2C1_EV_IRQHandler
 * I2C1 Event ISR handler 
 */
void I2C1_EV_IRQHandler (void)
{
    hal_i2c_irq_event(HAL_I2C_CH1);
}

/**
 * I2C1_ER_IRQHandler
 * I2C1 Error ISR handler 
 */
void I2C1_ER_IRQHandler (void)
{
    hal_i2c_irq_error(HAL_I2C_CH1);
}

/**
 * I2C2_EV_IRQHandler
 * I2C2 Event ISR handler 
 */
void I2C2_EV_IRQHandler (void)
{
    hal_i2c_irq_event(HAL_I2C_CH2);
}

/**
 * I2C2_ER_IRQHandler
 * I2C2 Error ISR handler 
 */
void I2C2_ER_IRQHandler (void)
{
    hal_i2c_irq_error(HAL_I2C_CH2);
}


/**
 * I2C3_EV_IRQHandler
 * I2C3 Event ISR handler 
 */
void I2C3_EV_IRQHandler (void)
{
    hal_i2c_irq_event(HAL_I2C_CH3);
}

/**
 * I2C3_ER_IRQHandler
 * I2C3 Error ISR handler 
 */
void I2C3_ER_IRQHandler (void)
{
    hal_i2c_irq_error(HAL_I2C_CH3);
}

