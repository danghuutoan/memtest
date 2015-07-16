/**
  ******************************************************************************
  * @file    hal_i2c.h
  * @author  Infonam Embedded Team
  * @version V1.0.0
  * @date    June-03-2015
  * @brief   This file contains definitions for #I2C driver
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_I2C_H_
#define _HAL_I2C_H_

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "std_type.h"

/* Exported define ----------------------------------------------------------*/

/**< for development software debuging */
#define HAL_I2C_DEBUG               STD_ON

/**< I2C Buffer size */
#define HAL_I2C_MAX_BUFFER_SIZE     128

/* Exported typedef ----------------------------------------------------------*/

/**
 * @brief hal_i2c_irq_event_t
 * The upper layer will handle all events in slave mode.
 * #I2C IRQ Event
 */
typedef enum _hal_i2c_irq_event {
    /**< IRQ Event */
    I2C_SLAVE_EVT_TRANSMITTER_ADDR_MATCHED,
    I2C_SLAVE_EVT_BYTE_TRANSMITTED,
    I2C_SLAVE_EVT_RECEIVER_ADDR_MATCHED,
    I2C_SLAVE_EVT_BYTE_RECEIVED,
    I2C_SLAVE_EVT_STOP,
    I2C_SLAVE_EVT_ACK_FAILURE
}hal_i2c_irq_event_t;

/**
 * @brief hal_i2c_irq_err_t
 *
 * #I2C IRQ Event
 */
typedef enum _hal_i2c_irq_err {
    /**< IRQ Error */
    I2C_ERROR
}hal_i2c_irq_err_t;

/**
 * @brief hal_i2c_irq_priority_t
 *
 * #i2c irq priority define
 */
typedef enum _hal_i2c_irq_prio {
    HAL_I2C_P1 = 1,
    HAL_I2C_P2 = 2,
    HAL_I2C_P3 = 3,
}hal_i2c_irq_priority_t;

/**
 * @brief hal_i2c_status_t
 *
 * #I2C module status
 */
typedef enum _hal_i2c_status {
    I2C_READY,
    I2C_RX_RECEIVING,
    I2C_RX_COMPLETED,
    I2C_TX_TRASMITTING,
    I2C_TX_COMPLETED,  
    I2C_BUS_BUSY
}hal_i2c_status_t;

/**
 * @brief hal_i2c_return_t
 *
 * #I2C return type
 */
typedef enum _hal_i2c_return {
    I2C_OK,
    I2C_NULL_PTR,
    I2C_INVALID_CHANNEL,
    I2C_CHANNEL_BUSY,
    I2C_INVALID_LENGTH,
    I2C_UNKNOWN_ERROR,
}hal_i2c_return_t;

/**
 * @brief hal_i2c_omode_t
 *
 * I2C operation mode 
 */
typedef enum _hal_i2c_omode {
    I2C_MASTER  = 0,
    I2C_SLAVE   = 1
}hal_i2c_omode_t;

/**
 * @brief hal_i2c_amode_t
 *
 * I2C address mode 
 */
typedef enum _hal_i2c_amode {
    I2C_7BIT    = 0,
    I2C_8BIT    = 1,
    I2C_10BIT   = 2
}hal_i2c_amode_t;

/**
 * @brief hal_i2c_channel_t
 *
 * #i2c channel define
 */
typedef enum _hal_i2c_channel {
    HAL_I2C_CH1,
    HAL_I2C_CH2,
    HAL_I2C_CH3,
    HAL_I2C_MAX_IDX,
}hal_i2c_channel_t;

/**
 * @brief  hal_i2c_clockspeed_t 
 *
 * #i2c clock speed define
 */
typedef enum _hal_i2c_clockspeed {
    I2C_SPEED_100KHz    = 100000,
    I2C_SPEED_400KHz    = 400000
}hal_i2c_clockspeed_t;


/**
 * @brief hal_i2c_buffer_t
 *
 * #i2c buffer define
 */
typedef struct _hal_i2c_buffer {
    uint16_t     len;         /**< tx Data Length */
    uint8_t*     buf;         /**< tx Data buffer */   
}hal_i2c_buffer_t;


/**
 * @brief hal_i2c_buffer_t
 *
 * #i2c buffer define
 */
typedef struct _hal_i2c_master_buffer {
    hal_i2c_buffer_t   rxbuf;         /**< rx buffer */
    hal_i2c_buffer_t   txbuf;         /**< tx buffer */
    uint16_t           slave_addr;    /**< Slave address to access */
    void               (*hdl)(uint16_t status);          /**< notification handle when transfer is completed */
}hal_i2c_master_buffer_t;


/**
 * @brief  hal_i2c_config_t 
 *
 * #i2c configuration parameters
 */
typedef struct _hal_i2c_config {
    hal_i2c_clockspeed_t        clock_speed;                  /**< hal i2c clock speed */
    hal_i2c_amode_t             amode;                        /**< hal i2c address mode */
    hal_i2c_omode_t             omode;                        /**< hal i2c operation mode */
    hal_i2c_irq_priority_t      priority;                     /**< hal i2c irq priority */
    uint16_t                    owner_address;                /**< owner address for master and slave mode */
}hal_i2c_config_t;

/**
 * @brief hal_i2c_t
 *
 * #COM driver structure
 */
typedef struct _hal_i2c {
    hal_i2c_channel_t           chid;                  /**< hal i2c channel id */
    hal_i2c_config_t            cfg;                   /**< point to i2c configuration structure */
    void                        (*evt_isr)                         
                                (uint16_t evt_code, void *hdl);   /**< Event handler call back function be implemented by upper layer */
    void                        (*err_isr)                         
                                (uint16_t err_code, void *hdl);   /**< Error handler call back function be implemented by upper layer */    
}hal_i2c_t;


/* Exported functions ------------------------------------------------------- */

/**
 * @brief hal_i2c_init
 * The function shall initialize independence #I2C channel
 * @param #I2C Point to hal_i2c_t structure
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_init( hal_i2c_t * i2c );

/**
 * @brief hal_i2c_deinit
 * The function stops operation on independence #I2C channel
 * @param #I2C Point to hal_i2c_t structure
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_deinit( hal_i2c_t * i2c );

/**
 * @brief hal_i2c_master_tx
 * The function write data from master to slave
 * @param chid - I2C channel
 * @param buf - point to #hal_i2c_master_buffer_t buffer
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_master_tx( hal_i2c_channel_t chid, hal_i2c_master_buffer_t * buf );

/**
 * @brief hal_i2c_master_rx
 * The function reads data from slave
 * @param chid - I2C channel
 * @param buf - point to #hal_i2c_master_buffer_t buffer
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_master_rx( hal_i2c_channel_t chid, hal_i2c_master_buffer_t * buf );

/**
 * @brief hal_i2c_get_bus_status
 * The function gets I2C status 
 * @param chid - I2C channel
 * @return reference to #hal_i2c_status_t
 */
int hal_i2c_get_bus_status( hal_i2c_channel_t chid );

/**
 * @brief hal_i2c_get_bus_status
 * The function gets I2C bus status 
 * @param chid - I2C channel
 * @return reference to #hal_i2c_status_t
 */
int hal_i2c_get_status( hal_i2c_channel_t chid );

/**
 * @brief hal_i2c_slave_writefifo
 * The function sends one byte to hardware fifo
 * @param chid - I2C channel
 * @param byte Point to buffer or variable
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_slave_writefifo( hal_i2c_channel_t chid, uint8_t * byte );

/**
 * @brief hal_i2c_slave_readfifo
 * The function reads one byte from hardware fifo
 * @param chid - I2C channel
 * @param byte Point to buffer or variable
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_slave_readfifo( hal_i2c_channel_t chid, uint8_t * byte );

/**
 * @brief hal_i2c_enable_irq
 * The function enables hardware irq
 * @param chid - I2C channel
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_enable_irq( hal_i2c_channel_t chid );

/**
 * @brief hal_i2c_disable_irq
 * The function disables hardware interrupt
 * @param chid - I2C channel
 * @return reference to #hal_i2c_return_t
 */
int hal_i2c_disable_irq( hal_i2c_channel_t chid );


#endif /* _HAL_I2C_H_ */
