/**
  ******************************************************************************
  * @file    DTM_Updater.h 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    July-2015
  * @brief   
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
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DTM_UPDATER_H
#define DTM_UPDATER_H

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_x_device.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  SPI_PROT_INIT_STATE = 0,                  /* Initialization phase         */
  SPI_PROT_CONFIGURED_STATE,                /* Configuration phase          */
  SPI_PROT_SLEEP_STATE,                     /* Sleep phase                  */
  SPI_PROT_CONFIGURED_HOST_REQ_STATE,       /* Host request phase           */
  SPI_PROT_CONFIGURED_EVENT_PEND_STATE,     /* Event pending phase          */
  SPI_PROT_WAITING_HEADER_STATE,            /* Waiting header phase         */
  SPI_PROT_HEADER_RECEIVED_STATE,           /* Header received phase        */
  SPI_PROT_WAITING_DATA_STATE,              /* Waiting data phase           */
  SPI_PROT_TRANS_COMPLETE_STATE,            /* Transaction complete phase   */
} SpiProtoType;


typedef void (*DTM_InterfaceHandler_Type)(void);

/* Exported constants --------------------------------------------------------*/
#define BUFFER_SIZE  (264) //(259+5)  /* 255 length + 4 ble header +5 SPI protocol header */

/* Exported macro ------------------------------------------------------------*/
#define SPI_STATE_TRANSACTION(NEWSTATE)       (spi_proto_state = NEWSTATE)
#define SPI_STATE_CHECK(STATE)                (spi_proto_state==STATE)
#define SPI_STATE_FROM(STATE)                 (spi_proto_state>=STATE)

#define DTM_INTERFACE_UART      (0)
#define DTM_INTERFACE_SPI       (1)
#define DTM_INTERFACE_UARTSLEEP (2)
#define DTM_INTERFACE_UNDEF     (3)

/* DTM interface GPIO = GPIO_Pin_12
 * if 1 => DTM_INTERFACE_UART
 * if 0 => DTM_INTERFACE_SPI
 */
#define DTM_INTERFACE_GPIO      ((uint32_t)0x00001000)  /*!< Pin 12 selected */

/* Exported functions ------------------------------------------------------- */
void updater(uint8_t reset_event, uint8_t dtm_interface);
void updater_init_uart(void);
void updater_init_spi(void);
#endif /* DTM_UPDATER_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/