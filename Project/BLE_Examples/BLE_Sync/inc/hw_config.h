/**
  ******************************************************************************
  * @file    hw_config.h 
  * @author  AMS - RF Application Team
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
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ------------------------------------------------------------*/

#define CE_TRIGGER_IN_PIN   GPIO_Pin_6
#define SYNC_OUT_PIN        GPIO_Pin_2

#define UART_DR_ADDRESS        (UART_BASE)
#define DMA_CH_UART_TX          DMA_CH1
#define DMA_FLAG_TC_UART_TX     DMA_FLAG_TC1
#define DMA_CH_UART_RX          DMA_CH0

#define DMA_IDLE        0
#define DMA_IN_PROGRESS 1

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t dma_state;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NVIC_Configuration(void);
void GPIO_Configuration(void);
void MFT_User_Init(void);
void MFT_Interrupt_Enable(void);
void MFT_Interrupt_Disable(void);
void MFT_Configuration(uint32_t offset);
void DMA_Configuration(void);
void DMA_Rearm(DMA_CH_Type* DMAy_Channelx, uint32_t buffer, uint32_t size);

#endif /* HW_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
