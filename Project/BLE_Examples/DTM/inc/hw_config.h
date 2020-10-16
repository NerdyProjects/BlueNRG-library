/**
  ******************************************************************************
  * @file    hw_config.h 
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
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ------------------------------------------------------------*/
#define UART_DR_ADDRESS        (UART_BASE)
#define DMA_CH_UART_TX          DMA_CH1
#define DMA_FLAG_TC_UART_TX     DMA_FLAG_TC1

#define DMA_CH_UART_RX          DMA_CH0
#define DMA_FLAG_TC_UART_RX     DMA_FLAG_TC0

/* SPI DMA channels */
#define DMA_CH_SPI_TX                   (DMA_CH5)
#define DMA_CH_SPI_RX                   (DMA_CH4)

/* UART port */
#define UART_TX_PIN       GPIO_Pin_8
#define UART_RX_PIN       GPIO_Pin_11
#define UART_CTS_PIN      GPIO_Pin_13
#define UART_RTS_PIN      GPIO_Pin_6

/* SPI port */
#define SPI_CS_PIN      GPIO_Pin_11
#define SPI_OUT_PIN     GPIO_Pin_2
#define SPI_IN_PIN      GPIO_Pin_3
#define SPI_CLCK_PIN    GPIO_Pin_0
#define GPIO_MODE_SPI   Serial0_Mode

#define SPI_IRQ_PIN     GPIO_Pin_7

/* DTM interface GPIO = GPIO_Pin_12
 * if 1 => DTM_INTERFACE_UART
 * if 0 => DTM_INTERFACE_SPI
 */
#define DTM_INTERFACE_GPIO      (GPIO_Pin_12)



/** Set the watchdog reload interval in [s] = (WDT_LOAD + 3) / (clock frequency in Hz). */
#define RC32K_FREQ		32768
#define RELOAD_TIME(SEC)        ((SEC*RC32K_FREQ)-3)

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void WDG_Configuration(void);

void HW_UART_Configuration(void);
void HW_UARTSLEEP_Configuration(void);
void HW_SPI_Configuration(void);

void GPIO_UARTSLEEP_Handler(void);
void GPIO_SPI_Handler(void);
void DMA_UARTSLEEP_Handler(void);
void DMA_UART_Handler(void);


uint8_t DMA_Rearm(DMA_CH_Type* DMAy_Channelx, uint32_t buffer, uint32_t size);
#define GPIO_CTS_Input() (GPIO_Init(&(GPIO_InitType){UART_CTS_PIN, GPIO_Input, DISABLE, DISABLE}))
void GPIO_CTS_Uart(void);
void GPIO_CTS_Irq(FunctionalState NewState);

#define NVIC_DisableCSnIrq()  (NVIC_Init(&(NVIC_InitType){GPIO_IRQn, MED_PRIORITY, DISABLE}))

#define NVIC_EnableCSnIrq()  (NVIC_Init(&(NVIC_InitType){GPIO_IRQn, MED_PRIORITY, ENABLE}))

#define NVIC_DisableRadioIrq()  (NVIC_Init(&(NVIC_InitType){BLUE_CTRL_IRQn, 0, DISABLE}))

#define NVIC_EnableRadioIrq()  (NVIC_Init(&(NVIC_InitType){BLUE_CTRL_IRQn, 0, ENABLE}))


#define GPIO_RTS_Uart()  (GPIO_Init(&(GPIO_InitType){UART_RTS_PIN, Serial1_Mode, DISABLE, DISABLE}))

#define GPIO_RTS_Output()   GPIO_SetBits(UART_RTS_PIN); \
                            GPIO_Init(&(GPIO_InitType){UART_RTS_PIN, GPIO_Output, DISABLE, DISABLE});


#endif /* HW_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
