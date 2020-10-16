/**
******************************************************************************
* @file    UART/Interrupt/main.c
* @author  VMA RF Application Team
* @version V1.1.0
* @date    27-March-2018
* @brief   HW configuration for the GUI application
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
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_x_device.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "hw_config.h"
#include "transport_layer.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup UART Interrupt Example
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern circular_fifo_t event_fifo;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  WDG configuration routine
* @param  None
* @retval None
*/
void WDG_Configuration(void)
{
#ifdef WATCHDOG  
  /* Enable watchdog clocks  */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_WDG, ENABLE);  
  
  /* Set watchdog reload period at 1 second */
  WDG_SetReload(RELOAD_TIME(WATCHDOG_TIME));
  
  /* Watchdog enable */
  WDG_Enable();
#endif
}


void HW_UART_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;
  UART_InitType UART_InitStructure;
  DMA_InitType DMA_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Configure GPIO_Pin_8 and GPIO_Pin_11 as UART_TXD and UART_RXD */
  GPIO_InitStructure.GPIO_Pin = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  
  GPIO_Init(&GPIO_InitStructure);
  
  /* Enable the UART Interrupt */
  NVIC_Init(&(NVIC_InitType){UART_IRQn, MED_PRIORITY, ENABLE});
  
  /* Enable the Blue Controller Interrupt */
  NVIC_EnableRadioIrq();
  
  /* Enable UART clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART, ENABLE);
  
  /* Init Structure */
  UART_StructInit(&UART_InitStructure);
  
  /* Configure UART */
  UART_InitStructure.UART_BaudRate = 115200;
  
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStructure.UART_FifoEnable = DISABLE;
  UART_Init(&UART_InitStructure);
  
  /* Enable the RX FIFO and setup the trigger points for receive FIFO interrupt to every byte */
  UART->LCRH_RX_b.FEN_RX = ENABLE;
  UART->IFLS_b.RXIFLSEL = 0;
  
  /* Enable UART interrupts */
  UART_ITConfig(UART_IT_RX, ENABLE);
  
  /* Enable UART */
  UART_Cmd(ENABLE);
  
#ifndef NO_DMA
  /* Configure DMA peripheral */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);
  
  /* Configure DMA SPI TX channel */
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
  
  DMA_InitStructure.DMA_PeripheralBaseAddr = UART_DR_ADDRESS;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_CH_UART_TX, &DMA_InitStructure);

  /* Enable DMA_CH_UART_TX Transfer Complete interrupt */
  DMA_FlagConfig(DMA_CH_UART_TX, DMA_FLAG_TC, ENABLE);
  
  /* DMA_CH_UART_RX Initialization */
  DMA_Cmd(DMA_CH_UART_RX, DISABLE);
  
  /* Enable UART_DMAReq_Tx */
  UART_DMACmd(UART_DMAReq_Tx, ENABLE);
  
  /* Enable the UARTx Interrupt */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}


void HW_SPI_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;
  DMA_InitType DMA_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);
  
  /** Configure GPIO pins for SPI.
  * GPIO_Pin_11 = CS
  * GPIO_Pin_0 = CLK
  * GPIO_Pin_2 = MOSI
  * GPIO_Pin_3 = MISO
  */
  GPIO_InitStructure.GPIO_Pin = SPI_CLCK_PIN | SPI_OUT_PIN | SPI_IN_PIN | SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_SPI;
  GPIO_Init(&GPIO_InitStructure);
  
  /* GPIO_Pin_7 = IRQ */
  GPIO_InitStructure.GPIO_Pin = SPI_IRQ_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_Init(&GPIO_InitStructure);
  GPIO_ResetBits(SPI_IRQ_PIN);
  
  /* Configures EXTI line */
  GPIO_EXTIConfig(&(GPIO_EXTIConfigType){SPI_CS_PIN, GPIO_IrqSense_Edge, GPIO_Event_Low});
  
  /* Clear pending interrupt */
  GPIO_ClearITPendingBit(SPI_CS_PIN);
  
  /* Enable the SPI Interrupt */
  NVIC_Init(&(NVIC_InitType){GPIO_IRQn, MED_PRIORITY, ENABLE});

  /* Enable the Blue Controller Interrupt */
  NVIC_EnableRadioIrq();
  
  /* SPI Configuration */
  SPI_InitType SPI_InitStructure;
  
  /* Enable SPI and GPIO clocks */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_SPI, ENABLE);
  
  /* Configure SPI in master mode */
  SPI_StructInit(&SPI_InitStructure);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_Init(&SPI_InitStructure);
      
  /* Clear RX and TX FIFO */
  SPI_ClearTXFIFO();
  SPI_ClearRXFIFO();
  
  /* Enable SPI interrupts */
  GPIO_EXTICmd(SPI_CS_PIN, ENABLE);
  
  /* Enable SPI */
  SPI_SendData(0xFF);
  SPI_Cmd(ENABLE);
  
  /* Configure DMA peripheral */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);
  
  /* Configure DMA SPI TX channel */
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (SPI_BASE + 0x08); 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_Init(DMA_CH_SPI_TX, &DMA_InitStructure);
    
  /* Configure DMA SPI RX channel */
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_CH_SPI_RX, &DMA_InitStructure);

  /* Enable the SPI RX DMA channel */
  DMA_Cmd(DMA_CH_SPI_RX, ENABLE);
  
  /* Enable SPI_TX/SPI_RX DMA requests */
  SPI_DMACmd(SPI_DMAReq_Tx | SPI_DMAReq_Rx, ENABLE);
  
  SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_STATE);
}


void HW_UARTSLEEP_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;
  UART_InitType UART_InitStructure;
  DMA_InitType DMA_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Configure GPIO_Pin_8 and GPIO_Pin_11 as UART_TXD and UART_RXD */
  GPIO_InitStructure.GPIO_Pin = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  
  GPIO_Init(&GPIO_InitStructure);

  /* Configure GPIO_Pin_6 (UART_RTS) as GPIO output */
  GPIO_RTS_Output(); 
  
  /* Configure GPIO_Pin_13 (UART_CTS) as GPIO input */
  GPIO_CTS_Input();

  /* Enable the UART Interrupt */
  NVIC_Init(&(NVIC_InitType){UART_IRQn, MED_PRIORITY, ENABLE});
  
  /* Configures EXTI line */
  GPIO_EXTIConfig(&(GPIO_EXTIConfigType){UART_CTS_PIN, GPIO_IrqSense_Edge, GPIO_Event_Low});
  
  /* Enable the SPI Interrupt */
  NVIC_Init(&(NVIC_InitType){GPIO_IRQn, MED_PRIORITY, ENABLE});

  GPIO_CTS_Irq(ENABLE);

  /* Enable the Blue Controller Interrupt */
  NVIC_EnableRadioIrq();
  
  /** Enable UART clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_UART, ENABLE);
  
  /** Init Structure */
  UART_StructInit(&UART_InitStructure);
  
  /** Configure UART */
  UART_InitStructure.UART_BaudRate = 115200;
  UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_RTS_CTS;
  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStructure.UART_FifoEnable = DISABLE;
  UART_Init(&UART_InitStructure);
  
  /* Enable the RX FIFO and setup the trigger points for receive FIFO interrupt to every byte */
  UART->LCRH_RX_b.FEN_RX = ENABLE;
  UART->IFLS_b.RXIFLSEL = 0;
  
  /* Enable UART interrupts */
  UART_ITConfig(UART_IT_RX, ENABLE);
  
  /* Enable UART */
  UART_Cmd(ENABLE);
  
  /* Configure DMA peripheral */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);
  
  /* Configure DMA SPI TX channel */
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
  
  DMA_InitStructure.DMA_PeripheralBaseAddr = UART_DR_ADDRESS;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA_CH_UART_TX, &DMA_InitStructure);

  /* Enable DMA_CH_UART_TX Transfer Complete interrupt */
  DMA_FlagConfig(DMA_CH_UART_TX, DMA_FLAG_TC, ENABLE);
  
  /* DMA_CH_UART_RX Initialization */
  DMA_Cmd(DMA_CH_UART_RX, DISABLE);
  
  /* Enable UART_DMAReq_Tx */
  UART_DMACmd(UART_DMAReq_Tx, ENABLE);
  
  /* Enable the UARTx Interrupt */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



void GPIO_CTS_Irq(FunctionalState NewState)
{
  if (NewState != DISABLE) {
    GPIO_ClearITPendingBit(UART_CTS_PIN);  
    GPIO_EXTICmd(UART_CTS_PIN, ENABLE);
  }
  else {
    GPIO_EXTICmd(UART_CTS_PIN, DISABLE);
  }
}

void GPIO_CTS_Uart(void)
{
  GPIO_InitType GPIO_InitStructure;
  
  /** Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Configure CTS pin as UART_CTS */
  GPIO_InitStructure.GPIO_Pin = UART_CTS_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  
  GPIO_CTS_Irq(DISABLE);
  
  GPIO_Init(&GPIO_InitStructure);
}



uint8_t DMA_Rearm(DMA_CH_Type* DMAy_Channelx, uint32_t buffer, uint32_t size)
{
  /* DMA_CH_SPI_TX reset */
  DMAy_Channelx->CCR_b.EN = RESET;
  
  /* Rearm the DMA_CH_SPI_TX */
  DMAy_Channelx->CMAR = buffer;
  DMAy_Channelx->CNDTR = size;
  
  /* DMA_CH_SPI_TX enable */
  DMAy_Channelx->CCR_b.EN = SET;
  
  return 0;
}

extern uint16_t command_fifo_dma_len;
extern uint32_t systick_counter_prev;

void GPIO_SPI_Handler(void)
{
  if(GPIO_GetITPendingBit(SPI_CS_PIN) == SET) {
    
    /* Edge mode is the normal mode.
     * Level mode is for SLEEP mode because:
     * when in sleep mode, all the registers setting are lost
     * the wake up occurs when the SPI_CS_PIN is low and this trigger also the irq.
     * This irq will be lost if in edge sensitive mode, because
     * once the BlueNRG-1 is woken up, the restore of the registers setting is done,
     * and so the irq setting on edge detection, but the event is lost due to this delay.
     * So, before go in SLEEP state, the sensitive is changed to the level.
     * In this way, once the restore of the registers setting is done, the event is not lost.
     */
    if(READ_BIT(GPIO->IS, SPI_CS_PIN)) {     /* if level sensitive */
      CLEAR_BIT(GPIO->IS, SPI_CS_PIN);       /* EDGE sensitive */
      DEBUG_NOTES(EDGE_SENSITIVE);
    }
    GPIO_ClearITPendingBit(SPI_CS_PIN);
    
    /* CS pin rising edge - close SPI communication */
    if(READ_BIT(GPIO->IEV, SPI_CS_PIN) != 0) {
      DEBUG_NOTES(GPIO_CS_RISING);
      CLEAR_BIT(GPIO->IEV, SPI_CS_PIN);
      
      if(SPI_STATE_FROM(SPI_PROT_CONFIGURED_EVENT_PEND_STATE)) {
        GPIO_ResetBits(SPI_IRQ_PIN);
        systick_counter_prev = 0;
        SysTick_State(DISABLE);
        SPI_STATE_TRANSACTION(SPI_PROT_TRANS_COMPLETE_STATE);
        DEBUG_NOTES(SPI_PROT_TRANS_COMPLETE);
        
        /* Pass the number of data received in fifo_command */
        advance_spi_dma((command_fifo_dma_len - DMA_GetCurrDataCounter(DMA_CH_SPI_RX)));
      }
    }
    /* CS pin falling edge - start SPI communication */
    else {
      DEBUG_NOTES(GPIO_CS_FALLING);
      SET_BIT(GPIO->IEV, SPI_CS_PIN);
      
      if(SPI_STATE_CHECK(SPI_PROT_CONFIGURED_EVENT_PEND_STATE)) {
        SPI_STATE_TRANSACTION(SPI_PROT_WAITING_HEADER_STATE);
      }
      else if(SPI_STATE_CHECK(SPI_PROT_SLEEP_STATE) || SPI_STATE_CHECK(SPI_PROT_CONFIGURED_STATE)) {
        SPI_ClearTXFIFO();
        SPI_SendData(0xFF);
        SPI_STATE_TRANSACTION(SPI_PROT_CONFIGURED_HOST_REQ_STATE);
      }
    }
    
  }
  
}


void GPIO_UARTSLEEP_Handler(void)
{
  if(GPIO_GetITPendingBit(UART_CTS_PIN) == SET) {
    
    if(READ_BIT(GPIO->IS, UART_CTS_PIN)) {     /* if level sensitive */
      CLEAR_BIT(GPIO->IS, UART_CTS_PIN);       /* EDGE sensitive */
    }
    GPIO_ClearITPendingBit(UART_CTS_PIN);
    
    if(READ_BIT(GPIO->IEV, UART_CTS_PIN) != 0) {
      CLEAR_BIT(GPIO->IEV, UART_CTS_PIN);
      GPIO_RTS_Output();
    }
    else {
      SET_BIT(GPIO->IEV, UART_CTS_PIN);
      GPIO_RTS_Uart();
    }
  }
}

void DMA_UART_Handler(void)
{
  uint8_t *ptr;
  uint16_t size;
  
  /* Check DMA_CH_UART_TX Transfer Complete interrupt */
  if(DMA_GetFlagStatus(DMA_FLAG_TC_UART_TX)) {
    DMA_ClearFlag(DMA_FLAG_TC_UART_TX);
    
    /* DMA1 finished the transfer of SrcBuffer */
    dma_state = DMA_IDLE;
    
    /* DMA_CH disable */
    DMA_CH_UART_TX->CCR_b.EN = RESET;
    DEBUG_NOTES(DMA_TC);
    
    fifo_discard_var_len_item(&event_fifo);
    
    if(fifo_size(&event_fifo) > 0) {
      if (fifo_get_ptr_var_len_item(&event_fifo, &size, &ptr) == 0) {
        transport_layer_uart_send_data(ptr+FIFO_ALIGNMENT, size);
      }
    }
    
  }
}


void DMA_UARTSLEEP_Handler(void)
{
  uint8_t *ptr;
  uint16_t size;
  
  /* Check DMA_CH_UART_TX Transfer Complete interrupt */
  if(DMA_GetFlagStatus(DMA_FLAG_TC_UART_TX)) {
    DMA_ClearFlag(DMA_FLAG_TC_UART_TX);
    
    /* DMA1 finished the transfer of SrcBuffer */
    dma_state = DMA_IDLE;
    
    /* DMA_CH disable */
    DMA_CH_UART_TX->CCR_b.EN = RESET;
    DEBUG_NOTES(DMA_TC);
    
    static uint32_t uart_sleep_cnt = 0;
    while( (UART_GetFlagStatus(UART_FLAG_TXFE) == RESET) || (UART_GetFlagStatus(UART_FLAG_BUSY) == SET) ) {
      uart_sleep_cnt++;
      if(uart_sleep_cnt > 0xFFFFF) break;
    }
    
    GPIO_SetBits(UART_RTS_PIN);     // high RTS
    GPIO_CTS_Input();
    GPIO_CTS_Irq(DISABLE);
    while(GPIO_ReadBit(UART_CTS_PIN) == Bit_RESET);    // wait for CTS high
    GPIO_CTS_Irq(ENABLE);
    
    if(uart_sleep_cnt< 0xFFFFF) {
      fifo_discard_var_len_item(&event_fifo);
      
      if((fifo_size(&event_fifo) > 0)  && (GPIO_ReadBit(UART_CTS_PIN) == Bit_SET) ) {
        if (fifo_get_ptr_var_len_item(&event_fifo, &size, &ptr) == 0) {
          transport_layer_uartsleep_send_data(ptr+FIFO_ALIGNMENT, size);
        }
      }
    }
    uart_sleep_cnt = 0;
  }
}
