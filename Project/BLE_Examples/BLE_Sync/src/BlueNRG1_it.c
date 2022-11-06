/**
  ******************************************************************************
  * @file    BlueNRG1_it.c 
  * @author  AMS - RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Com.h"
#include "clock.h"
#include "hw_config.h"
#include "vclock.h"
#include "sync.h"
    
/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Examples
  * @{
  */ 

/** @addtogroup GPIO_IOToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
  SysCount_Handler();
}

/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_BlueNRG1.c).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void UART_Handler(void)
{  
  SdkEvalComIOUartIrqHandler();
}

void Blue_Handler(void)
{
  // Disable Blue controller signal on GPIO6 before any radio ISR
  uint8_t i = 6;
  uint8_t mode = 0;
  MODIFY_REG(GPIO->MODE0, (0xF<<(i*4)), (mode << (i*4)) );

  //Call RAL_Isr
  RAL_Isr();
}

/**
* @brief  This function handles DMA Handler.
*/
void DMA_Handler(void)
{
  /* Check DMA_CH_UART_TX Transfer Complete interrupt */
  if(DMA_GetFlagStatus(DMA_FLAG_TC_UART_TX)) {
    DMA_ClearFlag(DMA_FLAG_TC_UART_TX);
    
    /* DMA1 finished the transfer of SrcBuffer */
    dma_state = DMA_IDLE;
    
    /* DMA_CH disable */
    DMA_CH_UART_TX->CCR_b.EN = RESET;
    
  }
}

#define MASTER_ROLE   0
#define SLAVE0_ROLE   1
#define SLAVE1_ROLE   2

#if ROLE!=MASTER_ROLE
extern uint16_t event_counter;
#endif

void MFT1A_Handler(void)
{
  uint32_t curr_systime = HAL_VTimerGetCurrentTime_sysT32();
  
#if ROLE==MASTER_ROLE
  extern uint8_t resync_vclock;
  
  if(resync_vclock){
    // First sync event
    ResyncVClock();
    resync_vclock = FALSE;
  }
  else {
    SaveSyncEventOnMaster(curr_systime, get_connection_event_counter(0), get_connection_event_counter(1));
  }
#else
  SaveSyncEventOnSlave(curr_systime, event_counter);
#endif
  
  MFT_ClearIT(MFT1, MFT_IT_TNA);  
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
