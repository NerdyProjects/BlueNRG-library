/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  RF Application Team - AMG
  * @version V1.0.0
  * @date    August, 2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32xx_it.h" 
#include "SDK_EVAL_Config.h"
#include "clock.h"

/** @addtogroup STM32L1XX_BlueNRG_Applications
 *  @{
 */

/** @addtogroup SampleApp
 *  @{
 */
 
/** @defgroup INTERRUPT_HANDLER 
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
  * @brief  NMI_Handler This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  HardFault_Handler This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  SVC_Handler This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  DebugMon_Handler This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  PendSV_Handler This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  SysTick_Handler This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  Clock_Inc_Tick();
}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/

#ifdef SPI_INTERFACE
volatile extern uint8_t spi_irq_flag;
void DTM_SPI_EXTI_IRQHandler(void)
{
  /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(DTM_SPI_IRQ_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(DTM_SPI_IRQ_EXTI_LINE);
    
    spi_irq_flag = TRUE;
  }
  
}

/**
* @brief  This function handles TIM2 update interrupt.
* @param  None
* @retval None
*/
extern volatile uint32_t clock_time;
void TIM2_IRQHandler(void)
{
  /* Check whether update interrupt is pending */
  if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_UPDATE(TIM2);
    clock_time ++;
  }
}
#endif

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  if(LL_USART_IsActiveFlag_RXNE(USART2) == 1) {
//    LL_USART_ClearFlag_RXNE(USART2);// Check with STM32L1
    /* Trasfer complete: read data */
    uint8_t tmp_rx_data = LL_USART_ReceiveData8(USART2);
    
    SdkEval_IO_Receive_Data(&tmp_rx_data, 1);
  }
}

#ifdef DTM_UART

#define UART_ARRAY_SIZE 1024
extern uint8_t DTM_write_data[UART_ARRAY_SIZE];
extern uint16_t DTM_write_data_head;
extern uint16_t DTM_write_data_tail;
extern uint16_t DTM_write_data_size;

void USART1_IRQHandler(void)
{  
  if(LL_USART_IsActiveFlag_RXNE(DTM_USART) == 1) {
//    LL_USART_ClearFlag_RXNE(DTM_USART);// Check with STM32L1
    
    DTM_write_data[DTM_write_data_head] = LL_USART_ReceiveData8(DTM_USART);
    DTM_write_data_size++;
    DTM_write_data_head = (DTM_write_data_head + 1)%UART_ARRAY_SIZE;
  }
}


#ifdef DTM_UART_HW_FLOW_CTRL
void DTM_USART_EXTI_IRQHandler(void)
{
    /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(DTM_USART_CTS_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(DTM_USART_CTS_EXTI_LINE);
    
    if( LL_GPIO_IsInputPinSet(DTM_USART_CTS_GPIO_PORT, DTM_USART_CTS_PIN) == 0  ) {
      DTM_Config_UART_RTS();
    }
    else {
      DTM_UART_RTS_OUTPUT_HIGH();
      DTM_Config_GPIO_Output_RTS();
    }
  }
  
}
#endif

#endif

/**
 * @}
 */ 

/**
 * @}
 */ 

/**
 * @}
 */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/