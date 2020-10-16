/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  RF Application Team - AMG
  * @version V1.0.0
  * @date    27-April-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32xx_it.h" 
#include "SDK_EVAL_Config.h"
#include "hci_parser.h"
#include "clock.h"
#include "hci.h"
    
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
/*            Cortex-M0+ Processor Exceptions Handlers                         */
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

/**
  * @brief  BNRG_SPI_EXTI_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
#ifdef SPI_INTERFACE
void DTM_SPI_EXTI_IRQHandler(void)
{
    /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(DTM_SPI_IRQ_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(DTM_SPI_IRQ_EXTI_LINE);
    HCI_Isr();
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

#ifdef UART_INTERFACE

void USART1_IRQHandler(void)
{
  uint8_t new_data_received = 0;
  
  if(LL_USART_IsActiveFlag_RXNE(USART1) == 1) {
//    LL_USART_ClearFlag_RXNE(USART1);// Check with STM32L1
    
    new_data_received = LL_USART_ReceiveData8(DTM_USART);
    
    hci_input_event(&new_data_received, 1);
  }
  
  if(LL_USART_IsActiveFlag_ORE(USART1) == 1) {
    LL_USART_ClearFlag_ORE(USART1);
  }
}

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

void USART2_IRQHandler(void)
{
  if(LL_USART_IsActiveFlag_RXNE(USART2) == 1) {
//    LL_USART_ClearFlag_RXNE(USART2);// Check with STM32L1
    LL_USART_ReceiveData8(USART2);
  }
  if(LL_USART_IsActiveFlag_ORE(USART2) == 1) {
    LL_USART_ClearFlag_ORE(USART2);
    LL_USART_ReceiveData8(USART2);
  }
}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*
void PPP_IRQHandler(void)
{
}
*/

/**
 * @}
 */ 

/**
 * @}
 */ 

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
