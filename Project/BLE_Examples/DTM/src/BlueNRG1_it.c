/**
******************************************************************************
* @file    BlueNRG1_it.c 
* @author  VMA RF Application Team
* @version V1.1.0
* @date    27-March-2018
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
#include "hci_parser.h"
#include "hw_config.h"
#include "transport_layer.h"
#include "miscutil.h" 

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
volatile extern uint8_t DTM_INTERFACE;

/* GPIO_HandlerCallback function */
void dtm_void_function(void) {};
const DTM_InterfaceHandler_Type GPIO_HandlerCallback[] = {
        dtm_void_function,
        GPIO_SPI_Handler,
        GPIO_UARTSLEEP_Handler};

/* DMA_HandlerCallback function */
const DTM_InterfaceHandler_Type DMA_HandlerCallback[] = {
        DMA_UART_Handler,
        dtm_void_function, 
        DMA_UARTSLEEP_Handler};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
NOSTACK_FUNCTION(NORETURN_FUNCTION(void NMI_Handler(void)))
{
  HAL_CrashHandler(__get_MSP(), NMI_SIGNATURE);  
  /* Go to infinite loop when NMI exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Hard Fault exception.
*/
NOSTACK_FUNCTION(NORETURN_FUNCTION(void HardFault_Handler(void)))
{
  HAL_CrashHandler(__get_MSP(), HARD_FAULT_SIGNATURE);  
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
extern volatile uint32_t systick_counter;
void SysTick_Handler(void)
{
  systick_counter++;
}


/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void GPIO_Handler(void)
{
  GPIO_HandlerCallback[DTM_INTERFACE]();
}



void UART_Handler(void)
{  
  uint8_t data;
  
  while (UART_GetITStatus(UART_IT_RX) != RESET)
  {
    /* The Interrupt flag is cleared from the FIFO manager */
    data = UART_ReceiveData() & 0xFF;
    hci_input(&data, 1);
  }
  
  if (UART_GetITStatus(UART_IT_TX) != RESET)
  {
    /* Clear the interrupt */
    UART_ClearITPendingBit(UART_IT_TX);
  }
}


/**
* @brief  This function handles DMA Handler.
*/
void DMA_Handler(void)
{
  DMA_HandlerCallback[DTM_INTERFACE]();
}



/**
* @brief  This function handles GPIO interrupt request.
* @param  None
* @retval None
*/

/* irq_count used for the aci_hal_transmitter_test_packets_process() command implementation */
uint32_t irq_count = 0; 

void Blue_Handler(void)
{
  // Call RAL_Isr
  RAL_Isr();
  irq_count++;
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
