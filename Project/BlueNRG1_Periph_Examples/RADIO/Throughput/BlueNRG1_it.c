/**
******************************************************************************
* @file    BlueNRG1_it.c 
* @author  VMA RF Application Team
* @version V1.0.1
* @date    April-2018
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
#include "SDK_EVAL_Com.h"

#include "clock.h"

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
/*NORETURN_FUNCTION(void NMI_Handler(void))
{
  CRASH_HANDLER(NMI_SIGNATURE);
  
  NVIC_SystemReset();
  //Go to infinite loop when Hard Fault exception occurs 
  while (1)
  {}
}*/

/**
* @brief  This function handles Hard Fault exception.
*/
/*NORETURN_FUNCTION(void HardFault_Handler(void))
{
  CRASH_HANDLER(HARD_FAULT_SIGNATURE);
  
  NVIC_SystemReset();
  //Go to infinite loop when Hard Fault exception occurs /
  while (1)
  {}
}*/

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
//  SysCount_Handler();
}


/******************************************************************************/
/*                 BlueNRG1LP Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_BlueNRG1lp.s).                                               */
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
  /* RAL_Isr();*/
  RADIO_IRQHandler();
}

uint32_t timer_reload = 0;

void MFT1A_Handler(void)
{
  if ( MFT_StatusIT(MFT1,MFT_IT_TNA) != RESET )
  { 
    timer_reload++;
    
    /** Clear MFT2 pending interrupt A */
    MFT_ClearIT(MFT1, MFT_IT_TNA);
  }
  if ( MFT_StatusIT(MFT1,MFT_IT_TNB) != RESET )
  { 
    timer_reload++;
    
    /** Clear MFT2 pending interrupt B */
    MFT_ClearIT(MFT1, MFT_IT_TNB);
  }  
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
