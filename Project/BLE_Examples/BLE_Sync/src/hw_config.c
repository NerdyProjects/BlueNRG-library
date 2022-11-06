/**
******************************************************************************
* @file    hw_config.c
* @author  AMS - RF Application Team
* @version V1.0.0
* @date    April-2020
* @brief   HW configuration functions
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
#include "sync.h"

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
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


uint8_t volatile dma_state = DMA_IDLE;

/**
* @brief  Multi Function Timer initialization
*
*         Configure the MFT in order to generate a pulse with an offset
*         compared to another one in input.
* @retval None
*/
void MFT_User_Init(void)
{
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1, ENABLE);
  
  MFT_SetCounter2(MFT1, 0);
  
  MFT_PulseTrainTriggerSelect(MFT1, MFT_TnB_TRIGGER);
  
  MFT_SelectCapturePin(MFT1_TIMERB, 6);  // Pin6
  
  MFT_TnEDGES(MFT1, MFT_RISING, MFT_RISING);
  
  // Enable interrupt when TnCRA is loaded, i.e. when timer expires with value loaded from TnCRB.
  MFT_EnableIT(MFT1, MFT_IT_TNA, ENABLE);
  
  /* Enable interrupt in NVIC. */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = MFT1A_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}
  
void MFT_Interrupt_Enable(void)
{
  NVIC_EnableIRQ(MFT1A_IRQn);
}

void MFT_Interrupt_Disable(void)
{
  NVIC_DisableIRQ(MFT1A_IRQn);
}

/**
* @brief  Multi Function Timer initialization
*
*         Configure the MFT in order to generate a pulse with an offset
*         compared to another one in input.
* @param  offset Delay between the input pulse and the output pulse.
*                Offset = (N/10) us
* @retval None
*/
void MFT_Configuration(uint32_t offset)
{
  MFT_InitType timer_init;
  
  MFT_Cmd(MFT1, DISABLE);
  
  MFT_SetCounter2(MFT1, 0);
  
  MFT_StructInit(&timer_init);  
  
  timer_init.MFT_Mode = MFT_MODE_1a;  
  /* MFT1 configuration */
  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
  timer_init.MFT_Clock2 = MFT_PRESCALED_CLK;
#if HS_SPEED_XTAL==HS_SPEED_XTAL_32MHZ
  timer_init.MFT_Prescaler = 4-1;      /* 125 ns clock */
#else
  timer_init.MFT_Prescaler = 2-1;      /* 125 ns clock */
#endif
  
  timer_init.MFT_CRA = PULSE_DURATION*8;
  
  timer_init.MFT_CRB = (offset*8)/10;
  MFT_SetCounter1(MFT1, timer_init.MFT_CRB);

  MFT_Init(MFT1, &timer_init);
  
  MFT_TnXEN(MFT1, MFT_TnA, ENABLE);
  MFT_TnXEN(MFT1, MFT_TnB, ENABLE);  
  
  MFT_Cmd(MFT1, ENABLE);  
}

/**
* @brief  GPIO Configuration.
*	  Configure outputs GPIO pins.
* @param  None
* @retval None
*/
void GPIO_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;
  
  /* GPIO Periph clock enable */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Init Structure */
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SYNC_OUT_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;  
  GPIO_Init(&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = CE_TRIGGER_IN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;  
  GPIO_Init(&GPIO_InitStructure);
  
  /* Configure the GPIO pin for debug */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = ENABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);
}

/**
* @brief  DMA Configuration.
*	  Configure the DMA.
* @param  None
* @retval None
*/
#if DMA_ENABLED
void DMA_Configuration(void)
{  
  DMA_InitType DMA_InitStructure;
  
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

/**
* @brief  DMA rearm
*	        Restart a DMA transfer
* @param  DMAy_Channelx - DMA channel
*         buffer - buffer containing data to be transmitted
*         size - size of data to be transmitted
* @retval None
*/
void DMA_Rearm(DMA_CH_Type* DMAy_Channelx, uint32_t buffer, uint32_t size)
{
  DMAy_Channelx->CCR_b.EN = RESET;
  
  DMAy_Channelx->CMAR = buffer;
  DMAy_Channelx->CNDTR = size;
  
  dma_state = DMA_IN_PROGRESS;
  
  DMAy_Channelx->CCR_b.EN = SET;
}
#endif

