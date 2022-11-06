
/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : MTFX/Mode_4/main.c 
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : September-2015
* Description        : Code demostrating the MFT Mode 4 functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "bluenrg_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples BlueNRG1 Standard Peripheral Examples
  * @{
  */


/** @addtogroup MTFX_Examples MTFX Examples
  * @{
  */

/** @addtogroup MTFX_Mode4 MTFX Mode 4 Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/    
#define IO_CAPTURE_PIN_MFT1_TIMERA	3 /* IO3 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void MFT_Configuration(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program code
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* GPIO configuration */
  GPIO_Configuration();
  
  /* MFT configuration */
  MFT_Configuration();
  
  /* IO3 for TnCRB */
  MFT_SelectCapturePin(MFT1_TIMERB, IO_CAPTURE_PIN_MFT1_TIMERA);
  
  /* Timer input sensitive for TnB to the rising edge. No matter for TnA pin. */
  MFT_TnEDGES(MFT1, MFT_RISING, MFT_RISING);
  
  /* Connect PWM output from timer to TnA pin */
  MFT_TnXEN(MFT1, MFT_TnA, ENABLE);
  
  /* Enable preset for CNT1 TnB event */
  MFT_TnXEN(MFT1, MFT_TnB, ENABLE);
  
  /* Enable MFT interrupts */
  MFT_EnableIT(MFT1, MFT_IT_TNB | MFT_IT_TND, ENABLE);
  
  /* Start MFT1 timer */
  MFT_Cmd(MFT1, ENABLE);
  
  /* Infinite loop */
  while (1)
  {
  }
}


/**
  * @brief  GPIO Configuration.
  *          Configure outputs GPIO pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;

  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

  /* Configure PWM pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
}

/**
  * @brief  MFT_Configuration.
  *          Configure MFT.
  * @param  None
  * @retval None
  */
void MFT_Configuration()
{
  MFT_InitType timer_init;
  NVIC_InitType NVIC_InitStructure;
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1, ENABLE);
  
  /* Init MFT1 in mode 4 */
  timer_init.MFT_Mode = MFT_MODE_4;
  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
  timer_init.MFT_Clock2 = MFT_PRESCALED_CLK;
  
#if (HS_SPEED_XTAL == HS_SPEED_XTAL_32MHZ)
	timer_init.MFT_Prescaler = 160-1;      /* 5 us clock */
#elif (HS_SPEED_XTAL == HS_SPEED_XTAL_16MHZ)
	timer_init.MFT_Prescaler = 80-1;       /* 5 us clock */
#endif
  
  timer_init.MFT_CRA = 10000-1; /* 250 ms */
  timer_init.MFT_CRB = 0xFFFF;
  
  MFT_Init(MFT1, &timer_init);
  MFT_SetCounter(MFT1, 10000-1, 0xFFFF);
  
  /* Enable MFT Interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = MFT1A_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = MFT1B_IRQn;
  NVIC_Init(&NVIC_InitStructure);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

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

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
