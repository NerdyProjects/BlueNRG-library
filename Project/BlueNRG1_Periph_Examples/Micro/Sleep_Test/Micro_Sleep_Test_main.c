/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : Micro/Sleep_Test/Micro_Sleep_Test_main.c 
* Author             : AMS - RF Application Team
* Version            : V2.0.0
* Date               : 10-January-2020
* Description        : BlueNRG-1,2 main file for sleep test based on Virtual/Sleep
*                      timer
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "sleep.h"
#include "SDK_EVAL_Led.h"
#include "osal.h"
#include "vtimer.h"

/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)

/* Calibration must be done */
#define INITIAL_CALIBRATION         TRUE

#define CALIBRATION_INTERVAL        1000

#else

/* No Calibration */
#define INITIAL_CALIBRATION         FALSE
#define CALIBRATION_INTERVAL        0

#endif


/** @addtogroup BlueNRG1_StdPeriph_Examples BlueNRG1 Standard Peripheral Examples
  * @{
  */

/** @addtogroup Micro_Examples Micro Examples
  * @{
  */

/** @addtogroup Micro_SleepTest  Micro Sleep Test Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define WAKEUP_TIMEOUT 5000 // 5 sec 

static uint8_t wakeup_source = 0;
static uint8_t wakeup_level = 0;

static VTIMER_HandleType timer_handle;


/* Extern function prototypes ------------------------------------------------*/
extern uint8_t __io_getcharNonBlocking(uint8_t *data);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


void help(void)
{
  printf("\r\n");
  printf("s:   Standby - SLEEPMODE_NOTIMER: wake on uart/button PUSH1 (Not supported on STEVAL-IDB007V1M)\r\n");
  printf("t:   Sleep - SLEEPMODE_WAKETIMER: wake on uart/timeout 5 sec/button PUSH1\r\n");
  printf("l:   Toggle led DL1\r\n");
  printf("p:   Print Hello World message\r\n");
  printf("r:   Reset the BlueNRG-1\r\n");
  printf("?:   Display this help menu\r\n");
  printf("PUSH1 button: toggle led DL1");
  printf("\r\n> ");
}
   
static void user_set_wakeup_source(SdkEvalButton Button)
{
  if (Button == BUTTON_1)
  {
     wakeup_source |= WAKEUP_IO13;
     wakeup_level |= (WAKEUP_IOx_LOW << WAKEUP_IO13_SHIFT_MASK);
  }
}


void printWakeupSource(void)
{
  uint16_t src;
  
  printf("WAKEUP Reason = ");

  src = BlueNRG_WakeupSource();
  
  if (src & WAKEUP_IO9)
    printf("IO-9 ");
  
  if (src & WAKEUP_IO10)
    printf("IO-10 ");

  if (src & WAKEUP_IO11)
    printf("IO-11 ");

  if (src & WAKEUP_IO12)
    printf("IO-12 ");

  if (src &WAKEUP_IO13)
    printf("IO-13 ");

  if (src & WAKEUP_SLEEP_TIMER1)
    printf("SLEEP TIMER1 ");

  if (src & WAKEUP_SLEEP_TIMER2)
    printf("SLEEP TIMER2 ");

  if (src & WAKEUP_BOR)
    printf("BOR ");

  if (src & WAKEUP_POR)
    printf("POR ");

  if (src & WAKEUP_SYS_RESET_REQ)
    printf("SYS RESET REQ ");

  if (src & WAKEUP_RESET_WDG)
    printf("WDG ");

  if (src & NO_WAKEUP_RESET)
    printf("NO WAKEUP RESET ");
  printf ("\r\n");
}

void HAL_VTimerTimeoutCallback(void *handle)
{
  /* Add app code to execute @ Sleep timeout */
}
 
/* Enable the Standby - SLEEPMODE_NOTIMER */
void sleep(void)
{
  uint8_t ret;

  wakeup_source = WAKEUP_IO11;
  wakeup_level = (WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK);

  user_set_wakeup_source(USER_BUTTON); 
	
  printf("\r\nEnter to Standby - SLEEPMODE_NOTIMER\r\n");
  while(SdkEvalComUARTBusy() == SET);
  ret = BlueNRG_Sleep(SLEEPMODE_NOTIMER, wakeup_source, wakeup_level);
  if (ret != SUCCESS) {
    printf("BlueNRG_Sleep() error 0x%02x\r\n", ret);
    while(1);
  }
  printWakeupSource();
}

/* Enable the Sleep - SLEEPMODE_WAKETIMER */
void sleep_timer(void)
{
  uint8_t ret;

  wakeup_source = WAKEUP_IO11;
  wakeup_level = (WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK);

  user_set_wakeup_source(USER_BUTTON); 
  
  printf("\r\nEnter to Sleep - SLEEPMODE_WAKETIMER\r\n");
  HAL_VTimer_Stop(&timer_handle);
  timer_handle.callback = HAL_VTimerTimeoutCallback;
  ret = HAL_VTimerStart_ms(&timer_handle, WAKEUP_TIMEOUT);
  if (ret != SUCCESS) {
    printf("HAL_VTimerStart_ms() error 0x%02x\r\n", ret);
    while(1);
  }

  while(SdkEvalComUARTBusy() == SET);
  ret = BlueNRG_Sleep(SLEEPMODE_WAKETIMER, wakeup_source, wakeup_level);
  if (ret != SUCCESS) {
    printf("BlueNRG_Sleep() error 0x%02x\r\n", ret);
    while(1);
  }
  printWakeupSource();
 
}


int main(void)
{
  uint8_t charRead;
  
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Enable UART */
  SdkEvalComIOConfig(SdkEvalComIOProcessInputData);
  
  /* Configure the BUTTON_1 button */
  SdkEvalPushButtonInit(BUTTON_1);
  /* IRQ_ON_FALLING_EDGE is set since wakeup level is low (mandatory setup for wakeup sources);  
        IRQ_ON_RISING_EDGE is the specific application level */ 
  SdkEvalPushButtonIrq(BUTTON_1, IRQ_ON_BOTH_EDGE); 

  /* Led DL1 configuration */
  SdkEvalLedInit(LED1);
  SdkEvalLedOff(LED1);

  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
 
  printf("Sleep Test\r\n");
  printf("Enter ? for list of commands\r\n");

  while(1) {

    /* Timer tick */
    HAL_VTIMER_Tick();


    if (__io_getcharNonBlocking(&charRead)) {
      switch (charRead) {
      case 's':
        sleep();
        break;
      case 't':
        sleep_timer();
        break;
      case 'l':
        SdkEvalLedToggle(LED1);
        break;
      case '?':
        help();
        break;
      case 'p':
        printf("Hello World Sleep !!!\r\n");
        break;
      case 'r':
        NVIC_SystemReset();
        break;
      case '\r':
      case '\n':
        // ignored
        break;
      default:
        printf("Unknown Command\r\n");
      }
    }
  }
}

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
    SleepModes returnValue = sleepMode;
    
    return returnValue;
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/


