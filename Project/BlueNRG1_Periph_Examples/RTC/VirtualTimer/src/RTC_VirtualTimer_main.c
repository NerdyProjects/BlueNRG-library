/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : RTC/RTC_VirtualTimer/BLE_RTC_main.c
* Author             : RF Application Team
* Version            : V2.0.0
* Date               : 12-December-2019
* Description        : RTC example using the VTimer functionality. The VTimer is
*                      used to wait for 60 seconds, then the LED2 turns on and 
*                      the application stop. Sleep is used.
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
#include "ble_const.h"
#include "sleep.h"
#include "SDK_EVAL_Config.h"

#include "vtimer.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
 *@{
 */

/** @addtogroup RTC_Examples
  * @{
  */

/** @addtogroup RTC_VirtualTimer RTC VirtualTimer
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define DEBUG 1 
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us

/* Sleep clock accuracy */
#if (LS_SOURCE == LS_SOURCE_INTERNAL_RO)

/* Calibration must be done */
#define INITIAL_CALIBRATION         TRUE

#define CALIBRATION_INTERVAL        500

#else

/* No Calibration */
#define INITIAL_CALIBRATION         FALSE
#define CALIBRATION_INTERVAL        0

#endif


/* Virtual Timer */
#define VTIMER_TIMEBASE_TARGET_MAX_SEC  (int32_t)(5200)


/** 
* Brief User target timeout
 */
#define VTIMER_USER_TARGET_SEC          (int32_t)(60) 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static VTIMER_HandleType VTimer_handle;
static volatile uint8_t VTimer_expired = FALSE;
static int32_t vtimer_user_target_sec = VTIMER_USER_TARGET_SEC;
static int32_t vtimer_user_target2 = 0;
static volatile int32_t vtimer_user_cycle = 1;
static volatile uint32_t vtimer_get_current_time = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void HAL_VTimerTimeoutCallback(void *handle1) 
{
    PRINTF("HAL_VTimerTimeoutCallback()\r\n");

    vtimer_get_current_time = HAL_VTimerGetCurrentTime_sysT32();
    vtimer_user_cycle--;

    VTimer_expired = TRUE;
    
}


int main(void) 
{
  uint8_t ret;
  
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG-1 platform */
  SdkEvalIdentification();
  
   /* Enable UART */
  SdkEvalComIOConfig(SdkEvalComIOProcessInputData);
  
  /* Initialize the LEDs */
  SdkEvalLedInit(LED1); /* Activity LED */
  SdkEvalLedInit(LED2);
  SdkEvalLedOn(LED1);
  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  

  if(vtimer_user_target_sec > VTIMER_TIMEBASE_TARGET_MAX_SEC) {
    vtimer_user_cycle += (vtimer_user_target_sec / VTIMER_TIMEBASE_TARGET_MAX_SEC);
    vtimer_user_target2 = vtimer_user_target_sec % VTIMER_TIMEBASE_TARGET_MAX_SEC;
    vtimer_user_target_sec = VTIMER_TIMEBASE_TARGET_MAX_SEC;
  }

  
  /* Start the VTimer */
  VTimer_handle.callback = HAL_VTimerTimeoutCallback;
  ret = HAL_VTimerStart_ms(&VTimer_handle, vtimer_user_target_sec*1000);
  if (ret != BLE_STATUS_SUCCESS) {
    while(1);
  }
  
  PRINTF("RTC Wakeup demo with standalone virtual/sleep timer(%d sec)\r\n", vtimer_user_target_sec);
  
  while(1) {

    /* Timer tick */
    HAL_VTIMER_Tick();
    
    
    if(VTimer_expired==TRUE) {
        
      if((vtimer_user_cycle == 1) || (vtimer_user_cycle == 0)) {
        
        if(vtimer_user_target2) {
          vtimer_get_current_time = HAL_VTimerAcc_sysT32_ms(vtimer_get_current_time, vtimer_user_target2*1000);
          HAL_VTimerStart_sysT32(&VTimer_handle, vtimer_get_current_time);
          vtimer_user_target2 = 0;
        }
        else {
          SdkEvalLedOn(LED2);
          while(1);
        }
      }
      else {
        vtimer_get_current_time = HAL_VTimerAcc_sysT32_ms(vtimer_get_current_time, vtimer_user_target_sec*1000);
        HAL_VTimerStart_sysT32(&VTimer_handle,  vtimer_get_current_time); 
      }
      VTimer_expired = FALSE;
    }

    /* Enable Power Save  */
    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0); 
    
  }
}

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
    if (SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy() )
    {
       return SLEEPMODE_RUNNING;
    }
    
    return sleepMode; 
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
