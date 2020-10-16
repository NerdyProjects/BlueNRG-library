/**
  ******************************************************************************
  * @file    user_timer.h
  * @author  RF Application Team - AMG
  * @version V1.0
  * @date    24-June-2015
  * @brief   Header for user_timer.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_TIMER_H
#define __USER_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "clock.h"

#ifdef STM32L476xx

#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_rcc.h"

#ifdef SYSCLK_MSI
#define SYSCLK_FREQ	  4000000
#else
#define SYSCLK_FREQ	  80000000
#endif

#endif
   
#ifdef STM32L152xE
   
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"

#ifdef SYSCLK_MSI
#define SYSCLK_FREQ	  4000000
#else
#define SYSCLK_FREQ	  32000000
#endif
#endif

#define SYSCLK_FREQ_SLEEP	LSI_VALUE


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


/* Definition for TIMx: TIM3 */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);  //__HAL_RCC_TIM3_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

#define USER_TIMER_PRESCALER    (64000-1)
/* Timeout in milliseconds. */
#define USER_TIMER_PERIOD_MSEC   250
#define USER_TIMER_PERIOD       USER_TIMER_PERIOD_MSEC*(SYSCLK_FREQ/(USER_TIMER_PRESCALER+1))/1000

#define USER_TIMER_PRESCALER_SLEEP  (USER_TIMER_PRESCALER+1)/(SYSCLK_FREQ/SYSCLK_FREQ_SLEEP) - 1


   
//extern TIM_HandleTypeDef    TimHandle;
extern tClockTime user_timer_expired; 


void Init_User_Timer(void);
void Start_User_Timer(void);
void Stop_User_Timer(void);
void User_Timer_Enter_Sleep(void);
void User_Timer_Exit_Sleep(void);

 
#ifdef __cplusplus
}
#endif

#endif /*__USER_TIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
