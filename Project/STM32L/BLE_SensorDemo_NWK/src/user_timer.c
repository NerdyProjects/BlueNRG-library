/**
  ******************************************************************************
  * @file    user_timer.c
  * @author  RF Application Team - AMG
  * @version V1.0.0
  * @date    24-June-2015
  * @brief   
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
    
#include "user_timer.h"

tClockTime user_timer_expired = FALSE;




/**
  * @brief  Initialize a timer for application usage.
  * @retval None
  */
void Init_User_Timer(void)
{ 
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();
    
  /* Configure the NVIC for TIMx */  
  NVIC_SetPriority(TIMx_IRQn, 4);
  
  /* Enable the TIMx global Interrupt */
  NVIC_EnableIRQ(TIMx_IRQn);
  
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_DOWN);
  
  LL_TIM_SetClockDivision(TIMx, LL_TIM_CLOCKDIVISION_DIV1);
  
  LL_TIM_SetAutoReload(TIMx, USER_TIMER_PERIOD); //12); //100);
  
  LL_TIM_SetPrescaler(TIMx, USER_TIMER_PRESCALER); //__LL_TIM_CALC_PSC(SystemCoreClock, 750));

  LL_TIM_GenerateEvent_UPDATE(TIMx);
  
  LL_TIM_ClearFlag_UPDATE(TIMx);  
}

/**
  * @brief  Start the user timer
  * @retval None
  */
void Start_User_Timer(void)
{
  LL_TIM_SetCounter(TIMx, 0);
  
  /* Enable the update interrupt */
  LL_TIM_EnableIT_UPDATE(TIMx);
    
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
  
}

/**
  * @brief   Stop the user timer
  * @retval None
  */
void Stop_User_Timer(void)
{
  /* Enable the update interrupt */
  LL_TIM_DisableIT_UPDATE(TIMx);
  
  /* Enable counter */
  LL_TIM_DisableCounter(TIMx);
}

/**
  * @brief  Adjust user  timer prescaler when entering sleep mode
  * @retval None
  */


void User_Timer_Enter_Sleep(void)
{
  uint32_t cnt;
  
  if(LL_TIM_IsEnabledCounter(TIMx))
  {
    LL_TIM_DisableIT_UPDATE(TIMx);
    LL_TIM_SetPrescaler(TIMx, USER_TIMER_PRESCALER_SLEEP);
    cnt = LL_TIM_GetCounter(TIMx);
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    LL_TIM_SetCounter(TIMx, cnt);
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);
  }
  
}


/**
  * @brief  Adjust user timer prescaler when exiting sleep mode
  * @retval None
  */
void User_Timer_Exit_Sleep(void)
{  
  uint32_t cnt;
  
  if(LL_TIM_IsEnabledCounter(TIMx))
  {
    LL_TIM_DisableIT_UPDATE(TIMx);
    LL_TIM_SetPrescaler(TIMx, USER_TIMER_PRESCALER);
    cnt = LL_TIM_GetCounter(TIMx);
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    LL_TIM_SetCounter(TIMx, cnt);
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);
  }
  
}

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : User Timer ISR
* Input          : None.
* Return         : None.
*******************************************************************************/
void TIMx_IRQHandler(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIMx) == 1) {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_UPDATE(TIMx);
  }
  
  user_timer_expired = TRUE;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
