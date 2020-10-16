/**
* @file    main.c
* @author  AMS - RF Application Team 
* @version V2.0.0
* @date    20-January-2020
* @brief   Network co-processor host reference project
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2020 STMicroelectronics</center></h2>
*/


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"
#include "hci_parser.h"
#include "hci.h"


void SystemInit_NWK(void);

/**
* @brief  System main function.
* @param  None
* @retval None
*/
int main (void)
{
  /* System Initialization */
  SystemInit_NWK();
  
  /* Infinite loop */
  while(1) {
    BTLE_StackTick();
  }
}


/* Private functions ---------------------------------------------------------*/


void SystemInit_NWK(void)
{
  SystemClock_Config();
  
  /* Configure the RESET pin */
  Sdk_Eval_Reset_Pin_Init();
  
#ifdef SPI_INTERFACE
  /* Init SPI interface */
  SdkEvalSpiInit();
#else

#ifdef UART_INTERFACE
  DTM_IO_Config();
#endif
#endif

  /* Init UART2 port - connection with ST-Link */
  SdkEval_IO_Config(hci_input);
  
  /* Enable the Select Pin for the communication interface, if supported */
  BlueNRG_Activate_Select_Pin();
  
  /* Reset BlueNRG */
  BlueNRG_RST();
}


#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
