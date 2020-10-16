/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : BLE_Stack_DevConfig_Updater_main.c
* Author             : AMS - RF Application 
* Version            : V1.0.0
* Date               : 03-February-2020
* Description        : BlueNRG-1,2 demo example for Device Configuration updater 
*                      and BLE stack updater
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "osal.h"
#include "gp_timer.h"
#include "bluenrg1_hal_aci.h"
#include "SDK_EVAL_Config.h"
#include "bluenrg_utils.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
//* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern const unsigned char FW_IMAGE[];
extern unsigned int FW_IMAGE_SIZE;
extern const devConfig_t deviceConfig;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void SystemInit_NWK(void)
{
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Configure the GPIO RESETN pin */
  Sdk_Eval_Reset_Pin_Init();
  
#ifdef SPI_INTERFACE
  /* Init SPI interface */
  SdkEvalSpiInit();
#else
#ifdef UART_INTERFACE
  DTM_IO_Config();
#endif
#endif
  
}

/* ********************************************************* 
 * Select the device by using the preprocessor symbols
 * BLUENRG1_DEVICE for the BlueNRG-1
 * BLUENRG2_DEVICE for the BlueNRG-2
 * ********************************************************* */
int main(void)
{
  int ret;
  
  /* System Initialization */
  SystemInit_NWK();
  
  ret = BlueNRG_Stack_Initialization();
  if (ret != BLE_STATUS_SUCCESS) {
    while(1);
  }
  
  SdkEvalLedInit();
  
#if defined (APPLY_BLUENRG_STACK_UPDATER)
  /* Update the DTM image using the FW_IMAGE provided in the file update_fw_image.c */
  ret = program_device(FW_IMAGE, FW_IMAGE_SIZE);
  if (ret) {
    while(1);
  }
#endif /* APPLY_BLUENRG_STACK_UPDATER */
  
#if defined (APPLY_BLUENRG_DEV_CONFIG_UPDATER)
  /* Verify if the DEV_CONFIG memory and then update it if needed */
  ret = verify_DEV_CONFIG(&deviceConfig);
  if (ret) {
    ret = program_DEV_CONFIG(&deviceConfig);
    if (ret) {
      while(1);
    }
  }
#endif  /* APPLY_BLUENRG_DEV_CONFIG_UPDATER */
  
  while(1) {
    SdkEvalLedToggle();
    Clock_Wait(500);
  }
}



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
