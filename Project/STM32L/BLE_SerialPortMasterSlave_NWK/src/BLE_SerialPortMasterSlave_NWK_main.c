/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_SerialPortMasterSlave_main.c
* Author             : AMS - RF Application Team 
* Version            : V2.1.0
* Date               : 24-August-2021
* Description        : BLE network coprocessor main file for Serial Port Master & Slave demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1_2 Serial Port Master and Slave demo \see BLE_SerialPortMasterSlave_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "hal_types.h"
#include "hci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "hci_const.h"
#include "bluenrg1_types.h"
#include "bluenrg1_gap.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "SDK_EVAL_Config.h"
#include "serial_port.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BLE_NEW_SERIAL_PORT_VERSION_STRING "2.0.0" 

#ifndef DEBUG
#define DEBUG 1
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void SystemInit_NWK(void);

/* Private functions ---------------------------------------------------------*/

  
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  uint8_t ret;

  /* System Initialization */
  SystemInit_NWK();
  
  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BlueNRG-1 BLE Serial Port Master & Slave Application (version: %s)\r\n", BLE_NEW_SERIAL_PORT_VERSION_STRING); 

  /* Init Serial Port Device */
  ret = SerialPort_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("SerialPort_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  PRINTF("BLE Stack Initialized & Device Configured\r\n");

  while(1) {
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();

    /* Application tick */
    APP_Tick();
  }
}



void SystemInit_NWK(void)
{
  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  
  /* Configure the BlueNRG-LP pins - RESET pin */
  Sdk_Eval_Reset_Pin_Init();
  
#ifdef SPI_INTERFACE
  /* Init SPI interface */
  SdkEvalSpiInit();
#else

#ifdef UART_INTERFACE
  DTM_IO_Config();
#endif
#endif

  SdkEval_IO_Config(Process_InputData);
  
}


/* Hardware Error event. 
   This event is used to notify the Host that a hardware failure has occurred in the Controller. 
   Hardware_Code Values:
   - 0x01: Radio state error
   - 0x02: Timer overrun error
   - 0x03: Internal queue overflow error
   After this event is recommended to force device reset. */

void hci_hardware_error_event(uint8_t Hardware_Code)
{
   NVIC_SystemReset();
}

/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
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
/** \endcond
 */
