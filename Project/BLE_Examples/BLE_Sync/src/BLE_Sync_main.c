
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Sync_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 10-September-2021
* Description        : Code demostrating a signal shynchronized between a Central and two Peripherals
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Sync_main.c
 * @brief  This application show how to have a signal shynchronized between a Central and two Peripherals,
 * with an high degree of accuracy.
 * 

* \section WiSE-Studio_project WiSE-Studio project
  To use the project with WiSE-Studio , please follow the instructions below:
  -# Open the WiSE-Studio  and select File->Import. 
  -# Select Existing Projects into Workspace. 
  -# Go to Project Explorer section
  -# Select desired configuration to build from Project->Project->Build Project.
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Sync\\MDK-ARM\\BlueNRG-1\\BLE_Sync.uvprojx </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Sync\\MDK-ARM\\BlueNRG-2\\BLE_Sync.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Sync\\EWARM\\BlueNRG-1\\BLE_Sync.eww </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Sync\\EWARM\\BlueNRG-2\\BLE_Sync.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Master - Configuration for master device
- \c Slave0 - Configuration for first slave
- \c Slave1 - Configuration for second slave


* \section Board_supported Boards supported
- \c STEVAL-IDB007V1
- \c STEVAL-IDB007V2
- \c STEVAL-IDB008V1
- \c STEVAL-IDB008V1M
- \c STEVAL-IDB008V2
- \c STEVAL-IDB009V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB00XV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name |            |  Description                                                                |
| JP1         |   JP2      |                                                                             |
----------------------------------------------------------------------------------------------------------
| ON 1-2      | ON 2-3     | USB supply power                                                            |
| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |


@endtable 

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB00XV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |          
| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |          
| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |


@endtable 
                        
* \section Pin_settings Pin settings
@table
|            |                                                           Master                                                            ||||||                                                           Slave0                                                            ||||||                                                           Slave1                                                            ||||||
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     GND    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|   RESETN   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    VBLUE   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |

@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 921600 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                                                           Master                                                            ||||||                                                           Slave0                                                            ||||||                                                           Slave1                                                            ||||||
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                           Master                                                            ||||||                                                           Slave0                                                            ||||||                                                           Slave1                                                            ||||||
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |

@endtable

* \section Usage Usage

This demo uses three eval boards: one Master and two Slaves. Once connected, Peripheral and Central devices generates some pulses
(the synchronized signal) on DIO2, at almost the same instant, with an accuracy of around 2 microseconds.
Moreover, a software library is used to maintain  a virtual common clock between the devices in the network. Upon connection, the master sends
the value of the virtual clock to the Peripheral. The synchronized signal is used to keep this clock always synchronized.
To demonstrate that the virtual clocks are synchronized between devices, a pulse is generated on DIO3 every 1 second, by polling the value of this clock.

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1,2 Sync demo \see BLE_Sync_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "app_state.h"
#include "sync.h"
#include "SDK_EVAL_Config.h"
#include "Sync_config.h"
#include "sleep.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_CHAT_VERSION_STRING "2.0" 
   
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void) 
{
  uint8_t ret;
  struct timer t;

  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Init Clock */
  Clock_Init();
  
  GPIO_Configuration();
   
  MFT_User_Init();

  /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
  SdkEvalComIOConfig(Process_InputData);

#if CLIENT && DMA_ENABLED
  DMA_Configuration();
#endif

  /* BlueNRG-1,2 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
#if SERVER
  printf("BlueNRG-1,2 BLE Sync Server Application\r\n");
#else
  printf("BlueNRG-1,2 BLE Sync Client Application\r\n"); 
#endif

  /* Init service */
  ret = Sync_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("CHAT_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("Bluetooth Stack Initialized \n");

  Timer_Set(&t, 1);
  
  while(1) {
    
    /* Disable UART IRQ to avoid calling BLE stack functions while BTLE_StackTick() is running. */
    NVIC_DisableIRQ(UART_IRQn);
    
    /* BlueNRG-1,2 stack tick */
    BTLE_StackTick();    
    
    NVIC_EnableIRQ(UART_IRQn);
    
    /* Application tick */
    APP_Tick();
      
#if SLEEP_ENABLED   
    if (Timer_Expired(&t)){
      extern volatile  uint8_t wakeupFromSleepFlag;
      wakeupFromSleepFlag = 0;
      SysTick->CTRL = 0;
      BlueNRG_Sleep(SLEEPMODE_WAKETIMER,WAKEUP_IO11,0);
      Clock_Init();
      if(wakeupFromSleepFlag && CKGEN_BLE->REASON_RST_b.WKP_IO11){
        // Wakeup because of GPIO.
        Timer_Set(&t, CLOCK_SECOND);
      }
    }
#endif
    
  }
  
} /* end main() */

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
