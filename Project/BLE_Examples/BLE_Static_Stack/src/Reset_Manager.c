
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : Reset_Manager.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-October-2021
* Description        : Bluetooth LE Static stack
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file Reset_Manager.c
 * @brief This application provides an example of the Bluetooth LE Static Stack building. 
 * It must be loaded on BlueNRG-1,2 device, as preliminary step for using the Bluetooth LE Static Stack approach.


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-1-2 DK x.x.x\\Project\\BLE_Examples\\BLE_Static_Stack\\MDK-ARM\\{BlueNRG-1|BlueNRG-2}\\BLE_StaticStack.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Debug->Start/Stop Debug Session  to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Static_Stack\\EWARM\\{BlueNRG-1|BlueNRG-2}\\BLE_StaticStack.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Static_Stack\\WiSE-Studio\\{BlueNRG-1|BlueNRG-2}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c OTA_BTL_ResetManager - Configuration with OTA support
- \c Release - Release configuration


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
|            |                                                           Release                                                           ||||||                                                                OTA_BTL_ResetManager                                                                 ||||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |    STEVAL-IDB008V1M    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     GND    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |          N.A.          |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |          N.A.          |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|   RESETN   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |          N.A.          |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|    VBLUE   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |          N.A.          |          N.A.          |          N.A.          |        Not Used        |          N.A.          |          N.A.          |

@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                                                           Release                                                           ||||||                                                                OTA_BTL_ResetManager                                                                 ||||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |    STEVAL-IDB008V1M    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                           Release                                                           ||||||                                                                OTA_BTL_ResetManager                                                                 ||||||
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |    STEVAL-IDB008V1M    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |     Reset BlueNRG1     |     Reset BlueNRG1     |     Reset BlueNRG2     |     Reset BlueNRG2     |     Reset BlueNRG2     |     Reset BlueNRG2     |

@endtable

* \section Usage Usage
   -  Bluetooth Low Energy Static Stack example: it allows to build a project containing the Bluetooth LE stack library with all (or part) of its APIs and build a second project that does not contains the stack and nevertheless it can use it. 
   - A second project, containing only the application, can be programmed into the device without reprogramming the Bluetooth LE stack.  

   - NOTES: This process assumes that Bluetooth Low Energy static stack application has been loaded as first step. The related demo application built for running with Bluetooth Low Energy static stack must be loaded without removing the previoulsy loaded Bluetooth LE static stack. BLE_SensorDemo_StaticStack project provides an example of Sensor Demo variant which uses the Bluetooth LE static stack approach.

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1,2 Reset manager \see Reset_Manager.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

#include "bluenrg_x_device.h"
#include "Reset_Manager_Config.h"

int main(void)
{
  if(*(uint32_t *)BLUE_FLAG_FLASH_BASE_ADDRESS == BLUE_FLAG_SET){
    
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) APP_ADDR);
    entryPoint();
  }  
  while(1);
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
