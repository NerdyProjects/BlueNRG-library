
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : DTM_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 27-June-2016
* Description        : DTM application which configures a BlueNRG-1,2 device as a network coprocessor (UART or SPI conficuration) in order to be used with the BlueNRG GUI or other instruments as CBT
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file DTM_main.c
 * @brief This application configures a BlueNRG-1,2 device as a network coprocessor (SPI or UART) in order to be used with the BlueNRG GUI or other instruments as CBT. Full stack  modular configuration option is used.
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
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\DTM\\MDK-ARM\\BlueNRG-1\\DTM.uvprojx </tt> or
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\DTM\\MDK-ARM\\BlueNRG-2\\DTM.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\DTM\\EWARM\\BlueNRG-1\\DTM.eww </tt> or
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\DTM\\EWARM\\BlueNRG-2\\DTM.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c SPI - Network coprocessor configuration: SPI mode
- \c UART - Network coprocessor configuration: UART mode
- \c UART_Throughput - Network coprocessor configuration: Throughput mode


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
|            |                                                            UART                                                             ||||||                                                             SPI                                                             ||||||                                                       UART_Throughput                                                       ||||||
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
|            |                                                            UART                                                             ||||||                                                             SPI                                                             ||||||                                                       UART_Throughput                                                       ||||||
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
|                |                                                            UART                                                             ||||||                                                             SPI                                                             ||||||                                                       UART_Throughput                                                       ||||||
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |

@endtable

* \section Usage Usage

DTM (Direct Test Mode) application allows to configure a BlueNRG-1,2 as a network coprocessor and target BLE technology evaluation & RF evaluation performances tests using the BlueNRG GUI or other instruments as CBT. 

Two network coprocessor configurations are available:
 - UART (DTM binary file: DTM_UART.hex, serial baudrate = 115200)
 - SPI  (DTM binary file: DTM_SPI.hex)

NOTE: UART_Throughput configuration must be used only for throughput evaluation tests through the BlueNRG GUI, Throughput tab. 

DTM_UART.hex and DTM_SPI.hex binary image files are built for also including the
updater code which allows to update the specific DTM FW version trough the BlueNRG GUI, Tools, BlueNRG Updater utility.

DTM binary images without updater code are also provided: DTM_UART_NOUPDATER.bin for UART configuration and DTM_SPI_NOUPDATER.bin for SPI configuration.

Once the STEVAL-IDB007Vx, BlueNRG-1 platform has been configured as network coprocessor (using related DTM_UART.hex or DTM_SPI.hex binary files),  or STEVAL-IDB008Vx, BlueNRG-2 platform has been configured as network coprocessor (using related DTM_UART.hex or DTM_SPI.hex binary files), new DTM binary files versions can be updated directly using the BlueNRG GUI, Tools, BlueNRG Updater utility.

The DTM project memory layout is as follows:
  
-------------- 0x10067FFF  
     
- <b> DTM </b>

--------------  0x10042000

- <b> DTM Updater </b>

--------------  0x10040000
 
The DTM Updater allows to access the memory flash through ACI_HAL commands. The DTM Updater can be activated in the following way:
 - Activation by using ACI_HAL_UPDATER_START
 - Activation by using IO3 pin (high level at start up).
 - <b> Note: if the IO3 pin is used and is high at start up, this will cause the DTM Updater starts. So, to avoid this, the support of the DTM Updater can be removed. </b>
  
If the DTM Updater support is not necessary, the DTM Updater code can be removed:
 - Disable/Remove the file DTM_Updater_xxx.c inside the folder project DTM_Updater.
 - Option/Linker: Change the symbol definition from MEMORY_FLASH_APP_OFFSET=0x800 to MEMORY_FLASH_APP_OFFSET=0.

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1,2 DTM application \see DTM_main.c for documentation.
 *
 *@{
 */
/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "BlueNRG1_conf.h"
#include "bluenrg1_stack.h"
#include "DTM_boot.h"
#include "sleep.h"
#include "transport_layer.h"
#include "hw_config.h"
#include "miscutil.h" 
#include "DTM_cmd_db.h"
#include "DTM_burst.h"

#define RESET_REASON_WDG        ((uint8_t)0x05)
#define RESET_REASON_LOCKUP     ((uint8_t)0x06)
#define RESET_REASON_BOR        ((uint8_t)0x07)
#define RESET_REASON_CRASH      ((uint8_t)0x08)

volatile extern uint8_t DTM_INTERFACE;
const uint8_t IO_WAKEUP_PIN[] = {0, 4, 0x10};
 
/* Add aci_blue_initialized_event() prototype */
void aci_blue_initialized_event(uint8_t Reason_Code);

/* Add aci_blue_crash_info_event() prototype */
void aci_blue_crash_info_event(uint8_t Crash_Type,
                               uint32_t SP,
                               uint32_t R0,
                               uint32_t R1,
                               uint32_t R2,
                               uint32_t R3,
                               uint32_t R12,
                               uint32_t LR,
                               uint32_t PC,
                               uint32_t xPSR,
                               uint8_t Debug_Data_Length,
                               uint8_t Debug_Data[]);
/*
 ******************************************************************************
 ******************************************************************************
 * The DTM project memory layout is as follows:
 * 
 * ------------- 0x10068000 (BlueNRG-1); 0x10080000 (BlueNRG-1)
 *      NVM         
 * ------------- 0x10067000 (BlueNRG-1); 0x1007F000 (BlueNRG-2)
 *
 *      DTM
 * 
 * ------------- 0x10042000 (MEMORY_FLASH_APP_OFFSET)
 *  DTM Updater
 * ------------- 0x10040000 (_MEMORY_FLASH_BEGIN_)
 *
 * The DTM UART/UART_Sleep, SPI project configurations include the DTM updater code + 
 * the DTM specific code (Configuration --> Preprocessor option: UART --> UART_INTERFACE; UART Sleep -->  UART_SLEEP; SPI --> SPI_INTERFACE).
 * The DTM SPI_NOUPDATER, UART_NOUPDATER project configurations allows to build the binary 
 * images (with based address = MEMORY_FLASH_APP_OFFSET) which includes only the DTM specific code. (Configuration --> Preprocessor option: UART_NOUPDATER --> UART_INTERFACE; SPI_NOUPDATER --> SPI_INTERFACE).
 * This binary image can be used for  upgrading the DTM binary image on a device: 
 * the assumption is that the device has been previosly loaded with the DTM full image (DTM + DTM Updater built with DTM UART/UART_Sleep, SPI project configurations), 
 * and that the user just wants to upgrade the DTM specific code with a new version, using the DTM SPI_NOUPDATER, UART_NOUPDATER project configurations
 * binary images. 
 * The DTM Updater allows to access the memory flash through ACI_HAL commands.
 * It is placed on the bootloader section from  _MEMORY_FLASH_BEGIN_ to  
 * _MEMORY_FLASH_BEGIN_ + MEMORY_FLASH_APP_OFFSET (0x2000). 
 * The DTM Updater can be activated in the following way:
 *  1) Activation by using ACI_HAL_UPDATER_START
 *  2) Activation by using IO3 pin (high level at start up).
 *     Note: if the IO3 pin is used and is high at start up, this will cause the 
 *     DTM Updater starts. So, to avoid this, the support of the DTM Updater
 *     can be removed.
 * 
 * If the DTM Updater support is not necessary, the DTM Updater code can be
 * removed:
 *  1) Disable/Remove the file DTM_Updater_xxx.c inside the folder project
 *     DTM_Updater.
 *  2) Option/Linker: Change the symbol definition from 
 *     MEMORY_FLASH_APP_OFFSET=0x2000 to MEMORY_FLASH_APP_OFFSET=0.
 *
 *  The same communication port (SPI or UART with related pins) of the DTM
 *  is used for the DTM Updater. Any change of relevant pins must be reported
 *  also in DTM Updater firmware.
 *
 *  NOTES: UART default baudrate is 115200; 
 * 
 *  UART_Throughput configuration must be used only for throughput evaluation tests 
 *  through the BlueNRG GUI, Throughput tab.
 *
 ******************************************************************************
 **                                WARNING                                   **
 ******************************************************************************
 * DTM v.3.2 available on BlueNRG-1,2 SDK v. 3.2.0 has an offset of 0x2000 (8 KB).
 * This image is not compatible with DTM_Updater image provided on previous SDK
 * versions (offset = 0x800).
 * In order to make it compatible with previous DTM_Updater versions, please 
 * rebuild the DTM UART or SPI configurations with linker symbol 
 * MEMORY_FLASH_APP_OFFSET=0x800.
 ******************************************************************************
 ******************************************************************************
*/

int main(void)
{
  crash_info_t crash_info;
  
  /* System Init */
  DTM_SystemInit();
  
  /* Stack Initialization */
  DTM_StackInit();
  
    
  /* Transport Layer Init */
  transport_layer_init();
  
  /* Get crash info */
  HAL_GetCrashInfo(&crash_info); 
  
#ifdef LL_ONLY
  uint8_t Value = 1;
  aci_hal_write_config_data(0x2C, 1, &Value);
  
#else
  uint16_t xResetReason;

  uint8_t reset_reason = 0x01;
  
  /* EVT_BLUE_INITIALIZED */  
  xResetReason = SysCtrl_GetWakeupResetReason();
  if(xResetReason & RESET_WDG) {
    reset_reason = RESET_REASON_WDG;
  }
  else if(xResetReason & RESET_LOCKUP) {
    reset_reason = RESET_REASON_LOCKUP;
  }
  else if(xResetReason & RESET_BLE_BOR) {
    reset_reason = RESET_REASON_BOR;
  }
  if((crash_info.signature&0xFFFF0000) == CRASH_SIGNATURE_BASE) {  
    reset_reason = RESET_REASON_CRASH;
  }

  aci_blue_initialized_event(reset_reason);

#endif

  if((crash_info.signature&0xFFFF0000) == CRASH_SIGNATURE_BASE) { 
    aci_blue_crash_info_event(crash_info.signature&0xFF,
                              crash_info.SP,
                              crash_info.R0,
                              crash_info.R1,
                              crash_info.R2,
                              crash_info.R3,
                              crash_info.R12,
                              crash_info.LR,
                              crash_info.PC,
                              crash_info.xPSR,
                              0,
                              NULL);
  }

  while(1) {
    /* BlueNRG-1,2 stack tick */
    BURST_Tick();
    BTLE_StackTick();
    transport_layer_tick();
    BlueNRG_Sleep(SLEEPMODE_NOTIMER, IO_WAKEUP_PIN[DTM_INTERFACE], 0); // 4: IO11 0: low level
  }
}
