
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : BLE_Beacon_FreeRTOS_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 25-October-2019
* Description        : Code demostrating the BLE Beacon application and the use of FreeRTOS with BlueNRG stack
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Beacon_FreeRTOS_main.c
 * @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device 
 * in order to advertise specific manufacturing data and allow another BLE device to
 * know if it is in the range of the BlueNRG-1,2 beacon device. It also shows how to
 * use FreeRTOS with BLE stack.
 * 

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FreeRTOS\\EWARM\\BlueNRG-1\\BLE_Beacon_FreeRTOS.eww </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FreeRTOS\\EWARM\\BlueNRG-2\\BLE_Beacon_FreeRTOS.eww </tt>
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FreeRTOS\\MDK-ARM\\BlueNRG-1\\BLE_Beacon_FreeRTOS.uvprojx </tt> or
     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FreeRTOS\\MDK-ARM\\BlueNRG-2\\BLE_Beacon_FreeRTOS.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
  -# Download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

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


* \subsection Project_configurations Project configurations
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
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
-------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     GND    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|   RESETN   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    VBLUE   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |

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
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
-------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
-----------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |

@endtable

* \section Usage Usage

The Beacon demo configures a BlueNRG-1,2 device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier            | 0x004C (Apple, Inc.)				                   |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

The BTLE_StackTick() is called from a FreeRTOS task (BLETask).
A task randomly changes the Minor number in the advertising data, every 500 ms. A message is sent through UART each time
this is done.
Another task sends other messages through UART every 200 ms and generates a short pulse on LED3 (visible with a logic
analyzer or oscilloscope).
A low priority has been assigned to the BLETask in this example. In general, assigning an high priority to BLE Task can give
better latency, especially if other tasks are CPU resource hungry. If some tasks require a lot of CPU time, it is recommended
to assign to those tasks a priority lower than the BLETask, otherwise BLE operations may be slowed down. Only for tasks that
perform very short sporadic operations before waiting for an event, it is still reasonable to choose a priority higher than
the BLETask.

NOTEs: When using FreeRTOS framework it is mandatory to add the following assembler preprocessor options: 

     - IAR EWARM:  Options, Assembler, Preprocessor : CONTEXT_SAVE_V2 and HS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ or HS_SPEED_XTAL=HS_SPEED_XTAL_16MHZ (depending on selected platform)
     - KEIL MDK-ARM: Options, Asm, Misc Controls: --cpreproc_opts=-DCONTEXT_SAVE_V2
     - The CONTEXT_SAVE_V2 option must be also used on related OTA Reset/Service Manager when integrating the OTA FW upgrade framework for BLE, FreeRTOS applications.

FreeRTOS framework is officialy supported within the BlueNRG-1,2 SDK package (STSW-BLUENRG1-DK) v3.2.0 or later. 

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
*  BlueNRG-1,2 Beacon FreeRTOS demo \see BLE_Beacon_FreeRTOS_main.c for documentation.
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
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "Beacon_config.h"
#include "OTA_btl.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_ble.h"

/* Binary semaphore used to synchronize Stack Tick and radio ISR. */
SemaphoreHandle_t radioActivitySemaphoreHandle;
/* Mutex used to avoid that the BLE Stack Tick can be interrupted by an ACI
   function in another thread. */
SemaphoreHandle_t BLETickSemaphoreHandle;
/* Mutex used to access UART resource */
SemaphoreHandle_t UARTSemaphoreHandle;


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.1.0"

/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.
   Assigning an high priority to BLE Task can give better latency, especially
   if other tasks are CPU resource hungry. */
#define TEST_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 2 )
#define	BLE_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 1 )

/*-----------------------------------------------------------*/
/* Wait time of the test task (numbe rof ticks) */
#define TEST_PERIOD         			    ( 200 / portTICK_PERIOD_MS )
#define ADV_CHANGE_PERIOD         			( 500 / portTICK_PERIOD_MS )

/* Private macro -------------------------------------------------------------*/

#define DEBUG 1

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) do{ xSemaphoreTake(UARTSemaphoreHandle, portMAX_DELAY);\
                      printf(__VA_ARGS__);                              \
                      xSemaphoreGive(UARTSemaphoreHandle); }while(0)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void createTasks( void );

void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t data_length, data[6];
  
  /* Set the TX Power to -2 dBm */
  ret = aci_hal_set_tx_power_level(1,4);
  if(ret != 0) {
    PRINTF ("Error in aci_hal_set_tx_power_level() 0x%02Xr\n", ret);
    while(1);
  }
  
  /* Init the GATT */
  ret = aci_gatt_init();
  if (ret != 0) 
    PRINTF ("Error in aci_gatt_init() 0x%02Xr\n", ret);
  else
    PRINTF ("aci_gatt_init() --> SUCCESS\r\n");
  
  /* Init the GAP */
  ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
    PRINTF ("Error in aci_gap_init() 0x%02X\r\n", ret);
  else
    PRINTF ("aci_gap_init() --> SUCCESS\r\n");
	
	ret = aci_hal_read_config_data(CONFIG_DATA_STORED_STATIC_RANDOM_ADDRESS, &data_length, data);
	if (ret == 0)
		PRINTF("Address: 0x%02X%02X%02X%02X%02X%02X\r\n",data[5],data[4],data[3],data[2],data[1],data[0]);
  else
    PRINTF ("aci_hal_read_config_data() 0x%02X\r\n", ret);	
	
}


/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret = BLE_STATUS_SUCCESS;
  
  /* Set AD Type Flags at beginning on Advertising packet  */
  uint8_t adv_data[] = {
    /* Advertising data: Flags AD Type */
    0x02, 
    0x01, 
    0x06, 
    /* Advertising data: manufacturer specific data */
    26, //len
    AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
    0x4C, 0x00, //Company identifier code
    0x02,       // ID
    0x15,       //Length of the remaining payload
    0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
    0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
    0x00, 0x00, // Major number 
    0x00, 0x00, // Minor number 
    0xC8        //2's complement of the Tx power (-56dB)};      
  };
  
  /* disable scan response */
  ret = hci_le_set_scan_response_data(0,NULL);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
    return;
  }
  else
    PRINTF ("hci_le_set_scan_resp_data() --> SUCCESS\r\n");
  
  /* put device in non connectable mode */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 0, NULL, 0, NULL, 0, 0); 
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
    return;
  }
  else
    PRINTF ("aci_gap_set_discoverable() --> SUCCESS\r\n");
  
  /* Set the  ADV data with the Flags AD Type at beginning of the 
  advertsing packet,  followed by the beacon manufacturer specific data */
  ret = hci_le_set_advertising_data (sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
    return;
  }
  else
    PRINTF ("hci_le_set_advertising_data() --> SUCCESS\r\n");
}

int main(void) 
{    
  tBleStatus ret;
  
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG-1,2 platform */
  SdkEvalIdentification();
  
  /* Init the UART peripheral */
  SdkEvalComUartInit(UART_BAUDRATE);  
  UARTSemaphoreHandle = xSemaphoreCreateMutex();
  
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED3);
  
  SdkEvalLedOn(LED1);
  
  /* Create a binary semaphore to sync with radio interrupts */
  radioActivitySemaphoreHandle = xSemaphoreCreateBinary();
  /* Create a mutex semaphore to avoid calling aci functions while
    BTLE_StackTick() is running.*/
  BLETickSemaphoreHandle =  xSemaphoreCreateMutex();
  if(radioActivitySemaphoreHandle==NULL || BLETickSemaphoreHandle == NULL){
    while(1);
  }
    
  /* BlueNRG-1,2 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }  
  
  createTasks();
}

static void BLETask( void *pvParameters )
{ 
  /* To make sure no other BLE functions are called from other tasks. */
  xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);	
	
  /* Init the BlueNRG-1,2 device */
  Device_Init();
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
	
	/* BLE is initialized. Let other tasks call BLE functions. */
  xSemaphoreGive(BLETickSemaphoreHandle);
  
  while(1)
  {
    /* Take the semaphore to avoid that other ACI functions can interrupt the
       execution of BTLE_StackTick();   */
    xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
    BTLE_StackTick();    
    xSemaphoreGive(BLETickSemaphoreHandle);
    if(BlueNRG_Stack_Perform_Deep_Sleep_Check() != SLEEPMODE_RUNNING)
    {
      xSemaphoreTake(radioActivitySemaphoreHandle, portMAX_DELAY);
    }
  }
}

/*-----------------------------------------------------------*/
/* Just a test task which makes a very short pulse on a GPIO. */
static void testTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, TEST_PERIOD );
    
    /* Only do a pulse. */
    SdkEvalLedOn( LED3 );
    __NOP();__NOP();__NOP();__NOP();
    SdkEvalLedOff( LED3 );
    
    PRINTF("Test Task\r\n");
  }  
}
/*-----------------------------------------------------------*/
/* Another task that changes the advertising data */
static void changeADVDataTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  uint8_t Random_Number[8];
    /* Set AD Type Flags at beginning on Advertising packet  */
  uint8_t adv_data[] = {
    /* Advertising data: Flags AD Type */
    0x02, 
    0x01, 
    0x06, 
    /* Advertising data: manufacturer specific data */
    26, //len
    AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
    0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
    0x02,       // ID
    0x15,       //Length of the remaining payload
    0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
    0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
    0x00, 0x00, // Major number 
    0x00, 0x00, // Minor number 
    0xC8        //2's complement of the Tx power (-56dB)};      
  };
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, ADV_CHANGE_PERIOD );
    
    BLE_ACI_PROTECTED(hci_le_rand(Random_Number));
    
    adv_data[28] = Random_Number[0];   
    
    BLE_ACI_PROTECTED(hci_le_set_advertising_data(sizeof(adv_data), adv_data));
    
    PRINTF("ADV change %d\r\n", adv_data[28]);
    
  }  
}

/*-----------------------------------------------------------*/

void createTasks( void )
{
  
  xTaskCreate(BLETask,"BLEStack", 512, NULL, BLE_TASK_PRIORITY, NULL);
  
  xTaskCreate( testTask, "Test", 80, NULL, TEST_TASK_PRIORITY, NULL );
  
  xTaskCreate( changeADVDataTask, "ADV", 140, NULL, TEST_TASK_PRIORITY, NULL );
  
  /* Start the tasks and timer running. */
  vTaskStartScheduler();
  
  /* If all is well, the scheduler will now be running, and the following
  line will never be reached.  If the following line does execute, then
  there was insufficient FreeRTOS heap memory available for the idle and/or
  timer tasks	to be created.  See the memory management section on the
  FreeRTOS web site for more details. */
  for( ;; );
}
/*-----------------------------------------------------------*/


/****************** BlueNRG-1,2 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  if(SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
    return SLEEPMODE_RUNNING;
  
  return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

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

void vApplicationMallocFailedHook( void )
{
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created.  It is also called by various parts of the
  demo application.  If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
  task.  It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()).  If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;
  
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
  /* This function will be called by each tick interrupt if
  configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
  added here, but the tick hook is called from an interrupt context, so
  code must not attempt to block, and only the interrupt safe FreeRTOS API
  functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

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
