/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : BLE_Stack_UpdaterYmodem_main.c
* Author             : AMS - VMA Division
* Version            : V1.0.0
* Date               : 03-February-2020
* Description        : BlueNRG-1,2demo example for stack updater through 
*                      ymodem protocol over serial uart interface
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "gp_timer.h"
#include "bluenrg1_hal_aci.h"
#include "SDK_EVAL_Config.h"
#include "bluenrg_utils.h"
#include "ymodem.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BLUE_FLAG_OFFSET 0x8C0
#define MIN_WRITE_BLOCK_SIZE 4

#define BASE_ADDRESS    (0x10040000)
#define FW_OFFSET       (8*1024)    // 2 KB
#define SECTOR_SIZE     (2*1024)    // 2 KB
#define DATA_SIZE       (64)        // 64 bytes

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes);

/* Private functions ---------------------------------------------------------*/

int program_device_UART(void)
{
  static volatile uint8_t my_buffer[1024+8];
  volatile static uint8_t status, data_size;
  uint8_t version;
  static uint8_t number_sectors;
  uint32_t j, i;
  int32_t ret;
  volatile static uint32_t packet = 0;
  volatile uint32_t size, fw_size;
  
  BlueNRG_HW_Updater();
  BTLE_StackTick(); // To receive the EVT_INITIALIZED

  if(aci_hal_get_updater_version(&version))
    return BLE_UTIL_ACI_ERROR;
  
  volatile uint32_t filesize;
  ret = Ymodem_Receive ((uint8_t *)my_buffer, 0, (uint32_t *) &fw_size, packet++);
  if (ret != YMODEM_CONTINUE) {
    Ymodem_Abort();
    return BLE_STATUS_ERROR;
  }
  Ymodem_SendAck();

  /* Calculate the number of sectors necessary to contain the fw image.*/
  number_sectors = ((fw_size + SECTOR_SIZE - 1) / SECTOR_SIZE);
  
  /***********************************************************************
  * Erase BLUE flag
  ************************************************************************/
  status = aci_hal_updater_erase_blue_flag();
  if (status != BLE_STATUS_SUCCESS)
    return status;

  /***********************************************************************
  * Erase and Program sectors
  ************************************************************************/  
  for(int i = FW_OFFSET; i < (FW_OFFSET + (number_sectors * SECTOR_SIZE)); i += SECTOR_SIZE) {
    status = aci_hal_updater_erase_sector(BASE_ADDRESS + i);
    if (status != BLE_STATUS_SUCCESS)
      return status;
  }
  
  i = 0;
  status = BLE_STATUS_SUCCESS;
  while (i <= (number_sectors * SECTOR_SIZE)) {
    
    ret = Ymodem_Receive ((uint8_t *)my_buffer, 0, (uint32_t *) &size, packet++);
    if (ret != YMODEM_CONTINUE && ret != YMODEM_DONE) {
      Ymodem_Abort(); // too many errors
      return BLE_STATUS_ERROR;
    }
    
    /* Discard bootloader data since it cannot and must not be programmed */
    if ((i/SECTOR_SIZE) < (number_sectors-1))
      data_size = DATA_SIZE;
    else
      data_size = MIN_WRITE_BLOCK_SIZE;
    for (j=0; (j<size); j += data_size) {
      status = aci_hal_updater_prog_data_blk(BASE_ADDRESS+FW_OFFSET+i+j, data_size, (uint8_t *)(my_buffer+j));
      if (status != BLE_STATUS_SUCCESS) {
        Ymodem_Abort();
        break;
      }
    }
    
    Ymodem_SendAck();
    i += size;
  }

  /***********************************************************************
  * Write BLUE flag
  ************************************************************************/
  status = aci_hal_updater_reset_blue_flag();
  if (status != BLE_STATUS_SUCCESS)
    return status;
  
  BlueNRG_RST();
  BTLE_StackTick(); // To receive the EVT_INITIALIZED
  
  return BLE_STATUS_SUCCESS;
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

  SdkEval_IO_Config(processInputData);
  
}

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_buffer_size = 0;
uint8_t *rx_buffer_ptr;
uint8_t *rx_buffer_tail_ptr;

void processInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  volatile int i;
  if (rx_buffer_size == 0) {
   rx_buffer_ptr = rx_buffer;
   rx_buffer_tail_ptr = rx_buffer_ptr;
  }

  for (i = 0; i < Nb_bytes; i++) {
    if( (rx_buffer_tail_ptr-rx_buffer) >= RX_BUFFER_SIZE){
      // Buffer is full
      return;
    }
    *rx_buffer_tail_ptr++ = data_buffer[i];
  }
  rx_buffer_size += Nb_bytes;
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
 
 /* BlueNRG-X stack init */
 ret = BlueNRG_Stack_Initialization();
 if (ret != BLE_STATUS_SUCCESS) {
   while(1);
 }
 
 Ymodem_Init();
 
 SdkEvalLedInit();
 
 /* Update the BlueNRG-2 image using the fw_image through UART */
 ret = program_device_UART();
 
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
/** \endcond
 */
