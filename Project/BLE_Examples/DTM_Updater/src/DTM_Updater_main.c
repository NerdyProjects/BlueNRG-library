/**
******************************************************************************
* @file    DTM_Updater_main.c 
* @author  AMS RF Application Team
* @version V2.0.0
* @date    February-2020
* @brief   DTM updater main. IMPORTANT: user must NOT modify the updater algorithm and code.
*          MEMORY_FLASH_APP_SIZE=0x2000 (DTM_updater size is 8K)
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
******************************************************************************
*/ 

#include "DTM_Updater.h"
#include "DTM_Updater_Config.h"

#define ENTERED_REASON_ACI_CMD          2
#define ENTERED_REASON_BAD_BLUEFLAG     3
#define ENTERED_REASON_IRQ_PIN          4

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05
#define GPIO_Pin_3                 (0x0008)  /*!< Pin 3 selected */

#define WA_DEVICE_VERSION   0x12
#define CLOCK_LOW_ENG_REG   0x3B
#define PMU_ANA_USER_REG    0x35
#define BOR_ENABLE          0x48
#define BLD_ENABLE          0x0B


void HAL_GetPartInfo(uint8_t *die_major, uint8_t *die_cut)
{
  *die_major    = CKGEN_SOC->DIE_ID_b.VERSION;
  *die_cut      = CKGEN_SOC->DIE_ID_b.REV;
  
  /* Patch for early samples */
  if (*die_major == 0)
    *die_major = 1;
}

/* TransportLayerTick function */
const DTM_InterfaceHandler_Type updater_init[] = {
        updater_init_uart,
        updater_init_spi};
 
int main(void)
{
  volatile uint8_t BOR_start_config[] = {0x02, CLOCK_LOW_ENG_REG,  BOR_ENABLE, 
                                         0x02, PMU_ANA_USER_REG,   BLD_ENABLE, 
                                         0x00};
  uint8_t die_cut, die_major;
  
  if ((CKGEN_SOC->REASON_RST == 0) && (CKGEN_BLE->REASON_RST > RESET_WAKE_DEEPSLEEP_REASONS)) {
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(DTM_APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) DTM_APP_ADDR);
    entryPoint();
    
    while(1);
  }
   
  /* Enable BOR */
  HAL_GetPartInfo(&die_major, &die_cut);
  if (((die_major<<4) | die_cut) >= WA_DEVICE_VERSION) {
    /* Set the 3 bit of the PMU_ANA_USER register */
    BOR_start_config[5] |= (1<<2);    
  }
  BLUE_CTRL->RADIO_CONFIG = 0x10000U | (uint16_t)((uint32_t)BOR_start_config & 0x0000FFFFU);
  while ((BLUE_CTRL->RADIO_CONFIG & 0x10000) != 0);
  /* Unlock the Flash */
  flash_sw_lock = FLASH_UNLOCK_WORD;
  
#if defined(UART_INTERFACE)
  volatile uint8_t DTM_INTERFACE = DTM_INTERFACE_UART;
  
#else
#if defined(SPI_INTERFACE)
  volatile uint8_t DTM_INTERFACE = DTM_INTERFACE_SPI;
  
#else
  volatile uint8_t DTM_INTERFACE = DTM_INTERFACE_UNDEF;
#endif
#endif
  
  if(DTM_INTERFACE == DTM_INTERFACE_UNDEF) {
    if((GPIO->DATA & DTM_INTERFACE_GPIO) == DTM_INTERFACE_GPIO) {
      DTM_INTERFACE = DTM_INTERFACE_UART;
    }
    else {
      DTM_INTERFACE = DTM_INTERFACE_SPI;
    }
  }

  /* if BLUE_FLAG_RESET => a previous programming operation went bad */
  if(*(uint32_t *)BLUE_FLAG_RAM_BASE_ADDRESS == BLUE_FLAG_RAM_RESET) {
    *(uint32_t *)BLUE_FLAG_RAM_BASE_ADDRESS = BLUE_FLAG_SET;
    updater_init[DTM_INTERFACE]();
    updater(ENTERED_REASON_ACI_CMD, DTM_INTERFACE);
  }
  else if(*(uint32_t *)BLUE_FLAG_FLASH_BASE_ADDRESS != BLUE_FLAG_SET) {
    updater_init[DTM_INTERFACE]();
    updater(ENTERED_REASON_BAD_BLUEFLAG, DTM_INTERFACE);
  }
  else if(READ_BIT(GPIO->DATA, GPIO_Pin_3) != 0) {  // IO3 -> SPI MOSI PIN
    updater_init[DTM_INTERFACE]();
    updater(ENTERED_REASON_IRQ_PIN, DTM_INTERFACE);
  }
  else {
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(DTM_APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) DTM_APP_ADDR);
    entryPoint();
    
    while(1);
  }
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
