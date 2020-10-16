/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : RADIO_Sniffer_main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : March-2018
* Description        : Sniffer application with multi state functionality.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "hal_radio.h"
#include "osal.h"
#include "fifo.h"
#include "vtimer.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup RADIO_Examples RADIO Examples
* @{
*/

/** @addtogroup RADIO_Sniffer_MultiState RADIO Sniffer MultiState Example
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

#define CALIBRATION_INTERVAL_CONF   1000

#if LS_SOURCE==LS_SOURCE_INTERNAL_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#else

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

#define HS_STARTUP_TIME         (uint16_t)(1)  /* start up time min value */
#define N_STATE_MACHINES        STATEMACHINE_COUNT /* The number of state machines */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Blue FIFO */
circular_fifo_t blueRec_fifo;
uint8_t blueRec_buffer[MAX_PACKET_LENGTH*2];

uint8_t receivedData[MAX_PACKET_LENGTH];

uint32_t delay = 1000;
uint32_t rx_timeout = 500000;
int32_t rssi_val = 0;
uint32_t timestamp = 0;
uint8_t actual_ch = 0;
uint8_t channel_map[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};

uint32_t network_id_values[STATEMACHINE_COUNT] = {0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6,0x8E89BED6};
uint8_t channel_values[STATEMACHINE_COUNT] = {21,22,23,24,25,26,27,28};
uint8_t encryption_values[STATEMACHINE_COUNT] = {0,0,0,1,1,1,1,0};

uint32_t network_id[N_STATE_MACHINES];
uint8_t channel[N_STATE_MACHINES];
uint8_t encryption[N_STATE_MACHINES];
uint8_t No_Packet_Per_StateMachineNo = 5;

static ActionPacket aPacket[N_STATE_MACHINES]; 

uint8_t count_tx[5]     = {0x00,0x00,0x00,0x00,0x00};
uint8_t count_rcv[5]    = {0x00,0x00,0x00,0x00,0x00};
uint8_t enc_key[16]     = {0xBF,0x01,0xFB,0x9D,0x4E,0xF3,0xBC,0x36,0xD8,0x74,0xF5,0x39,0x41,0x38,0x68,0x4C};
uint8_t enc_iv[8]       = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint8_t nullfunction(ActionPacket* p)
{
  return TRUE;
}


uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
  static uint8_t count_t = 0;
  static uint8_t StateMachine_index = 0;
  
  /* received a packet */
  if( (p->status & BIT_TX_MODE) == 0) {
    SdkEvalLedToggle(LED1);
    
    /* Number of packet capture per State Machine */
    if(count_t < No_Packet_Per_StateMachineNo) {
      count_t++;
      
      if((p->status & IRQ_RCV_OK) != 0) {
        fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, p->data);
        fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->rssi) );
        fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->timestamp_receive));
        fifo_put(&blueRec_fifo, 1, (uint8_t*)(&channel[p->StateMachineNo]));
        
        RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
        RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);       
      }
      else if((p->status & IRQ_TIMEOUT) != 0) {
        //        printf("Timeout:%dus\r\n",rx_timeout);
        printf("%d",count_t);
        RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
        RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
      }
      else if((p->status & IRQ_CRC_ERR) != 0) {
        //        printf("CRC error\r\n");
        printf("%d",count_t);
        RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
        RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
      }              
    }
    else {
      if((p->status & IRQ_RCV_OK) != 0) {
        fifo_put_var_len_item(&blueRec_fifo, p->data[1]+2, p->data);
        fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->rssi) );
        fifo_put(&blueRec_fifo, 4, (uint8_t*)(&p->timestamp_receive));
        fifo_put(&blueRec_fifo, 1, (uint8_t*)(&channel[p->StateMachineNo])); 
      }
      count_t = 0;
      StateMachine_index++;
      StateMachine_index = StateMachine_index%N_STATE_MACHINES; /* Update the State Machine */

      RADIO_SetChannel(StateMachine_index,(uint8_t)channel[StateMachine_index], 0);
      RADIO_MakeActionPacketPending(&aPacket[StateMachine_index]);
//      printf("\r\n >> Sniffing channel..%d StateMachineNo:%d Encryption:%d Network_ID:0x%X\r\n",(uint8_t)channel[StateMachine_index],StateMachine_index,encryption[StateMachine_index],network_id[StateMachine_index]);
      printf("\r\n >> Sniffing channel..%d\r\n", (uint8_t)channel[StateMachine_index]);
    }
  }
  
  return TRUE;   
}  


int sniffer_init(uint8_t StateMachineNo)
{
  RADIO_SetChannelMap(StateMachineNo, channel_map);
  RADIO_SetChannel(StateMachineNo, (uint8_t)channel[StateMachineNo], 0);
  RADIO_SetTxAttributes(StateMachineNo, network_id[StateMachineNo], 0x555555);  
  
  aPacket[StateMachineNo].StateMachineNo = StateMachineNo;   
  aPacket[StateMachineNo].ActionTag =  INC_CHAN | PLL_TRIG | RELATIVE | TIMER_WAKEUP;
  aPacket[StateMachineNo].WakeupTime = delay;
  aPacket[StateMachineNo].ReceiveWindowLength = rx_timeout; 
  aPacket[StateMachineNo].data = receivedData; 
  aPacket[StateMachineNo].next_true = NULL_0;
  aPacket[StateMachineNo].next_false = NULL_0;    
  aPacket[StateMachineNo].condRoutine = nullfunction;
  aPacket[StateMachineNo].dataRoutine = RxCallback;
  
  RADIO_SetEncryptionAttributes(StateMachineNo, enc_iv, enc_key);
  RADIO_SetEncryptionCount(StateMachineNo, &count_tx[0], &count_rcv[0]); 
  RADIO_SetEncryptFlags(StateMachineNo,(FunctionalState)encryption[StateMachineNo],(FunctionalState)encryption[StateMachineNo]);    
  RADIO_SetReservedArea(&aPacket[StateMachineNo]);
  
  printf(" >> Sniffing channel..%d StateMachineNo:%d Encryption:%d Network_ID:0x%X\r\n", (uint8_t)channel[StateMachineNo], StateMachineNo, encryption[StateMachineNo], network_id[StateMachineNo]);
  return 0;
}

int main(void)
{
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  uint8_t packet[MAX_PACKET_LENGTH];
  uint16_t length;
  
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG-1 platform */
  SdkEvalIdentification();
  SdkEvalLedInit(LED1);
  
  /* Set the parameters */
  for(uint8_t i = 0; i < N_STATE_MACHINES; i++) {
    network_id[i] = network_id_values[i];
    channel[i] = channel_values[i];
    encryption[i] = encryption_values[i];
  }

  /* Configure I/O communication channel */
  SdkEvalComUartInit(UART_BAUDRATE);
  
  /* Create the blueRec FIFO */
  fifo_init(&blueRec_fifo, MAX_PACKET_LENGTH*2, blueRec_buffer, 1);
  
  /* Radio configuration */
  RADIO_Init(NULL, ENABLE);
  /* Timer Init */
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  /* Initilize the sniffer State Machines */
  for(uint8_t i = 0; i<N_STATE_MACHINES; i++) {
    sniffer_init(i);
  }
  
  /* Call this function for the first action packet to be executed */
  printf("\r\n >> Sniffing channel..%d\r\n", (uint8_t)channel[0]);
  RADIO_MakeActionPacketPending(&aPacket[0]);
  
  /* Infinite loop */
  while(1) {
    HAL_VTIMER_Tick();
    
    /* Print the packets sniffed */
    if(fifo_size(&blueRec_fifo) !=0) {
      
      /* Get the length of the packet and the packet itself */
      fifo_get_var_len_item(&blueRec_fifo, &length, packet);

      /* Get the RSSI information */
      fifo_get(&blueRec_fifo, 4, (uint8_t*)&rssi_val);
      
      /* Get the timestamp */
      fifo_get(&blueRec_fifo, 4, (uint8_t*)&timestamp);
      
      /* Get the channel */
      fifo_get(&blueRec_fifo, 1, (uint8_t*)&actual_ch);
      
      printf("\r\nchannel: %d, RSSI: %d dBm\r\n", actual_ch, rssi_val);
      printf("Timestamp: %d.%02d ms\r\n", PRINT_INT(((float)timestamp)*625/256000.0),PRINT_FLOAT(((float)timestamp)*625/256000.0)) ;
      printf("Frame: ");
      for(uint16_t i= 0; i<(length/*-MIC_FIELD_LENGTH*/); i++) {
        printf("%02x:", packet[i]);
      }
      printf("\r\n");
      //      printf("\r\nMIC: ");
      //      for(uint16_t i= (length+2-MIC_FIELD_LENGTH); i<(length+2); i++) {
      //        printf("%02x:", temp[i]);
      //      }
    }
  }
}



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


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
