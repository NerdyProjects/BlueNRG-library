/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : sync.c
* Author             : AMS - RF  Application team
* Description        : This file handles Bluetooth activity.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "sync.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "hw_config.h"
#include "vclock.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define CMD_BUFF_SIZE 256

// Expected size of notifications. It depends also on ATT MTU.
#define NOTIFICATION_LENGTH  60
#define WRITE_LENGTH          8

#define DEBUG         1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SLAVE0_OFFSET      (SLAVE1_DELAY - SLAVE0_DELAY)
#define SLAVE1_OFFSET      0


#ifndef ROLE
#error Define role!
#else
#if ROLE==MASTER_ROLE
#define CLIENT  1
#else
#define SERVER  1
#endif
#endif

#define NUM_SLAVES 2

#define MASTER {0xaa, 0x01, 0x00, 0xE1, 0x80, 0x02}
#define SLAVE0 {0xab, 0x01, 0x00, 0xE1, 0x80, 0x02}
#define SLAVE1 {0xac, 0x01, 0x00, 0xE1, 0x80, 0x02}

/* Private variables ---------------------------------------------------------*/

static uint32_t next_vtime = 0;

#if ROLE==MASTER_ROLE

/* The list of the slave addresses. Now it is hard-coded. In the future, the slave
  will be discovered before pairing phase.
*/
Whitelist_Entry_t slave_addresses[NUM_SLAVES] = {
  {
    .Peer_Address_Type = PUBLIC_ADDR,
    .Peer_Address = SLAVE0,
  },
  {
    .Peer_Address_Type = PUBLIC_ADDR,
    .Peer_Address = SLAVE1,
  }
};

// Offsets in the two cases: unaligned pulse on first slot and unaligned pulse on second slot.
const uint32_t master_offsets[2] = {SLAVE1_DELAY, SLAVE1_DELAY_PULSE_ON_2ND_SLOT};

// Number of the slot where pulse is generated on DIO6: it can be 0 or 1.
uint8_t slot_pulse_index = 0;

uint8_t change_slot_pulse_index = FALSE;;

uint8_t resync_vclock = FALSE;

#endif

volatile int app_flags = SET_CONNECTABLE;

#if ROLE != MASTER_ROLE
uint32_t offset = 0xFFFFFFFF;
#endif

static char cmd[CMD_BUFF_SIZE];

#if ROLE==MASTER_ROLE

/* States of the state machine used to discover services, enable notifications and
  write the offset into the slaves.
*/
typedef enum{
  IDLE = 0,
  START_SERVICE_DISCOVERY,
  DISCOVERING_SERVICES,
  START_TX_CHAR_DISCOVERY,
  DISCOVERING_TX_CHAR,
  START_RX_CHAR_DISCOVERY,
  DISCOVERING_RX_CHAR,
  START_OFFSET_CHAR_DISCOVERY,
  DISCOVERING_OFFSET_CHAR,
  START_VCLOCK_CHAR_DISCOVERY,
  DISCOVERING_VCLOCK_CHAR,
  ENABLE_NOTIFICATIONS,
  ENABLING_NOTIFICATIONS,
  WRITE_OFFSET,
  WRITING_OFFSET,
  EXCHANGE_CONFIG,
  EXCHANGING_CONFIG,
  WRITE_VCLOCK,
  WRITING_VCLOCK,
  DONE
} SlaveState; 

// Type of the structure used to store the state related to each server/slave
typedef struct _slave {
  uint8_t  address[6];
  uint16_t conn_handle;
  SlaveState state;
  uint16_t chat_serv_start_handle;
  uint16_t chat_serv_end_handle;
  uint16_t sync_serv_start_handle;
  uint16_t sync_serv_end_handle;
  uint16_t tx_handle;
  uint16_t rx_handle;
  uint16_t offset_handle;
  uint16_t vclock_handle;
  uint16_t event_counter;
}slave_device;

slave_device slaves[NUM_SLAVES];


uint8_t num_connected_slaves = 0;

#else

uint16_t connection_handle;
uint16_t event_counter;

#endif

/* Private function prototypes -----------------------------------------------*/
void SlaveInit(uint8_t slave_index);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : address_is_set.
* Description    : Returns is address is set, i.e. if it is different
*                  from 0x000000000000
* Input          : the address.
* Return         : TRUE is addres is set, FALSE otherwise.
*******************************************************************************/
uint8_t address_is_set(uint8_t address[6])
{
  int i;
  
  for(i = 0; i < 6; i++){
    if(address[i] != 0)
      break;
  }
  if(i == 6)
    return FALSE;
  else  
    return TRUE;
}

/*******************************************************************************
* Function Name  : Sync_DeviceInit.
* Description    : Init the Bluetooth stack.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t Sync_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G', '1'};
  
#if ROLE == SLAVE0_ROLE
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = SLAVE0;
#elif ROLE == SLAVE1_ROLE
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = SLAVE1;
#else  
  uint8_t role = GAP_CENTRAL_ROLE;
  uint8_t bdaddr[] = MASTER;
  /* Used to specify maximum supported tx and rx octets. Needed to reduce time allocated for master slots.
  2 bytes for each value. In order: max tx  octets, max tx time, max rx octets, max rx time.   */
  uint8_t data[] = {27,0, 0x48, 0x01, 0xfb, 0x00, 0x48, 0x08};  
  aci_hal_write_config_data(0xD1, 8, data);
#endif 
  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS){
    printf("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
  }
  
  PRINTF("Setting address 0x%02X%02X%02X%02X%02X%02X\n", bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);

  aci_hal_set_radio_activity_mask(0x04|0x08|0x20); // slave, master and scan slots

  /* Set the TX power to -2 dBm */
  aci_hal_set_tx_power_level(1, 4);

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gatt_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00, 0x08, &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Set the device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle,
                                   0, sizeof(name), name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Gatt Update characteristic value 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_update_char_value() --> SUCCESS\r\n");
  }

#if  SERVER
  
  ret = Add_Chat_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_Chat_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_Chat_Service() --> SUCCESS\r\n");
  }
    
  ret = Add_Sync_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Add_Sync_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("Add_Sync_Service() --> SUCCESS\r\n");
  }
  
#else
  
  for(int i = 0; i < NUM_SLAVES; i++)
    SlaveInit(i);
  
#endif
  
  InitVClock();
  
  return BLE_STATUS_SUCCESS;
}

uint8_t go_to_sleep = TRUE;

/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  static uint16_t end = 0;
  uint8_t i;
  
  for (i = 0; i < Nb_bytes; i++) {
    if (end >= CMD_BUFF_SIZE - 1) {
      end = 0;
    }
    
    cmd[end] = data_buffer[i];
    SdkEvalComIOSendData(data_buffer[i]);
    end++;
    
    go_to_sleep = FALSE;
    
    if (cmd[end - 1] == '\r' || cmd[end - 1] == '\n') {
      go_to_sleep = TRUE;
      end--;
      if(end != 0){        
        //GPIO_WriteBit(GPIO_Pin_3, Bit_SET);
#if SERVER        
        uint32_t len = MIN(NOTIFICATION_LENGTH, end);          
        aci_gatt_update_char_value_ext(connection_handle, chatServHandle, TXCharHandle, 1, len, 0, len, (uint8_t *)cmd);          
#elif CLIENT
        uint32_t len = MIN(WRITE_LENGTH, end);
        for(int i = 0; i < NUM_SLAVES; i++){
          if(slaves[i].conn_handle != 0){
            aci_gatt_write_without_resp(slaves[i].conn_handle, slaves[i].rx_handle+1, len, (uint8_t *)cmd);
          }
        }
#endif
        //GPIO_WriteBit(GPIO_Pin_3, Bit_RESET);
        
      }
      end = 0;
    }  
    
  }
  
}

/*******************************************************************************
* Function Name  : App_SleepMode_Check.
* Description    : Check if the device can go to sleep. See sleep.h
* Input          : Requested sleep mdoe.
* Return         : Allowed sleep mode
*******************************************************************************/
#if SLEEP_ENABLED
SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  if(SdkEvalComIOTxFifoNotEmpty() || MFT_PulseTrainEventTriggerStatus(MFT1) == SET){
    return SLEEPMODE_RUNNING;
  }
  
  if(go_to_sleep)
    return SLEEPMODE_NOTIMER;
  else
    return SLEEPMODE_CPU_HALT;
}
#endif


/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Connect(void)
{  
  tBleStatus ret;
  
#if ROLE == MASTER_ROLE
  
  ret = aci_gap_start_auto_connection_establish_proc(100/0.625, 100/0.625, PUBLIC_ADDR, 12.5/1.25, 12.5/1.25, 0, 20, CE_LENGTH, CE_LENGTH,
                                                     NUM_SLAVES,
                                                     slave_addresses);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting connection: 0x%02X\n", ret);    
  }
  else {
    printf("Connecting...\n");
  }
  
#else
  
  uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','1','_','C','h','a','t'};
  
  hci_le_set_scan_response_data(0,NULL);
  
  ret = aci_gap_set_discoverable(ADV_IND, 22.5/0.625, 22.5/0.625, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF ("Error in aci_gap_set_discoverable(): 0x%02x\r\n", ret);
  else
    PRINTF ("aci_gap_set_discoverable() --> SUCCESS\r\n");
  
#endif
}

#if ROLE == MASTER_ROLE

/*******************************************************************************
* Function Name  : SlaveInit.
* Description    : Init the slave state
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void SlaveInit(uint8_t slave_index)
{  
  Osal_MemSet(&slaves[slave_index].address, 0, sizeof(slave_device));
}

/*******************************************************************************
* Function Name  : StartDiscovery.
* Description    : Begin discovery of the services for the selected slave
* Input          : Index of the slave
* Return         : none.
*******************************************************************************/
void StartDiscovery(uint8_t slave_index)
{  
  slaves[slave_index].state = START_SERVICE_DISCOVERY;
}

/*******************************************************************************
* Function Name  : PerSlaveStateMachine.
* Description    : State machine handling the disocvery of the services, setting
*                  of the client characteristic configuratino descriptors and
*                  writing into the characteristics.
* Input          : none
* Return         : none.
*******************************************************************************/
void PerSlaveStateMachine(void)
{
  tBleStatus ret;
  
  for(int i = 0; i < NUM_SLAVES; i++){
  
    switch(slaves[i].state){
      
      case START_SERVICE_DISCOVERY:
      {
        /* Start discovery of all primary services */
        
        ret = aci_gatt_disc_all_primary_services(slaves[i].conn_handle);
        PRINTF ("aci_gatt_disc_all_primary_services(): %d\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_SERVICES;
        }
        else {
          slaves[i].state = IDLE;
        }      
      }
      break;      
      
    case START_TX_CHAR_DISCOVERY:
      {        
        /* Start discovery of TX characteristic */
        
        UUID_t char_UUID;
        
        Osal_MemCpy(&char_UUID.UUID_128, TX_char_uuid, 16);
        ret = aci_gatt_disc_char_by_uuid(slaves[i].conn_handle, slaves[i].chat_serv_start_handle, slaves[i].chat_serv_end_handle, UUID_TYPE_128, &char_UUID);
        PRINTF ("aci_gatt_disc_char_by_uuid() for TX characteristic: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_TX_CHAR;
        }
        else {
          slaves[i].state = IDLE;
        }      
      }
      break;
    case START_RX_CHAR_DISCOVERY:
      {
        
        /* Discovery RX characteristic */
        
        UUID_t char_UUID;
        
        Osal_MemCpy(&char_UUID.UUID_128, RX_char_uuid, 16);
        ret = aci_gatt_disc_char_by_uuid(slaves[i].conn_handle, slaves[i].chat_serv_start_handle, slaves[i].chat_serv_end_handle, UUID_TYPE_128, &char_UUID);
        printf ("aci_gatt_disc_char_by_uuid() for RX characteristic: %d\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_RX_CHAR;
        }
        else {
          slaves[i].state = IDLE;
        }      
      }
      break;
    case START_OFFSET_CHAR_DISCOVERY:
      {
        /* Start discovery of offset characteristic */
        
        UUID_t char_UUID;
        
        Osal_MemCpy(&char_UUID.UUID_128, offset_char_uuid, 16);
        ret = aci_gatt_disc_char_by_uuid(slaves[i].conn_handle, slaves[i].sync_serv_start_handle, slaves[i].sync_serv_end_handle, UUID_TYPE_128, &char_UUID);
        PRINTF ("aci_gatt_disc_char_by_uuid() for Offset characteristic: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_OFFSET_CHAR;
        }
        else {
          slaves[i].state = IDLE;
        }
      }
      break;
    case START_VCLOCK_CHAR_DISCOVERY:
      {
        /* Start discovery of offset characteristic */
        
        UUID_t char_UUID;
        
        Osal_MemCpy(&char_UUID.UUID_128, vclock_char_uuid, 16);
        ret = aci_gatt_disc_char_by_uuid(slaves[i].conn_handle, slaves[i].sync_serv_start_handle, slaves[i].sync_serv_end_handle, UUID_TYPE_128, &char_UUID);
        PRINTF ("aci_gatt_disc_char_by_uuid() for VClock characteristic: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = DISCOVERING_VCLOCK_CHAR;
        }
        else {
          slaves[i].state = IDLE;
        }
      }
      break;
    case ENABLE_NOTIFICATIONS:
      {
        /* Enable notifications for TX characteristic */
        
        uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
        
        ret = aci_gatt_write_char_desc(slaves[i].conn_handle, slaves[i].tx_handle+2, 2, client_char_conf_data);
        PRINTF ("aci_gatt_write_char_desc() to enable TX characteristic notifications: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = ENABLING_NOTIFICATIONS;
        }
        else if(ret == BLE_STATUS_NOT_ALLOWED){
          // Retry later
        }
        else {
          slaves[i].state = IDLE;
        }
      }
      break;
    case WRITE_OFFSET:
      {
        /* Write offset inside slave's characteristic in the sync service */
        
        uint32_t slave_offset = 0;
        
        if(i == 0)
          slave_offset = SLAVE0_OFFSET;
        else if(i == 1)
          slave_offset = SLAVE1_OFFSET;
          
        ret = aci_gatt_write_char_value(slaves[i].conn_handle, slaves[i].offset_handle+1, 4, (uint8_t *)&slave_offset);
        
        PRINTF ("aci_gatt_write_char_value() to write sync offset: 0x%02X\r\n", ret);
        if(ret == 0){
          slaves[i].state = WRITING_OFFSET;
        }
        else {
          slaves[i].state = IDLE;
        }
        
      }
      break;     
    case EXCHANGE_CONFIG:
      {
        /* Exchange ATT MTU */        
        ret = aci_gatt_exchange_config(slaves[i].conn_handle);
        PRINTF ("aci_gatt_exchange_config(): %d\r\n", ret);
        if(ret == 0){
          slaves[i].state = EXCHANGING_CONFIG;
        }
        else {
          slaves[i].state = IDLE;
        }      
        
      }
      break;
    case WRITE_VCLOCK:
      {
        /* Write vtime inside slave's characteristic in the sync service */
        
        uint32_t vtime;
        uint16_t event_counter;
        uint8_t  charac_val[6];
        
        MFT_Interrupt_Disable();
        GetVTimeOnLastSync(i, &vtime, &event_counter);
        MFT_Interrupt_Enable();
        
        HOST_TO_LE_16(charac_val, event_counter);
        HOST_TO_LE_32(charac_val+2, vtime);
          
        ret = aci_gatt_write_char_value(slaves[i].conn_handle, slaves[i].vclock_handle+1, 6, charac_val);
        
        PRINTF ("aci_gatt_write_char_value() to write vtime (event counter %d): 0x%02X\r\n", event_counter, ret);
        if(ret == 0){
          slaves[i].state = WRITING_VCLOCK;
        }
        else {
          slaves[i].state = IDLE;
        }
        
      }
      break; 
    default:
    	break;
      
    }
  
  }
  
}

/*******************************************************************************
* Function Name  : get_slave_index.
* Description    : Get the index in the 'slaves' array corresponding to the given
*                  connection handle.
* Input          : conn_handle Connection handle
* Return         : index of the slave. If -1, slave has not been found
*******************************************************************************/
int get_slave_index(uint16_t conn_handle)
{
  int i;
  
  for(i = 0; i < NUM_SLAVES; i++){
    if(slaves[i].conn_handle == conn_handle){
      return i;
    }
  }  
  return -1;
}

#endif

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
  
  if(APP_FLAG(SET_CONNECTABLE))
  {
#if ROLE == MASTER_ROLE
    if(num_connected_slaves < NUM_SLAVES){
        Connect();
        APP_FLAG_CLEAR(SET_CONNECTABLE);
    }
#else
    Connect();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
#endif
    
  }
  
#if CLIENT
  PerSlaveStateMachine();
#endif
  
  uint32_t vtime = 0;
  uint8_t  ret;
  MFT_Interrupt_Disable();
  ret = GetCurrentVTime(&vtime);
  MFT_Interrupt_Enable();
  if(ret != 0 && vtime >= next_vtime){
    GPIO_WriteBit(GPIO_Pin_3, Bit_SET);
    printf("%d\r\n", vtime);    
    next_vtime += VCLOCK_SECONDS;
    GPIO_WriteBit(GPIO_Pin_3, Bit_RESET);
  }
  
}/* end APP_Tick() */

#if ROLE == MASTER_ROLE
uint16_t get_connection_event_counter(uint8_t slave_num)
{
  if(slave_num > NUM_SLAVES)
    return 0;
  
  return slaves[slave_num].event_counter;
}
#endif

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{  
   PRINTF("hci_le_connection_complete_event %d\n", Status);
   
#if ROLE == MASTER_ROLE
   
   if(Status == 0){
     
     for(int i = 0; i < NUM_SLAVES; i++){
       
       if(!address_is_set(slaves[i].address)){
         memcpy(slaves[i].address, Peer_Address, 6);
         slaves[i].conn_handle = Connection_Handle;
         PRINTF("Connected with slave %d: %02X-%02X-%02X-%02X-%02X-%02X\n", i, Peer_Address[5], Peer_Address[4], Peer_Address[3], Peer_Address[2], Peer_Address[1], Peer_Address[0]);
         StartDiscovery(i);
         break;
       }
       
       // event counter set to 0 at disconnection event. No need to set it now.
     }
     
     num_connected_slaves++;
     
     if(num_connected_slaves == 1){
       /* From 0 to 1 slave connected. The only option is to have the DIO6 signal on first slot */
       slot_pulse_index = 0;
       APP_FLAG_SET(SET_CONNECTABLE);
       resync_vclock = TRUE;
     }
     else if(num_connected_slaves == 2){
       /* From 1 to 2 slaves connected. The best option is to have the DIO6 signal on second slot */
       change_slot_pulse_index = TRUE;
     }
     
     SetSyncInterval(Conn_Interval*25); // Converting Conn_Interval to units of 50 us
     
   }
  
#else
  
  connection_handle = Connection_Handle;
  event_counter = 0;
  SetSyncInterval(Conn_Interval*25); // Converting Conn_Interval to units of 50 us (1.25/0.05 = 25)
  
  APP_FLAG_SET(CONNECTED);
  
  PRINTF("Connected\n");
  
#endif

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{     
  if(Status != 0)
    return;
  
#if ROLE == MASTER_ROLE
  
  int i;
    
  i = get_slave_index(Connection_Handle);  
  if(i < 0) // This should not happen
    return;
  
  SlaveInit(i);
  PRINTF("Disconnected from slave %d.\n", i);
  
  APP_FLAG_SET(SET_CONNECTABLE);
  
  num_connected_slaves--;
  
  if(num_connected_slaves == 0){
    // All slaves disconnected. Stop scanning that was running to find the second slave
    // In this way the duty cycle can be higher and the timebase is also reset.
    aci_gap_terminate_gap_proc(GAP_AUTO_CONNECTION_ESTABLISHMENT_PROC);
    Clock_Wait(4);
    slot_pulse_index = 0;
  }
  else if(num_connected_slaves == 1 && i == 1){
    /* Only first slave is connected. The unaligned trigger signal must be on the first slot.
       Offsets must be changed accordingly.
    */
    slot_pulse_index = 0;
  }
  else {
    slot_pulse_index = 1;
  }
  
#else
  
  connection_handle = 0;
  offset = 0xFFFFFFFF;
  
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
  
#endif
  
}/* end hci_disconnection_complete_event() */

#if CLIENT

/*******************************************************************************
 * Function Name  : aci_gatt_notification_event.
 * Description    : This event occurs when a notification is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{ 
  int i;
  
  i = get_slave_index(Connection_Handle);  
  if(i < 0){ // This should not happen
    return;
  }
 
  if(Attribute_Handle == slaves[i].tx_handle+1)
  {
    PRINTF("%d: ", i);
#if DMA_ENABLED
    DMA_Rearm(DMA_CH_UART_TX, (uint32_t) Attribute_Value, Attribute_Value_Length);
    while(dma_state==DMA_IN_PROGRESS); // For the moment do not exit till transfer is complete. As an alternative we can copy into a buffer and exit.
#else
    for(int i = 0; i < Attribute_Value_Length; i++)
      PRINTF("%c", Attribute_Value[i]);
#endif /* DMA_ENABLED */
    PRINTF("\r\n");
  }
}
                                                 
 /*******************************************************************************
 * Function Name  : aci_att_read_by_group_type_resp_event.
 * Description    : This event occurs during a primary service discovery
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/                                                
void aci_att_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t Attribute_Data_Length,
                                           uint8_t Data_Length,
                                           uint8_t Attribute_Data_List[])
{
  int slave_index;
  
  PRINTF("aci_att_read_by_group_type_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
  
  slave_index = get_slave_index(Connection_Handle);  
  if(slave_index < 0) // This should not happen
    return;
  
  switch(slaves[slave_index].state){
    
  case DISCOVERING_SERVICES:
    if(Attribute_Data_Length == 20){ // Only 128bit UUIDs
      for(int i = 0; i < Data_Length; i += Attribute_Data_Length){
        if(memcmp(&Attribute_Data_List[i+4],chat_service_uuid,16) == 0){
          memcpy(&slaves[slave_index].chat_serv_start_handle, &Attribute_Data_List[i], 2);
          memcpy(&slaves[slave_index].chat_serv_end_handle, &Attribute_Data_List[i+2], 2);
          PRINTF("Slave %d, Chat service handles: 0x%04X 0x%04X\n", slave_index, slaves[i].chat_serv_start_handle, slaves[i].chat_serv_end_handle);
        }
        else if(memcmp(&Attribute_Data_List[i+4],sync_service_uuid,16) == 0){
          memcpy(&slaves[slave_index].sync_serv_start_handle, &Attribute_Data_List[i], 2);
          memcpy(&slaves[slave_index].sync_serv_end_handle, &Attribute_Data_List[i+2], 2);
          PRINTF("Slave %d, Sync service handles: 0x%04X 0x%04X\n", slave_index, slaves[i].sync_serv_start_handle, slaves[i].sync_serv_end_handle);
        }
      }
    }
    break;
  default:
	break;
  }
  
}

/*******************************************************************************
 * Function Name  : aci_gatt_disc_read_char_by_uuid_resp_event.
 * Description    : This event occurs during a 
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle,
                                                uint16_t Attribute_Handle,
                                                uint8_t Attribute_Value_Length,
                                                uint8_t Attribute_Value[])
{
  int i;
  
  PRINTF("aci_gatt_disc_read_char_by_uuid_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
  
  i = get_slave_index(Connection_Handle);  
  if(i < 0) // This should not happen
    return;
  
  switch(slaves[i].state){
    
  case DISCOVERING_TX_CHAR:
    slaves[i].tx_handle = Attribute_Handle;
    PRINTF("TX Char handle for slave %d: 0x%04X\n", i, Attribute_Handle);
    break;
  case DISCOVERING_RX_CHAR:
    slaves[i].rx_handle = Attribute_Handle;
    printf("RX Char Handle for slave %d: 0x%04X\n", i, Attribute_Handle);
    break;
  case DISCOVERING_OFFSET_CHAR:
    slaves[i].offset_handle = Attribute_Handle;
    PRINTF("Offset handle for slave %d: 0x%04X\n", i, Attribute_Handle);
    break;
  case DISCOVERING_VCLOCK_CHAR:
    slaves[i].vclock_handle = Attribute_Handle;
    PRINTF("VClock handle for slave %d: 0x%04X\n", i, Attribute_Handle);
    break;
  default:
	break;
  }
  
}

/*******************************************************************************
 * Function Name  : aci_gatt_proc_complete_event.
 * Description    : This event occurs when a GATT procedure complete is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  int i;
  
  i = get_slave_index(Connection_Handle);  
  if(i < 0) // This should not happen
    return;
  
  switch(slaves[i].state){
  case DISCOVERING_SERVICES:
    PRINTF("Discovering services ended.\r\n");
    if(slaves[i].chat_serv_start_handle != 0)
      slaves[i].state = START_TX_CHAR_DISCOVERY;
    else if(slaves[i].sync_serv_start_handle != 0)
      slaves[i].state = START_OFFSET_CHAR_DISCOVERY;
    else
      slaves[i].state = DONE;
    break;    
  case DISCOVERING_TX_CHAR:
    PRINTF("Discovering TX charac ended.\r\n");
    slaves[i].state = START_RX_CHAR_DISCOVERY;
    break;
  case DISCOVERING_RX_CHAR:
    PRINTF("Discovering RX charac ended.\r\n");
    if(slaves[i].sync_serv_start_handle != 0)
      slaves[i].state = START_OFFSET_CHAR_DISCOVERY;
    else
      slaves[i].state = DONE;
    break;
  case DISCOVERING_OFFSET_CHAR:
    PRINTF("Discovering offset charac ended.\r\n");
      slaves[i].state = START_VCLOCK_CHAR_DISCOVERY;
    break;
  case DISCOVERING_VCLOCK_CHAR:
    PRINTF("Discovering VClock charac ended.\r\n");
    if(slaves[i].tx_handle != 0)
      slaves[i].state = ENABLE_NOTIFICATIONS;
    else 
      slaves[i].state = DONE;
    break;
  case ENABLING_NOTIFICATIONS:
    PRINTF("Notifications enabled\r\n");
    if(slaves[i].offset_handle != 0)
      slaves[i].state = WRITE_OFFSET;
    else 
      slaves[i].state = DONE;
    break;
  case WRITING_OFFSET:
    PRINTF("Offset written\r\n");
    slaves[i].state = EXCHANGE_CONFIG;    
    break;
  case EXCHANGING_CONFIG:
    PRINTF("Configuration exchanged\r\n");
    if(slaves[i].vclock_handle != 0)
      slaves[i].state = WRITE_VCLOCK;
    else 
      slaves[i].state = DONE;
    break;
  case WRITING_VCLOCK:
    PRINTF("VClock written\r\n");
    slaves[i].state = DONE;
    
    uint32_t vtime;
    MFT_Interrupt_Disable();
    GetCurrentVTime(&vtime);
    MFT_Interrupt_Enable();
    if(next_vtime == 0){
      /* This is to print at fixed interval for demo. Search for next multiple of a second. */
      next_vtime = (vtime/VCLOCK_SECONDS + 1)*VCLOCK_SECONDS;
    }
    
    break;
  default:
    break;
  }
}

#else /* SERVER */

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  PRINTF("aci_gatt_attribute_modified_event, handle 0x%04X\n",Attr_Handle);
  if(Attr_Handle == offsetCharHandle + 1)
  {
    Osal_MemCpy(&offset, Attr_Data, 4);
    PRINTF("Offset: %.1f us \r\n", ((float)offset)/10);
  }
  else if(Attr_Handle == RXCharHandle + 1)
  {
    for(int i = 0; i < Attr_Data_Length; i++)
      printf("%c", Attr_Data[i]);
    printf("\r\n");
  }
  else if(Attr_Handle == vclockCharHandle + 1)
  {
    uint16_t event_counter = LE_TO_HOST_16(Attr_Data);
    uint32_t vtime = LE_TO_HOST_32(Attr_Data+2);
    
    printf("VClock set: event counter: %d, vtime: %d\r\n", event_counter, vtime);
    
    MFT_Interrupt_Disable();
    uint8_t ret = SetVClockFromMaster(event_counter, vtime);
    MFT_Interrupt_Enable();
    
    printf("set_vclock_from_master %d\r\n", ret);
    
    MFT_Interrupt_Disable();
    GetCurrentVTime(&vtime);
    MFT_Interrupt_Enable();
  
    next_vtime = (vtime/VCLOCK_SECONDS + 1)*VCLOCK_SECONDS;
    
  }
}

#endif

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
} 

uint8_t call_stack_tick = FALSE;

#define CONN_EVENT_MASTER 0x05

/*******************************************************************************
 * Function Name  : aci_hal_end_of_radio_activity_event.
 * Description    : event generated at the end of a radio activity. Called by
 *                  the BTLE_StackTick. See file bluenrg1_events.h.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
  
#if ROLE==MASTER_ROLE
  
  static uint32_t prev_state_sysTime;
  
  if(num_connected_slaves){
    
    if(slot_pulse_index == 0){
      /* Only first slave is connected. Program the virtual timer to expire
      just before the first slot. */
      
      if(Next_State_SysTime - prev_state_sysTime > MASTER_SLOT_DELAY_TH){
        // Next slot is first slot of the anchor period.
        HAL_VTimerStart_sysT32(0,Next_State_SysTime+40);
      }
    }
    else {
      /* Second slave is connected. Program the virtual timer to expire
      just before the second slot. */
      
      if(Next_State_SysTime - prev_state_sysTime < MASTER_SLOT_DELAY_TH){
        // Next slot is for second slave.
        HAL_VTimerStart_sysT32(0,Next_State_SysTime+40);
      }
      
    }
      
    // Update connection event counter
    if(Last_State == CONN_EVENT_MASTER){
      if(num_connected_slaves == 2){
        if(Next_State_SysTime - prev_state_sysTime < MASTER_SLOT_DELAY_TH){
          // This is the slot for the first slave
          slaves[0].event_counter++;
        }
        else {
          // This is the slot for the second slave
          slaves[1].event_counter++;
        }
      }
      else if(num_connected_slaves == 1){
        if(slaves[0].conn_handle){
          // This is the slot for the first slave
          slaves[0].event_counter++;
        }
        else {
          slaves[1].event_counter++;
        }
      }
    }
    
#if TEST    
    for(int i = 0; i < 1; i++){
      tBleStatus ret;
      //static uint8_t test_data[20] = {'D','A','T','A','5','6','7','8','9','A','B','C','D','E','F','G',};
      static uint8_t test_data[WRITE_LENGTH] = {'1','2','3','4'};
      static uint32_t counter = 0;
      //memcpy(test_data+16, &counter, 4);
      memcpy(test_data+WRITE_LENGTH-4, &counter, 4);
      //GPIO_WriteBit(GPIO_Pin_3, Bit_SET);
      for(int i = 0; i < NUM_SLAVES; i++){
        if(slaves[i].conn_handle != 0){
          ret = aci_gatt_write_without_resp(slaves[i].conn_handle, slaves[i].rx_handle+1, sizeof(test_data), test_data);
          if(ret){
            printf("Error %02X\n",ret);
          }
        }
      }
      //GPIO_WriteBit(GPIO_Pin_3, Bit_RESET);
      counter++;
    } 
#endif
    
  }
  
  prev_state_sysTime = Next_State_SysTime;
    
#else // ROLE != MASTER_ROLE
  
  event_counter++;
  
  if (APP_FLAG(CONNECTED) && offset != 0xFFFFFFFF) {  // If connected and offset has been set by the peer.
    HAL_VTimerStart_sysT32(0,Next_State_SysTime+40);
    
#if TEST   
    
    for(int i = 0; i < 1; i++){
      tBleStatus ret;
      //static uint8_t test_data[20] = {'D','A','T','A','5','6','7','8','9','A','B','C','D','E','F','G',};
      static uint8_t test_data[NOTIFICATION_LENGTH] = {'D','A','T','A','1','2','3','4','5','6','7','8','9','0','1','2','3','4','5','6','7','8','9','0',
      '1','2','3','4','5','6','7','8','9','0','1','2','3','4','5','6','7','8','9','0','1','2','3','4','5','6','7','8','9','0','A','B'};
      static uint32_t counter = 0;
      //memcpy(test_data+16, &counter, 4);
      memcpy(test_data+56, &counter, 4);
      //GPIO_WriteBit(GPIO_Pin_3, Bit_SET);
      ret = aci_gatt_update_char_value_ext(connection_handle, chatServHandle, TXCharHandle, 1, sizeof(test_data), 0, sizeof(test_data), test_data);
      if(ret){
        PRINTF("Error %02X\n",ret);
      }
      //GPIO_WriteBit(GPIO_Pin_3, Bit_RESET);
      counter++;
    }
#endif
    
  }
  
#endif

}

/*******************************************************************************
 * Function Name  : HAL_VTimerTimeoutCallback.
 * Description    : function called when a Virtual timers expires.
 *                  See file bluenrg1_stack.h.
 * Input          : See file bluenrg1_stack.h
 * Output         : See file bluenrg1_stack.h
 * Return         : See file bluenrg1_stack.h
 *******************************************************************************/
void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{ 
  if(timerNum != 0)
    return;
  
  // Enable Blue controller signal on GPIO6
  uint8_t i = 6;
  uint8_t mode = 3;
  MODIFY_REG(GPIO->MODE0, (0xF<<(i*4)), (mode << (i*4)) );
  
#if ROLE == MASTER_ROLE
  MFT_Configuration(master_offsets[slot_pulse_index]);
  
  if(change_slot_pulse_index){
    slot_pulse_index = slot_pulse_index ? 0 : 1;
    change_slot_pulse_index = FALSE;
  }
#else
  MFT_Configuration(offset);
#endif
}

/*******************************************************************************
 * Function Name  : aci_att_exchange_mtu_resp_event.
 * Description    : function called when an update of the ATT MTU is received.
 *                  See file bluenrg1_stack.h.
 * Input          : Connection_Handle handle of the connection
 *                  RX_MTU New ATT MTU on this connection handle
 * Output         : None
 * Return         : None
 *******************************************************************************/
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t RX_MTU)
{
  PRINTF("aci_att_exchange_mtu_resp_event, handle: 0x%04X, RX_MTU: %d \n", Connection_Handle, RX_MTU);
}
