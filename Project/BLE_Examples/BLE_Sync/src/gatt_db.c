
#include <stdio.h>
#include <string.h>
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "osal.h"
#include "app_state.h"
#include "SDK_EVAL_Config.h"
#include "sync.h"


uint16_t chatServHandle, TXCharHandle, RXCharHandle;
uint16_t syncServHandle, offsetCharHandle, vclockCharHandle;

  /*
  Chat service UUIDs:
  D973F2E0-B19E-11E2-9E96-0800200C9A66
  D973F2E1-B19E-11E2-9E96-0800200C9A66
  */
const uint8_t chat_service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
const uint8_t TX_char_uuid[16] =      {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
const uint8_t RX_char_uuid[16] =      {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};

  /*
Sync service UUIDs:
  3295a048-e6ed-11e7-80c1-9a214cf093ae
  3295a318-e6ed-11e7-80c1-9a214cf093ae
  2288d3c3-2f6a-474a-ae0d-a51342811cb5
  
  09678c8d-0e85-42f9-bd46-7b6dde89b610
  f237cc07-bf7a-4c2a-9117-de70485c0aae
  c641900b-846a-4aa6-9b55-984729b4b7da
  */
const uint8_t sync_service_uuid[16] = {0xae,0x93,0xf0,0x4c,0x21,0x9a,0xc1,0x80,0xe7,0x11,0xed,0xe6,0x48,0xa0,0x95,0x32};
const uint8_t offset_char_uuid[16] =  {0xae,0x93,0xf0,0x4c,0x21,0x9a,0xc1,0x80,0xe7,0x11,0xed,0xe6,0x18,0xa3,0x95,0x32};
const uint8_t vclock_char_uuid[16] =  {0xda,0xb7,0xb4,0x29,0x47,0x98,0x55,0x9b,0xa6,0x4a,0x6a,0x84,0x0b,0x90,0x41,0xc6};

/*******************************************************************************
* Function Name  : Add_Chat_Service
* Description    : Add the Chat service. This service has one characteristic with
*                  notify property, that is used to send data to the client.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Chat_Service(void)
{
  uint8_t ret;
  UUID_t uuid;

  Osal_MemCpy(&uuid.UUID_128, chat_service_uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*)&uuid, PRIMARY_SERVICE, 6, &chatServHandle); 
  if (ret != BLE_STATUS_SUCCESS) goto fail;    

  Osal_MemCpy(&uuid.UUID_128, TX_char_uuid, 16);
  ret =  aci_gatt_add_char(chatServHandle, UUID_TYPE_128, (Char_UUID_t *)&uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                16, 1, &TXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  Osal_MemCpy(&uuid.UUID_128, RX_char_uuid, 16);
  ret =  aci_gatt_add_char(chatServHandle, UUID_TYPE_128, (Char_UUID_t *)&uuid, 8, CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 1, &RXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  printf("Chat Service added.\nTX Char Handle 0x%04X\nRX Char Handle 0x%04X\n", TXCharHandle, RXCharHandle);
  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Chat service.\n");
  return BLE_STATUS_ERROR ;
}

/*******************************************************************************
* Function Name  : Add_Sync_Service
* Description    : Add the Sync service. This service has one writable characteristic.
*                  A client writes into this characteristic to set the offset that
*                  server/slave has to apply to the unaligned sync signal in order to obtain
*                  an aligned one.
* Input          : None
* Return         : Status.
*******************************************************************************/
uint8_t Add_Sync_Service(void)
{
  uint8_t ret;
  UUID_t uuid;

  Osal_MemCpy(&uuid.UUID_128, sync_service_uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t*)&uuid, PRIMARY_SERVICE, 5, &syncServHandle); 
  if (ret != BLE_STATUS_SUCCESS) goto fail;    

  Osal_MemCpy(&uuid.UUID_128, offset_char_uuid, 16);
  ret =  aci_gatt_add_char(syncServHandle, UUID_TYPE_128, (Char_UUID_t*)&uuid, 4, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 0, &offsetCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  // First 2 bytes for event counter, last 4 bytes for vtime
  Osal_MemCpy(&uuid.UUID_128, vclock_char_uuid, 16);
  ret =  aci_gatt_add_char(syncServHandle, UUID_TYPE_128, (Char_UUID_t*)&uuid, 6, CHAR_PROP_WRITE, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                16, 0, &vclockCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  printf("Sync Service added.\nOffset Char Handle 0x%04X\nVClock Char Handle 0x%04X\n", offsetCharHandle, vclockCharHandle);
  return BLE_STATUS_SUCCESS; 

fail:
  printf("Error while adding Sync service.\n");
  return BLE_STATUS_ERROR ;
}



