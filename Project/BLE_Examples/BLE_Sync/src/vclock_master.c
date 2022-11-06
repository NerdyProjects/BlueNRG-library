/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : vclock_master.c
* Author             : AMS - RF  Application team
* Description        : This library implements a virtual clock that can be
*                      synchronized with another clock through sync events. To
*                      be used on master side.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <hal_types.h>
#include "bluenrg1_stack.h"
#include "bluenrg_x_device.h"

#define FROM_SYS_TO_VCLOCK(sys_time) ((uint64_t)(sys_time))*625/(256*50)

struct sync_evt_master {  
  uint32_t sys_time; // Value of the system timer when last sync has been received
  uint16_t slave_event_counter[2]; // event counters for slave 0 and 1 when last sync has been received
  uint32_t virtual_time; // Virtual time when last sync has been received (in units of 50 us)
};

static struct sync_evt_master last_sync_evt_master;

static uint16_t sync_interval = 0; // In units of 50 us

void InitVClock(void)
{
  last_sync_evt_master.sys_time = HAL_VTimerGetCurrentTime_sysT32();
  last_sync_evt_master.virtual_time = 0;
}

void SetSyncInterval(uint16_t interval)
{
  sync_interval = interval;
}

void SaveSyncEventOnMaster(uint32_t sys_time, uint16_t slave0_event_counter, uint16_t slave1_event_counter)
{
  last_sync_evt_master.sys_time = sys_time;
  last_sync_evt_master.slave_event_counter[0] = slave0_event_counter;
  last_sync_evt_master.slave_event_counter[1] = slave1_event_counter;
  last_sync_evt_master.virtual_time += sync_interval;
}

void ResyncVClock(void)
{
  uint32_t vtime_from_last_sync_evt;
  uint32_t current_vtime;
  uint32_t current_sys_time = HAL_VTimerGetCurrentTime_sysT32();
  
  vtime_from_last_sync_evt = FROM_SYS_TO_VCLOCK(current_sys_time-last_sync_evt_master.sys_time);
  current_vtime = vtime_from_last_sync_evt + last_sync_evt_master.virtual_time;
  
  // Treat this time as if there has been a sync event.
  last_sync_evt_master.sys_time = current_sys_time;
  last_sync_evt_master.virtual_time = current_vtime;
  last_sync_evt_master.slave_event_counter[0] = 0;
  last_sync_evt_master.slave_event_counter[1] = 0;
}

void GetVTimeOnLastSync(uint8_t slave, uint32_t *vtime, uint16_t *event_counter)
{  
  *vtime = last_sync_evt_master.virtual_time;
  *event_counter = last_sync_evt_master.slave_event_counter[slave];    
}

uint8_t GetCurrentVTime(uint32_t *vtime)
{
  uint32_t vtime_from_last_sync_evt;
  
  vtime_from_last_sync_evt = FROM_SYS_TO_VCLOCK(HAL_VTimerGetCurrentTime_sysT32()-last_sync_evt_master.sys_time);
  *vtime = vtime_from_last_sync_evt + last_sync_evt_master.virtual_time;
  
  return TRUE;    
}
