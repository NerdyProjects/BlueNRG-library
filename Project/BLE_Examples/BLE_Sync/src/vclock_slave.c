/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : vclock_slave.c
* Author             : AMS - RF  Application team
* Description        : This library implements a virtual clock that can be
*                      synchronized with another clock through sync events. To
*                      be used on slave side.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <hal_types.h>
#include <string.h>
#include "bluenrg1_stack.h"
#include "bluenrg_x_device.h"

// Master will send a packet containing the value of the virtual timer and the value of the connection event counter at that time.
// This connection event counter may not be the latest one. So, at least last 3 events are stored.
#define NUM_SYNC_EVENTS 3

#define FROM_SYS_TO_VCLOCK(sys_time) ((uint32_t)(((uint64_t)(sys_time))*625/(256*50)))

struct sync_evt_slave {  
  uint32_t sys_time; // Value of the system timer when last sync has been received
  uint16_t event_counter; // event counter when last sync has been received
  uint32_t virtual_time; // Virtual time when last sync has been received (in units of 50 us)
  uint8_t  valid;         // If this is a valid sync event
};

static struct sync_evt_slave sync_evts[NUM_SYNC_EVENTS];
static int8_t last_saved_event_index = 0;

static uint32_t vtime_offset = 0; // Offset between the local vclock and the master vclock
static uint8_t  vtime_offset_set = FALSE; // If vtime_offset has been set or not
static uint16_t sync_interval = 0; // In units of 50 us

void InitVClock(void)
{
  memset(sync_evts, 0, sizeof(sync_evts));
  vtime_offset_set = FALSE;
  last_saved_event_index = 0;
  sync_evts[last_saved_event_index].sys_time = HAL_VTimerGetCurrentTime_sysT32();
  sync_evts[last_saved_event_index].virtual_time = 0;
}

void SetSyncInterval(uint16_t interval)
{
  sync_interval = interval;
}

void SaveSyncEventOnSlave(uint32_t sys_time, uint16_t event_counter)
{  
  uint32_t old_vtime = sync_evts[last_saved_event_index].virtual_time;
  uint32_t old_systime = sync_evts[last_saved_event_index].sys_time;
  uint32_t vtime_from_last_sync = FROM_SYS_TO_VCLOCK(sys_time - old_systime);
  uint32_t num_sync = 1;
  
  if(vtime_from_last_sync > sync_interval+sync_interval/2){
    // Some sync events have been missed, estimate how many
    num_sync = vtime_from_last_sync/sync_interval;
    if(num_sync > 20){
      /* Too many missed sync events.
         TODO: what to do? Local virtual clock may have been diverged too much.
         For the moment set vclock as unreliable. It will be set to reliable
         after a call to set_vclock_from_master. */
      vtime_offset_set = FALSE;
    }
    
    /* Evaluate if the rest the devision vtime_from_last_sync/sync_interval is greater
      than sync_interval/2  */
    if((vtime_from_last_sync - (num_sync * sync_interval)) > sync_interval/2){
      // Round to the next multiple of sync_interval
      num_sync++;
    }
    
  }
  
  last_saved_event_index = (last_saved_event_index+1)%NUM_SYNC_EVENTS;
  
  sync_evts[last_saved_event_index].sys_time = sys_time;
  sync_evts[last_saved_event_index].event_counter = event_counter;
  sync_evts[last_saved_event_index].virtual_time = old_vtime + num_sync*sync_interval;
  sync_evts[last_saved_event_index].valid = TRUE;
}

uint8_t SetVClockFromMaster(uint16_t event_counter, uint32_t vtime)
{
  // Search for the sync event with the given event counter
  for(uint8_t i = 0; i < NUM_SYNC_EVENTS; i++){
    if(sync_evts[i].valid && sync_evts[i].event_counter == event_counter){
      // Sync event found!
      vtime_offset = vtime - sync_evts[i].virtual_time;
      vtime_offset_set = TRUE;
      return TRUE;
    }
  }
  
  return FALSE;    
}

uint8_t GetCurrentVTime(uint32_t *vtime)
{
  uint32_t from_last_sync_evt;
  uint8_t synchronized = TRUE;

  if(!vtime_offset_set){
    // Clock is not yet synchronized
    synchronized = FALSE;
  }
  
  from_last_sync_evt = FROM_SYS_TO_VCLOCK(HAL_VTimerGetCurrentTime_sysT32()-sync_evts[last_saved_event_index].sys_time);
  *vtime = from_last_sync_evt + sync_evts[last_saved_event_index].virtual_time + vtime_offset;
  
  return synchronized;    
}

uint8_t GetTimeFromLastSync(uint32_t *vtime)
{
  if(!vtime_offset_set){
    // Clock is not yet synchronized
    return FALSE;
  }
  
  *vtime = FROM_SYS_TO_VCLOCK(HAL_VTimerGetCurrentTime_sysT32()-sync_evts[last_saved_event_index].sys_time);
  
  return TRUE;  
}


