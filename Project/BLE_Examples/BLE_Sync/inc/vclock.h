/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : vclock.h
* Author             : AMS - RF  Application team
* Description        : This library implements a virtual clock that can be
*                      synchronized with another clock through sync events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#define VCLOCK_SECONDS (1000000/50)

/**
* @brief  Initialize the Virtual Clock.
*
*         Call this function to set the start of the virtual clock to the current
*         system  time.
*
* @param  None
*
* @retval None
*/
void InitVClock(void);

/**
* @brief  Set the connection interval.
*
*         Call this function when the connection has been established.
*
* @param  connection_interval The connection interval of the current connection, in units of 50 us.
*
* @retval None
*/
void SetSyncInterval(uint16_t interval);

/**
* @brief  Store a sync event (on master side)
*
*         Function to be called when a sync event is received, in order to update the
*         value of the virtual clock. To be used on the master. The sync events has to be 
*         received at a time interval equal to the connection interval
*
* @param  sys_time Curent value of the system timer (in system time units)
* @param  slave0_event_counter The connection event counter for the first slave (on first slot)
* @param  slave1_event_counter The connection event counter for the second slave (on second slot)
*
* @retval None
*/
void SaveSyncEventOnMaster(uint32_t sys_time, uint16_t slave0_event_counter, uint16_t slave1_event_counter);

/**
* @brief  Store a sync event (on slave side)
*
*         Function to be called when a sync event is received, in order to update the
*         value of the virtual clock. To be used on the slave.
*
* @param  sys_time Curent value of the system timer (in system time units)
* @param  event_counter The connection event counter for the current connection
* @retval None
*/
void SaveSyncEventOnSlave(uint32_t sys_time, uint16_t event_counter);

/**
* @brief  Pass the information about the master virtual clock to the library (on slave side)
*
*         Function to be called as soon as the value of the virtual clock on the master and the
*         related connection event counter has been received. It adjusts the value of the virtual clock
*         in order ot be synchronized with the master clock without offset.
*         WARNING: this function cannot be called while SaveSyncEventOnMaster()
*         or SaveSyncEventOnSlave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param  event_counter Event counter on the master when the value of the virtual clock has been captured.
* @param  vtime Value of the virtual clock on the master at the given connection event counter
*
* @retval 1 (TRUE) the virtual clock has been adjusted with the correct offset. If the sync event with the
*         passed event counter has not been found among the stored records, 0 (FALSE) is returned.
*/
uint8_t SetVClockFromMaster(uint16_t event_counter, uint32_t vtime);

/**
* @brief  Get the value of the virtual clock at the last sync event (master side).
*
*         Function to retrieve the value of the virtual clock (only on the master)
*         and the related value of the connection event counter for the given slave.
*         compared to another one in input.
*         WARNING: this function cannot be called while save_sync_event_on_master()
*         or save_sync_event_on_slave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param  slave The slave number (0 or 1) from which retrieve the event counter at the last
*               sync event
* @param[out]  vtime The value of the virtual clock at the last sync event
* @param[out]  event_counter The value of the event counter for the salve specified by 'slave' parameter.
* @retval None
*/
void GetVTimeOnLastSync(uint8_t slave, uint32_t *vtime, uint16_t *event_counter);

/**
* @brief  Get the time passed from last sync events.
*
*         This function may be useful to determine if the slave virtual clock may have deviated too much from
*         the virtual clock on the master. The less the value is, the more accurate the virtual clock is.
*         Deviation depends on the accuracy of the sleep clocks on both master and slave.
*         WARNING: this function cannot be called while save_sync_event_on_master()
*         or save_sync_event_on_slave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param[out]  vtime The virtual clock time passed from last sync event
*
* @retval None
*/
uint8_t GetTimeFromLastSync(uint32_t *vtime);

/**
* @brief  Get the current value of the virtual clock
*
*         The function returns the current value of the virtual clock. This
*         clock is synchonized using the sync events as a reference. 
*         WARNING: this function cannot be called while save_sync_event_on_master()
*         or save_sync_event_on_slave() are called, therefore interrupts from which these
*         functions are executed must be temporarily disabled.
*
* @param[out]  vtime The current value of the virtual clock (in 50 us units)
*                    The less time has passed from last sync event, the more accurate the
*                    value of the virtual clock is (compared to the master's one).
*
* @retval Returns if the value of the virtual clock is synchronized (with offset 0) with master virtual clock
*         (1: TRUE, 0: FALSE).
*/
uint8_t GetCurrentVTime(uint32_t *vtime);

/**
* @brief  Synchronize the virtual clock to the current system clock.
*
*         The function takes the current value of the virtual clock and sets a new sync event
*         with the current value of the system clock. This function has to be called at the first
*         sync event on the master. Not available on slave.
*
* @param  None
*
* @retval None
*/
void ResyncVClock(void);
