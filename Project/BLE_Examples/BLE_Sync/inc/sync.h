
#ifndef _SYNC_H_
#define _SYNC_H_

#define CE_LENGTH 7
#define MASTER_SLOT_DELAY_TH 2500

// Measured delays between slots when unaligned pulse is on first master slot
#define SLAVE0_DELAY                    580   // Delay between master and slave0
#define SLAVE1_DELAY                    59955 // Delay between master and slave1

// Measured delays between slots when unaligned pulse is on second master slot
#define SLAVE1_DELAY_PULSE_ON_2ND_SLOT  582   // Delay between master and slave1

#define MASTER_ROLE   0
#define SLAVE0_ROLE   1
#define SLAVE1_ROLE   2

#define PULSE_DURATION    40

uint8_t Sync_DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

uint16_t get_connection_event_counter(uint8_t slave_num);

#endif // _SYNC_H_
