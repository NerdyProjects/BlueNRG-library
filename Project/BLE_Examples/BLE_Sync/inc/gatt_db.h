
#ifndef _GATT_DB_H_
#define _GATT_DB_H_

tBleStatus Add_Chat_Service(void);
uint8_t Add_Sync_Service(void);

extern uint16_t chatServHandle, TXCharHandle, RXCharHandle;
extern uint16_t offsetCharHandle, vclockCharHandle;

extern const uint8_t chat_service_uuid[16];
extern const uint8_t TX_char_uuid[16];
extern const uint8_t RX_char_uuid[16];
extern const uint8_t sync_service_uuid[16];
extern const uint8_t offset_char_uuid[16];
extern const uint8_t vclock_char_uuid[16];

#endif /* _GATT_DB_H_ */
