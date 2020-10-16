/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : DTM_burst.h
* Author             : AMS - RF Application team
* Description        : Header file for module implementing test burst commands
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef _DTM_BURST_H_
#define _DTM_BURST_H_

#include "ble_status.h"

tBleStatus BURST_TXNotificationStart(uint16_t Connection_Handle, uint16_t Service_Handle,
                                      uint16_t Char_Handle, uint16_t Value_Length);

tBleStatus BURST_TXWriteCommandStart(uint16_t Connection_Handle, uint16_t Attr_Handle,
                                     uint16_t Value_Length);

tBleStatus BURST_RXStart(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Notifications_WriteCmds);

tBleStatus BURST_TXStop(void);

tBleStatus BURST_RXStop(void);

uint8_t BURST_NotificationReceived(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Value_Length, uint8_t Value[]);

uint8_t BURST_WriteReceived(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Value_Length, uint8_t Value[]);

uint8_t BURST_BufferAvailableNotify(void);

uint32_t BURST_TXReport(void);

uint32_t BURST_RXReport(uint16_t *Data_Length, uint32_t *Sequence_Errors);

void BURST_Tick(void);


#endif /* _DTM_BURST_H_ */
