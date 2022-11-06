
#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

uint8_t SerialPort_DeviceInit(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
void APP_Tick(void);

#endif
