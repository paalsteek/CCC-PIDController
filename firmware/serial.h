#ifndef UART_H_
#define UART_H_

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>

#define NEWLINESTR "\r\n"

#ifdef __cplusplus
extern "C" {
#endif

void SerialPutString(char* data);
void SerialPutChar(char data);

void SerialPutInt(int i);
void SerialPutLongInt(long int i);
void SerialPutFloat(float f);

void SerialPutHexByte(char byte);

uint8_t SerialReadLine(char* buf, uint8_t size);
#ifdef __cplusplus
}
#endif

#endif //UART_H_
