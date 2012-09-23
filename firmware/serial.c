#include "serial.h"

#include "stdlib.h"

void SerialPutString(char* data)
{
		/* Select the Serial Tx Endpoint */
		Endpoint_SelectEndpoint(CDC_TX_EPADDR);

		/* Write the String to the Endpoint */
		Endpoint_Write_Stream_LE(data, strlen(data), NULL);

		/* Remember if the packet to send completely fills the endpoint */
		bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* If the last packet filled the endpoint, send an empty packet to release the buffer on
		 * the receiver (otherwise all data will be cached until a non-full packet is received) */
		if (IsFull)
		{
			/* Wait until the endpoint is ready for another packet */
			Endpoint_WaitUntilReady();

			/* Send an empty packet to ensure that the host does not buffer data sent to it */
			Endpoint_ClearIN();
		}
}

void SerialPutInt(int i)
{
	char buf[6];
	itoa(i, buf, 10);
	SerialPutString(buf);
}

void SerialPutLongInt(long int i)
{
	char buf[11];
	ltoa(i, buf, 10);
	SerialPutString(buf);
}

void SerialPutHexByte(char byte)
{
	char buf[3];
	itoa(byte, buf, 16);
	SerialPutString(buf);
}

void SerialPutFloat(float f)
{
	char buf[10];
	dtostrf(f, 3, 5, buf);
	SerialPutString(buf);
}
