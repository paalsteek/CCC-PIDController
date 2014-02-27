#include "serial.h"

#include "stdlib.h"

void SerialPutString(char* data)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	uint8_t endpoint = Endpoint_GetCurrentEndpoint();

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
		Endpoint_SelectEndpoint(endpoint);
}

void SerialPutChar(char data)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	uint8_t endpoint = Endpoint_GetCurrentEndpoint();

		/* Select the Serial Tx Endpoint */
		Endpoint_SelectEndpoint(CDC_TX_EPADDR);

		/* Write the String to the Endpoint */
		Endpoint_Write_8(data);

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
		Endpoint_SelectEndpoint(endpoint);
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

uint8_t SerialReadLine(char* buf, uint8_t size)
{
	uint8_t pos = 0;
	while (true)
	{
		USB_USBTask();

		/* Device must be connected and configured for the task to run */
		if (USB_DeviceState == DEVICE_STATE_Configured)
		{
			/* Select the Serial Rx Endpoint */
			Endpoint_SelectEndpoint(CDC_RX_EPADDR);

			/* Throw away any received data from the host */
			if (Endpoint_IsOUTReceived())
			{
				if ( (pos + Endpoint_BytesInEndpoint()) >= size )
				{
					SerialPutString(NEWLINESTR "Error reading input: Buffer to small!" NEWLINESTR);
					buf[size - 1] = '\0';
					Endpoint_ClearOUT();
					return 0;
				} else {
					for ( uint16_t i = 0; i < Endpoint_BytesInEndpoint(); i++ )
					{
						char in = Endpoint_Read_8();
						switch ( in )
						{
							case '\n':
							case '\r':
								Endpoint_ClearOUT();
								SerialPutString(NEWLINESTR);
								buf[pos] = '\0';
								return pos;
							/*case '':
								pos--;
								buf[pos] = '\0';
								SerialPutChar('\r');
								SerialPutString(buf);
								break;*/
							default:
								SerialPutChar(in);
								buf[pos] = in;
								pos++;
						}
					}
				}
				Endpoint_ClearOUT();
			}
		}
	}
}
