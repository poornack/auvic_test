/*
 * UART.c
 */

#include "UART.h"

#include <stdbool.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "Task.h"
#include <string.h>
#include "circBuffer2D.h"
#include "circBuffer1D.h"
#include "pb_common.h"
#include "pb_decode.h"

// Move to common units
#define BITVALUE(x) ( 1U << (x))

//Register bit for enabling TXEIE bit. This is used instead of the definitions in stm32f4xx_usart.h
#define USART_TXEIE	0b10000000
#define USART_RXEIE	0b100000

typedef enum
{
	UART_STATE_IDLE,
	UART_START_PROCESS_RECEIVED_DATA
} UART_state_E;

typedef enum
{
	UART_TX_STATE_IDLE,
	UART_TX_STATE_TRANSMIT,
} UART_TXState_E;

typedef enum
{
	UART_RX_STATE_RECEIVE_PAYLOAD_SIZE,
	UART_RX_STATE_RECEIVE_PAYLOAD,
	UART_RX_STATE_RECEIVE_PAYLOAD_CRC,
} UART_RXState_E;

typedef struct
{
	uint16_t payloadSize;
	uint8_t payload[UART_RX_BUFFER_LENGTH];
	uint32_t crc;
} UART_RXBuffer_S;

typedef struct
{
	TaskHandle_t taskHandle;
	UART_state_E state;
	UART_TXState_E TXState;
	UART_RXState_E RXState;
	uint16_t expectedRXPayloadSize;
	UART_RXBuffer_S RXBuffer;
	uint16_t bytesReceived; 
} UART_data_S;

// Receive buffer for UART, no DMA
uint8_t inputString[UART_RX_BUFFER_LENGTH]; //string to store individual bytes as they are sent
uint8_t inputStringIndex = 0;

static UART_data_S UART_data;
extern UART_config_S UART_config;

static void UART_configureGPIO(void)
{
	// Check that all configs are valid
	configASSERT(IS_GPIO_PIN_SOURCE(UART_config.HWConfig->rxPin));
	configASSERT(IS_GPIO_PIN_SOURCE(UART_config.HWConfig->txPin));
	configASSERT(IS_GPIO_ALL_PERIPH(UART_config.HWConfig->GPIOPort));

	// Clock should be enabled in the enable clock callback

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// Configure GPIOs
	GPIO_InitStructure.GPIO_Pin = BITVALUE(UART_config.HWConfig->rxPin) | BITVALUE(UART_config.HWConfig->txPin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART_config.HWConfig->GPIOPort, &GPIO_InitStructure);

	// Attatch GPIO AF to UART. This section needs to change if UART6 needs to be used
	GPIO_PinAFConfig(UART_config.HWConfig->GPIOPort, UART_config.HWConfig->rxPin, GPIO_AF_USART1);
	GPIO_PinAFConfig(UART_config.HWConfig->GPIOPort, UART_config.HWConfig->txPin, GPIO_AF_USART1);
}

static void UART_configureUARTPeriph(void)
{
	configASSERT(IS_USART_1236_PERIPH(UART_config.HWConfig->UARTPeriph)); // Switch to `IS_USART_APP_PERIPH` if needed

	// Clock should be enabled in the enable clock callback

	USART_InitTypeDef USART_InitStruct;
	USART_StructInit(&USART_InitStruct);

	USART_InitStruct.USART_BaudRate = 9600;	// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(UART_config.HWConfig->UARTPeriph, &USART_InitStruct);

	UART_config.HWConfig->UARTPeriph->CR1 |= USART_RXEIE; //Enable the receive interrupt

	NVIC_SetPriority(UART_config.HWConfig->UARTInterruptNumber, 7); /* (3) */
	NVIC_EnableIRQ(UART_config.HWConfig->UARTInterruptNumber); /* (4) */

	USART_Cmd(UART_config.HWConfig->UARTPeriph, ENABLE);
}

static void UART_run(void)
{
	while(1)
	{
		switch(UART_data.state)
		{
			case UART_START_PROCESS_RECEIVED_DATA:
			{
				if(UART_config.receiveCallback != NULL)
				{
					uint8_t receivedData[UART_RX_BUFFER_LENGTH];
					circBuffer2D_pop(CIRCBUFFER2D_CHANNEL_UART_RX, receivedData);
					pb_istream_t istream = pb_istream_from_buffer(receivedData, sizeof(receivedData));
					UART_TO_BOARD_MESSAGE_TYPE decodedMessage;
					if(pb_decode(&istream, UART_TO_BOARD_MESSAGE_FIELDS, &decodedMessage))
					{
						UART_config.receiveCallback(&decodedMessage);
					}
				}

				UART_data.state = UART_STATE_IDLE;

				break;
			}
			case UART_STATE_IDLE:
			default:
			{
				if(ulTaskNotifyTake(pdFALSE, portMAX_DELAY) != 0U)
				{
					UART_data.state = UART_START_PROCESS_RECEIVED_DATA;
				}
				break;
			}
		}
	}
}

extern void UART_init()
{
	if(UART_config.HWConfig->enablePeripheralsClockCallback != NULL)
	{
		UART_config.HWConfig->enablePeripheralsClockCallback();
	}
	else
	{
		configASSERT(0U);
	}

	UART_configureGPIO();
	UART_configureUARTPeriph();

	UART_data.state = UART_STATE_IDLE;

	(void)xTaskCreate((TaskFunction_t)UART_run,       /* Function that implements the task. */
					  "UARTTask",          /* Text name for the task. */
					  configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
					  NULL,    /* Parameter passed into the task. */
					  tskIDLE_PRIORITY + 1,/* Priority at which the task is created. */
					  &UART_data.taskHandle);      /* Used to pass out the created task's handle. */

}

/*
 * ERROR CODE:
 * -1 = String length is not 1 or greater
 * -2 = OutputBuffer will overflow. Wait some time and retry
 * 1  = Added to buffer successfully
 */
extern bool UART_write(char* mesg)
{
	return UART_push_out_len(mesg, strnlen(mesg, UART_TX_BUFFER_SIZE));
}

extern bool UART_writeLen(char* mesg, int len)
{
	bool ret = false;
	if((mesg != NULL) && (len <= UART_TX_BUFFER_SIZE))
	{
		ret = circBuffer1D_push(CIRCBUFFER1D_CHANNEL_UART_TX, (uint8_t *)mesg, len);
		UART_config.HWConfig->UARTPeriph->CR1 |= USART_TXEIE;
	}

	return ret;
}

// This is handling two cases. The interrupt will run if a character is received
// and when data is moved out from the transmit buffer and the transmit buffer is empty
static inline void UART_commonInterruptHandler(void)
{
	if((UART_config.HWConfig->UARTPeriph->SR & USART_FLAG_RXNE) == USART_FLAG_RXNE) //If character is received
	{
		uint8_t tempInput = UART_config.HWConfig->UARTPeriph->DR;

		switch(UART_data.RXState)
		{
			case UART_RX_STATE_RECEIVE_PAYLOAD_SIZE:
			{
				UART_data.RXBuffer.payloadSize = (UART_data.RXBuffer.payloadSize << 8U) | tempInput; // Need to switch endianess
				UART_data.bytesReceived += 1U;

				if(inputStringIndex >= 2U)
				{
					UART_data.bytesReceived = 0U;
					UART_data.RXState = UART_RX_STATE_RECEIVE_PAYLOAD;
				}
				break;
			}
			case UART_RX_STATE_RECEIVE_PAYLOAD:
			{
				UART_data.RXBuffer.payload[UART_data.bytesReceived] = tempInput;
				UART_data.bytesReceived += 1U;

				if(UART_data.bytesReceived >= UART_RX_BUFFER_LENGTH)
				{
					UART_data.bytesReceived = 0U;
					UART_data.RXState = UART_RX_STATE_RECEIVE_PAYLOAD_CRC;
				}
				break;
			}
			case UART_RX_STATE_RECEIVE_PAYLOAD_CRC:
			{
				UART_data.RXBuffer.crc = (UART_data.RXBuffer.crc << 8U) | tempInput; // Need to switch endianess
				UART_data.bytesReceived += 1U;

				if(UART_data.bytesReceived >= 4U)
				{
					//Verify CRC
					UART_data.bytesReceived = 0U;

					// Notify task if CRC okay
					circBuffer2D_push(CIRCBUFFER2D_CHANNEL_UART_RX, UART_data.RXBuffer.payload, UART_data.RXBuffer.payloadSize);
					memset(&UART_data.RXBuffer, 0U, sizeof(UART_data.RXBuffer));
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					vTaskNotifyGiveFromISR(UART_data.taskHandle, &xHigherPriorityTaskWoken);

					UART_data.RXState = UART_RX_STATE_RECEIVE_PAYLOAD_SIZE;
				}
				break;
			}
			default:
			{
				break;
			}
		}
	}
	else if ((UART_config.HWConfig->UARTPeriph->SR & USART_FLAG_TXE) == USART_FLAG_TXE) // If Transmission is complete
	{
		uint8_t dataToSend;
		if(circBuffer1D_popByte(CIRCBUFFER1D_CHANNEL_UART_TX, &dataToSend))
		{
			UART_config.HWConfig->UARTPeriph->DR = dataToSend;
		}
		else
		{
			UART_config.HWConfig->UARTPeriph->CR1 &= ~USART_TXEIE;
		}
	}
	else
	{
		// Unexpected interrupt. handle it
	}
}

void USART1_IRQHandler()
{
	UART_commonInterruptHandler();
}

void USART2_IRQHandler()
{
	UART_commonInterruptHandler();
}

void USART6_IRQHandler()
{
	UART_commonInterruptHandler();
}
