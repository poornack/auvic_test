/*
 * simple_UART.c
 *
 *  Created on: Aug 19, 2017
 *      Author: abates
 */


#include "UART.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "Task.h"
#include <string.h>
#include "circBuffer2D.h"
#include "circBuffer1D.h"
#include <stdbool.h>

//Register bit for enabling TXEIE bit. This is used instead of the definitions in stm32f4xx_usart.h
#define USART_TXEIE	0b10000000
#define USART_RXEIE	0b100000

// Receive buffer for UART, no DMA
char inputString[MAX_BUFFER_DATA]; //string to store individual bytes as they are sent
uint8_t inputStringIndex = 0;

// Task handle to notify FSM task
TaskHandle_t UARTTaskToNotify = NULL;

typedef struct
{
	Buffer_t 	 rxBuffer;
	TaskHandle_t runTaskHandle;
} UART_data_S;

static UART_data_S UART_data;

static void Configure_GPIO_USART1(void) {
	/* Enable the peripheral clock of GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Configuration: TIM5 CH1 (PA0) */
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect USART1 pins to AF */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

/**
 * @brief  This function configures USART1.
 * @param  None
 * @retval None
 */
static void Configure_USART1(void) {
	/* Enable the peripheral clock USART1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//RCC->CFGR3 |= RCC_CFGR3_USART1SW_1;
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization

	USART_InitStruct.USART_BaudRate = 9600;	// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);

	USART1->CR1 |= USART_RXEIE; //Enable the USART1 receive interrupt

	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 7); /* (3) */
	NVIC_EnableIRQ(USART1_IRQn); /* (4) */

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

extern UART_config_S UART_config;
static void UART_run(void)
{
	while(1) {

		(void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		if(UART_config.receiveCallback != NULL)
		{
			uint8_t data[40]; // Whatever the buffer length is
			Buffer_pop(&UART_data.rxBuffer, (char *)data);
			UART_config.receiveCallback(data, 40);
		}
	}
}

extern void UART_init() {

	//initialize the input buffer
	Buffer_init(&UART_data.rxBuffer);

	//initialize the UART driver
	Configure_GPIO_USART1();
	Configure_USART1();

	(void)xTaskCreate((TaskFunction_t)UART_run,       /* Function that implements the task. */
					  "UARTTask",          /* Text name for the task. */
					  configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
					  NULL,    /* Parameter passed into the task. */
					  tskIDLE_PRIORITY + 1,/* Priority at which the task is created. */
					  &UART_data.runTaskHandle);      /* Used to pass out the created task's handle. */


}

/*
 * ERROR CODE:
 * -1 = String length is not 1 or greater
 * -2 = OutputBuffer will overflow. Wait some time and retry
 * 1  = Added to buffer successfully
 */
extern bool UART_push_out(char* mesg) {

	 return UART_push_out_len(mesg, strlen(mesg));
}

extern bool UART_push_out_len(char* mesg, int len) {

	bool ret = false;
	ret = circBuffer1D_push(CIRCBUFFER1D_CHANNEL_UART_TX, (uint8_t *)mesg, len);

	USART1->CR1 |= USART_TXEIE;
	return ret;
}

// This is handling two cases. The interrupt will run if a character is received
// and when data is moved out from the transmit buffer and the transmit buffer is empty
void USART1_IRQHandler() {

	if((USART1->SR & USART_FLAG_RXNE) == USART_FLAG_RXNE) { //If character is received

		char tempInput[1];
		tempInput[0] = USART1->DR;

		//Check for new line character which indicates end of command
		if (tempInput[0] == '\n' || tempInput[0] == '\r') {

			if(strlen(inputString) > 0) {
				Buffer_add(&UART_data.rxBuffer, inputString, MAX_BUFFER_DATA);
				memset(inputString, 0, MAX_BUFFER_DATA);
				inputStringIndex = 0;

				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				vTaskNotifyGiveFromISR(UARTTaskToNotify, &xHigherPriorityTaskWoken);
			}

		} else {
			inputString[inputStringIndex] = tempInput[0];
			inputStringIndex = (inputStringIndex + 1) % MAX_BUFFER_DATA;
		}

	} else if ((USART1->SR & USART_FLAG_TXE) == USART_FLAG_TXE) { // If Transmission is complete

		uint8_t dataToSend;
		if(circBuffer1D_popByte(CIRCBUFFER1D_CHANNEL_UART_TX, &dataToSend))
		{
			USART1->DR = dataToSend;
		} else
		{
			USART1->CR1 &= ~USART_TXEIE;
		}
	}
}
