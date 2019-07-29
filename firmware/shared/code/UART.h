/*
 * simple_UART.h
 *
 *  Created on: Aug 19, 2017
 *      Author: abates
 */

#ifndef SIMPLE_UART_H_
#define SIMPLE_UART_H_


#include "UART_componentSpecific.h"
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	// GPIO
	uint8_t rxPin;
	uint8_t txPin;
	GPIO_TypeDef *	GPIOPort;

	// UART
	USART_TypeDef * UARTPeriph;
	uint32_t UARTInterruptNumber;
	uint32_t baudRate;

	//Common
	void (* enablePeripheralsClockCallback)(void);

} UART_HWConfig_S;

typedef const struct
{
	void (* receiveCallback)(UART_TO_BOARD_MESSAGE_TYPE const * const message);
	const UART_HWConfig_S * const HWConfig;
} UART_config_S;


extern void UART_init();
extern bool UART_push_out(char* mesg);
extern bool UART_push_out_len(char* mesg, int len);

#endif /* SIMPLE_UART_H_ */
