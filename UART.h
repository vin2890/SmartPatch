/*
 * UART.h
 *
 *  Created on: Feb 20, 2019
 *      Author: Kyle Green
 */

#ifndef UART_H_
#define UART_H_
#include "RingBuffer.h"


static rbd_t _inputrb_d;

static rbd_t _outputrb_d;

int UART0_Initialize(void);

int uart_puts(const char *);

int uart_putrb(rbd_t );


#define INPUT_RB_SIZE 32 //must be a power of two

#define OUTPUT_RB_SIZE 128 //must be a power of two

static unsigned char _input_rb_mem[INPUT_RB_SIZE];

static unsigned char _output_rb_mem[OUTPUT_RB_SIZE];

#endif /* UART_H_ */
