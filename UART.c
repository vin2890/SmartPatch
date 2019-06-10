/*
 * UART.c
 *
 *  Created on: Feb 20, 2019
 *      Author: Kyle Green
 */
#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "types.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "RingBuffer.h"
#include "UART.h"
#include "Events.h"
#include "Executive.h"

int UART0_Initialize(void){

    //Setup ring buffers
    //http://www.simplyembedded.org/tutorials/msp430-uart/
    rb_attr_t _rb_input_attr = {sizeof(_input_rb_mem[0]), INPUT_RB_SIZE,  _input_rb_mem};
    rb_attr_t _rb_output_attr = {sizeof(_output_rb_mem[0]), OUTPUT_RB_SIZE,  _output_rb_mem};
    if(ring_buffer_init(&_inputrb_d, &_rb_input_attr)!=0){
        return (-1);
    }
    if(ring_buffer_init(&_outputrb_d, &_rb_output_attr)!=0){
        return (-1);
    }
    //

    WDTCTL = WDTPW | WDTHOLD;   // Stop Watchdog

    // Configure GPIO
    P4SEL0 |= BIT2 | BIT3;                    // USCI_A0 UART operation
    P4SEL1 &= ~(BIT2 | BIT3);

    //Disable GPIO power-on high-impedance mode

    PM5CTL0 &= ~LOCKLPM5;

    //Clocks should already be set up at this point
    // Startup clock system with max DCO setting ~8MHz
//    CSCTL0_H = CSKEY >> 8;                    // Unlock clock registers
//    CSCTL1 = DCOFSEL_3 | DCORSEL;             // Set DCO to 8MHz
//    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
//    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
//    CSCTL0_H = 0;                             // Lock CS registers

    //
    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;
//    UCA0BR0 = 8;                             // BR-4, BRF-10, BRS val - 0xf700 corresponds to 115200 baud at 8MHz DCO clk
//    UCA0BR1 = 0x00;
//    UCA0MCTLW |= UCOS16 | UCBRF_10 | 0xF700;
    UCA0BR0 = 4;                             // BR-4, BRF-5, BRS val - 0x5500 corresponds to 115200 baud at 8MHz DCO clk
    UCA0BR1 = 0x00;
    UCA0MCTLW |= UCOS16 | UCBRF_5 | 0x5500;
    //IFG &= ~(UCA0RXIFG);

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
    uart_puts("BPBSB");
    __enable_interrupt();
    //    __bis_SR_register(LPM3_bits | GIE);       // Enter LPM3, interrupts enabled
    __no_operation();                         // For debugger
    return 0;

}
// int uart_puts(const char *) - Function to simply put a simple string (null terminated) onto UART0
// If line feeds ('\n') are present in the string, a carriage return is appended to make the text make sense in a terminal
// returns - 0 if not empty string, -1 else
int uart_puts(const char *str)
{
    int status = -1; // this status code is only returned if a null string is sent to the function

    if (str != NULL) {
        status = 0;

        while (*str != '\0') {      // loop until the null-termination is the char to be sent

            // Wait til it's okay to send data
            while (!(UCA0IFG & UCTXIFG));

            /* Transmit data */
            UCA0TXBUF = *str; //reference current pointer to get the char in the array

            // If there is a line-feed, add a carriage return
            if (*str == '\n') {
                while (!(UCA0IFG & UCTXIFG)); //wait
                UCA0TXBUF = '\r'; //carriage return
            }
            str++; //move pointer to next char in string
        }
    }

    return status;
}

// int uart_putrb - This function dumps a ringbuffer unto the uart, this function should
// be called by the process filling the uart after some sufficient condition is met
//
int uart_putrb(rbd_t ringbuf){

    char c;
    while(!ring_buffer_get(ringbuf,&c)){   //Empty ring buffer onto the UART
        while(!(UCA0IFG & UCTXIFG));
        UCA0TXBUF = c;
    }
    return 0;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{

  switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:

      while(!(UCA0IFG&UCTXIFG));
      char c = (char)UCA0RXBUF;
      //No need to check to make sure the input buffer is not full, as it will be cleared at the first carriage return by the executive
      if(c=='t'||c=='a'||c=='g'||c=='h'||c=='s'||c=='i')
      {
          ring_buffer_put(_inputrb_d,&c); //small critical section to ensure data is received correctly
          PUBLISH_EVENT(EV_UART_RX);
      }//Publish UART_RX event, now we need to kick the mcu awake on a delimiter (new line for now)
      c=NULL;
     // __bic_SR_register_on_exit(LPM3_bits);
      //__enable_interrupt();
      break;
    case USCI_UART_UCTXIFG:
        break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
  //__bic_SR_register_on_exit(LPM3_bits);
}
