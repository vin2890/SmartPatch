/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//#############################################################################
//
//! \file   eUSCIA1_SPI.c
//!
//! \brief  Please see USCIA1_SPI.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

//*****************************************************************************
// the includes
//*****************************************************************************
#include "driverlib.h"
#include "eUSCIA1_SPI.h"
//TODO:The naming in this file is incorrect as we're using SPI on eUSCIB1... not changing for now
void USCIA1_InitializeRegisters(void)
{

	  //P3SEL0 |= ((enum PORT3_ADC_CONTROL)SPI_CLK + (enum PORT3_ADC_CONTROL)SPI_SIMO + (enum PORT3_ADC_CONTROL)SPI_SOMI);  				// Set SPI peripheral bits
    P3SEL0 |= (1<<0) + (1<<1) + (1<<2); // Ports 3.0-3.2 are SPI_CLK, SPI_SIMO, and SPI_SOMI respectively
	  //P3DIR |= ((enum PORT3_ADC_CONTROL)SPI_CLK + (enum PORT3_ADC_CONTROL)SPI_SIMO);
    P3DIR |= (1<<0) + (1<<1); // Ports 3.0 (SPI_CLK) and 3.1 (SPI_SIMO) are output
	  //P3DIR &= ~((enum PORT3_ADC_CONTROL)SPI_SOMI);                         	// Din as input
    P3DIR &= ~(1<<2); // Port 3.2 (SPI_SOMI) is input



	  UCB1CTLW0 = UCSWRST;
	  //UCA1CTL1 |= UCSWRST;               		// Enable SW reset
	  //UCSYNC - SPI mode is selected when the UCSYNC bit is set.
	  //UCMSB - The UCMSB bit controls the direction of the transfer and selects LSB or MSB first.
	  //UCMST - When UCMST = 1, the bit clock is provided by the USCI bit clock generator on the UCxCLK pin.
	  //        When UCMST = 0, the USCI clock is provided on the UCxCLK pin by the master.
	  //UCA1CTL0 |= UCMSB + UCMST + UCSYNC;		//[b0]   1 -  Synchronous mode
	  UCB1CTLW0 |= UCMST | UCSYNC | UCMSB;
												//[b2-1] 00-  3-pin SPI mode
												//[b3]   1 -  Master mode
												//[b4]   0 - 8-bit data
												//[b5]   1 - MSB first
												//[b6]   0 - The inactive state is low.
												//[b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

	  //USCI clock source select.
	  //UCA1CTL1 |= UCSSEL__ACLK;               	// ACLK
	  //UCB1CTL1 |= UCSSEL__ACLK; //This should be a faster clock than ACLK, will try sourcing SMCLK instead
	  UCB1CTL1 |= UCSSEL__SMCLK; //run spi on SMCLK
	  //UCA1CTLW0 |= UCSSEL__SMCLK;
	  //UCA1BR0 = 24;                             // 1 MHz
	  //UCA1BR1 = 0;                              //
	  UCB1BR0 = 0x00;                           // /8MHz SPI clk
	  UCB1BR1 = 0;                              //
	  //UCA1CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation

	  //UCA1MCTLW = 0;                            // No modulation
	  //Upon switching code to UCB1 there is no modulation capability to turn off on UCB1 --Kyle
	  UCB1CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**

}

void USCIA1_InitializePorts(void)
{
    //Data direction for Port 2
        //P2DIR |= ((enum PORT2_ADC_CONTROL)ADC_RESET + (enum PORT2_ADC_CONTROL)ADC_START + (enum PORT2_ADC_CONTROL)ADC_CLK_SEL); //Set port 2 pin 1 and 2 as output
    P2DIR |= (1<<1) + (1<<2) + (1<<3); //Ports 2.1, 2.2 and 2.3 are ADS1292 clk select, reset and start pins respectively
        //P2OUT &= ~((enum PORT2_ADC_CONTROL)ADC_RESET + (enum PORT2_ADC_CONTROL)ADC_START);//Set port pins 1 and 2 to low
    P2OUT &= ~((1<<2) + (1<<3)); //Set Port 2.2 and 2.3 to low
        //P2OUT |=  ((enum PORT2_ADC_CONTROL)ADC_RESET +  (enum PORT2_ADC_CONTROL)ADC_CLK_SEL); //Set the reset MSP430 pin as high
    P2OUT |= (1<<2) + (1<<3);  //Set Port 2.2 and 2.3 to high

	  //P3DIR |=  ((enum PORT3_ADC_CONTROL)SPI_CS); //Set SPI chip select to output
    P5DIR |= (1<<3);  //Port 5.3 (SPI_CS) to output
	  //P3OUT &= ~((enum PORT3_ADC_CONTROL)SPI_CS);//Set SPI chip select to low
    P5OUT &= ~(1<<3); //Port 5.3 (SPI_CS) to low
	  //P3OUT |=  ((enum PORT3_ADC_CONTROL)SPI_CS);	//Set SPI chip select to high
    P5OUT |= (1<<3); //Port 5.3 (SPI_CS) to high

}


