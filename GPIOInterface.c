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
//! \file   GPIOInterface.c
//!
//! \brief  Please see GPIOInterface.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#include <msp430.h>
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "GPIO.h"
#include "driverlib.h"

void GPIOInterface_initGPIO(void)
{
    P1OUT = 0x00;
    P1DIR = 0x00;

    P2OUT = 0x00;
    P2DIR = 0x00;
    //put in place an interrupt on RX pin so we can wake from uart
//    P2IN |= BIT1;
//    P2IES = P2IV_P2IFG1;
//    P2IE = P2IV_P2IFG1;
    //
    P3OUT = 0x00;
    P3DIR = 0x00;

    P4OUT = 0x00;
    P4DIR = 0x00;

    P5OUT = 0x00;
    P5DIR = 0x00;

    P6OUT = 0x00;
    P6DIR = 0x00;

    P7OUT = 0x00;
    P7DIR = 0x00;

    P8OUT = 0x00;
    P8DIR = 0x00;

	//P3DIR = P3DIR | 0x01;
	//P3DIR = P3DIR | 0x08;

    //GPIO1 and 2 from ADS1292 (pin 9 (port 2.7) and 25 (port 3.3) respectively)
    P2DIR |= (1<<7); //Port 2.7
    P3DIR |= (1<<3); //Port 3.3


    //P2SEL0 |= BIT0 | BIT1;
    //P2SEL1 &= ~(BIT0 | BIT1);
}

void GPIOInterface_initClocks(void)
{
	volatile uint32_t ACLK_Clock;
	volatile uint32_t MCLK_Clock;
	volatile uint32_t SMCLK_Clock;

    //Set DCO Frequency to 8MHz --- This original comment is wrong, the DCO used to be actually set to 4MHz, unsure if this was intentional or just a mistake
	//TODO: Investigate necessary clock rates for peripherals
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_3);
    //CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_3); //yet another mistake I found... this doesn't actually set the DCO Frequency to 8MHz, this sets it to 4 MHz, CS_DCORSEL_1 and CS_DCOFSEL_3 is needed for 8Mhz
    //I'm unsure what the clock needs to be at this time.  This codebase is a nightmare plate of spaghetti code. - Kyle

    //configure MCLK, SMCLK and ACLK to be source by DCOCLK
    //Set ACLK = DCO with frequency divider of 1
    CS_clockSignalInit(CS_ACLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2); // This initialization seems incorrect due to using CS_DCOCLK_SELECT, leaving it tho -kyle
    //Set SMCLK = DCO with frequency divider of 1
    CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_clockSignalInit(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);


    ACLK_Clock = CS_getACLK();
//
    SMCLK_Clock = CS_getSMCLK();
//
    MCLK_Clock =  CS_getMCLK();
//
    MCLK_Clock =  CS_getMCLK();
}
