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
//! \file   TimerInterface.c
//!
//! \brief  Please see TimerInterface.h
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
#include <msp430.h>
#include "Timer_A.h"
#include "TimerInterface.h"
#include <stdint.h>

void TimerInterface_Init(void)
{

	/* This is probably the most important Timer_A function call. It configures the Timer_A peripheral by configuring its registers.  */
	TimerA_ConfigTimer();

	/* Configure CCR0 to overflow/interrupt every 2 seconds. */
	TimerA_ConfigCCR0(100);

	/* Configure CC1 to interrupt every 1 second. */
	TimerA_ConfigCCR1(50);

	/* Configure CC1 to interrupt every 2 seconds. */
	TimerA_ConfigCCR2(100);

	/* For now disable the timer overflow interrupt. */
	TimerA_overflowInterruptEnabled();

	/* For now disable the CCR0 interrupt. */
	TimerA_CCR0InterruptDisabled();

	/* For now disable the CCR1 interrupt. */
	TimerA_CCR1InterruptEnable();

	/* For now disable the CCR2 interrupt. */
	TimerA_CCR2InterruptEnable();

	/* Set the Timer overflow interrupt as the trigger source for the ADC12_B converter. */
	TimerA_ADC12BTriggerSource();

	/* To conserve power, place the Timer in standby mode. */
	TimerInterface_powerMode1();

}

void TimerInterface_powerMode0(void)
{
	/* Set the timer to UpMode. This function also causes the timer to continue counting if the timer was paused. */
	TimerA_UpMode();
}

void TimerInterface_powerMode1(void)
{
	/* Pause the timer. */
	TimerA_StopMode();
}

void TimerInterface_timer1ClearTimer(void)
{
	/* Clear the timer. */
	TimerA_clearTimer();
}

void TimerInterface_timer1Frequency(uint16_t timerCounterLimit,  uint16_t IDClockDivider, uint16_t EX0ClockDivider )
{
	/* Dynamically configure the upper limit of the CCRO timer. */
	TimerA_ConfigCCR0(timerCounterLimit);

	/* Dynamically configure the clock frequency. */
	TimerA_clockDivider(IDClockDivider, EX0ClockDivider);

}

void TimerInterface_enableTimer1Interrupt0(uint16_t compareValue)
{
	/* Enable the CCR0 interrupt. */
	TimerA_CCR0InterruptEnable();

	/* When set to UpMode, this  CCR0 interrupt is unique it determine the upper count limit for the timer.  */
	TimerA_ConfigCCR0(compareValue);
}

void TimerInterface_disableTimer1Interrupt0(void)
{
	/* Disable CCR0 interrupt */
	TimerA_CCR0InterruptDisabled();
}

void TimerInterface_enableTimer1Interrupt1(uint16_t compareValue)
{
	/* Enable the CCR0 interrupt. */
	TimerA_CCR1InterruptEnable();

	/* Set the count value that will generate a CCR1 interrupt. */
	TimerA_ConfigCCR1(compareValue);
}

void TimerInterface_disableTimer1Interrupt1(void)
{
	/* Disable CCR0 interrupt */
	TimerA_CCR1InterruptDisabled();
}

void TimerInterface_enableTimer1Interrupt2(uint16_t compareValue)
{
	/* Disable CCR2 interrupt */
	TimerA_CCR2InterruptEnable();

	/* Set the count value that will generate a CCR2 interrupt. */
	TimerA_ConfigCCR2(compareValue);
}

void TimerInterface_disableTimer1Interrupt2(void)
{
	/* Disable CCR1 interrupt */
	TimerA_CCR2InterruptDisabled();
}


