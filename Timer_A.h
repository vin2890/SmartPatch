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
//! \file   Timer_A.h
//!
//! \brief  This is the Timer_A peripheral driver module. All of the functions
//!			found in this peripheral driver module are specific to the Timer_A
//!			peripheral. Furthermore, all the functions found on this module
//!			configure only local MSP430 registers hence removing the need to
//!			enable the watchdog timer whenever executing them. This peripheral
//!			driver module is designe specifically for and will only work for
//!			Timer_A. Configure the peripheral clock source, peripheral frequency,
//!			peripheral counting mode, disable/enable peripheral, clear peripheral
//!			timer, overflow interrupt enable, compare interrupt enable/disable,
//!			trigger source are the functions that are found on this peripheral
//!			driver module. This module was lightly commented as the majority
//!			is explained in great detail using the ADC12_B datasheet. Module
//!			comments are by no means peripheral datasheet supplement.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#ifndef TIMER_A_H_
#define TIMER_A_H_

#include <stdint.h>


//*****************************************************************************
//
//! \brief  This function is called only once and it is called when the MSP430
//!			is initializing. Furthermore, the function is called indirectly by
//!			the Timer peripheral interface module when it itself is initializing.
//!			This function configures the Timer_A peripheral to be clock source
//!			by ACLK. The function also pre-scales the clock by 128 so as to
//!			conserve as much power as possible.
//
//! \param none
//
//! \return	None
//
//*****************************************************************************
void TimerA_ConfigTimer(void);

//*****************************************************************************
//
//! \brief  This function stops the Timer_A counter. To halt the counter in the
//!			Timer_A peripheral, simply clear the MC bits in the Timer_Ax Control
//!			Register.
//
//! \param none
//
//! \return	None
//
//*****************************************************************************
void TimerA_StopMode(void);

//*****************************************************************************
//
//! \brief  This function starts or continues the Timer_A counter. To enbable/continue
//!			the counter in the Timer_A peripheral, simply set the MC bits in the
//!			Timer_Ax Control Register. In up mode, CCR0 determines what the upper
//!			limit of the counter is.
//
//! \param 	    None
//
//! \return		None
//
//*****************************************************************************
void TimerA_UpMode(void);

//*****************************************************************************
//
//! \brief  This function clears the Timer_A counter. To clear the counter in
//!			the Timer_A peripheral, simply set the TACLR bits in the Timer_A0
//!			Control Register.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_clearTimer(void);

//*****************************************************************************
//
//! \brief  This function configures the timer source frequency. To scale down
//!			the timersource frequency simply configure bits ID in teh TA0CTL
//!			register and bits TAIDEX in the TA0EX0 register. The maximum that
//!			can be scaled down in Frequency/64.
//
//! \param IDClockDivider     Input divider. These bits along with the TAIDEX
//!							  bits select the divider for the input clock.
//! \param EX0ClockDivider    Input divider expansion. These bits along with
//!							  the ID bits select the divider for the input clock.
//
//! \return		None
//
//*****************************************************************************
void TimerA_clockDivider(uint16_t IDClockDivider, uint16_t EX0ClockDivider);

//*****************************************************************************
//
//! \brief  This function configures the Timer_A overflow interrupt	as trigger
//!			source for the ADC12_B. The scale down the timersource frequency
//!			simply configure bits ID in teh TA0CTL register and bits TAIDEX in
//!			the TA0EX0 register. The maximum that can be scaled down in
//!			Frequency/64.
//
//! \param 		None
//
//! \return		None
//
//*****************************************************************************
void TimerA_ADC12BTriggerSource(void);

//*****************************************************************************
//
//! \brief  This function enables the Timer_A counter overflow interrupt. To
//!			enable the Timer_A counter overflow interrupt, simply set the TAIE
//!			bits in the Timer_A0 Control Register.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_overflowInterruptEnabled(void);

//*****************************************************************************
//
//! \brief  This function disables the Timer_A counter overflow interrupt.
//!			To enable the Timer_A counter overflow interrupt, simply set the TAIE
//!			bits in the Timer_A0 Control Register.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_overflowinterruptDisabled(void);

//*****************************************************************************
//
//! \brief  This function enables the Timer_A CCR0 interrupt. To enable the
//!			Timer_ACCR0 interrupt, simply set the CCIE bits in the Timer_A0
//!			Control Register. Whenever the TA0R counter reaches TA0CCR0, an
//!			TA0IV_TACCR0 interrupt will be generated if this function is called.
//!			If TimerA_interrupt Enabled is called before this function, the a
//!			back to back TA0IV_TAIFG and TA0IV_TACCR0 Interrupts will be generated.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR0InterruptEnable(void);

//*****************************************************************************
//
//! \brief  This function disables the Timer_A CCR0 interrupt. To disable the
//!			Timer_ACCR0 interrupt, simply clear the CCIE bits in the Timer_A0
//!			Control Register.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR0InterruptDisabled(void);

//*****************************************************************************
//
//! \brief  This function modifies the Timer_A compare value for CCR0 interrupt.
//!			To modify the Timer_ACCR0 interrupt, simply set the TA0CCR0 bits
//!			in the Timer_A Capture/Compare 0 Register. If the Timer_A is set up
//!			in up-mode, parameters delayCycles determines the upper limit of
//!			the TA0R counter.
//
//! \param delayCycles     if(TA0R == delayCycles) generate CCR0 interrupt
//
//! \return		None
//
//*****************************************************************************
void TimerA_ConfigCCR0(unsigned int delayCycles);

//*****************************************************************************
//
//! \brief  This function enables the Timer_A CCR1 interrupt. To enable the
//!			Timer_ACCR0 interrupt, simply set the CCIE bits in the Timer_A0
//!			Control Register. Whenever the TA0R counter	reaches TA0CCR1, an
//!			interrupt will be generated if this function is called.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR1InterruptEnable(void);


//*****************************************************************************
//
//! \brief  This function disables the Timer_A CCR1 interrupt. To disable the
//!			Timer_ACCR1	interrupt, simply clear the CCIE bits in the Timer_A0
//!			Control Register.
//
//! \param 	    None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR1InterruptDisabled(void);

//*****************************************************************************
//
//! \brief  This function modifies Timer_A compare value for CCR1 interrupt.
//!			To modify the Timer_ACCR1 interrupt, simply set the TA0CCR1 bits
//!			in the Timer_A Capture/Compare 0 Register.
//
//! \param delayCycles      if(TA0R == delayCycles) generate CCR1 interrupt
//
//! \return	None
//
//*****************************************************************************
void TimerA_ConfigCCR1(unsigned int delayCycles);

//*****************************************************************************
//
//! \brief  This function enables the Timer_A CCR2 interrupt. To enable the
//!			Timer_ACCR0 interrupt, simply set the CCIE bits in the Timer_A0
//!			Control Register. Whenever the TA0R counter reaches TA0CCR2, an
//!			interrupt will be generated if this function is called.
//
//! \param      None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR2InterruptEnable(void);

//*****************************************************************************
//
//! \brief  This function disables the Timer_A CCR2 interrupt. To disable the
//!			Timer_ACCR2 interrupt, simply clear the CCIE bits in the Timer_A0
//!			Control Register.
//
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerA_CCR2InterruptDisabled(void);

//*****************************************************************************
//
//! \brief  This function modifies Timer_A compare value for CCR2 interrupt.
///!		To modify the Timer_ACCR2 interrupt, simply clear the TA0CCR2 bits
//!			in the Timer_A Capture/Compare 0 Register.
//
//! \param delayCycles      if(TA0R == delayCycles) generate CCR2 interrupt
//
//! \return		None
//
//*****************************************************************************
void TimerA_ConfigCCR2(unsigned int delayCycles);

#endif /* TIMER_A_H_ */
