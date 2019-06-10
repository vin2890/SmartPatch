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
//! \file   TimerInterface.h
//!
//! \brief  This is the Timer peripheral interface module for the Timer_A
//! 		peripheral driver module. This interface module provides functions common to
//! 		all Timer peripherals. Most people familiar with MCU timers should be
//! 		familiar with the functions found in this module however will be not so
//! 		familiar with the functions found in the Timer_A peripheral driver module.
//! 		Call the functions found on the Timer_A peripheral driver file in order
//! 		to initialize the Timer_A peripheral module, start timer, stop timer,
//! 		power down timer and power up timer are some of the functions that
//! 		are found in this peripheral interface module. All of the functions
//!			found on this module are local and hence there is no chance of program
//!			hanging.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef TIMER_H_
#define TIMER_H_

//*****************************************************************************
//
//! \brief  This function is called only once and it is called when the MSP430
//!			is initializing. All the function calls called by this function are
//!			found only the Timer_A peripheral driver module. This peripheral
//!			interface function initializes the Timer_A peripheral and its
//!			interrupts. All the functions called by this function are found only
//!			the Timer_A peripheral driver file. This function configures the
//!			Timer_A peripheral clock source, peripheral clock frequency,
//!			peripheral compare interrupt value, and it initializes peripheral
//!			trigger sources. Finally to conserve power the function powers
//!			down the Timer_A peripheral.
//
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_Init(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to activate the Timer_A
//!			peripheral. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. The function calls called are local
//!			hence there is no chance of program hanging. To activate/reactivate
//!			the Timer_A simply set the TA0CTL bit in the Timer_A0 Control Register
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to deactivate the Timer_A
//!			peripheral. Although, in this case the function simply pipes out the
//!			command sent to it, that is not always the case with peripheral
//!			interface functions. The function calls called are local hence there
//!			is no chance of program hanging. To deactivate the Timer_A simply
//!			clear the TA0CTL bit in the Timer_A0 Control Register
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_powerMode1(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to clear the Timer_A
//!			counter. Although, in this case the function simply pipes out the
//!			command sent to it, that is not always the case with peripheral
//!			interface functions. The function calls called are local hence there
//!			is no chance of program hanging. To clear the counter Timer_A simply
//!			clear the TACLR bit in the Timer_A0 Control Register
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_timer1ClearTimer(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to configures the
//!			timer frequency and the timer counter limit. All the function calls
//!			called by this function are found only the Timer_A peripheral driver
//!			module. The function calls called are local hence there is no
//!			chance of program hanging. By dynamically adjusting the timer
//!			counter limit and the timer clock frequency, it is possible to adjust
//!			the timer in order to maximize performance and power consumption.
//!
//! \param 	timerCounterLimit	Timer Counter Limit
//! \param 	IDClockDivider		ID ClockDivider
//! \param 	EX0ClockDivider		EX0C lockDivider
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_timer1Frequency(uint16_t timerCounterLimit,  uint16_t IDClockDivider, uint16_t EX0ClockDivider );

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to enable the Capture
//!			Compare 0 interrupt. All the function calls called by this function
//!			are found only the Timer_A peripheral driver module. The function
//!			calls called are local hence there is no chance of program hanging.
//!			By dynamically adjusting the timer CCR0 interrupt capare value, it
//!			is possible to adjust the interrupt time.
//!
//! \param compareValue	Compare Value
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_enableTimer1Interrupt0(uint16_t compareValue);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to disable the Capture
//!			Compare 0 interrupt. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. The function call called is local
//!			hence there is no chance of program hanging.
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_disableTimer1Interrupt0(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to enable the Capture
//!			Compare 1 interrupt. All the function calls called by this function
//!			are found only the Timer_A peripheral driver module. The function
//!			calls called are local hence there is no chance of program hanging.
//!			By dynamically adjusting the timer CCR1 interrupt capare value, it
//!			is possible to adjust the interrupt time.
//!
//! \param compareValue	Compare Value
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_enableTimer1Interrupt1(uint16_t compareValue);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to disable the Capture
//!			Compare 1 interrupt. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. The function call called is local
//!			hence there is no chance of program hanging.
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_disableTimer1Interrupt1(void);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to enable the Capture
//!			Compare 2 interrupt. All the function calls called by this function
//!			are found only the Timer_A peripheral driver module. The function
//!			calls called are local hence there is no chance of program hanging.
//!			By dynamically adjusting the timer CCR2 interrupt capare value, it
//!			is possible to adjust the interrupt time.
//!
//! \param compareValue	Compare Value
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_enableTimer1Interrupt2(uint16_t compareValue);

//*****************************************************************************
//
//! \brief  This function is called whenever it is desired to disable the Capture
//!			Compare 2 interrupt. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. The function call called is local
//!			hence there is no chance of program hanging.
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void TimerInterface_disableTimer1Interrupt2(void);


#endif /* TIMER_H_ */
