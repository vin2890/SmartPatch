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
//! \file   Executive.h
//!
//! \brief  At the heart of the MPBSM system software is the Executing module.
//!			The module keeps track of which events are pending, executing new
//!			events, prioritizing events, and finally putting the MSP430 in sleep
//!			mode whenever no events are pending.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef EXECUTIVE_H
#define EXECUTIVE_H

//*****************************************************************************
// the includes
//*****************************************************************************
#include "Events.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"

unsigned char fastEventBitmask;
volatile uint16_t counterMotionSensor;
volatile uint16_t  desiredCounterValue;

/* Extern Data */
extern uint8_t fastEventBitmask;
extern uint8_t Exec_eventFifo[];
extern uint8_t Exec_eventFifoHead;
extern uint8_t Exec_eventFifoTail;

#define PUBLISH_EVENT(event) Exec_writeEventFifo(event)
#define PUBLISH_FAST_EVENT(event) fastEventBitmask |= event
	

//*****************************************************************************
//
//! \brief   This function is responsible for running the main control loop.
//!		     The control loop is based on checking both the fast-event bitmask
//!			 (for high priority events) and the event FIFO to determine if an
//!			 event needs to be handled.  The event is then dispatched to the
//!			 appropriate handler.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void Exec_run(void);

//*****************************************************************************
//
//! \brief   This function is responsible for reading a single event out of the
//!			 event fifo.
//
//! \param  none
//
//! \return  Event read
//
//*****************************************************************************
//static uint8_t Exec_readEventFifo(void);

//*****************************************************************************
//
//! \brief   This function is responsible for writing a single event to the event
//!			 fifo and updating the appropriate pointers.
//
//! \param event     the byte to write to the Fifo
//
//! \return  None
//
//*****************************************************************************
void Exec_writeEventFifo(uint8_t event);

	
#endif	





