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
//! \file   Events.h
//!
//! \brief  This module does not contain a .c counterpart. This file contains
//!			all the possible events that can be triggered. Some of the events
//!			are internal and some are external. Say for example when a
//!			temperature data command is sent by the handset, the initial
//!			command is external event however to maintain logging of temperature
//!			then only internal events are used.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef EVENTS_H
#define EVENTS_H


//! \brief Originally, all events were passed in a bitmask...however, an event
//!		 FIFO was finally used, but the coding of the event definitions were
//!		 never translated back....doesn't make a difference, but looks a little
//!		 weird
#define EV_BMA_WATERMARK_INTERRUPT					0x01
#define EV_BMA_NEW_DATA_READY						0x02

#define EV_ACQUIRE_TEMPERATURE_INIT					0x03
#define EV_ACQUIRE_TEMPERATURE						0x04
#define EV_ACQUIRE_TEMPERATURE_STREAMING			0x05
#define EV_ACQUIRE_TEMPERATURE_STREAMING_STOP		0x06

#define EV_ACQUIRE_GSR_DATA_LOGGER_INIT				0x07
#define EV_ACQUIRE_GSR_DATA_LOGGER					0x08
#define EV_ACQUIRE_GSR_STREAMING					0x09
#define EV_ACQUIRE_GSR_STREAMING_RESET				0x0B

#define EV_ACQUIRE_ACCELERATION_DATA_INIT				0x0C
#define EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE				0x0D
#define EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE_UPDATE_STEPCOUNT_AND_SAMPLE_NUMBER		0x0E
#define EV_ACQUIRE_PEDOMETER_STOP						0x0F

#define EV_WRITE_REG						0x10
#define EV_READ_REG							0x11
#define EV_DATA_STREAMING					0x12
#define EV_ACQUIRE_DATA						0x13
#define EV_ACQUIRE_ECG_SAMPLES				0x14
#define EV_DATA_DOWNLOAD					0x15
#define EV_START_RECORDING					0x16
#define EV_FIRMWARE_UPGRADE					0x17
#define EV_FIRMWARE_VERSION					0x18
#define EV_STATUS_INFO						0x19
#define EV_FILTER_SELECT					0x1A
#define EV_ERASE_MEMORY						0x1B
#define EV_ECG_DATA_READY					0x1C
#define EV_ECG_KEY_PRESSED					0x1D
#define EV_ECG_KEY_NOTPRESSED				0x1E
#define EV_UART_RX                          0x1F
#define EV_UART_TX                          0x20



//! \brief This is used to pass fast events through the system so there is a minimum of processing time needed
//! between lines of tracking data
#define FEV_RF430CL331_INTERRUPT 	0x01

//! \brief This is used to pass fast events through the system so there is a minimum of processing time needed
//! between lines of tracking data
#define FEV_PROCESS_LINE_COMPLETE 	0x02

//! \brief This is needed for the event fifo
#define EXEC_EVENT_FIFO_SIZE 8
#define EXEC_EVENT_FIFO_MASK EXEC_EVENT_FIFO_SIZE-1 

#endif

