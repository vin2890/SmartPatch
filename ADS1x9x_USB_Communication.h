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
//! \file   ADS1x9x_USB_Communication.h
//!
//! \brief  This module is the heart of the ADS1292R IC. Heart Rate, Respiration
//!			rate, and ECG filtering are all calculated in this module. There are
//!			only two functions found on this module. Both the functions are called
//!			whenever the ADS1292 ready bit is set. It is recomended that this file
//!			be rename.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ADS1x9x_USB_COMMUNICATION_H_
#define ADS1x9x_USB_COMMUNICATION_H_

//! \brief There are command that can be received.
#ifndef NULL
#define NULL						0x00
#endif
#define START_DATA_HEADER			0x02
#define WRITE_REG_COMMAND			0x91
#define READ_REG_COMMAND			0x92
#define DATA_STREAMING_COMMAND		0x93
#define DATA_STREAMING_PACKET		0x93
#define ACQUIRE_DATA_COMMAND		0x94
#define ACQUIRE_DATA_PACKET 		0x94
#define PROC_DATA_DOWNLOAD_COMMAND	0x95
#define DATA_DOWNLOAD_COMMAND		0x96
#define FIRMWARE_UPGRADE_COMMAND	0x97
#define START_RECORDING_COMMAND		0x98
#define FIRMWARE_VERSION_REQ		0x99
#define STATUS_INFO_REQ 			0x9A
#define FILTER_SELECT_COMMAND		0x9B
#define ERASE_MEMORY_COMMAND		0x9C
#define RESTART_COMMAND				0x9D
#define END_DATA_HEADER				0x03
#define CARRIAGE_RETURN 			0x0D

//! \brief There are responses to the Android app
#define PATCH_NOT_BUSY				0x04
#define PATCH_BUSY					0x05
#define ACK							0X06
#define NCK							0x07

#define ECG_DATA_PACKET_LENGTH 6 // 3 Bytes (24 bits) * 1 Ch status + 2 Ch data = 3 * 3 = 9
#define ECG_ACQUIRE_PACKET_LENGTH 54

//*****************************************************************************
//
//! \brief   This function acquire requested number of samples and send packet
//!			 by packet to PC application. It uses
//!		 			1. ECGRecorder_data_Buf[]:Produced by ISR and consumed by main
//!		 			2. Recorder_head		:Used by ISR
//!		 			3. Recorder_tail		: Used by main
//!			 		4. 4KSPS and below sampling rate data foworded to USB port
//!					   packet by packet of 8 samples.
//!			 		5. 8KSPS case adc data is stored first in memory and after
//!					   storing all samples then the sored samples are forworded to USB port.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void Accquire_ECG_Samples(void);

//*****************************************************************************
//
//! \brief   Input 	: ECG_Data_rdy : ADC sets this flag after reading ADC readings.
//!  		: ECG_Data_rdy : ADC sets this flag after reading ADC readings.
//!			: ADS1x9x_ECG_Data_buf[]: Satatus,ECG&RESP dada
//!			: sampleCNT : Static variable to track number of samples in a packet
//!
//! Constants: PACK_NUM_SAMPLES Number of samples in a packet
//!
//! 	Output	:
//! 			: ECGTxPacket[] : Tx array.
//! 			: ECGTxPacketRdy : Flag to indicate datapacket is ready
//! 			: ECGTxCount : Holds number of bytes need to send
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
int Stream_ECG_data_packets(void);


#endif /*ADS1x9x_USB_COMMUNICATION_H_*/
