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
//! \file   NFCForumType4TagType.h
//!
//! \brief  This module serves two purposes. First it is responsible for
//!			implementing the NFC Forum Type 4 Tag Type protocol. Mainly this
//!			module creates files, writes to files, reads from files, deletes
//!		    file, and checks to see if the file exist. The two types of files
//!			found are proprietary files and NDEF files. The second part of this
//!			module serves as an event sniffer. Functions inside the module are
//!			constantly analyzing which files are read, written, and deleted
//!		    converting those actions into events for a state machine( StateMachine.h/.c ).
//!			In order to fully understand this module, first the NFC Forum Type 4
//!			Tag Type specifications should first be read.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef REQUEST_PROCESSOR_H_
#define REQUEST_PROCESSOR_H_

#include <stdint.h>

//! \brief This enum defines what is returned if the file is found and not found
//!
enum FileExistType
{
	FileFound 		= 1,
	FileNotFound 	= 0
};

extern uint8_t FileTextE104[];
extern uint8_t FileTextE105[];
extern uint8_t FileTextE106[];
extern uint8_t FileTextE107[];
extern uint8_t FileTextE108[];

//! \brief This data type is common to all files.
//!
typedef struct NdefFile_Type
{
	unsigned char FileID[2];
	unsigned char * FilePointer;
	unsigned int FileLength;
}NdefFileType;


//*****************************************************************************
//
//! \brief   Initializes the file management
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void AppInit();

//*****************************************************************************
//
//! \brief   Service an interrupt from the RF430
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void ServiceInterrupt(unsigned int flags);

//*****************************************************************************
//
//! \brief   See if the file exist. Returned the file descriptor if the file
//!			 does exist.
//
//! \param none     None
//
//! \return  \b 1- FILE_FOUND, or \b 0 - FILE_NOT_FOUND
//
//*****************************************************************************
enum FileExistType SearchForFile(unsigned char *fileId);


//*****************************************************************************
//
//! \brief   Tag has data to be read out on the selected file. This would be a
//!			 result of the UpdateBinary NDEF command
//
//! \param selectedFile     File number. Can be anywhere from 1 and above
//! \param buffer_start     This is the buffere in tehRF430CL331.
//! \param file_offset     	Where to start reading from the file
//! \param length     		The length of the data to read
//
//! \return None
//
//*****************************************************************************
void ReadDataOnFile(unsigned int selectedFile, unsigned int buffer_start, unsigned int file_offset, unsigned int length);


//*****************************************************************************
//
//! \brief   Tag is requesting data on the selected file. This would be a result
//!			 of the ReadBinary NDEF command return how much written
//
//! \param selectedFile     File number. Can be anywhere from 1 and above
//! \param buffer_start     This is the buffere in tehRF430CL331.
//! \param file_offset     	Where to start reading from the file
//! \param length     		The length of the data to read
//
//! \return None
//
//*****************************************************************************
uint16_t SendDataOnFile(unsigned int selectedFile, unsigned int buffer_start, unsigned int file_offset, unsigned int length);


#endif /* REQUEST_PROCESSOR_H_ */
