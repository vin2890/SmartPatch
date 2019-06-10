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
//! \file   NFCInterface.h
//!
//! \brief  This is the IC interface file for NFC transponder ICs.
//! 		This IC interface file contains function calls which are only found
//!			in the RF430CL331 IC driver file. If its corresponding RF430CL331
//!			IC driver files changes, then this IC interface file should also
//!			change as well. This IC interface file has generic function calls
//!			which should be found in similar NFC transponder IC’s. For example
//!			power up IC, power down IC, initialize IC registers.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef NFCINTERFACE_H_
#define NFCINTERFACE_H_

#include "types.h"

//*****************************************************************************
//
//! \brief   This function initializes the RF430CL331 IC by calling functions
//!			 found in the RF430CL331 IC driver file.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void NFCInterface_Init(void);

//*****************************************************************************
//
//! \brief   This function puts the NFC IC in Active mode. According to the
//!			 datasheet the current consumption in this mode is 40uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL NFCInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief   This function puts the NFC IC in Shutdown mode. According to
//!			 the datasheet the current consumption in this mode is 10uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL NFCInterface_powerMode1(void);



#endif /* NFCINTERFACE_H_ */
