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
//! \file   NFCInterface.c
//!
//! \brief  Please see NFCInterface.h
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
#include "NFCInterface.h"
#include "RF430CL331.h"
#include "NFCForumType4TagType.h"
#include "I2CInterface.h"
#include "types.h"

void NFCInterface_Init(void)
{
	//This function call initializes the GPIO port 4 as an input port with a
	//low to high trasition interrupt. This is needed by the RF430CL331 INT pin.
	RF430CL331_INT0Init();

	//This function call sets up the MSP430 port pin P9.3 as an output. This
	//port pin is tied to the RF430CL331 reset pin
	RF430CL331_ResetInit();

	//This function call sets up the MSP430 port pin P4.0 as an input. This port
	//pin is tied to the RF430CL331 ready pin
	RF430CL331_I2CReadyInit();

	//This function call actually reset the RF430CL331 IC. Later change will
	//include a state machine to remove the delay_cycles.
	RF430CL331_reset();

	//This function call calls the RF430CL331_RegisterInit IC driver file to
	//configure the RF430CL331 registers. The RF430CL331 datasheet should be
	//read in order to understand the function calls found in the
	//RF430CL331_RegisterInit IC driver file
	RF430CL331_RegisterInit();

}

BOOL NFCInterface_powerMode0(void)
{
	if(RF430CL331_remoteStandbyDisabled() == false)
		return false;

	if(RF430CL331_remoteRFEnabled() == false)
		return false;

	return true;
}

BOOL NFCInterface_powerMode1(void)
{
	if(RF430CL331_remoteStandbyEnabled() == false)
		return true;

	if(RF430CL331_remoteRFDisabled() == false)
		return false;

	return true;
}





