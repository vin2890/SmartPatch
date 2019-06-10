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
//! \file   I2CInterface.c
//!
//! \brief  Please see I2CInterface.h
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
#include "eUSCIB0_I2C.h"
#include <stdint.h>

void I2CInterface_Initialize(void)
{
	USCIBO_InitializePorts();
	USCIBO_InitializeRegisters();
}

void I2CInterface_readGenericData(uint8_t ic_address, uint16_t reg_addr, uint8_t* read_data, uint16_t data_length)
{

	USCIBO_readContinuous(ic_address, reg_addr, (uint8_t *)read_data, data_length);

}

uint16_t I2CInterface_readRegisterData(uint8_t ic_address, uint16_t reg_addr, uint8_t bytesInAddr, uint8_t bytesToRead)
{

	return USCIBO_readRegister(ic_address, reg_addr, bytesInAddr, bytesToRead);

}

void I2CInterface_writeGenericData(uint8_t ic_address, uint16_t reg_addr, uint8_t  * write_data, uint16_t data_length)
{

	USCIBO_writeContinuous(ic_address, reg_addr, (uint8_t  *) write_data, data_length);

}

void I2CInterface_writeRegisterData(uint8_t ic_address, uint16_t reg_addr, uint16_t value, uint8_t bytesInAddr, uint8_t bytesInValue)
{

	USCIBO_writeRegister(ic_address, reg_addr, value, bytesInAddr, bytesInValue);

}

bool_t I2CInterface_isI2cBusy(void)
{
//	bool_t retVal = FALSE;
//	if ( (status & (1<<BUSY)) != 0)
//	{
//		retVal = TRUE;
//	}

//	return(retVal);
	return 0;
}

