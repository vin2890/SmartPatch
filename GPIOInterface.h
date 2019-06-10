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
//! \file   GPIOInterface.h
//!
//! \brief  This module does three things. First, it configures all the MPS320
//!			GPIOs to input. Second, it configures the DCO clock speed and finally
//!			it configures the clock speed of ACLK, MCLK, and SMCLK clcock.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef GPIO__H_
#define GPIO__H_

//*****************************************************************************
//
//! \brief   This module is responsible for initializing the MSP430 GPIO. This
//!			 functions drives all the outputs low which may not be optimal for a
//!			 different board.
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void GPIOInterface_initGPIO(void);

/***********************************************************
	Function Name: GPIOInterface_initClocks
	Function Description: This module is responsible for initializing
	the MSP clocks. Set DCO Frequency to 8MHz and configure MCLK,
	SMCLK and ACLK to be source by DCOCLK

	Inputs:  none
	Outputs: none
***********************************************************/
//*****************************************************************************
//
//! \brief   This module is responsible for initializing the MSP clocks. Set DCO
//!			 Frequency to 8MHz and configure MCLK, SMCLK and ACLK to be source by
//!			 DCOCLK
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void GPIOInterface_initClocks(void);


#endif /* GPIO__H_ */
