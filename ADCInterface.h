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
//! \file   ADCInterface.h
//!
//! \brief  This is the ADC peripheral interface module for the ADC12_B
//! 		peripheral driver module. This interface module provides functions common
//! 		to all ADC peripherals. Most people familiar with ADC peripherals should
//! 		be familiar with the functions found in this module however will be not
//! 		so familiar with the functions found in the ADC12_B driver module. Call
//! 		the functions found on the ADC12_B peripheral driver module in order to
//! 		initialize the ADC12_B peripheral module, power up ADC12_B peripheral
//! 		module, power down ADC12_B peripheral module, get ADC reading are some
//! 		of the functions that should be found on this peripheral interface
//! 		module.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ADCINTERFACE_H_
#define ADCINTERFACE_H_

//*****************************************************************************
//
//! \brief   This function is called only once and it is called when the MSP430
//!			 is initializing. All the function calls called by this function are
//!			 found only the ADC12_B peripheral driver module. Furthermore, all
//!			 the function calls are local hence there is no chance of program
//!			 hanging. First function configures the port pin P1.2 for peripheral
//!			 (specifically ADC12_B peripheral) use instead of GPIO use. Next the
//!			 function calls a ADC12_B peripheral driver function which configures
//!			 the ADC12_B registers. Finally to conserve power the function powers
//!			 down the ADC12_B peripheral.
//
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void ADCInterface_Init(void);

//*****************************************************************************
//
//! \brief   This function is called whenever it is desired to activate the ADC12_B
//!			 peripheral. All the function calls called by this function are found
//!			 only the ADC12_B peripheral driver module. Furthermore, all the
//!			 function calls are local hence there is no chance of program hanging.
//!			 To activate the ADC12_B actually requires three separation instructions.
//!			 Some bits (like the ADC12ON) in the ADC12_B configuration registers
//!			 require the ADC12ENC to be cleared before they can be modified. Once
//!			 the ADC12ON is set, ADC12ENC is once again set.
//
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void ADCInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief   This function is called whenever it is desired to power down the ADC12_B
//!			 peripheral. All the function calls called by this function are found
//!			 only the ADC12_B peripheral driver module. Furthermore, all the function
//!			 calls are local hence there is no chance of program hanging. To power
//!			 down the ADC12_B actually requires three separation instructions. Some
//!			 bits (like the ADC12ON) in the ADC12_B configuration registers require
//!			 the ADC12ENC to be cleared before they can be modified. Once the ADC12ON
//!			 is cleared, ADC12ENC is once again set.
//
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void ADCInterface_powerMode1(void);

#endif /* ADC_H_ */
