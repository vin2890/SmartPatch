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
//! \file   eUSCIA1_SPI.h
//!
//! \brief  This is the eUSCIA1 peripheral driver module. All of the functions
//!			found in this peripheral driver module are specific to the eUSCIA1 peripheral.
//!			if a peripheral module other than eUSCIA1 is used, then this module must be
//!			replaced. Configure the eUSCIA1 to act as a SPI peripheral, configure the
//!			peripheral clock source, peripheral frequency, initialize peripheral ports
//!			and interrupts, store the returned data and transfer data are the functions
//!			that are found on this peripheral driver module.
//
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#ifndef USCIA_SPI_H_
#define USCIA_SPI_H_

//! \brief This enum list all the possible ICs in the ADS1x9x family.
//!
enum ADS129x
{
		ADS1191_16BIT	 = 0,
		ADS1192_16BIT,
		ADS1291_24BIT,
		ADS1292_24BIT
};

//! \brief This enum list the port bits needed by the eUSCIA1 peripheral.
//!
enum PORT3_ADC_CONTROL
{
		ADC_GPIO1 = 1,
		Unconnected = 2,//unknown
		ADC_DRDY = 4,
		ADC_GPIO2 = 8, //0k //
		SPI_SIMO = 16,//ok //
		SPI_SOMI = 32,//
		SPI_CLK = 64, //ok
		SPI_CS = 128 //ok
};

//! \brief This enum list the port bits needed by the eUSCIA1 peripheral.
//!
enum PORT2_ADC_CONTROL
{
		Unconnected0 = 1,   //bit 0
		ADC_CLK_SEL = 2,    //bit 1
		ADC_RESET = 4,		//bit 2
		ADC_START = 8,		//bit 3
		ALT_ADC_CLK_SEL = 16,	//bit 4
		Unconnected2 = 32,	//bit 5
		Unconnected3 = 64,	//bit 6
		Unconnected4 = 128	//bit 7


};

//*****************************************************************************
//
//! \brief   This peripheral driver function initializes the eUSCIA1 peripheral.
//!			 This function configures the eUSCIA1 to act as an SPI peripheral,
//!			 configures the peripheral clock source, peripheral frequency, and
//!			 initialize peripheral ports.
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void USCIA1_InitializeRegisters(void);

//*****************************************************************************
//
//! \brief   This peripheral driver function initializes the eUSCIA1 peripheral.
//!			 This function configures the eUSCIA1 to act as an SPI peripheral,
//!			 configures the peripheral clock source, peripheral frequency, and
//!			 initialize peripheral ports.
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void USCIA1_InitializePorts(void);

#endif /* USCIA_SPI_H_ */
