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
//! \file   ADC12_B.h
//!
//! \brief  This is the ADC_B peripheral driver module. All of the functions
//!			found in this peripheral driver module are specific to the ADC_B
//!			peripheral. Furthermore, all the functions found on this module
//!			configure only local MSP430 registers hence removing the need to
//!			enable the watchdog timer whenever executing them. If a peripheral
//!			module other than ADC_B is used, then this module must be replaced.
//!			Configuration of peripheral clock source, peripheral frequency,
//!			disabling peripheral, enabling peripheral, enabling peripheral
//!			interrupt, disabling peripheral interrupts, reading pin ADC are
//!			just a few of the functions that are found on this peripheral
//!			driver module. This module was lightly commented as the majority
//!			is explained in great detail using the ADC12_B datasheet. Module
//!			comments are by no means peripheral datasheet supplement.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ADC_H_
#define ADC_H_

//*****************************************************************************
// typedefs
//*****************************************************************************
#include <stdint.h>
#include <msp430.h>
#include "types.h"

//*****************************************************************************
//defines
//*****************************************************************************

//! \brief This define makes only requires one change to this file if the ADC
//!  	port is every changed.
#define PORT_ADC_SEL	P1SEL0

//! \brief This define makes only requires one change to this file if the ADC
//! 	 port is every changed.
#define PORT_ADC_DIR	P1DIR

//! \brief This define makes only requires one change to this file if the ADC
//!  	port is every changed.
#define A2 BIT2

extern int16_t ADCValues;

//*****************************************************************************
//
//! \brief	This function is called only once and it is called when the MSP430
//!			is initializing. Furthermore, the function is called indirectly by
//!			the ADC peripheral interface module when it itself is initializing.
//!			This function configures the registers of the ADC12_B peripheral.
//!			The ADC12_B peripheral is set to be source from the ACLK, set to
//!			12-bit resolution and interrupt driven with Timer_A overflow interrupt
//!			as the trigger source.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12B_init(void);

//*****************************************************************************
//
//! \brief	This function is called only once and it is called when the MSP430
//!			is initializing. Normally, the function gets called almost immediately
//!			after ADC12B_init gets called. The instructions executed in this
//!			function applies specifically to the MSP430. If another MCU is used,
//!			this this function must be completely modified. The function config-
//!			ures the port pins P1.2 for peripheral (specifically ADC12_B
//!			peripheral) use instead of GPIO use. Port pin P1.2 is set for A2.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12_PortsInit(void);

//*****************************************************************************
//
//! \brief  This function checks to see if the ADC12_B peripheral is busy
//!			executing a previous command. To check to see if the ADC12_B is
//!			busy  the ADC12BUSY bit in the ADC12_B Control 1 Register is checked.
//!			If the bit is set to 0, then no operation is active, otherwise if
//!			the bit is set A sequence, sample, or conversion is active.
//
//! \param	None
//
//! \return  \b TRUE, or \b FALSE
//
//*****************************************************************************
BOOL ADC12_busy(void);

//*****************************************************************************
//
//! \brief  This function enables the ADC12_B core. To configure the ADC12ON bit,
//!			first the ADC12ENC bit must be set. After setting the ADC12ENC, then
//!			set the ADC12ON.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12_enableADC_B(void);

//*****************************************************************************
//
//! \brief  This function disables the ADC12_B core. To configure the ADC12ON bit,
//!			first the ADC12ENC bit must be set. After setting the ADC12ENC, then
//!			set the ADC12ON.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12_disableADC_B(void);

//*****************************************************************************
//
//! \brief	This function enables ADC12_B conversion. ADC12ENC must be set to 1
//!			before any ADC12_B conversion can take place.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12_enableConversion(void);

//*****************************************************************************
//
//! \brief	Disable ADC12_B conversion. With few exceptions, an application can
//!			modify the ADC12_B control bits only when ADC12ENC = 0. ADC12SHT1x,
//!			ADC12SHT0x, ADC12MSC, ADC12ON, ADC12PDIV, ADC12SHSx, ADC12SHP,
//!			ADC12ISSH, ADC12DIVx, ADC12SSELx, ADC12ICH3MA, ADC12ICH2MA, ADC12ICH1MA,
//!			ADC12ICH0MA, ADC12TCMAP, ADC12BATMA, ADC12CSTARTADDx, ADC12WINC,
//!			ADC12DIF, ADC12VRSEL, ADC12EOS and ADC12INCHx are ADC12_B control bits
//!			and can only be modified when ADC12ENC is set to zero.
//
//! \param	None
//
//! \return	none
//
//*****************************************************************************
void ADC12_disableConversion(void);

//*****************************************************************************
//
//! \brief	ADC12SHSx bits in the “ADC12_B Control 1 Register”, configure the
//!			ADC12_B trigger source. If the ADC12SHSx bit in the “ADC12_B Control
//!			1 Register” are set to 000b, then this function starts a ADC
//!			conversion by setting ADC12SC bit to one in the “ADC12_B Control 0
//!			Register”.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void ADC12_SoftwareStartConversion(void);

//*****************************************************************************
//
//! \brief	The ADC12SHS bits define the ADC12_B trigger source. If ADC12SHS is
//!			set to 000 then the trigger source is Software (ADC12SC), else if
//!			set to 001, then the trigger source is Timer_A TA0 CCR1 output, else
//!			if set to 010, then the trigger source is Timer_B TB0 CCR0 output,
//!			else if set to 011 then the trigger source is Timer_B TB0 CCR1 output,
//!			else if set to 100 then the trigger source is Timer_A TA1 CCR1 output,
//!			else if set to 101 then the trigger source is Timer_A TA2 CCR1 output,
//!			else if set to 110 then the trigger source is Timer_A TA3 CCR1 output,
//!			else if set to 111 then the trigger source is Reserved (DVSS). This
//!			function sets the ADC12_B trigger source to be the ADC12SC bit.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void ADC12_SHISourceSC(void);

//*****************************************************************************
//
//! \brief	The ADC12SHS bits define the ADC12_B trigger source. If ADC12SHS is
//!			set to 000 then the trigger source is Software (ADC12SC), else if
//!			set to 001, then the trigger source is Timer_A TA0 CCR1 output, else
//!			if set to 010, then the trigger source is Timer_B TB0 CCR0 output,
//!			else if set to 011 then the trigger source is Timer_B TB0 CCR1 output,
//!			else if set to 100 then the trigger source is Timer_A TA1 CCR1 output,
//!			else if set to 101 then the trigger source is Timer_A TA2 CCR1 output,
//!			else if set to 110 then the trigger source is Timer_A TA3 CCR1 output,
//!			else if set to 111 then the trigger source is Reserved (DVSS). This
//!			function sets the ADC12_B trigger source to be the Timer_A TA0 CCR1
//!			output.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void ADC12_SHISourcenCCR1(void);

//*****************************************************************************
//
//! \brief  This function sets the ADC12 to low power mode. The ADC12PWRMD bit
//!			optimizes the ADC12_B power consumption at two ADC12CLK ranges.
//!			Select the lowest ADC12CLK frequency that meets or exceeds the
//!			application requirements. If ADC12CLK is 1/4 or less of datasheet
//!			specified maximum for ADC12PWRMD=0, ADC12PWRMD=1 may be set to save
//!			power.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void ADC12_lowPowerModeEnable(void);

//*****************************************************************************
//
//! \brief  This function sets the ADC12 to low power mode. The ADC12PWRMD bit
//!			optimizes the ADC12_B power consumption at two ADC12CLK ranges.
//!			Select the lowest ADC12CLK frequency that meets or exceeds the
//!			application requirements. If ADC12CLK is 1/4 or less of datasheet
//!			specified maximum for ADC12PWRMD=0, ADC12PWRMD=1 may be set to save
//!			power.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void ADC12_lowPowerModeDisable(void);


#endif /* ADC_H_ */
