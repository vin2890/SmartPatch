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
//! \file   TMP112A.h
//!
//! \brief  This is the IC driver module for the TMP112A IC. This IC driver
//!			module contains function calls which are only found in the I2C Interface
//!			module. If the corresponding (I2C) Interface module is modified, then
//!			this IC driver module must also be modified. Although, in this case
//!			the file is set up to interface to the/a I2C peripheral interface
//!			file, it can be modified to interact with a SPI interface module,
//!			UART interface module, USB interface module, etc. The reason why this IC
//!			driver module is set up to interface with I2C peripheral interface
//!			module is because the TMP112A IC uses I2C communication protocol.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#ifndef TMP112A_H_
#define TMP112A_H_

//! \brief This define contains the address of the TMP112A IC.
#define IC_ADDRESS_TMP112_ADDRESS 		0x0048

//! \brief TMP112A temperature register
#define TEMPERATURE_REGISTER		0x00

//! \brief TMP112A configuration register
#define CONFIGURATION_REGISTER		0x01

//! \brief TMP112A temperature low register
#define TLOW_REGISTER				0x02

//! \brief TMP112A temperature high register
#define THIGH_REGISTER				0x03

//! \brief TMP112A SHUTDOWN MODE bit
#define SD	BIT0

//! \brief TMP112A THERMOSTAT MODE bit
#define TM	BIT1

//! \brief TMP112A POLARITY bit
#define POL	BIT2

//! \brief TMP112A FAULT QUEUE bit
#define F0	BIT3

//! \brief TMP112A FAULT QUEUE bit
#define F1	BIT4

//! \brief TMP112A CONVERTER RESOLUTION bit
#define R0	BIT5

//! \brief TMP112A CONVERTER RESOLUTION bit
#define R1	BIT6

//! \brief TMP112A ONE-SHOT/CONVERSION READY bit
#define OS	BIT7

//! \brief TMP112A EXTENDED MODE bit
#define EM	BITC

//! \brief TMP112A ALERT bit
#define AL	BITD

//! \brief TMP112A CONVERSION RATE bit
#define CR0	BITE

//! \brief TMP112A CONVERSION RATE bit
#define CR1	BITF


//*****************************************************************************
//
//! \brief   This function initializes the TMP112A IC registers. The user should
//!			 read the BMA280 datasheet if deep a understanding of the
//!			 initialization procure is desired. Because the functions call is
//!			 remote, it has the possibility to hang. Caution should be taken
//!			 when calling this function.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t TMP112A_remoteRegisterInit(void);

 //*****************************************************************************
 //
 //! \brief   The TMP112A IC contains an Alert interrupt pin. This function call
//!			  configures MSP430 port pin P2.7 as input and which will generate
//!			  an interrupt in a high to low transition of the line. The TMP112A
//!			  Alert pin which is tied to MSP430 port pin P2.7.
 //
 //! \param none     None
 //
 //! \return  \b NO_ERROR, or \b ERROR
 //
 //*****************************************************************************
uint16_t TMP112A_localGPIOInit(void);

//*****************************************************************************
//
//! \brief   This function disables the TMP112A Alert interrupt which is tied to
//!			 MSP430 port pin P2.7.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
void TMP112A_localEnableAlertDetect(void);

//*****************************************************************************
//
//! \brief   This function disables the TMP112A Alert interrupt which is tied
//!			 to MSP430 port pin P2.7.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
void TMP112A_localDissableAlertDetect(void);

//*****************************************************************************
//
//! \brief   This function puts the TMP112A IC in Shutdown mode. According to
//!			 the datasheet the current consumption in this mode is 1uA.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t TMP112A_remoteShutDownMode(void);

//*****************************************************************************
//
//! \brief   This function puts the TMP112A IC in Active mode. According to the
//!			 datasheet the current consumption in this mode is 10uA.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t TMP112A_remoteActiveMode(void);

//*****************************************************************************
//
//! \brief   This function handles everything needed to get a temperature reading
//!			 out of the TMP112A IC. The function must 1st wake up the TMP112A IC
//!			 by putting it in Active mode, then it must read the temperature,
//!			 finally it must put the TMP112A IC back to Shutdown mode.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t TMP112A_remoteReadTemperature(void);

#endif /* TMP112A_H_ */
