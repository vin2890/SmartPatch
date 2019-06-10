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
//! \file   GSRInterface.h
//!
//! \brief  This module configures the MSP430 for GSR streaming mode or GSR data
//!			log mode. This module does everything needed to enter the particular
//!			mode (such as enable Timers and/or Enabled the ADC), stay in the
//!			mode(Reset timer, read GSR) and finally exit the mode(once again
//!			disable Timer and/or ADC). The file writes its results to E107 file.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#ifndef GSRINTERFACE_H_
#define GSRINTERFACE_H_

//! \brief This enum mentions the states that the BMA280 can be in when a
//! 	temperature reading is taking place.
typedef enum {
	St_GSR_Streaming_Init,
	St_GSR_Streaming_Reset,
	St_GSR_Streaming_PowerDown,
} GSR_STREAMING_STATES;

#define GSR_INTERFACE_ACQUIRE_GSR_STREAMING_COMMAND	0xA8
#define GSR_INTERFACE_ACQUIRE_GSR_COMMAND			0x88
#define GSR_INTERFACE_ACQUIRE_GSR_RESPONSE_COMMAND	0x89
#define GSR_INTERFACE_ACQUIRE_GSR_DONE				0x8A
#define GSR_INTERFACE_ACQUIRE_GSR_NOT_DONE			0x8B

//*****************************************************************************
//
//! \brief   This function initializes the TMP112A IC by calling functions
//!			 found in the TMP112A IC driver file.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
uint16_t GSRInterface_Initialize(void);

//*****************************************************************************
//
//! \brief   This function initialized the TMP112A registers. Because the functions
//!			 call is remote, it has the possibility to hang. Caution should be
//!			 used when calling this function.
//
//! \param none     none
//
//! \return  \b NO_ERROR_INITIALIZING, or \b ERROR_INITIALIZING
//
//*****************************************************************************
uint16_t GSRInterface_registerInit(void);

//*****************************************************************************
//
//! \brief   This function puts the TMP112A IC in Active mode. According to the
//!			 datasheet the current consumption in this mode is 10uA.
//
//! \param none     none
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t GSRInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief   This function puts the TMP112A IC in Shutdown mode. According to
//!			 the datasheet the current consumption in this mode is 1uA.
//
//! \param none     none
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t GSRInterface_powerMode1(void);

//*****************************************************************************
//
//! \brief   This function handles everything needed to get a temperature reading
//!			 out of the TMP112A IC. The function must 1st wake up the TMP112A IC
//!			 by putting it in Active mode, then it must read the temperature,
//!			 finally it must put the TMP112A IC back to Shutdown mode.
//
//! \param none     none
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t GSRInterface_readGSR(void);

//*****************************************************************************
//
//! \brief   This function is similar to the TemperatureInterface_readTemperature
//!			 except it runs in the background.
//
//! \param none     none
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t GSRInterface_readGSRWriteToFile(void);

//*****************************************************************************
//
//! \brief   This initializes the read procedure.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void GSRInterface_readGSRInitialize(void);

//*****************************************************************************
//
//! \brief   This function contains the GSR streaming state machine. Whenever
//!			 GSR streaming is desired, this function is being used.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void GSRInterface_streamingStateMachine(void);

//*****************************************************************************
//
//! \brief   Every time a read operation occurs, the “GSR Stream counter” is
//!			 reset.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void GSRInterface_streamingCounterReset(void);

////*****************************************************************************
////
////! \brief   This initializes GSR streaming.
////
////! \param none     none
////
////! \return  none
////
////*****************************************************************************
//void GSRInterface_streamingGSRInitialize(void);
//
////*****************************************************************************
////
////! \brief   This function is tied to the timer. Every time the timer overflows,
////!			 the counter increments. Every time a read from the phone occurs, the
////!			 counter is reset. This guarantees streaming.
////
////! \param none     none
////
////! \return  none
////
////*****************************************************************************
//void GSRInterface_streamingCounterIncrement(void);


#endif /* TEMPERATURE_H_ */
