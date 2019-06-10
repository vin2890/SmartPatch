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
//! \file   TemperatureInterface.h
//!
//! \brief  This is the IC interface file for temperature ICs. This IC
//! 		interface file contains function calls which are only found in the
//!			TMP112A IC driver file. If its corresponding TMP112A IC driver
//!			files changes, then this IC interface file should also change as
//!			well. This IC interface file has generic function calls which
//!			should be found in similar temperature IC’s. For example power up
//!			IC, power down IC, get temperature in Fahrenheit, get temperature
//!			in Celsius, etc. This module is responsible for executing the
//!			Temperature streaming and temperature data logging command. All the
//!			results are written to the E106 NFC file. The functions inside the
//!			module configure, power up, power down the temperature sensor.
//!			Furthermore, this module utilizes a MSP430 timer which it enables
//!			and disables when needed.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

//! \brief This enum mentions the states that the TMP112A can be in when a
//! 	temperature reading is taking place.
typedef enum {
	St_Temperature_Idle,
	St_Temperature_PowerUp_IC,
	St_Temperature_Read_Temperature,
	St_Temperature_PowerDown_IC,
} TEMPERATURE_DATALOGGER_STATES;

//! \brief This enum mentions the states that the TMP112A can be in when a
//! 	temperature streaming reading is taking place.
typedef enum {
	St_Temperature_Streaming_Init,
	St_Temperature_Streaming_Reset,
	St_Temperature_Streaming_PowerDown,
} TEMPERATURE_STREAMING_STATES;

#define TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_STREAMING_COMMAND	0x74
#define TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_COMMAND			0x84
#define TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_RESPONSE_COMMAND	0x85
#define TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_DONE				0x86
#define TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_NOT_DONE			0x87

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
uint16_t TemperatureInterface_Initialize(void);

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
uint16_t TemperatureInterface_registerInit(void);

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
uint16_t TemperatureInterface_powerMode0(void);

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
uint16_t TemperatureInterface_powerMode1(void);

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
uint16_t TemperatureInterface_readTemperature(void);

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
uint16_t TemperatureInterface_readTemperatureWriteToFile(void);

//*****************************************************************************
//
//! \brief   This function sets up the MPBSM system to streaming mode.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void TemperatureInterface_readTemperatureInitialize(void);

//*****************************************************************************
//
//! \brief   This function gets called when this function gets called, that means
//!	         stop steaming mode. The handset should be reading the temperature
//!			 file every second.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void TemperatureInterface_streamingStateMachinePowerDown(void);

//*****************************************************************************
//
//! \brief   As the goes MPBSM into streaming mode, it traverses this state machine.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void TemperatureInterface_streamingStateMachine(void);





#endif /* TEMPERATURE_H_ */
