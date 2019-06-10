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
//! \file   AccelerometerInterface.h
//!
//! \brief  This is the IC interface file for accelerometer ICs. This
//!			IC interface file contains function calls which are only found in the
//!			BMA280 IC driver file. If its corresponding BMA280 IC driver files
//!			changes, then this IC interface file should also change as well. This
//!			IC interface file has generic function calls which should be found in
//!			similar accelerometer IC’s. For example power up IC, power down IC,
//!			get x acceleration, get y acceleration, get z acceleration.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ACCELEROMETERINTERFACE_H_
#define ACCELEROMETERINTERFACE_H_

//*****************************************************************************
// the includes
//*****************************************************************************
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "types.h"


//! \brief This enum mentions the states that the BMA280 can be in when a
//! 	acceleration reading is taking place.
typedef struct _acc_data
{
	signed int x_raw;
	signed int y_raw;
	signed int z_raw;
	signed int x_offset;
	signed int y_offset;
	signed int z_offset;
	signed int x;
	signed int y;
	signed int z;
	unsigned int ready;
}acc_data_t;


#define ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_COMMAND				0x8C
#define ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_RESPONSE_COMMAND	0x8D
#define ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_DONE				0x8E
#define ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_NOT_DONE			0x8F


//*****************************************************************************
//
//! \brief	This function is called only once and it is called when the MSP430 is
//!			initializing. This function does both local and remote configuration.
//!			The BMA280 is equipped with two programmable interrupt I/0 interrupt
//!			pins and these pins were tied to the MSP320 P1.1 and P2.2 pins. This
//!			function configures the MSP430 P1.1 and P2.2 to be interruptible.
//!			Because when this function is called the MPBSM is not cadence mode,
//!			both MSP430 interrupts are disabled at least for now. Furthermore,
//!			this function makes remote calls to the BMA280 to reset it, configure
//!			its registers, and finally put in in low power mode.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t AccelerometerInterface_Initialize(void);


//*****************************************************************************
//
//! \brief	This function contains a state machine and is constantly executed
//!			when the MPBSM is in Cadence Data log mode. The state machine enters
//!			states  St_Pedometer_Initialize_Pedometer_Algorithm and
//!			St_Pedometer_PowerDown_IC only once when entering and leaving Cadence
//!			Data log mode. On the contrary the state machine stays in the
//!			St_Pedometer_Data_Ready for the majority of time when the MPBSM is
//!			in Cadence Data log mode.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
void AccelerometerInterface_dataLoggerMode(void);

//*****************************************************************************
//
//! \brief	This function gets executed when the MPBSM system is in Cadence Data
//!			log mode. This function gets executed on every timer overflow interrupt
//!			occurs. How many times this function gets called per second is dependent
//!			on the value in “Time between samples” is. Every time this function
//!			gets called, the E108 file increments in size by 1. The function
//!			basically does two things; records the current step count (which is
//!			calculated by the cadence algorithm) into the cadence file which is
//!			E108 and also updates the sample number in the E108 file.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
void AccelerometerInterface_updateStepcountAndSampleNumber(void);

//*****************************************************************************
//
//! \brief	This function if the first function to get called when the MPBSM system
//!			is getting ready to be set to Cadence Data log mode. The function updates
//!			the command file PATCH_BUSY and ACK parameters to inform the MPBSM app
//!			that it is busy and any re-scan is not allowed, reads the “time between
//!			samples” and “desired acceleration samples” from the command file which
//!			were provided by the Android app, and finally configures the Timer based
//!			on the read “time between samples” and “desired acceleration samples”
//!			variables.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
void AccelerometerInterface_setDataLoggerModeInitialize(void);


//*****************************************************************************
//
//! \brief   The BMA280 IC contains two programmable I/O interrupt pins. In this
//!			 case Port2 pin 0 is configured as the data ready pin. Whenever the
//!			 BMA280 has new accelerometer data for the MSP430, it interrupts the
//!			 MSP430 by pulling the line low. By calling this function, an interrupt
//!			 will not occur if the line is pulled low.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_disableDataReadyInterrupt(void);

//*****************************************************************************
//
//! \brief   The BMA280 IC contains two programmable I/O interrupt pins. In this
//!			 case Port2 pin 0 is configured as the data ready pin. Whenever the
//!			 BMA280 has new accelerometer data for the MSP430, it interrupts the
//!			 MSP430 by pulling the line low. By calling this function, an interrupt
//!			 will occur if the line is pulled low.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_enableDataReadyInterrupt(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in normal mode. According to the
//!			 datasheet the current consumption in this mode is 130uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in Low-power Mode 2. According to the
//!			 datasheet the current consumption in this mode is 66uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode1(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in Standby mode. According to the
//!			 datasheet the current consumption in this mode is 130uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode2(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in Low-power Mode 1. According to
//!			 the datasheet the current consumption in this mode is 6.5uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode3(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in Suspend mode. According to the
//!			 datasheet the current consumption in this mode is 2.1uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode4(void);

//*****************************************************************************
//
//! \brief   This function puts the BMA280 in Deep Suspend mode. According to
//!			 the datasheet the current consumption in this mode is 1uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void AccelerometerInterface_powerMode5(void);

//*****************************************************************************
//
//! \brief   This function reads the X acceleration value from the BMA280.In reality
//!			 all the heavy lifting is being done by the IC driver module.
//
//! \param none     none
//
//! \return  The x acceleration data
//
//*****************************************************************************
uint16_t AccelerometerInterface_ReadXAcceleration(void);


//*****************************************************************************
//
//! \brief   This function reads the Y acceleration value from the BMA280. In reality
//!			 all the heavy lifting is being done by the IC driver module.
//
//! \param none     none
//
//! \return  The y acceleration data
//
//*****************************************************************************
uint16_t AccelerometerInterface_readYAcceleration(void);

//*****************************************************************************
//
//! \brief   This function reads the Z acceleration value from the BMA280. In reality
//!			 all the heavy lifting is being done by the IC driver module.
//
//! \param none     none
//
//! \return  The z acceleration data
//
//*****************************************************************************
uint16_t AccelerometerInterface_readZAcceleration(void);

//*****************************************************************************
//
//! \brief   This function reads the XYZ acceleration value from the BMA280. In reality
//!			 all the heavy lifting is being done by the IC driver module.
//
//! \param none     none
//
//! \return  The XYZ acceleration data
//
//*****************************************************************************
uint16_t AccelerometerInterface_readxyzAcceleration(void);

#endif /* ACCELEROMETER_H_ */
