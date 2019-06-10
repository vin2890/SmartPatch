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
//! \file   TemperatureInterface.c
//!
//! \brief  Please see TemperatureInterface.h
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
#include <stdint.h>
#include "I2CInterface.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "RF430CL331.h"
#include "TemperatureInterface.h"
#include "TMP112A.h"
#include "Events.h"
#include "Executive.h"
#include "TimerInterface.h"
#include "Timer_A.h"
#include "NFCForumType4TagType.h"
#include "StateMachine.h"
#include "AccelerometerInterface.h"
#include "NFCInterface.h"
#include "DEBUG.h"
#include "UART.h"
#include "RingBuffer.h"

int16_t desiredSamples, acquiredSamples, timeBetweenSamples;
int16_t temperature, temperatureSampleCounter=0, E106PLEN = 0;
_Bool acquireTemperatureDataLogger = false;
_Bool acquireTemperatureStreaming = false;
extern _Bool commandExecutionState;

TEMPERATURE_STREAMING_STATES static currentTemperatureStreamingState = St_Temperature_Streaming_Init;

uint16_t TemperatureInterface_Initialize(void)
{

	/*This function might be enabled in the future.  */
	//TMP112A_remoteRegisterInit();

	/* By disabling the IC, 10uAmps are saved. */
    if(!DEBUG){
	TemperatureInterface_powerMode1();
    }

	/*Initialize the MSP430 GPIO port to interrupt. */
	//TMP112A_localGPIOInit();

	/*Since we are barely starting up, we are not interested in an alert. */
//	TMP112A_localDissableAlertDetect();

	return 0;
}

uint16_t TemperatureInterface_registerInit(void)
{

	return 1;
}

uint16_t TemperatureInterface_powerMode0(void)
{
	TMP112A_remoteActiveMode();
	return 1;
}

uint16_t TemperatureInterface_powerMode1(void)
{
	TMP112A_remoteShutDownMode();
	return 1;
}

uint16_t TemperatureInterface_readTemperature(void)
{

	int16_t  temperature;

	TMP112A_remoteActiveMode();

	temperature = TMP112A_remoteReadTemperature();

	TMP112A_remoteShutDownMode();

	return temperature;
}

uint16_t TemperatureInterface_readTemperatureWriteToFile(void)
{

	TEMPERATURE_DATALOGGER_STATES static currentReadTempState = St_Temperature_Idle;

	/*Reading the temperature from the TMP is broken down into states so as to (1)
	 * decrease power consumption and (2) enable other higher priority interrupts
	 * to interrupt the reading temperature process. The reason why the temperature
	 * is not just done in one step is because it takes time for the TMP112A to
	 * stabilize after it has been woken up. */
	switch(currentReadTempState)
	{
		/* The purpose of this state is to wake up the TMP112A. */
		/* 2.83 ms */
		case (St_Temperature_Idle):

			/* This call does not make sense for the first call to this function however subsequent functions calls need it.        */
			TimerInterface_powerMode1();

			/* Wake up TMP112A temperature sensor from ShutDown mode. */
			TMP112A_remoteActiveMode();

			/* The state to advance to. */
			currentReadTempState = St_Temperature_PowerUp_IC;

			/* Enable timer. This will wake up the MCU when its goes into sleep mode */
			TimerInterface_powerMode0();

			break;

		/* The purpose of this state is to read the temperature from the TMP112A since it is now awoken. */
		/* 1.19 ms */
		case (St_Temperature_PowerUp_IC):

			/* Clear timer */
			TimerInterface_timer1ClearTimer();

			/* Finally read temperature from TMP112A */
			temperature = TMP112A_remoteReadTemperature();

			/* The state to advance to. */
			currentReadTempState = St_Temperature_Read_Temperature;

			/* Enable timer. This will wake up the MCU when its goes into sleep mode */
			TimerInterface_powerMode0();

			break;

		/* The purpose of this state is to once again power down the TMP112A. */
		/* 2.81 ms */
		case (St_Temperature_Read_Temperature):

			/* Place the TMP112A sensor once again in low power mode. */
			TMP112A_remoteShutDownMode();

			/* The state to advance to. */
			currentReadTempState = St_Temperature_PowerDown_IC;

			/* Enable timer. This will wake up the MCU when its goes into sleep mode */
			TimerInterface_powerMode0();

			break;

		/* The purpose of this state is to write the temperature to the temperature file. */
		case (St_Temperature_PowerDown_IC):

			/* This is probably the heart of the TemperatureInterface. Here is where the data is stored in the TextFile. */
			FileTextE106[8+temperatureSampleCounter] = temperature;

			/* Increment the temperature sample counter. Later this variable will be compared with the desired number of samples.  */
			temperatureSampleCounter++;

			/* The temperature Sample Counter variable is constantly updating the Temperature file so if incase the battery drains. High byte */
			FileTextE106[5] = temperatureSampleCounter>>8;

			/* The temperature Sample Counter variable is constantly updating the Temperature file so if incase the battery drains. Low byte */
			FileTextE106[6] = temperatureSampleCounter & 0x00FF;

			currentReadTempState = St_Temperature_Idle;

			/* Determine whether the number of desired samples equals the number of samples acquired. */
			if(temperatureSampleCounter == desiredSamples)
			{

				/* Also change PLEN*/
				E106PLEN = temperatureSampleCounter + 9;

				/* This piece of data is meant for the app. This lets the app know how big the data size is. This is for the high byte. */
				FileTextE106[0] = (E106PLEN >>8);

				/* This piece of data is meant for the app. This lets the app know how big the data size is. This is for the low byte.  */
				FileTextE106[1] = (E106PLEN & 0x00FF);

				/* This piece of data is meant for the app. This lets the app know that the MPBSM system is now once again free to execute a command. */
				FileTextE106[2] = PATCH_NOT_BUSY;

				/* This piece of data is meant for the app. This lets the app that the MPBSM system is busy executing a previous command. */
				FileTextE105[2] = PATCH_NOT_BUSY;

				/* This piece of data is meant for the app. This lets the app that the MPBSM system is busy executing a previous command. */
				FileTextE106[7] = TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_DONE;

				/* This is legacy code. This lets the app know that the end of data has been reached. The app can compare the PLEN against END_DATA_HEADER to see that data is not corrupted.   */
				FileTextE106[8+temperatureSampleCounter++] = END_DATA_HEADER;

				/* This is legacy code. This lets the app know that the end of data has been reached. The app can compare the PLEN against END_DATA_HEADER to see that data is not corrupted.   */
				FileTextE106[8+temperatureSampleCounter++] = END_DATA_HEADER;

				/* This is legacy code. This lets the app know that the end of data has been reached. The app can compare the PLEN against END_DATA_HEADER to see that data is not corrupted.   */
				FileTextE106[8+temperatureSampleCounter++] = '\n';

				/* Reset the temperatureSampleCounter*/
				temperatureSampleCounter = 0;

				acquireTemperatureDataLogger = false;

				/* Place the TMP112A sensor in low power mode. */
				TemperatureInterface_powerMode1();

				/* Not sure why this function must be called but if it is not called then the system does not go into low power mode.  */
				TimerA_ADC12BTriggerSource();

				/* Probably not needed however there is no harm in resetting the timer.  */
				TimerInterface_timer1ClearTimer();

				/*Since we don’t need it and since it consumes power, might as well power off the timer. */
				TimerInterface_powerMode1();

				/* Now re-enable the NFC interface. */
				//NFCInterface_powerMode0();
			}

			else
			{
				/* Enable timer. This will wake up the MCU when its goes into sleep mode */
				TimerInterface_powerMode0();
			}

			break;

	}

	//Additions from Dominique
    char header = 0x10; //TEMP HEADER
    char TEMPHeader = 0x1F;// TEMP LIVE STREAMING
    unsigned char endDataHeader = 0x03;
    ring_buffer_put(_outputrb_d,&header);
    ring_buffer_put(_outputrb_d,&TEMPHeader);
    ring_buffer_put(_outputrb_d,&temperature);
    ring_buffer_put(_outputrb_d,&endDataHeader);
    unsigned char newline = 0x0D;
    ring_buffer_put(_outputrb_d,&newline);
    PUBLISH_EVENT(EV_UART_TX);
    return temperature;
}



void TemperatureInterface_readTemperatureInitialize(void)
{

	/* This informs the app that the patch is in the process of acquiring the temperature */
	FileTextE105[2] = PATCH_BUSY;

	/* This is used for data logger mode. */
	FileTextE105[12] = ACK;

	/* The time between samples variable is read from the command file which is provided by the Android app. */
	timeBetweenSamples = FileTextE105[5];

	/* The time between samples variable is read from the command file which is provided by the Android app. */
	timeBetweenSamples = ((timeBetweenSamples << 8)  | FileTextE105[6] );

	/* The bumber of samples variable is read from the command file which is provided by the Android app. */
	desiredSamples = FileTextE105[7];
	/*  */
	desiredSamples = ((desiredSamples << 8)  | FileTextE105[8] );

	/* This flag is used by the timer. */
	acquireTemperatureDataLogger = true;

	/* Set the Timer1 frequency. */
	TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);

	/* In data logger since the NFC interface is not needed mode while the MPBSM system is logging in the data
	 * values, it makes sense to disable the NFC IC. When the data values are gathered, re-enable the NFC interface.  */
	//NFCInterface_powerMode1();

	/* Start the process to acquire desiredSamples number of temperature samples at timeBetweenSamples specified intervals. */
	PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE);

}


void TemperatureInterface_streamingStateMachinePowerDown(void)
{
	currentTemperatureStreamingState = St_Temperature_Streaming_PowerDown;

	/* Start the process to acquire desiredSamples number of temperature samples at timeBetweenSamples specified intervals. */
	PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_STREAMING);

}

void TemperatureInterface_streamingStateMachine(void)
{

	TEMPERATURE_STREAMING_STATES static currentTemperatureStreamingState = St_Temperature_Streaming_Init;

	switch(currentTemperatureStreamingState)
	{

		case (St_Temperature_Streaming_Init):

			/* Wake up TMP112A temperature sensor from ShutDown mode. */
			TMP112A_remoteActiveMode();

			/* The time between samples variable is read from the command file which is provided by the Android app. */
			timeBetweenSamples = 500;

			/* Set the Timer1 frequency. */
			TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);

			/* Enable timer. This will wake up the MCU when its goes into sleep mode */
			TimerInterface_powerMode0();

			/* This flag is needed by the timer interrupt. */
			acquireTemperatureStreaming = true;

			/* We have finished initializing the streaming state machine. Now it’s time to get the temperature and reset the timer.  */
			currentTemperatureStreamingState = St_Temperature_Streaming_Reset;

			break;

		case (St_Temperature_Streaming_Reset):

			/* Copy the temperature reading to the Temperature file. */

		    temperature=TemperatureInterface_readTemperature();
		    FileTextE106[8] = temperature;
			//char header = 0x10; //TEMP HEADER
			//char TEMPHeader = 0x1F;// TEMP LIVE STREAMING
			////unsigned char endDataHeader = 0x03;
			//ring_buffer_put(_outputrb_d,&header);
			//ring_buffer_put(_outputrb_d,&TEMPHeader);
			//ring_buffer_put(_outputrb_d,&temperature);
			//ring_buffer_put(_outputrb_d,&endDataHeader);
			//unsigned char newline = 0x0D;
			//ring_buffer_put(_outputrb_d,&newline);
			//PUBLISH_EVENT(EV_UART_TX);
			/* Reset timer but don't turn it off */
			TimerInterface_timer1ClearTimer();

			break;

		case (St_Temperature_Streaming_PowerDown):

			/* Since we don't need the timer anymore, disable it. */
			TimerInterface_powerMode1();

			/* To conserve power shutdown the TMP112A sensor */
			TMP112A_remoteShutDownMode();

			/* Since the timer will be disabled, disable the temperature streaming timer flag. */
			acquireTemperatureStreaming = false;

			/* We need to reset the primary state machine. */
			commandExecutionState = St_Command_Execution_Idle;

			/* We have finished initializing the streaming state machine. Now it’s time to get the temperature and reset the timer.  */
			currentTemperatureStreamingState = St_Temperature_Streaming_Init;

			break;

	}

}

