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
//! \file   GSRInterface.c
//!
//! \brief  Please see GSRInterface.h
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
//#include "RF430CL331.h"
#include "GSRInterface.h"
#include "TMP112A.h"
#include "Events.h"
#include "Executive.h"
#include "TimerInterface.h"
#include "Timer_A.h"
#include "NFCForumType4TagType.h"
#include "StateMachine.h"
#include "ADCInterface.h"
#include "NFCInterface.h"
#include "UART.h"
#include "RingBuffer.h"

int16_t desiredSamples, acquiredSamples, timeBetweenSamples;
int16_t GSR;
uint16_t GSRSampleCounter = 0, GSRFileIndex = 8, GSRCounter = 0;
_Bool acquireGSRDataLogger = false;
_Bool acquireGSRStreaming = false;
extern int16_t ADCValues;
extern _Bool commandExecutionState;

GSR_STREAMING_STATES static currentGSRStreamingState = St_GSR_Streaming_Init;

uint16_t GSRInterface_Initialize(void)
{
	GSRInterface_powerMode0();

	return 0;
}

uint16_t GSRInterface_registerInit(void)
{
	return 1;
}

uint16_t GSRInterface_powerMode0(void)
{
	ADCInterface_powerMode0();
	return 1;
}

uint16_t GSRInterface_powerMode1(void)
{
	ADCInterface_powerMode1();
	return 1;
}

uint16_t GSRInterface_readGSR(void)
{
    return 0; //Warning thrown, added stub
}

uint16_t GSRInterface_readGSRWriteToFile(void)
{

	FileTextE107[GSRFileIndex++] = ADCValues >> 8;
	FileTextE107[GSRFileIndex++] = ADCValues & 0x00FF;

	FileTextE107[5] = GSRSampleCounter>>8;
	FileTextE107[6] = GSRSampleCounter & 0x00FF;
	//GSRSampleCounter++;

	/* Determine whether the number of desired samples equals the number of samples acquired. */
	if(GSRSampleCounter++ == desiredSamples)
	{

		/* Also change PLEN*/
		int16_t E107PLEN = 0;

		/*Since we don’t need it and since it consumes power, might as well power off the timer. */
		TimerInterface_powerMode1();

		ADCInterface_powerMode1();

		FileTextE107[2] = PATCH_NOT_BUSY;
		FileTextE105[2] = PATCH_NOT_BUSY;

		FileTextE107[7] = GSR_INTERFACE_ACQUIRE_GSR_DONE;

		FileTextE107[8+GSRFileIndex++] = END_DATA_HEADER;
		FileTextE107[8+GSRFileIndex++] = END_DATA_HEADER;
		FileTextE107[8+GSRFileIndex++] = '\n';

		/* Calculate the size of the GSR data logger data. This is used by the app. */
		E107PLEN = GSRFileIndex - 2;

		/* This lets the app know how many bytes the file size is and hence how many bytes to read. High byte */
		FileTextE107[0] = (E107PLEN >>8);

		/* This lets the app know how many bytes the file size is and hence how many bytes to read. Low byte */
		FileTextE107[1] = (E107PLEN & 0x00FF);

		/* Reset the temperatureSampleCounter*/
		GSRSampleCounter = 0;

		GSRFileIndex = 8;

		acquireGSRDataLogger = false;

		/* Now re-enable the NFC interface. */
		NFCInterface_powerMode0();
		//P3OUT ^= 1 << 0;
	}

	else
	{
		/* Enable timer. This will wake up the MCU when its goes into sleep mode */
		TimerInterface_powerMode0();
	}
    char header = 0x40; //GSR HEADER
    char GSRHeader = 0x4F;// GSR STREAMING
    unsigned char endDataHeader = 0x03;
    ring_buffer_put(_outputrb_d,&header);
    ring_buffer_put(_outputrb_d,&GSRHeader);
    ring_buffer_put(_outputrb_d,&GSR);
    ring_buffer_put(_outputrb_d,&endDataHeader);
    unsigned char newline = 0x0D;
    ring_buffer_put(_outputrb_d,&newline);
    PUBLISH_EVENT(EV_UART_TX);
	return GSR;
}



void GSRInterface_readGSRInitialize(void)
{

	/* In data logger since the NFC interface is not needed mode while the MPBSM system is logging in the data
	 * values, it makes sense to disable the NFC IC. When the data values are gathered, re-enable the NFC interface.  */
	//NFCInterface_powerMode1();

	/* This informs the app that the patch is in the process of acquiring the temperature */
	FileTextE105[2] = PATCH_BUSY;

	/* This informs the app that the patch is in the process of acquiring the temperature */
	FileTextE105[4] = GSR_INTERFACE_ACQUIRE_GSR_COMMAND;

	/* This is used in data logger mode.  */
	FileTextE105[12] = ACK;

	/* The time between samples variable is read from the command file which is provided by the Android app. */
	timeBetweenSamples = FileTextE105[5];

	/* TimeBetweenSamples is a 16 bit number which is composed of FileTextE105[5] and FileTextE105[6] */
	timeBetweenSamples = ((timeBetweenSamples << 8)  | FileTextE105[6] );

	/* The bumber of samples variable is read from the command file which is provided by the Android app. */
	desiredSamples = FileTextE105[7];

	/* DesiredSamples is a 16 bit number which is composed of FileTextE105[8] and FileTextE105[9] */
	desiredSamples = ((desiredSamples << 8)  | FileTextE105[8] );

	/* Set the patch to GSR datalogger mode. This flag is needed by the timer interrupt routine. */
	acquireGSRDataLogger = true;

	/* Enable the ADC for GSR conversion */
	ADCInterface_powerMode0();

	/* Set the Timer1 frequency. */
	TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);

	/* Enable timer. This will wake up the MCU when its goes into sleep mode */
	TimerInterface_powerMode0();

	PUBLISH_EVENT(EV_ACQUIRE_GSR_DATA_LOGGER);

}

void GSRInterface_streamingStateMachine(void)
{

	switch(currentGSRStreamingState)
	{

		case (St_GSR_Streaming_Init):

			/* This informs the app that the patch is in the process of acquiring the temperature */
			FileTextE105[4] = GSR_INTERFACE_ACQUIRE_GSR_STREAMING_COMMAND;

			/* TimeBetweenSamples is a 16 bit number which is composed of FileTextE105[5] and FileTextE105[6] */
			//timeBetweenSamples = 300;
			timeBetweenSamples = 500;

			/* Set the patch to GSR datalogger mode. This flag is needed by the timer interrupt routine. */
			acquireGSRStreaming = true;

			/* Enable the ADC for GSR conversion */
			ADCInterface_powerMode0();

			/* Set the Timer1 frequency. */
			TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);

			/* Enable timer. This will wake up the MCU when its goes into sleep mode */
			TimerInterface_powerMode0();

			currentGSRStreamingState = St_GSR_Streaming_Reset;

			break;

		case (St_GSR_Streaming_Reset):

			/* Reactivate the timer. */
			TimerInterface_powerMode0();

			/* Enabel the GSR counter.*/
			GSRCounter++;

			/*  */
			if(GSRCounter == 2)currentGSRStreamingState = St_GSR_Streaming_PowerDown;

			break;

		case (St_GSR_Streaming_PowerDown):

			/* Reset the GSR counter */
			GSRCounter = 0;

			/* Disable the timer since we don't need it. */
			TimerInterface_powerMode1();

			/* Since we don't need it and to conserve power, power down the ADC. */
			ADCInterface_powerMode1();

			/* We need to reset the primary state machine. */
			commandExecutionState = St_Command_Execution_Idle;

			/* GSR streaming has been disable hence disable this flag. */
			acquireGSRStreaming = false;

			/* Reset the state machine for the next streaming command. */
			currentGSRStreamingState = St_GSR_Streaming_Init;

			break;
	}
	char header = 0x40; //GSR HEADER
    char GSRHeader = 0x4F;// GSR STREAMING
    unsigned char endDataHeader = 0x03;
    ring_buffer_put(_outputrb_d,&header);
    ring_buffer_put(_outputrb_d,&GSRHeader);
    ring_buffer_put(_outputrb_d,&ADCValues);
    ring_buffer_put(_outputrb_d,&endDataHeader);
    unsigned char newline = 0x0D;
    ring_buffer_put(_outputrb_d,&newline);
    PUBLISH_EVENT(EV_UART_TX);

}

void GSRInterface_streamingCounterReset(void)
{

	/* Reset the counter value because the phone is still nearby. */
	GSRCounter = 0;

	/* The ADC is a 16 bit integer hence it need two file locations to store its value. Here is the high byte. */
	FileTextE107[8] = ADCValues >> 8;

	/* The ADC is a 16 bit integer hence it need two file locations to store its value. Here is the low byte. */
	FileTextE107[9] = ADCValues & 0x00FF;

}







//void GSRInterface_streamingGSRInitialize(void)
//{
//
//	/* This informs the app that the patch is in the process of acquiring the temperature */
//	FileTextE105[4] = GSR_INTERFACE_ACQUIRE_GSR_STREAMING_COMMAND;
//
//	/* TimeBetweenSamples is a 16 bit number which is composed of FileTextE105[5] and FileTextE105[6] */
//	timeBetweenSamples = 300;
//
//	/* Set the patch to GSR datalogger mode. This flag is needed by the timer interrupt routine. */
//	acquireGSRStreaming = true;
//
//	/* Enable the ADC for GSR conversion */
//	ADCInterface_powerMode0();
//
//	/* Set the Timer1 frequency. */
//	TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);
//
//	/* Enable timer. This will wake up the MCU when its goes into sleep mode */
//	TimerInterface_powerMode0();
//
//}
//
//
//void GSRInterface_streamingCounterIncrement(void)
//{
//
//	/* This if statement determines whether the phone has been removed.  */
//	if(GSRCounter == 3)
//	{
//
//		/* Reset the GSR counter */
//		GSRCounter = 0;
//
//		/*Re-enable the timer.  */
//		TimerInterface_powerMode1();
//
//		/* Since we don't need it and to conserve power, power down the ADC. */
//		ADCInterface_powerMode1();
//
//	}
//
//	/* This means that the phone has been removed and hence we must reset everything. */
//	else
//	{
//		/* Do not turn off the timer. Simply pause it */
//		TimerInterface_powerMode0();
//
//		/* Enabel the GSR counter.*/
//		GSRCounter++;
//	}
//
//
//}

