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
//! \file   Executive.c
//!
//! \brief  Please see Executive.h
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
#include <msp430.h>
#include <stdio.h>
#include "Executive.h"
#include "Events.h"
#include "types.h"
#include "driverlib.h"
#include "GPIO.h"
#include "RF430CL331.H"
#include "I2CInterface.h"
#include "NFCForumType4TagType.h"
#include "GSRInterface.h"
#include "BMA280.h"
#include "AccelerometerInterface.h"
#include "TemperatureInterface.h"
#include "ADS1x9x_USB_Communication.h"
#include "ECGInterface.h"
#include "UART.h"
#include "RingBuffer.h"
#include "string.h"
#include "ADS1x9x.h"
#include "DEBUG.h"
#include "Timer_A.h" // for debug

/*  Local Variables */
uint8_t Exec_eventFifo[EXEC_EVENT_FIFO_SIZE];
uint8_t Exec_eventFifoHead=0;
uint8_t Exec_eventFifoTail=0;

/*  Local Function Definitions */
static uint8_t Exec_readEventFifo(void);

//uint16_t flags = 0;

/*  Definitions */
#define IS_DATA_IN_EVENT_FIFO() (!(Exec_eventFifoHead == Exec_eventFifoTail))

void Exec_run(void)
{

	uint8_t eventGenerated;
	fastEventBitmask = 0x00;
	while(1)
	{
		//if (fastEventBitmask)
		//{
			/* an event needing fast processing has been received */
			/* a received line needs to be processed...this
			needs to be processed as quickly as possible */
			//if (fastEventBitmask & FEV_RF430CL331_INTERRUPT)
			//{
               // DISABLE_INTS();
				//fastEventBitmask &= ~FEV_RF430CL331_INTERRUPT;
               // ENABLE_INTS();

                //Only NFC interface functions are supposed to be called. Fix.
                //RF430CL331_serviceInterrupt();

			//}
		//}
		

		if (IS_DATA_IN_EVENT_FIFO() == TRUE)
		{			
            eventGenerated = Exec_readEventFifo();
			switch(eventGenerated)
			{
				case (EV_BMA_WATERMARK_INTERRUPT):
					BMA280_enableINT1Detect();
					break;
				case (EV_BMA_NEW_DATA_READY):

					break;

				/* This is all realted to temperature */
				case (EV_ACQUIRE_TEMPERATURE_INIT):
					TemperatureInterface_readTemperatureInitialize();
					break;
				case (EV_ACQUIRE_TEMPERATURE):
					TemperatureInterface_readTemperatureWriteToFile();
					break;
				case (EV_ACQUIRE_TEMPERATURE_STREAMING):
					TemperatureInterface_streamingStateMachine();
					break;
				case (EV_ACQUIRE_TEMPERATURE_STREAMING_STOP):
					TemperatureInterface_streamingStateMachinePowerDown();
					break;



				/* This is all related to GSR. */
				case (EV_ACQUIRE_GSR_DATA_LOGGER_INIT):
					GSRInterface_readGSRInitialize();
					break;
				case (EV_ACQUIRE_GSR_DATA_LOGGER):
					GSRInterface_readGSRWriteToFile();
					break;


				case (EV_ACQUIRE_GSR_STREAMING):
					GSRInterface_streamingStateMachine();
					break;
				case (EV_ACQUIRE_GSR_STREAMING_RESET):
					GSRInterface_streamingCounterReset();
					break;



				/* Accelerometer */
				case (EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE_UPDATE_STEPCOUNT_AND_SAMPLE_NUMBER):
					AccelerometerInterface_updateStepcountAndSampleNumber();
					break;

				case (EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE):
					AccelerometerInterface_dataLoggerMode();
					break;

				case (EV_ACQUIRE_ACCELERATION_DATA_INIT):
					AccelerometerInterface_setDataLoggerModeInitialize();
					break;




				case (EV_WRITE_REG):
					ECGInterface_RemoteWriteRegister();
					break;

				case (EV_READ_REG):
					ECGInterface_RemoteReadRegister();
					break;
				case (EV_FIRMWARE_UPGRADE):
					ECGInterface_FirmwareUpgrade();
					break;

				case (EV_FIRMWARE_VERSION):
					ECGInterface_FirmwareVersion();
					break;

				case (EV_STATUS_INFO):
					break;

				case (EV_FILTER_SELECT):
					ECGInterface_SelectFilter();
					break;

				case (EV_ERASE_MEMORY):
					break;
				default:
					break;


				/* EV_DATA_STREAMING */
				case (EV_DATA_STREAMING):
					ECGInterface_StreamData();
					break;
				/* EV_ECG_DATA_READY */
				case (EV_ECG_DATA_READY):
					Stream_ECG_data_packets();
					break;
				case (EV_ACQUIRE_DATA):
					ECGInterface_AcquireData();
					break;
				case (EV_ACQUIRE_ECG_SAMPLES):
					Accquire_ECG_Samples();
					break;



				case (EV_DATA_DOWNLOAD):
					ECGInterface_DownloadData();
					break;

				case (EV_START_RECORDING):

					break;

				case (EV_ECG_KEY_PRESSED):
					ECGInterface_KeyPressed();
					break;
				case (EV_ECG_KEY_NOTPRESSED):
					ECGInterface_KeyNotPressed();
					break;
				case (EV_UART_RX):
				    //process input buffer _inputrb_d to determine what action to take
				    //uses the custom build strfind function--see it's function brief in ringbuffer
				    //if the number of commands gets really big this method isn't the fastest for parsing input

				    if(ring_buffer_strfind(_inputrb_d,"h"))
				    {
				        PUBLISH_EVENT(EV_DATA_STREAMING);
				        //uart_puts("stream requested\n");
				        ring_buffer_clear(_inputrb_d);
				    }
				    else if(ring_buffer_strfind(_inputrb_d,"s"))
				    {
				        Disable_ADS1x9x_DRDY_Interrupt();
				        //uart_puts("stop requested");
				    }
				    else if(ring_buffer_strfind(_inputrb_d,"i"))
				    {
				        //uart_puts("ADS1292 default register initialization...\n\r");
				        ADS1x9x_Default_Reg_Init();
				    }
				    else if(ring_buffer_strfind(_inputrb_d,"t")){
				        PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_INIT);
				        //uart_puts("temp requested\n");
				        ring_buffer_clear(_inputrb_d);
				    }
				    else if(ring_buffer_strfind(_inputrb_d,"a"))
				    {
                       PUBLISH_EVENT(EV_ACQUIRE_ACCELERATION_DATA_INIT);
                       //uart_puts("accelerometer requested\n");
                       ring_buffer_clear(_inputrb_d);
				    }
				    else if(ring_buffer_strfind(_inputrb_d,"g"))
				    {
                       PUBLISH_EVENT(EV_ACQUIRE_GSR_DATA_LOGGER_INIT);
                       //uart_puts("gsr requested\n");
                       ring_buffer_clear(_inputrb_d);
				    }
                    ring_buffer_clear(_inputrb_d); //clear input buffer after every command
				    break;
				case (EV_UART_TX):
				    //Empty output buffer onto UART0
				    uart_putrb(_outputrb_d);
                    break;
			}			
		}
        else
        {
            __enable_interrupt();
 			//__bis_SR_register(LPM3_bits + GIE); //go to low power mode and enable interrupts. Here we are waiting for an RF read or write
			__no_operation();
        }
        /* toggle the debug line */

	}
}

static unsigned char Exec_readEventFifo(void)
{
	uint8_t dataByte, tmpTail;
	
	DISABLE_INTS();
	/* just return the current tail from the tx fifo */
	dataByte = Exec_eventFifo[Exec_eventFifoTail];	
	tmpTail = (Exec_eventFifoTail+1) & (EXEC_EVENT_FIFO_MASK);
	Exec_eventFifoTail = tmpTail;
	ENABLE_INTS();
	
	return(dataByte);
}

void Exec_writeEventFifo(unsigned char event)
{
	uint8_t tmpHead;

	DISABLE_INTS();
	Exec_eventFifo[Exec_eventFifoHead] = event;

    /* now move the head up */
    tmpHead = (Exec_eventFifoHead + 1) & (EXEC_EVENT_FIFO_MASK);
    Exec_eventFifoHead = tmpHead;
	ENABLE_INTS();
}

