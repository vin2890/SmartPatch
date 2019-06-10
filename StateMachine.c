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
//! \file   StateMachine.c
//!
//! \brief  Please see StateMachine.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#include <msp430.h>
#include "StateMachine.h"
#include "types.h"
#include "NFCForumType4TagType.h"
#include "ADS1x9x.h"
#include "TemperatureInterface.h"
#include "GSRInterface.h"
#include "ADC12_B.h"
#include "Executive.h"
#include "AccelerometerInterface.h"
//#include "ADS1x9x_Nand_Flash.h"
#include "ADS1x9x_Version.h"

/*  Extern Variables */
EVENTS_PROCESS_TRACING StateMachine_processTracingRxEventFifo[STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_SIZE];
unsigned char StateMachine_processTracingRxEventFifoHead=0;
unsigned char StateMachine_processTracingRxEventFifoTail=0;

EVENTS_PROCESS_TRACING StateMachine_processTracingTxEventFifo[STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_SIZE];
unsigned char StateMachine_processTracingTxEventFifoHead=0;
unsigned char StateMachine_processTracingTxEventFifoTail=0;


STATES_PROCESS_TRACING StateMachine_processTracingRxStateFifo[STATE_MACHINE_PROCESS_TRACING_RX_STATE_FIFO_SIZE];
unsigned char StateMachine_processTracingRxStateFifoHead=0;
unsigned char StateMachine_processTracingRxStateFifoTail=0;

STATES_PROCESS_TRACING StateMachine_processTracingTxStateFifo[STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_SIZE];
unsigned char StateMachine_processTracingTxStateFifoHead=0;
unsigned char StateMachine_processTracingTxStateFifoTail=0;


EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionRxEventFifo[STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_SIZE];
unsigned char StateMachine_commandExecutionRxEventFifoHead=0;
unsigned char StateMachine_commandExecutionRxEventFifoTail=0;

EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionTxEventFifo[STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_SIZE];
unsigned char StateMachine_commandExecutionTxEventFifoHead=0;
unsigned char StateMachine_commandExecutionTxEventFifoTail=0;


STATES_COMMAND_EXECUTION StateMachine_commandExecutionRxStateFifo[STATE_MACHINE_COMMAND_EXECUTION_RX_STATE_FIFO_SIZE];
unsigned char StateMachine_commandExecutionRxStateFifoHead=0;
unsigned char StateMachine_commandExecutionRxStateFifoTail=0;

STATES_COMMAND_EXECUTION StateMachine_commandExecutionTxStateFifo[STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_SIZE];
unsigned char StateMachine_commandExecutionTxStateFifoHead=0;
unsigned char StateMachine_commandExecutionTxStateFifoTail=0;



/*  Definitions */
#define IS_PROCESS_TRACING_DATA_IN_TX_EVENT_FIFO() (!(StateMachine_processTracingTxEventFifoHead == StateMachine_processTracingTxEventFifoTail))
#define IS_PROCESS_TRACING_DATA_IN_RX_EVENT_FIFO() (!(StateMachine_processTracingRxEventFifoHead == StateMachine_processTracingRxEventFifoTail))

#define IS_PROCESS_TRACING_DATA_IN_TX_STATE_FIFO() (!(StateMachine_processTracingTxStateFifoHead == StateMachine_processTracingTxStateFifoTail))
#define IS_PROCESS_TRACING_DATA_IN_RX_STATE_FIFO() (!(StateMachine_processTracingRxStateFifoHead == StateMachine_processTracingRxStateFifoTail))

STATES_PROCESS_TRACING processTracingState = St_Process_Tracing_Idle;


/*  Definitions */
#define IS_COMMAND_EXECUTION_DATA_IN_TX_EVENT_FIFO() (!(StateMachine_commandExecutionTxEventFifoHead == StateMachine_commandExecutionTxEventFifoTail))
#define IS_COMMAND_EXECUTION_DATA_IN_RX_EVENT_FIFO() (!(StateMachine_commandExecutionRxEventFifoHead == StateMachine_commandExecutionRxEventFifoTail))

#define IS_COMMAND_EXECUTION_DATA_IN_TX_STATE_FIFO() (!(StateMachine_commandExecutionTxStateFifoHead == StateMachine_commandExecutionTxStateFifoTail))
#define IS_COMMAND_EXECUTION_DATA_IN_RX_STATE_FIFO() (!(StateMachine_commandExecutionRxStateFifoHead == StateMachine_commandExecutionRxStateFifoTail))

STATES_COMMAND_EXECUTION commandExecutionState = St_Command_Execution_Idle;

int temperatureCounter = 0;

struct ADS1x9x_state ECG_Recoder_state;

//#pragma NOINIT(ECGRxPacket)
//unsigned char ECGRxPacket[64],ECGRxCount, dumy ;
//
//#pragma NOINIT(ECGTxPacket)
//unsigned char ECGTxPacket[64],ECGTxCount,ECGTxPacketRdy;

//extern unsigned regval, Live_Streaming_flag;
unsigned Req_Dwnld_occured;

unsigned char ECG_Proc_data_cnt = 0;



void StateMachine_initialize(void)
{
//	StateMachine_processTracingWriteStateRxFifo(St_Process_Tracing_Idle);
//	StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_Null);
//
//	StateMachine_commandExecutionWriteStateRxFifo(St_Command_Execution_Idle);
//	StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_Null);
}


void StateMachine_commandExecution(void)
{
    /* Read the next incoming event. Usually this is a blocking function. */
	EVENTS_COMMAND_EXECUTION event;


//	if((ADS1x9x_Reg_Read(0x08) == 12 ))
//	{
//		FileTextE105[2] = PATCH_BUSY;//This lets the Android phone app know not to give another CMD until this CMD finishes.
////		FileTextE105[6] = ACK;		//This lets the Android phone app know that the Patch acknowledges its received its CMD and in the process of executing it.
//		processTracingState = St_Process_Tracing_Idle;
//	}
//	else
//	{
//		FileTextE105[2] = PATCH_NOT_BUSY;
//	}

	if(StateMachine_processTracing()==false)
		return;


	while(IS_COMMAND_EXECUTION_DATA_IN_RX_EVENT_FIFO() == TRUE)
	{
		event = StateMachine_commandExecutionReadEventRxFifo();

		/* Switch the state and the event to execute the right transition. */
		switch(commandExecutionState)
		{
			case St_Command_Execution_Idle:
				switch(event)
				{

					case EVT_Command_Execution_CMDFile_Read_1:
						commandExecutionState = St_Command_Execution_CMDFile_Read_1;
						StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);

						//WDTCTL = WDTPW + WDTCNTCL;
						break;
					default:
						commandExecutionState = St_Command_Execution_Idle;
						StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
						break;
				}
				break;

			case St_Command_Execution_CMDFile_Read_1:
				switch(event)
				{
				case EVT_Command_Execution_CMDFile_Written:
					commandExecutionState = St_Command_Execution_CMDFile_write_CMD;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
					break;
				default:
					commandExecutionState = St_Command_Execution_Idle;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
					break;
				}
				break;

			case St_Command_Execution_CMDFile_write_CMD:
				switch(event)
				{
				case EVT_Command_Execution_CMDFile_Read_1:
					commandExecutionState = St_Command_Execution_CMDFile_Read_2;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);


		    		//The phone read the CMD file to make sure the Busy bit is not set.
					ECG_Recoder_state.command = FileTextE105[4];
					Decode_Recieved_Command();

					//commandExecutionState = St_Command_Execution_Idle;
					commandExecutionState = St_Command_Execution_CMDFile_Read_2;
					break;
				default:
					commandExecutionState = St_Command_Execution_Idle;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
					break;
				}
				break;

			case St_Command_Execution_CMDFile_Read_2:
				switch(event)
				{
				case EVT_Command_Execution_E106File_Read:

					PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_STREAMING);

					break;
				case EVT_Command_Execution_E107File_Read:

					PUBLISH_EVENT(EV_ACQUIRE_GSR_STREAMING_RESET);
					break;
				case EVT_Command_Execution_E108File_Read:
					commandExecutionState = St_Command_Execution_E106File_Read;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);

		    		//The phone read the CMD file to make sure the Busy bit is not set.
					ECG_Recoder_state.command = FileTextE105[4];
					Decode_Recieved_Command();


					commandExecutionState = St_Command_Execution_CMDFile_Read_2;

					//if(9 == temperatureCounter++)
					//{
					//	temperatureCounter = 0;
					//	commandExecutionState = St_Command_Execution_Idle;

					//	FileTextE105[2] = PATCH_NOT_BUSY;//This lets the Android phone app know not to give another CMD until this CMD finishes.
//						FileTextE105[6] = NCK;		//This lets the Android phone app know that the Patch acknowledges its received its CMD and in the process of executing it.

					//}

					//WDTCTL = WDTPW + WDTCNTCL;
					break;
				default:
					commandExecutionState = St_Command_Execution_Idle;
					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
					break;
				}
				break;


//			case St_Command_Execution_Execute_Command:
//				switch(event)
//				{
//				default:
//					commandExecutionState = St_Command_Execution_Idle;
//					StateMachine_commandExecutionWriteStateRxFifo(commandExecutionState);
//
//
//
//					break;
//				}
//				break;
		}//switch(NFCstate)
	}//while(IS_DATA_IN_RX_FIFO() == TRUE)
}//void fuctionStateMachine(void)


BOOL StateMachine_processTracing (void)
{
    /* Read the next incoming event. Usually this is a blocking function. */
	EVENTS_PROCESS_TRACING event;

	while(IS_PROCESS_TRACING_DATA_IN_RX_EVENT_FIFO() == TRUE)
	{
		event = StateMachine_processTracingReadEventRxFifo();

		/* Switch the state and the event to execute the right transition. */
		switch(processTracingState)
		{
			case St_Process_Tracing_Idle:
				switch(event)
				{
					case EVT_Process_Tracing_E103_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E103_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E104_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E104_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E105_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E106_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E107_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E108_File_Selected:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;

				}
				break;

/*E103*/
			case St_Process_Tracing_E103_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E104_File_Selected:
						processTracingState = St_Process_Tracing_E104_File_Selected;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E103_Read_Message:
						processTracingState = St_Process_Tracing_E103_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
					}
				break;

			case St_Process_Tracing_E103_PLEN_Zero:
				switch(event)
				{
					case EVT_Process_Tracing_E105_Write_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_Message_Written;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;

				}
				break;
/*E104*/
			case St_Process_Tracing_E104_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E104_Read_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E104_PLEN_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

//					case EVT_Process_Tracing_E104_Write_PLEN_Zero:
//						/* Change the state */
//						processTracingState = St_Process_Tracing_E105_PLEN_Zero;
//						StateMachine_processTracingWriteStateRxFifo(processTracingState);
//						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E104_PLEN_Read:
				switch(event)
				{
					case EVT_Process_Tracing_E104_Read_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E104_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
/*E105*/
			case St_Process_Tracing_E105_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E105_Read_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_PLEN_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E105_Write_PLEN_Zero:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_PLEN_Zero;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E105_PLEN_Read:
				switch(event)
				{
					case EVT_Process_Tracing_E105_Read_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);

						StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_CMDFile_Read_1);
						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}

			case St_Process_Tracing_E105_PLEN_Zero:
				switch(event)
				{
					case EVT_Process_Tracing_E105_Write_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_Message_Written;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E105_Message_Written:
				switch(event)
				{
					case EVT_Process_Tracing_E105_Write_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E105_PLEN_Updated;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);

						StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_CMDFile_Written);

						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
/*E106*/
			case St_Process_Tracing_E106_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E106_Read_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_PLEN_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E106_Write_PLEN_Zero:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_PLEN_Zero;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E106_PLEN_Read:
				switch(event)
				{
					case EVT_Process_Tracing_E106_Read_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);

						StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_E106File_Read);

						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}

			case St_Process_Tracing_E106_PLEN_Zero:
				switch(event)
				{
					case EVT_Process_Tracing_E106_Write_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_Message_Written;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E106_Message_Written:
				switch(event)
				{
					case EVT_Process_Tracing_E106_Write_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E106_PLEN_Updated;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
/*E107*/
			case St_Process_Tracing_E107_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E107_Read_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_PLEN_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E107_Write_PLEN_Zero:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_PLEN_Zero;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E107_PLEN_Read:
				switch(event)
				{
					case EVT_Process_Tracing_E107_Read_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);

						StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_E107File_Read);

						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}


			case St_Process_Tracing_E107_PLEN_Zero:
				switch(event)
				{
					case EVT_Process_Tracing_E107_Write_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_Message_Written;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;

				}
				break;

			case St_Process_Tracing_E107_Message_Written:
				switch(event)
				{
					case EVT_Process_Tracing_E107_Write_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E107_PLEN_Updated;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
/*E108*/
			case St_Process_Tracing_E108_File_Selected:
				switch(event)
				{
					case EVT_Process_Tracing_E108_Read_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_PLEN_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					case EVT_Process_Tracing_E108_Write_PLEN_Zero:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_PLEN_Zero;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}
				break;

			case St_Process_Tracing_E108_PLEN_Read:
				switch(event)
				{
					case EVT_Process_Tracing_E108_Read_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_Message_Read;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);

						StateMachine_commandExecutionWriteEventRxFifo(EVT_Command_Execution_E108File_Read);

						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}


			case St_Process_Tracing_E108_PLEN_Zero:
				switch(event)
				{
					case EVT_Process_Tracing_E108_Write_Message:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_Message_Written;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						break;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;

				}
				break;

			case St_Process_Tracing_E108_Message_Written:
				switch(event)
				{
					case EVT_Process_Tracing_E108_Write_PLEN:
						/* Change the state */
						processTracingState = St_Process_Tracing_E108_PLEN_Updated;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						processTracingState = St_Process_Tracing_Idle;
						return true;

					default:
						processTracingState = St_Process_Tracing_Idle;
						StateMachine_processTracingWriteStateRxFifo(processTracingState);
						return false;
				}


		}//switch(NFCstate)
	}//while(IS_DATA_IN_RX_FIFO() == TRUE)

	return false;
}//void fuctionStateMachine(void)


EVENTS_PROCESS_TRACING StateMachine_processTracingReadEventRxFifo(void){

	unsigned char tmpTail;
	EVENTS_PROCESS_TRACING dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_processTracingRxEventFifo[StateMachine_processTracingRxEventFifoTail];
//	StateMachine_processTracingRxEventFifo[StateMachine_processTracingRxEventFifoTail] = EVT_Process_Tracing_Null;
	tmpTail = (StateMachine_processTracingRxEventFifoTail+1) & (STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_MASK);
	StateMachine_processTracingRxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_processTracingWriteEventRxFifo(EVENTS_PROCESS_TRACING data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_processTracingRxEventFifo[StateMachine_processTracingRxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_processTracingRxEventFifoHead + 1) & (STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_MASK);
    StateMachine_processTracingRxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

EVENTS_PROCESS_TRACING StateMachine_processTracingReadEventTxFifo(void){

	unsigned char tmpTail;
	EVENTS_PROCESS_TRACING dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_processTracingTxEventFifo[StateMachine_processTracingTxEventFifoTail];
//	StateMachine_processTracingTxEventFifo[StateMachine_processTracingTxEventFifoTail] = EVT_Process_Tracing_Null;
	tmpTail = (StateMachine_processTracingTxEventFifoTail+1) & (STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_MASK);
	StateMachine_processTracingTxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_processTracingWriteEventTxFifo(EVENTS_PROCESS_TRACING data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_processTracingTxEventFifo[StateMachine_processTracingTxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_processTracingTxEventFifoHead + 1) & (STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_MASK);
    StateMachine_processTracingTxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

STATES_PROCESS_TRACING StateMachine_processTracingReadStateRxFifo(void){

	unsigned char tmpTail;
	STATES_PROCESS_TRACING dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_processTracingRxStateFifo[StateMachine_processTracingRxStateFifoTail];
	StateMachine_processTracingRxStateFifo[StateMachine_processTracingRxStateFifoTail] = St_Process_Tracing_Idle;
	tmpTail = (StateMachine_processTracingRxStateFifoTail+1) & (STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_MASK);
	StateMachine_processTracingRxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_processTracingWriteStateRxFifo(STATES_PROCESS_TRACING data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_processTracingRxStateFifo[StateMachine_processTracingRxStateFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_processTracingRxStateFifoHead + 1) & (STATE_MACHINE_PROCESS_TRACING_RX_STATE_FIFO_MASK);
    StateMachine_processTracingRxStateFifoHead = tmpHead;
	ENABLE_INTS();

}

STATES_PROCESS_TRACING StateMachine_processTracingReadStateTxFifo(void){

	unsigned char tmpTail;
	STATES_PROCESS_TRACING dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_processTracingTxStateFifo[StateMachine_processTracingTxStateFifoTail];
//	StateMachine_processTracingTxStateFifo[StateMachine_processTracingTxStateFifoTail] = St_Process_Tracing_Idle;
	tmpTail = (StateMachine_processTracingTxStateFifoTail+1) & (STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_MASK);
	StateMachine_processTracingTxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_processTracingWriteStateTxFifo(STATES_PROCESS_TRACING data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_processTracingTxStateFifo[StateMachine_processTracingTxStateFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_processTracingTxStateFifoHead + 1) & (STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_MASK);
    StateMachine_processTracingTxStateFifoHead = tmpHead;
	ENABLE_INTS();

}

/***********************************************************
	Function Name: UIMgr_flushTxBuffer
	Function Description: This function is responsible for
	sending all data currently in the serial tx buffer
	to the user.
	Inputs:  none
	Outputs: none
***********************************************************/
void StateMachine_flushTxBuffer(void)
{
//	while(IS_DATA_IN_TX_FIFO() == TRUE)
//	{
//		//UartInt_txByte(UIMgr_readTxFifo() );
//	}
}

EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionReadEventRxFifo(void){

	unsigned char tmpTail;
	EVENTS_COMMAND_EXECUTION dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_commandExecutionRxEventFifo[StateMachine_commandExecutionRxEventFifoTail];
	StateMachine_commandExecutionRxEventFifo[StateMachine_commandExecutionRxEventFifoTail] = EVT_Command_Execution_Null;
	tmpTail = (StateMachine_commandExecutionRxEventFifoTail+1) & (STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_MASK);
	StateMachine_commandExecutionRxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_commandExecutionWriteEventRxFifo(EVENTS_COMMAND_EXECUTION data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_commandExecutionRxEventFifo[StateMachine_commandExecutionRxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_commandExecutionRxEventFifoHead + 1) & (STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_MASK);
    StateMachine_commandExecutionRxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionReadEventTxFifo(void){

	unsigned char tmpTail;
	EVENTS_COMMAND_EXECUTION dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_commandExecutionTxEventFifo[StateMachine_commandExecutionTxEventFifoTail];
//	StateMachine_commandExecutionTxEventFifo[StateMachine_commandExecutionTxEventFifoTail] = EVT_Command_Execution_Null;
	tmpTail = (StateMachine_commandExecutionTxEventFifoTail+1) & (STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_MASK);
	StateMachine_commandExecutionTxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}


void StateMachine_commandExecutionWriteEventTxFifo(EVENTS_COMMAND_EXECUTION data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_commandExecutionTxEventFifo[StateMachine_commandExecutionTxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_commandExecutionTxEventFifoHead + 1) & (STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_MASK);
    StateMachine_commandExecutionTxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

STATES_COMMAND_EXECUTION StateMachine_commandExecutionReadStateRxFifo(void){

	unsigned char tmpTail;
	STATES_COMMAND_EXECUTION dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	//StateMachine_rxFifo[StateMachine_rxFifoTail] = EVT_Null;
	dataByte = StateMachine_commandExecutionRxStateFifo[StateMachine_commandExecutionRxStateFifoTail];
	StateMachine_commandExecutionRxStateFifo[StateMachine_commandExecutionRxStateFifoTail] = St_Command_Execution_Idle;
	tmpTail = (StateMachine_commandExecutionRxStateFifoTail+1) & (STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_MASK);
	StateMachine_commandExecutionRxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_commandExecutionWriteStateRxFifo(STATES_COMMAND_EXECUTION data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_commandExecutionRxStateFifo[StateMachine_commandExecutionRxStateFifoHead] = data;


    /* now move the head up */
    tmpHead = (StateMachine_commandExecutionRxStateFifoHead + 1) & (STATE_MACHINE_COMMAND_EXECUTION_RX_STATE_FIFO_MASK);
    StateMachine_commandExecutionRxStateFifoHead = tmpHead;
	ENABLE_INTS();

}

STATES_COMMAND_EXECUTION StateMachine_commandExecutionReadStateTxFifo(void){

	unsigned char tmpTail;
	STATES_COMMAND_EXECUTION dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_commandExecutionTxStateFifo[StateMachine_commandExecutionTxStateFifoTail];
//	StateMachine_commandExecutionTxStateFifo[StateMachine_commandExecutionTxStateFifoTail] = St_Command_Execution_Idle;
	tmpTail = (StateMachine_commandExecutionTxStateFifoTail+1) & (STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_MASK);
	StateMachine_commandExecutionTxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_commandExecutionWriteStateTxFifo(STATES_COMMAND_EXECUTION data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_commandExecutionTxStateFifo[StateMachine_commandExecutionTxStateFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_commandExecutionTxStateFifoHead + 1) & (STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_MASK);
    StateMachine_commandExecutionTxStateFifoHead = tmpHead;
	ENABLE_INTS();

}

void Decode_Recieved_Command(void)
{
	if (ECG_Recoder_state.state == IDLE_STATE)
	{
			switch(ECG_Recoder_state.command)
			{

				case TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_COMMAND:
				{
					PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_INIT);
				}
				break;

				case TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_STREAMING_COMMAND:
				{
					PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_STREAMING);
				}
				break;


				case GSR_INTERFACE_ACQUIRE_GSR_COMMAND:
				{
					PUBLISH_EVENT(EV_ACQUIRE_GSR_DATA_LOGGER_INIT);
				}
				break;

				case GSR_INTERFACE_ACQUIRE_GSR_STREAMING_COMMAND:
				{
					PUBLISH_EVENT(EV_ACQUIRE_GSR_STREAMING);
				}
				break;




				case ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_COMMAND:
				{
					PUBLISH_EVENT(EV_ACQUIRE_ACCELERATION_DATA_INIT);
				}




		       	case WRITE_REG_COMMAND: 	// Write Reg
				{
					PUBLISH_EVENT(EV_WRITE_REG);
		  		}
		  		break;
		       	case READ_REG_COMMAND: 	// Read Reg
				{
					PUBLISH_EVENT(EV_READ_REG);
		  		}
		  		break;
		       	case DATA_STREAMING_COMMAND: 	// Data streaming
				{
					PUBLISH_EVENT(EV_DATA_STREAMING);
		  		}
		  		break;
		       	case ACQUIRE_DATA_COMMAND: 	// Acquire Data
				{
					PUBLISH_EVENT(EV_ACQUIRE_DATA);
		  		}
		  		break;

		       	case DATA_DOWNLOAD_COMMAND: 	// RAW DATA DUMP
				{
					PUBLISH_EVENT(EV_DATA_DOWNLOAD);

				}
		  		break;

		       	case START_RECORDING_COMMAND: 	// Processed Data Dump
				{
					PUBLISH_EVENT(EV_START_RECORDING);
				}
		  		break;

		       	case FIRMWARE_UPGRADE_COMMAND: 	// FIRMWARE UPGRADE
				{
					PUBLISH_EVENT(EV_FIRMWARE_UPGRADE);
		  		}

		  		break;
		       	case FIRMWARE_VERSION_REQ: 	// firmware Version request
				{
					PUBLISH_EVENT(EV_FIRMWARE_VERSION);
				}
		  		break;

		       	case STATUS_INFO_REQ: 	// Status Request
				{
					PUBLISH_EVENT(EV_STATUS_INFO);
				}
		  		break;
		       	case FILTER_SELECT_COMMAND: 	// Filter Select request
				{
					PUBLISH_EVENT(EV_FILTER_SELECT);

				}
		  		break;
		       	case ERASE_MEMORY_COMMAND: 	// MEMORY ERASE Command
				{
					//PUBLISH_EVENT(EV_ERASE_MEMORY);

					//Erase_NAND_Flash();
				}

				default:

				break;
			}
	}
	else
	{

			switch(ECG_Recoder_state.command)
			{

		       	case DATA_STREAMING_COMMAND:
				{

					Disable_ADS1x9x_DRDY_Interrupt();		// Disable DRDY interrupt
					Stop_Read_Data_Continuous();			// SDATAC command
	  				ECG_Recoder_state.state = IDLE_STATE;	// Switch to Idle state
					Clear_ADS1x9x_Chip_Enable();			// Disable Chip select
					ECG_Data_rdy = 0;
					Live_Streaming_flag = FALSE;			// Disable Live streaming flag
				}
				break;
				default:

				break;
			}
	}
	ECG_Recoder_state.command = 0;
}




