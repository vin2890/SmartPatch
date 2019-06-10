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
//! \file   StateMachine.h
//!
//! \brief  This module contains a two stage state machine. All the events are
//!			feed by the NFCForumType4TagType.c/.h module.  This module is
//!			responsible for determining when a command has been given and
//!			executing the given command. All the events which are passed by
//!			the NFCForumType4TagType module are stored in a FIFO ring buffer.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

/********************************************************************/
#include <stdio.h>
#include "types.h"

/********************************************************************/

/* Defines */
#define STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_MASK STATE_MACHINE_PROCESS_TRACING_RX_EVENT_FIFO_SIZE-1
#define STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_MASK STATE_MACHINE_PROCESS_TRACING_TX_EVENT_FIFO_SIZE-1

#define STATE_MACHINE_PROCESS_TRACING_RX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_PROCESS_TRACING_RX_STATE_FIFO_MASK STATE_MACHINE_PROCESS_TRACING_RX_STATE_FIFO_SIZE-1
#define STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_MASK STATE_MACHINE_PROCESS_TRACING_TX_STATE_FIFO_SIZE-1


#define STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_MASK STATE_MACHINE_COMMAND_EXECUTION_RX_EVENT_FIFO_SIZE-1
#define STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_MASK STATE_MACHINE_COMMAND_EXECUTION_TX_EVENT_FIFO_SIZE-1

#define STATE_MACHINE_COMMAND_EXECUTION_RX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_COMMAND_EXECUTION_RX_STATE_FIFO_MASK STATE_MACHINE_COMMAND_EXECUTION_RX_STATE_FIFO_SIZE-1
#define STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_MASK STATE_MACHINE_COMMAND_EXECUTION_TX_STATE_FIFO_SIZE-1




typedef enum {
		St_Process_Tracing_Idle,

		St_Process_Tracing_E103_File_Selected,
		St_Process_Tracing_E103_PLEN_Read,
		St_Process_Tracing_E103_Message_Read,

		//St_Process_Tracing_E105_File_Selected,
		St_Process_Tracing_E103_PLEN_Zero,
		St_Process_Tracing_E103_Message_Written,
		St_Process_Tracing_E103_PLEN_Updated,


		St_Process_Tracing_E104_File_Selected,
		St_Process_Tracing_E104_PLEN_Read,
		St_Process_Tracing_E104_Message_Read,

		//St_Process_Tracing_E105_File_Selected,
		St_Process_Tracing_E104_PLEN_Zero,
		St_Process_Tracing_E104_Message_Written,
		St_Process_Tracing_E104_PLEN_Updated,

		St_Process_Tracing_E105_File_Selected,
		St_Process_Tracing_E105_PLEN_Read,
		St_Process_Tracing_E105_Message_Read,

		//St_Process_Tracing_E105_File_Selected,
		St_Process_Tracing_E105_PLEN_Zero,
		St_Process_Tracing_E105_Message_Written,
		St_Process_Tracing_E105_PLEN_Updated,

		St_Process_Tracing_E106_File_Selected,
		St_Process_Tracing_E106_PLEN_Read,
		St_Process_Tracing_E106_Message_Read,

		//St_Process_Tracing_E107_File_Selected,
		St_Process_Tracing_E106_PLEN_Zero,
		St_Process_Tracing_E106_Message_Written,
		St_Process_Tracing_E106_PLEN_Updated,

		St_Process_Tracing_E107_File_Selected,
		St_Process_Tracing_E107_PLEN_Read,
		St_Process_Tracing_E107_Message_Read,

		//St_Process_Tracing_E108_File_Selected,
		St_Process_Tracing_E107_PLEN_Zero,
		St_Process_Tracing_E107_Message_Written,
		St_Process_Tracing_E107_PLEN_Updated,

		St_Process_Tracing_E108_File_Selected,
		St_Process_Tracing_E108_PLEN_Read,
		St_Process_Tracing_E108_Message_Read,

		//St_Process_Tracing_E108_File_Selected,
		St_Process_Tracing_E108_PLEN_Zero,
		St_Process_Tracing_E108_Message_Written,
		St_Process_Tracing_E108_PLEN_Updated,
} STATES_PROCESS_TRACING;

typedef enum {
		EVT_Process_Tracing_Null,

        EVT_Process_Tracing_E103_File_Selected,
        EVT_Process_Tracing_E103_Read_PLEN,
        EVT_Process_Tracing_E103_Read_Message,

        //EVT_Process_Tracing_E105_File_Selected,
        EVT_Process_Tracing_E103_Write_PLEN_Zero,
        EVT_Process_Tracing_E103_Write_Message,
        EVT_Process_Tracing_E103_Write_PLEN,

		EVT_Process_Tracing_E104_File_Selected,
        EVT_Process_Tracing_E104_Read_PLEN,
        EVT_Process_Tracing_E104_Read_Message,

        //EVT_Process_Tracing_E105_File_Selected,
        EVT_Process_Tracing_E104_Write_PLEN_Zero,
        EVT_Process_Tracing_E104_Write_Message,
        EVT_Process_Tracing_E104_Write_PLEN,

        EVT_Process_Tracing_E105_File_Selected,
        EVT_Process_Tracing_E105_Read_PLEN,
        EVT_Process_Tracing_E105_Read_Message,

        //EVT_Process_Tracing_E105_File_Selected,
        EVT_Process_Tracing_E105_Write_PLEN_Zero,
        EVT_Process_Tracing_E105_Write_Message,
        EVT_Process_Tracing_E105_Write_PLEN,

        EVT_Process_Tracing_E106_File_Selected,
        EVT_Process_Tracing_E106_Read_PLEN,
        EVT_Process_Tracing_E106_Read_Message,

        //EVT_Process_Tracing_E106_File_Selected,
        EVT_Process_Tracing_E106_Write_PLEN_Zero,
        EVT_Process_Tracing_E106_Write_Message,
        EVT_Process_Tracing_E106_Write_PLEN,

        EVT_Process_Tracing_E107_File_Selected,
        EVT_Process_Tracing_E107_Read_PLEN,
        EVT_Process_Tracing_E107_Read_Message,

        //EVT_Process_Tracing_E106_File_Selected,
        EVT_Process_Tracing_E107_Write_PLEN_Zero,
        EVT_Process_Tracing_E107_Write_Message,
        EVT_Process_Tracing_E107_Write_PLEN,

        EVT_Process_Tracing_E108_File_Selected,
        EVT_Process_Tracing_E108_Read_PLEN,
        EVT_Process_Tracing_E108_Read_Message,

        //EVT_Process_Tracing_E106_File_Selected,
        EVT_Process_Tracing_E108_Write_PLEN_Zero,
        EVT_Process_Tracing_E108_Write_Message,
        EVT_Process_Tracing_E108_Write_PLEN,

} EVENTS_PROCESS_TRACING;

typedef enum {
		St_Command_Execution_Idle,
		St_Command_Execution_CMDFile_Read_1,
		St_Command_Execution_CMDFile_write_CMD,
		St_Command_Execution_CMDFile_Read_2,

		St_Command_Execution_E103File_Read,
		St_Command_Execution_E104File_Read,
		St_Command_Execution_E105File_Read,
		St_Command_Execution_E106File_Read,
		St_Command_Execution_E107File_Read,
		//St_Command_Execution_Execute_Command
} STATES_COMMAND_EXECUTION;

typedef enum {
		EVT_Command_Execution_Null,
		EVT_Command_Execution_CMDFile_Read_1,
		EVT_Command_Execution_CMDFile_Written,
		EVT_Command_Execution_CMDFile_Read_2,

		EVT_Command_Execution_E103File_Read,
		EVT_Command_Execution_E103File_Written,
		EVT_Command_Execution_E103File_Read_2,

		EVT_Command_Execution_E104File_Read,
		EVT_Command_Execution_E104File_Written,
		EVT_Command_Execution_E104File_Read_2,

		EVT_Command_Execution_E105File_Read,
		EVT_Command_Execution_E105File_Written,
		EVT_Command_Execution_E105File_Read_2,

		EVT_Command_Execution_E106File_Read,
		EVT_Command_Execution_E106File_Written,
		EVT_Command_Execution_E106File_Read_2,


		EVT_Command_Execution_E107File_Read,
		EVT_Command_Execution_E107File_Written,
		EVT_Command_Execution_E107File_Read_2,

		EVT_Command_Execution_E108File_Read,
		EVT_Command_Execution_E108File_Written,
		EVT_Command_Execution_E108File_Read_2


} EVENTS_COMMAND_EXECUTION;

#ifndef NULL
#define NULL						0x00
#endif
#define START_DATA_HEADER			0x02
#define WRITE_REG_COMMAND			0x91
#define READ_REG_COMMAND			0x92
#define DATA_STREAMING_COMMAND		0x93
#define DATA_STREAMING_PACKET		0x93
#define ACQUIRE_DATA_COMMAND		0x94
#define ACQUIRE_DATA_PACKET 		0x94
#define PROC_DATA_DOWNLOAD_COMMAND	0x95
#define DATA_DOWNLOAD_COMMAND		0x96
#define FIRMWARE_UPGRADE_COMMAND	0x97
#define START_RECORDING_COMMAND		0x98
#define FIRMWARE_VERSION_REQ		0x99
#define STATUS_INFO_REQ 			0x9A
#define FILTER_SELECT_COMMAND		0x9B
#define ERASE_MEMORY_COMMAND		0x9C
#define RESTART_COMMAND				0x9D
#define END_DATA_HEADER				0x03
#define CARRIAGE_RETURN 			0x0D

//#define ACQUIRE_TEMPERATURE_COMMAND		0X84
#define ACQUIRE_MOTION_DATA				0X85
#define ACQUIRE_GSR_ADC_VALUE			0X86

#define PATCH_NOT_BUSY				0x04
#define PATCH_BUSY					0x05
#define ACK							0X06
#define NCK							0x07


struct ADS1x9x_state{
	unsigned char state;
	unsigned char SamplingRate;
	unsigned char command;
};

typedef enum stECG_RECORDER_STATE {

	IDLE_STATE =0,
	DATA_STREAMING_STATE,
	ACQUIRE_DATA_STATE,
	ECG_DOWNLOAD_STATE,
	ECG_RECORDING_STATE,

	ACQUIRE_TEMPERATURE_DATALOGGER_STATE


}ECG_RECORDER_STATE;

extern struct NANDAddress Read_Recorder_NANDAddress;
extern struct NANDAddress Recorder_NANDAddress;
extern struct NANDAddress Acquire_NANDAddress;

extern unsigned char ADS1x9xRegVal[16];
extern unsigned char ECG_Data_rdy;
extern unsigned short NumPackets,ReqSamples;
extern unsigned char NumFrames;
extern unsigned short BlockNum;
extern unsigned char Store_data_rdy;
//extern unsigned char ECGRecorder_data_Buf[512], Recorder_head,Recorder_tail;
extern unsigned int packetCounter , AcqpacketCounter;
extern unsigned char Filter_Option;

struct ADS1x9x_state ECG_Recoder_state;

extern unsigned regval, Live_Streaming_flag;

void Decode_Recieved_Command(void);


void StateMachine_initialize(void);


BOOL StateMachine_processTracing(void);

/***********************************************************
	Function Name: StateMachine_processTracingReadEventRxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
	Note: Used
***********************************************************/
EVENTS_PROCESS_TRACING StateMachine_processTracingReadEventRxFifo(void);

/***********************************************************
	Function Name: StateMachine_processTracingWriteEventRxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
	Note:	Used
***********************************************************/
void StateMachine_processTracingWriteEventRxFifo(EVENTS_PROCESS_TRACING data);


/***********************************************************
	Function Name: StateMachine_processTracingReadEventTxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
EVENTS_PROCESS_TRACING StateMachine_processTracingReadEventTxFifo(void);

/***********************************************************
	Function Name: StateMachine_processTracingWriteEventTxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
***********************************************************/
void StateMachine_processTracingWriteEventTxFifo(EVENTS_PROCESS_TRACING data);

/***********************************************************
	Function Name: StateMachine_processTracingReadStateRxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
STATES_PROCESS_TRACING StateMachine_processTracingReadStateRxFifo(void);

/***********************************************************
	Function Name: StateMachine_processTracingWriteStateRxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
	Note: Used
***********************************************************/
void StateMachine_processTracingWriteStateRxFifo(STATES_PROCESS_TRACING data);

/***********************************************************
	Function Name: StateMachine_processTracingReadStateTxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
STATES_PROCESS_TRACING StateMachine_processTracingReadStateTxFifo(void);

/***********************************************************
	Function Name: StateMachine_processTracingWriteStateTxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
***********************************************************/
void StateMachine_processTracingWriteStateTxFifo(STATES_PROCESS_TRACING data);

/***********************************************************
	Function Name: UIMgr_flushTxBuffer
	Function Description: This function is responsible for
	sending all data currently in the serial tx buffer
	to the user.
	Inputs:  none
	Outputs: none
***********************************************************/
//void StateMachine_flushTxBuffer(void)

void StateMachine_commandExecution(void);
/***********************************************************
	Function Name: StateMachine_commandExecutionReadEventRxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
	Note: Used
***********************************************************/
EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionReadEventRxFifo(void);


/***********************************************************
	Function Name: StateMachine_commandExecutionWriteEventRxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
	Notes: Used
***********************************************************/
void StateMachine_commandExecutionWriteEventRxFifo(EVENTS_COMMAND_EXECUTION data);

/***********************************************************
	Function Name: StateMachine_commandExecutionReadEventTxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
EVENTS_COMMAND_EXECUTION StateMachine_commandExecutionReadEventTxFifo(void);

/***********************************************************
	Function Name: StateMachine_commandExecutionWriteEventTxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
***********************************************************/
void StateMachine_commandExecutionWriteEventTxFifo(EVENTS_COMMAND_EXECUTION data);

/***********************************************************
	Function Name: StateMachine_commandExecutionReadStateRxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
STATES_COMMAND_EXECUTION StateMachine_commandExecutionReadStateRxFifo(void);

/***********************************************************
	Function Name: StateMachine_commandExecutionWriteStateRxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
	Note: Used
***********************************************************/
void StateMachine_commandExecutionWriteStateRxFifo(STATES_COMMAND_EXECUTION data);

/***********************************************************
	Function Name: StateMachine_commandExecutionReadStateTxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: unsigned char-the data read
***********************************************************/
STATES_COMMAND_EXECUTION StateMachine_commandExecutionReadStateTxFifo(void);

/***********************************************************
	Function Name: StateMachine_commandExecutionWriteStateTxFifo
	Function Description: This function is responsible for
	writing a single byte to the RxFifo and
	updating the appropriate pointers.
	Inputs:  data - the byte to write to the Fifo
	Outputs: none
***********************************************************/
void StateMachine_commandExecutionWriteStateTxFifo(STATES_COMMAND_EXECUTION data);


#endif /* STATEMACHINE_H_ */
