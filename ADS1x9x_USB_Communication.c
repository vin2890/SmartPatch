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
//! \file   ADS1x9x_USB_Communication.c
//!
//! \brief  Please see ADS1x9x_USB_Communication.h
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
#include "ADS1x9x_USB_Communication.h"
#include "ADS1x9x.h"
//#include "ADS1x9x_Nand_Flash.h"
#include "ADS1x9x_ECG_Processing.h"
#include <msp430.h>
#include "StateMachine.h"
#include <stdint.h>
#include "UART.h"
#include "RingBuffer.h"
#include "Executive.h"
#include "DEBUG.h"

void Accquire_ECG_Samples(void);


unsigned short NumPackets,ReqSamples;
uint8_t NumFrames;
uint8_t NumFrames1;

unsigned int FRAMCounter = 9;

extern uint8_t ECGTxPacket[64],ECGTxCount,ECGTxPacketRdy , SPI_Rx_buf[];
extern uint8_t ECG_Data_rdy;
extern uint8_t ADS1x9xRegVal[];
extern struct ADS1x9x_state ECG_Recoder_state;
extern unsigned short timeCtr;
unsigned int packetCounter , AcqpacketCounter;
extern struct NANDAddress  Acquire_NANDAddress;
extern long ADS1x9x_ECG_Data_buf[6];
extern short ECGRawData[4],ECGFilteredData[4] ;
extern unsigned short QRS_Heart_Rate;
extern unsigned short Respiration_Rate ;
unsigned char LeadStatus;

extern uint8_t FileTextE104[9000];
extern uint8_t FileTextE105[50];
unsigned short BlockNum, t1;

uint8_t *ECGPacketAcqPrt, *ECGPacketAcqHoldPrt;


uint8_t ECGRecorder_data_Buf[80], Recorder_head,Recorder_tail;

uint8_t Store_data_rdy;

void Accquire_ECG_Samples(void)
{
	uint8_t *ptr;
   	//unsigned short cPointer;

   if ( Recorder_head != Recorder_tail)
   {
		if ((ADS1x9xRegVal[1] & 0x07) == 6)
		{
				while ( Recorder_tail != Recorder_head)
				{
					Recorder_tail++;							// Increment tail

					if ( (Recorder_tail % 8)  == 0)				// Reset tail after 32 samples
					{
						NumPackets -= 8;						// Reduce number of samples to be captured by 8
					/* After every 8 samples store  ECG data to memory */
//R						Store_Acquired_ECG_Samples_to_Flash(&ECGRecorder_data_Buf[(Recorder_tail-8)<<3] , NumPackets);
						if ( Recorder_tail  == 32)				// Reset tail after 32 samples
							Recorder_tail = 0;
					}
					if ( NumPackets == 0)
					{
						/* Terminate Acquired data after specified sample count*/
						Disable_ADS1x9x_DRDY_Interrupt();		// Disable DRDY interrupt
						Stop_Read_Data_Continuous();			// SDATAC command

			   	   		ECG_Recoder_state.state = IDLE_STATE;	// Switch to Idle state


						__delay_cycles(500);

						/* Set SPI clock to 1 MHz */

						UCA1CTLW0 = UCSWRST;
						UCA1CTLW0 |= UCMST | UCSYNC | UCMSB;
						UCA1CTLW0 |= UCSSEL__ACLK;
						UCA1BR0 = 0x02;                           // /2
						UCA1BR1 = 0;
						UCA1MCTLW = 0;                            // No modulation
						UCA1CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**


						__delay_cycles(500);
						//timeCtr =199;

						/* Construct 7 byte packet */
						ECGTxPacket[0] = START_DATA_HEADER;	//Start of packet
						ECGTxPacket[1] = ACQUIRE_DATA_COMMAND;	// Acquire header
						ECGTxPacket[2] = (unsigned char)((ReqSamples >> 8) & 0xFF);
						ECGTxPacket[3] = (unsigned char)(ReqSamples & 0xFF);
						ECGTxPacket[4] = END_DATA_HEADER;
						ECGTxPacket[5] = END_DATA_HEADER;		//endof packet
						ECGTxPacket[6] = '\n';
//Save the data to FRAM.
//Ricky					cdc_sendDataInBackground((BYTE*)&ECGTxPacket,7,0,0);          // Send the response over USB

						Recorder_head = 0;					// Reset Head
						Recorder_tail =0;					// Reset tail
//						Acquire_NANDAddress.usBlockNum = 5;	//Set Memory block
//						Acquire_NANDAddress.ucPageNum = 0;	// Set Memory page
//						Acquire_NANDAddress.usColNum = 0;	// Set Memory colum
						AcqpacketCounter = 0;				// Initialize pointer
						/* Send stored data from memory*/
//						Send_Acquired_ECG_Samples_to_USB(ReqSamples);
					}	//if ( NumPackets == 0)
				}	//while ( Recorder_tail != Recorder_head)
		}	// if ((ADS1x9xRegVal[1] & 0x07) == 6)
		else
		{
			//You add the plus 4 because the packet must have the START_DATA_HEADER, ACQUIRE_DATA_PACKET, Status, Status
		   	ECGPacketAcqPrt = &ECGTxPacket[NumFrames * ECG_DATA_PACKET_LENGTH + 4]; //Packet pointer
			ptr = &ECGRecorder_data_Buf[(Recorder_tail << 3)+ 2];	//pointer for data read from circular buffer

		  	*ECGPacketAcqPrt++ = *ptr++;			//CH0 D15-D8
		  	*ECGPacketAcqPrt++ = *ptr++;			//CH0 D15-D8
		  	*ECGPacketAcqPrt++ = *ptr++;			//CH0 D7-D0
		  	*ECGPacketAcqPrt++ = *ptr++;			//CH1 D23-D16
		  	*ECGPacketAcqPrt++ = *ptr++;			//CH1 D15-D8
		  	*ECGPacketAcqPrt++ = *ptr++;			//CH1 D7-D0

			Recorder_tail++;						// Increment tail
			if ( Recorder_tail == 32)				// Reset tail after 32 samples
				Recorder_tail = 0;

			NumFrames++;							// increment packet counter
			if (NumFrames == 8)
			{
				NumFrames = 0;						// Reset packet counter

			/* After every 8 samples send  ECG data to USB port */

				*ECGPacketAcqPrt++ = END_DATA_HEADER;			// End-of-packet
				//*ECGPacketAcqPrt++ = '\n';					//

				ECGTxPacket[0]= START_DATA_HEADER;				//Start-of-packet.
				ECGTxPacket[1]= ACQUIRE_DATA_PACKET;			// Packet header

				ECGTxPacket[2]= ECGRecorder_data_Buf[Recorder_tail<<3];			//Status
				ECGTxPacket[3]= ECGRecorder_data_Buf[(Recorder_tail <<3) + 1];	//Status

				NumPackets -= 8;					// Reduce numer of samples by 8


				//ECG_ACQUIRE_PACKET_LENGTH = 56;
				/* Send packet of 8 smples in packet of length ECG_ACQUIRE_PACKET_LENGTH */
//Ricky			cdc_sendDataInBackground((BYTE*)&ECGTxPacket,ECG_ACQUIRE_PACKET_LENGTH,0,0);          // Send the response over USB


				if ( NumPackets == 0)
				{
				/* Terminate Acquired data after specified sample count*/

					Disable_ADS1x9x_DRDY_Interrupt();		// Disable DRDY interrupt
					Stop_Read_Data_Continuous();			// SDATAC command
					__delay_cycles(500);

					/* Set SPI Clock to 1 MHz */
//	   	   		UCB0CTL1 |= UCSWRST;               		// Enable SW reset
//				UCB0CTL0 |= UCMSB+UCMST+UCSYNC;			//[b0]   1 -  Synchronous mode
															//[b2-1] 00-  3-pin SPI
															//[b3]   1 -  Master mode
															//[b4]   0 - 8-bit data
															//[b5]   1 - MSB first
															//[b6]   0 - Clock polarity low.
															//[b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

//				UCB0CTL1 |= UCSSEL__ACLK;              	// ACLK
//				UCB0BR0 = 24;                           // 1 MHz
//				UCB0BR1 = 0;                            //
//				UCB0CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation
					__delay_cycles(500);
//				timeCtr =199;
		   	   		ECG_Recoder_state.state = IDLE_STATE;	// Switch to Idle state
				}	//if ( NumPackets == 0)
			}	//if (NumFrames == 8)
		}	// else of if ((ADS1x9xRegVal[1] & 0x07) == 6)
   }	// if ( Recorder_head != Recorder_tail)
   
} // void Accquire_ECG_Samples(void)

int Stream_ECG_data_packets(void)
{
uint8_t StreamCount = 0, ucLoopCnt;
static uint8_t sampleCNT = 0;
#define PACK_NUM_SAMPLES 14
   if (ECG_Data_rdy == 1 )
   {
	   ECG_Data_rdy = 0;
	   ADS1x9x_Filtered_ECG();
	   if ( sampleCNT == 0)
	   {
	   	   StreamCount = 0;
	   	   //FileTextE105[StreamCount++]= 0x00;	// Packet start Header
	   	   char header = 0x00;
	   	   ring_buffer_put(_outputrb_d,&header);
	   	   //FileTextE105[StreamCount++]= 0x3F;	// Live ECG Streaming Header
	   	   char liveECGHeader = 0x3F;
	   	   ring_buffer_put(_outputrb_d,&liveECGHeader);


	   	   //FileTextE105[StreamCount++]= START_DATA_HEADER;	//0x02			// Packet start Header
	   	   char start_data_header = START_DATA_HEADER;
	   	   ring_buffer_put(_outputrb_d,&start_data_header);
	   	   //FileTextE105[StreamCount++]= DATA_STREAMING_PACKET;	//0x93		// Live ECG Streaming Header
	   	   char liveECGStreamingHeader = DATA_STREAMING_PACKET;
	   	   ring_buffer_put(_outputrb_d,&liveECGStreamingHeader);

	   	   LeadStatus = 0x00;
		   ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] & 0x0F8000;
		   ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] >> 15;
		   LeadStatus = (unsigned char ) ADS1x9x_ECG_Data_buf[0];

		   // Set the Current Heart rate//
		   //FileTextE105[StreamCount++] = QRS_Heart_Rate;					// Heart Rate
		   char heartRate = QRS_Heart_Rate;
		   ring_buffer_put(_outputrb_d,&heartRate);

		   // Set the current Leadoff status//
		   //FileTextE105[StreamCount++] = Respiration_Rate;				// Respiration Rate
		   char respRate = Respiration_Rate;
		   ring_buffer_put(_outputrb_d,&respRate);

		   //FileTextE105[StreamCount++] = LeadStatus ;					// Lead Status
		   ring_buffer_put(_outputrb_d,&LeadStatus);
	   }
	   if ( sampleCNT > PACK_NUM_SAMPLES) sampleCNT =0;
	   StreamCount = sampleCNT << 2;									// Get Packet pointer
	   StreamCount+=5;													// Offset of 5 bytes header
	   for ( ucLoopCnt =0 ; ucLoopCnt < 2; ucLoopCnt++)
	   {
		   //FileTextE105[StreamCount++]= (ECGFilteredData[ucLoopCnt] & 0x00FF);			// High Byte B15-B8
		   unsigned char highByte = (ECGFilteredData[ucLoopCnt]&0x00FF);
		   ring_buffer_put(_outputrb_d,&highByte);
	       //FileTextE105[StreamCount++]= (ECGFilteredData[ucLoopCnt] & 0xFF00) >> 8 ;		// Low byte B7-B0
		   unsigned char lowByte = (ECGFilteredData[ucLoopCnt]&0xFF00) >> 8;
		   ring_buffer_put(_outputrb_d,&lowByte);
	   }
	   sampleCNT++;
	   if ( sampleCNT == PACK_NUM_SAMPLES)
	   {
		   sampleCNT = 0;
		   //FileTextE105[StreamCount++]= END_DATA_HEADER;	// Packet end header
		   unsigned char endDataHeader = END_DATA_HEADER;
		   ring_buffer_put(_outputrb_d,&endDataHeader);
		   //FileTextE105[StreamCount++]= '\n';
		   unsigned char newline = 0x0D;
		   ring_buffer_put(_outputrb_d,&newline);
		   ECGTxPacketRdy = TRUE;						// Set packet ready flag after every 14th sample.
		   //I'm gonna leave this flag in place, but rather issue a UART_TX event to empty the output buffer to the uart
		   PUBLISH_EVENT(EV_UART_TX);
		   //


		   ECGTxCount = StreamCount;					// Define number of bytes to send as 54.

		   return 1;
	   }
	}
   return 0;
}


