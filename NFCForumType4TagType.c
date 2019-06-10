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
//! \file   NFCForumType4TagType.c
//!
//! \brief  Please see I2CInterface.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#include "NFCForumType4TagType.h"
#include "RF430CL331.h"
#include <stdint.h>
#include "msp430.h"
#include "types.h"
#include "StateMachine.h"
#include "ADS1x9x_USB_Communication.h"
#include "I2CInterface.h"
#include "TemperatureInterface.h"
#include "GSRInterface.h"
#include "AccelerometerInterface.h"

#pragma NOINIT(NdefFiles)
struct NdefFile_Type NdefFiles[8];
unsigned int NumberOfFiles = 0;
const unsigned int E104_Length;
//volatile unsigned int i=0;

unsigned int propriRead = 0;

extern BOOL commandFileWritten;
extern void StateMachine_processTracingWriteEventRxFifo(EVENTS_PROCESS_TRACING data);

//To detect and access NFC-Forum-defined data, the NFC Forum Device retrieves and uses the Capability
//Container (CC) file contained inside the NDEF Tag Application. The CC file contains management data
//and it is stored inside a read-only EF file (see [ISO/IEC_7816-4]). The NFC Forum Device SHALL accept
//NDEF Tag Applications having a CC file with a file identifier equal to E103h.
const uint8_t CCFileText[100] = 	{
//Alex							0x00, 0x0F,	/* CCLEN */


		/* CCLEN - 2 Bytes -Offset 00h - Indicates the size of this capability container
		 * (including this field).*/
//		0x00, 0x17,
		//0x00, 0x1F,
//		0x00, 0x27,
		0x00, 0x2F,
		/* Mapping version - 1 Byte - Offset 02h - Indicates the mapping specification version
		 * it is compliant to (see Section 5.1.1). The most significant nibble (the 4 most
		 * significant bits) SHALL indicate the major version number, and the least significant
		 * nibble (the 4 least significant bits) SHALL indicate the minor version number. */
		0x20,

		/* MLe (bytes); Maximum R-APDU data size - 2 Bytes - Offset 03h - Defines the maximum data size
		 * that can be read from the Type 4 Tag using a single ReadBinary command. [RQ_T4T_NDA_004]
		 * The valid values are MLe = 000Fh-FFFFh. The values between 0000h-0000Eh are RFU.*/
		0x00, 0xF9,	/* MLe (249 bytes); Maximum R-APDU data size */

		/*MLc (bytes); Maximum C-APDU data size - 2 Bytes - Offset 05h - Defines the maximum data
		 * size that can be sent to the Type 4 Tag using a single UpdateBinary command. [RQ_T4T_NDA_005]
		 * The valid range is MLc = 0001h-FFFFh bytes. The value 0000h is RFU.*/
		0x00, 0xF6, /* MLc (246 bytes); Maximum C-APDU data size */


			/********************************************************************************************************
			 * One NDEF File Control TLV SHALL be present at offset 0007h. The NDEF File Control TLV is always present
			 * inside the CC file and it provides control information about the EF file containing the NDEF message
			 * (see Section 5.2). The NFC Forum Device SHALL be able to read and process the NDEF File Control TLV.
			 * The NFC Forum Device SHALL check that the CC file contains an NDEF File Control TLV at offset 0007h.
			 *
			 * TLV Block name - NDEF File Control TLV
			 * Bytes Size - 8 Bytes -
			 * Offset - 07h
			 *
			 * Tag Field Value - 04h
			 * Lenght FIeld Value - 06h
			 * Value Field - 6 Bytes.
			 * Short Description - Contains control information concerning the EF file containing the NDEF message
			*********************************************************************************************************/


			/* T The tag field identifies the type of the TLV block and consists of a single byte encoding a
			 * number from 00h to FEh. The tag field values from 00h to 03h and from 06h to FFh are RFU by
			 * the NFC Forum.*/
			0x04, 		/* Tag, File Control TLV (4 = NDEF file) */

			/* L The length field provides the size in bytes of the value field. It has two different formats
			 * composed of one or three bytes. The NFC Forum Device SHALL implement both length field formats
			 * as shown in Figure 1. However, depending on the tag field value, the length field may not be
			 * present.
			 *
			 * One-byte format: The NFC Forum Device SHALL use the one-byte format to code the length of the
			 * value field between 00h and FEh bytes. The NFC Forum Device SHALL interpret this byte as a
			 * cardinal if the value is between 00h and FEh. If it contains FFh, the NFC Forum Device SHALL
			 * interpret the value as a flag that specifies that the length field is composed of more than
			 * one byte.*/
			0x06, 		/* Length, File Control TLV (6 = 6 bytes of data for this tag) */


					/**********************************************************************************************************
					 * V is composed of 6 bytes that specify size, read access conditions, write access conditions, and the
					 * EF identifier of the EF file containing the NDEF message. If the length field is equal to 00h or there
					 * is no length field, the value field is not present (i.e. the TLV block is empty). If there is a length
					 * field and it indicates a length of the value field N bigger than zero (N>0), the value field consists
					 * of N consecutive bytes.The 6 bytes are encoded as follows:
					 * File Identifier - 2 bytes
					 * Maximum NDEF file size
					 * NDEF file read access condition
					 * NDEF file write access condition
					************************************************************************************************************/


					/* File Identifier indicates a valid NDEF file. The valid ranges are 0001h to E101h, E104h to 3EFFh, 3F01h to
					 * 3FFEh and 4000h to FFFEh. The values 0000h, E102h, E103h, 3F00h and 3FFFh are reserved and FFFFh is RFU.*/
					0xE1, 0x04,
					//0x2F, 0x44, /* Max NDEF size (12100 bytes of useable memory) */

					/* Maximum NDEF file size, 2 bytes. Maximum size in bytes of the NDEF file. This size does not reflect
					 * the size of the contained NDEF message as such but rather the size of the file containing the NDEF
					 * message. The valid range is 0005h to FFFEh. The values 0000h-0004h and FFFFh are RFU.
					 * Max NDEF size - 8990 bytes of useable memory */
					0x23, 0x1E,

					/* NDEF file read access condition, 1 byte:
					 ? 00h indicates read access granted without any security
					 ? 01h to 7Fh and FFh are RFU
					 ? 80h to FEh are proprietary */
					0x00, 		/* NDEF file read access condition, read access without any security */

					/* NDEF file write access condition, 1 byte:
						? 00h indicates write access granted without any security
						? FFh indicates no write access granted at all (read-only)
						? 01h to 7Fh are RFU
						? 80h to FEh are proprietary */
					0x00, 		/* NDEF file write access condition; write access without any security */


			/**********************************************************************************************************
			 * The CC file contains zero, one, or more Proprietary File Control TLV block. The Proprietary File
			 * Control TLV blocks MAY be present from offset 000Fh
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Lenght FIeld Value - 06h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an
			 * EF file containing proprietary data
			************************************************************************************************************/

			/* T The tag field identifies the type of the TLV block and consists of a single byte encoding a
			 * number from 00h to FEh. The tag field values from 00h to 03h and from 06h to FFh are RFU by
			 * the NFC Forum.
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an EF file
			 * 					   containing proprietary data 															*/
			0x05, 		/* Tag, File Control TLV (4 = NDEF file) */

			/* L The length field provides the size in bytes of the value field. It has two different formats
			 * composed of one or three bytes. The NFC Forum Device SHALL implement both length field formats
			 * as shown in Figure 1. However, depending on the tag field value, the length field may not be
			 * present.
			 * One-byte format: The NFC Forum Device SHALL use the one-byte format to code the length of the
			 * value field between 00h and FEh bytes. The NFC Forum Device SHALL interpret this byte as a
			 * cardinal if the value is between 00h and FEh. If it contains FFh, the NFC Forum Device SHALL
			 * interpret the value as a flag that specifies that the length field is composed of more than
			 * one byte.*/
			0x06, 		/* Length, File Control TLV (6 = 6 bytes of data for this tag) */

					/**********************************************************************************************************
					 * V is composed of 6 bytes that specifies size, read access conditions and write access conditions, and EF
					 * identifier of the EF file containing the proprietary data. The 6 bytes are encoded as follows:
					 * File Identifier
					 * Maximum Proprietary file size
					 * Proprietary file read access condition
					 * Proprietary file write access condition
					************************************************************************************************************/

					/* File Identifier, 2 bytes. Indicates a valid Proprietary file. The valid ranges are 0001h to E101h, E104h
					 * to 3EFFh, 3F01h to 3FFEh, and 4000h to FFFEh. The values 0000h, E102h, E103h, 3F00h, and 3FFFh are reserved
					 * and FFFFh is RFU.*/
					0xE1, 0x05,	/* File Identifier */
					//0x2F, 0x44, /* Max NDEF size (12100 bytes of useable memory) */



					/* Maximum Proprietary file size, 2 bytes. Maximum size in bytes of the Proprietary file. The valid range is
					 * 0003h-FFFEh. The value FFFFh is RFU.
					 * Max NDEF size (8990 bytes of useable memory)
					 * */
					0x23, 0x1E,

					/* Proprietary file read access condition, 1 byte:
					 * ? 00h indicates read access granted without any security
					   ? 01h to 7Fh and FFh are RFU
					   ? 80h to FEh are proprietary */
					0x00,

					/* Proprietary file write access condition, 1 byte:
						? 00h indicates write access granted without any security
						? FFh indicates no write access granted at all (read-only)
						? 01h to 7Fh are RFU
						? 80h to FEh are proprietary*/
					0x00,

			/**********************************************************************************************************
			 * The CC file contains zero, one, or more Proprietary File Control TLV block. The Proprietary File
			 * Control TLV blocks MAY be present from offset 000Fh
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Lenght FIeld Value - 06h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an
			 * EF file containing proprietary data
			************************************************************************************************************/

			/* T The tag field identifies the type of the TLV block and consists of a single byte encoding a
			 * number from 00h to FEh. The tag field values from 00h to 03h and from 06h to FFh are RFU by
			 * the NFC Forum.
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an EF file
			 * 					   containing proprietary data 															*/
			0x05, 		/* Tag, File Control TLV (4 = NDEF file) */

			/* L The length field provides the size in bytes of the value field. It has two different formats
			 * composed of one or three bytes. The NFC Forum Device SHALL implement both length field formats
			 * as shown in Figure 1. However, depending on the tag field value, the length field may not be
			 * present.
			 * One-byte format: The NFC Forum Device SHALL use the one-byte format to code the length of the
			 * value field between 00h and FEh bytes. The NFC Forum Device SHALL interpret this byte as a
			 * cardinal if the value is between 00h and FEh. If it contains FFh, the NFC Forum Device SHALL
			 * interpret the value as a flag that specifies that the length field is composed of more than
			 * one byte.*/
			0x06, 		/* Length, File Control TLV (6 = 6 bytes of data for this tag) */

					/**********************************************************************************************************
					 * V is composed of 6 bytes that specifies size, read access conditions and write access conditions, and EF
					 * identifier of the EF file containing the proprietary data. The 6 bytes are encoded as follows:
					 * File Identifier
					 * Maximum Proprietary file size
					 * Proprietary file read access condition
					 * Proprietary file write access condition
					************************************************************************************************************/

					/* File Identifier, 2 bytes. Indicates a valid Proprietary file. The valid ranges are 0001h to E101h, E104h
					 * to 3EFFh, 3F01h to 3FFEh, and 4000h to FFFEh. The values 0000h, E102h, E103h, 3F00h, and 3FFFh are reserved
					 * and FFFFh is RFU.*/
					0xE1, 0x06,	/* File Identifier */
					//0x2F, 0x44, /* Max NDEF size (12100 bytes of useable memory) */



					/* Maximum Proprietary file size, 2 bytes. Maximum size in bytes of the Proprietary file. The valid range is
					 * 0003h-FFFEh. The value FFFFh is RFU.
					 * Max NDEF size (8990 bytes of useable memory)
					 * */
					0x23, 0x1E,

					/* Proprietary file read access condition, 1 byte:
					 * ? 00h indicates read access granted without any security
					   ? 01h to 7Fh and FFh are RFU
					   ? 80h to FEh are proprietary */
					0x00,

					/* Proprietary file write access condition, 1 byte:
						? 00h indicates write access granted without any security
						? FFh indicates no write access granted at all (read-only)
						? 01h to 7Fh are RFU
						? 80h to FEh are proprietary*/
					0x00,

			/**********************************************************************************************************
			 * The CC file contains zero, one, or more Proprietary File Control TLV block. The Proprietary File
			 * Control TLV blocks MAY be present from offset 000Fh
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Lenght FIeld Value - 06h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an
			 * EF file containing proprietary data
			************************************************************************************************************/

			/* T The tag field identifies the type of the TLV block and consists of a single byte encoding a
			 * number from 00h to FEh. The tag field values from 00h to 03h and from 06h to FFh are RFU by
			 * the NFC Forum.
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an EF file
			 * 					   containing proprietary data 															*/
			0x05, 		/* Tag, File Control TLV (4 = NDEF file) */

			/* L The length field provides the size in bytes of the value field. It has two different formats
			 * composed of one or three bytes. The NFC Forum Device SHALL implement both length field formats
			 * as shown in Figure 1. However, depending on the tag field value, the length field may not be
			 * present.
			 * One-byte format: The NFC Forum Device SHALL use the one-byte format to code the length of the
			 * value field between 00h and FEh bytes. The NFC Forum Device SHALL interpret this byte as a
			 * cardinal if the value is between 00h and FEh. If it contains FFh, the NFC Forum Device SHALL
			 * interpret the value as a flag that specifies that the length field is composed of more than
			 * one byte.*/
			0x06, 		/* Length, File Control TLV (6 = 6 bytes of data for this tag) */

					/**********************************************************************************************************
					 * V is composed of 6 bytes that specifies size, read access conditions and write access conditions, and EF
					 * identifier of the EF file containing the proprietary data. The 6 bytes are encoded as follows:
					 * File Identifier
					 * Maximum Proprietary file size
					 * Proprietary file read access condition
					 * Proprietary file write access condition
					************************************************************************************************************/

					/* File Identifier, 2 bytes. Indicates a valid Proprietary file. The valid ranges are 0001h to E101h, E104h
					 * to 3EFFh, 3F01h to 3FFEh, and 4000h to FFFEh. The values 0000h, E102h, E103h, 3F00h, and 3FFFh are reserved
					 * and FFFFh is RFU.*/
					0xE1, 0x07,	/* File Identifier */
					//0x2F, 0x44, /* Max NDEF size (12100 bytes of useable memory) */



					/* Maximum Proprietary file size, 2 bytes. Maximum size in bytes of the Proprietary file. The valid range is
					 * 0003h-FFFEh. The value FFFFh is RFU.
					 * Max NDEF size (8990 bytes of useable memory)
					 * */
					0x23, 0x1E,

					/* Proprietary file read access condition, 1 byte:
					 * ? 00h indicates read access granted without any security
					   ? 01h to 7Fh and FFh are RFU
					   ? 80h to FEh are proprietary */
					0x00,

					/* Proprietary file write access condition, 1 byte:
						? 00h indicates write access granted without any security
						? FFh indicates no write access granted at all (read-only)
						? 01h to 7Fh are RFU
						? 80h to FEh are proprietary*/
					0x00,

			/**********************************************************************************************************
			 * The CC file contains zero, one, or more Proprietary File Control TLV block. The Proprietary File
			 * Control TLV blocks MAY be present from offset 000Fh
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Lenght FIeld Value - 06h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an
			 * EF file containing proprietary data
			************************************************************************************************************/

			/* T The tag field identifies the type of the TLV block and consists of a single byte encoding a
			 * number from 00h to FEh. The tag field values from 00h to 03h and from 06h to FFh are RFU by
			 * the NFC Forum.
			 * TLV block name - Proprietary File Control TLV
			 * Tag Field Value - 05h
			 * Short Description - Contains control information concerning a ?Proprietary file?, which is an EF file
			 * 					   containing proprietary data 															*/
			0x05, 		/* Tag, File Control TLV (4 = NDEF file) */

			/* L The length field provides the size in bytes of the value field. It has two different formats
			 * composed of one or three bytes. The NFC Forum Device SHALL implement both length field formats
			 * as shown in Figure 1. However, depending on the tag field value, the length field may not be
			 * present.
			 * One-byte format: The NFC Forum Device SHALL use the one-byte format to code the length of the
			 * value field between 00h and FEh bytes. The NFC Forum Device SHALL interpret this byte as a
			 * cardinal if the value is between 00h and FEh. If it contains FFh, the NFC Forum Device SHALL
			 * interpret the value as a flag that specifies that the length field is composed of more than
			 * one byte.*/
			0x06, 		/* Length, File Control TLV (6 = 6 bytes of data for this tag) */

					/**********************************************************************************************************
					 * V is composed of 6 bytes that specifies size, read access conditions and write access conditions, and EF
					 * identifier of the EF file containing the proprietary data. The 6 bytes are encoded as follows:
					 * File Identifier
					 * Maximum Proprietary file size
					 * Proprietary file read access condition
					 * Proprietary file write access condition
					************************************************************************************************************/

					/* File Identifier, 2 bytes. Indicates a valid Proprietary file. The valid ranges are 0001h to E101h, E104h
					 * to 3EFFh, 3F01h to 3FFEh, and 4000h to FFFEh. The values 0000h, E102h, E103h, 3F00h, and 3FFFh are reserved
					 * and FFFFh is RFU.*/
					0xE1, 0x08,	/* File Identifier */
					//0x2F, 0x44, /* Max NDEF size (12100 bytes of useable memory) */



					/* Maximum Proprietary file size, 2 bytes. Maximum size in bytes of the Proprietary file. The valid range is
					 * 0003h-FFFEh. The value FFFFh is RFU.
					 * Max NDEF size (8990 bytes of useable memory)
					 * */
					0x23, 0x1E,

					/* Proprietary file read access condition, 1 byte:
					 * ? 00h indicates read access granted without any security
					   ? 01h to 7Fh and FFh are RFU
					   ? 80h to FEh are proprietary */
					0x00,

					/* Proprietary file write access condition, 1 byte:
						? 00h indicates write access granted without any security
						? FFh indicates no write access granted at all (read-only)
						? 01h to 7Fh are RFU
						? 80h to FEh are proprietary*/
					0x00


};//CC file text



/* This is a NDEF file. It contains NLEN and NDEF message*/
#pragma PERSISTENT(FileTextE104)
uint8_t FileTextE104[1000] = {
		0x00, 0x15, /* NLEN; NDEF length (3 byte long message) */
			/************ This is the NDEF message that is stored inside a NDEF file. ***************/
			0xD1, 0x01, 0x11,
			0x54, /* T = text */
			0x02, /*Starting here is the payload*/
			0x65, 0x6E, /* 'e', 'n', */
			/* 'Hello, world!' NDEF data; Empty NDEF message, length should match NLEN*/
			0x48, 0x48, 0x65, 0x6C, 0x6C, 0x6f, 0x2c, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x21
			/************** This is the message that is stored inside a NDEF file. ***************/
};

/* COMMAND FILE */
/* This is a proprietary a file.It contains PLEN and Proprietary message.*/
/* This would be the command file*/
#pragma NOINIT(FileTextE105)
uint8_t FileTextE105[13];
//= {
//
//		/* PLEN */
//		0x00, /*PLEN Low Byte*/
//		0x0B, /*PLEN High Byte*/
//
//		/* Proprietary Message */
//		PATCH_NOT_BUSY, //Busy?
//		0x00, //START_DATA_HEADER
//		0x00, //COMMAND
//		0x00, //Command parameter #1
//		0x00, //Command parameter #2
//		0x00, //Command parameter #3
//		0x00, //Command parameter #4
//		END_DATA_HEADER, //END_DATA_HEADER
//		END_DATA_HEADER, //END_DATA_HEADER
//		0x00, //'\n';
//		NCK //Smart Patch ACK response.
//
//};

/* TEMPERATURE FILE */
/* This is a proprietary a file.It contains PLEN and Proprietary message.*/
/* This file will contain the output of the cadence algorithm. */
#pragma NOINIT(FileTextE106)
uint8_t FileTextE106[2000];
//= {
//		/* PLEN */
//		0x00, /* HIGH BYTE */
//		0x1E, /* LOW BYTE */
//
//		/* Proprietary Message */
//		PATCH_NOT_BUSY,
//		START_DATA_HEADER,
//		TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_RESPONSE_COMMAND,
//		0x00,
//		0x00,
//		TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_NOT_DONE,
//		0x00,
//		0x00,
//		0x00
//
//};


/* This is a proprietary a file.It contains PLEN and Proprietary message.*/
/* This file will contain the output of the cadence algorithm. */
#pragma NOINIT(FileTextE107)
uint8_t FileTextE107[2000];
//= {
//		/* PLEN */
//		0x00,
//		0x09, /* NLEN; NDEF length (3 byte long message) */
//
//		/* Proprietary Message */
//		PATCH_NOT_BUSY,
//		START_DATA_HEADER,
//		GSR_INTERFACE_ACQUIRE_GSR_RESPONSE_COMMAND,
//		0x00,
//		0x00,
//		GSR_INTERFACE_ACQUIRE_GSR_NOT_DONE,
//		0x00,
//		0x00,
//		0x00
//
//};


/* This is a proprietary a file.It contains PLEN and Proprietary message.*/
/* This file will contain the output of the cadence algorithm. */
#pragma NOINIT(FileTextE108)
uint8_t FileTextE108[2000];
//= {
//		/* PLEN */
//		0x00,
//		0x09, /* NLEN; NDEF length (3 byte long message) */
//
//		/* Proprietary Message */
//		PATCH_NOT_BUSY,
//		START_DATA_HEADER,
//		ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_RESPONSE_COMMAND,
//		0x00,
//		0x00,
//		ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_NOT_DONE,
//		0x00,
//		0x00,
//		0x00
//
//};


unsigned int SelectedFile;		//the file that is currently selected

void AppInit()
{
	//The NFC Forum Device SHALL accept NDEF Tag Applications having a CC file with a file identifier equal to E103h.
	// Init CC file info
	NdefFiles[0].FileID[0] = 0xE1;
	NdefFiles[0].FileID[1] = 0x03;
	NdefFiles[0].FilePointer = (unsigned char *)CCFileText;
	NdefFiles[0].FileLength = 0;		//?


	// Init Ndef file info
	NdefFiles[1].FileID[0] = 0xE1;
	NdefFiles[1].FileID[1] = 0x04;
	NdefFiles[1].FilePointer = (unsigned char *)FileTextE104;
	//NdefFiles[1].FileLength = 0;		//?

	// Init Ndef file info
	NdefFiles[2].FileID[0] = 0xE1;
	NdefFiles[2].FileID[1] = 0x05;
	NdefFiles[2].FilePointer = (unsigned char *)FileTextE105;
	//NdefFiles[1].FileLength = 0;		//?

	// Init Ndef file info
	NdefFiles[3].FileID[0] = 0xE1;
	NdefFiles[3].FileID[1] = 0x06;
	NdefFiles[3].FilePointer = (unsigned char *)FileTextE106;
	//NdefFiles[1].FileLength = 0;		//?


	// Init Ndef file info
	NdefFiles[4].FileID[0] = 0xE1;
	NdefFiles[4].FileID[1] = 0x07;
	NdefFiles[4].FilePointer = (unsigned char *)FileTextE107;
	//NdefFiles[1].FileLength = 0;		//?

	// Init Ndef file info
	NdefFiles[5].FileID[0] = 0xE1;
	NdefFiles[5].FileID[1] = 0x08;
	NdefFiles[5].FilePointer = (unsigned char *)FileTextE108;
	//NdefFiles[1].FileLength = 0;		//?


	NumberOfFiles = 6; 			//the number if NDEF files available
	SelectedFile = 0;			//default to CC file



	/* PLEN */
	FileTextE105[0] = 0x00; /*PLEN Low Byte*/
	FileTextE105[1] = 0x0B; /*PLEN High Byte*/
	/* Proprietary Message */
	FileTextE105[2] = PATCH_NOT_BUSY; //Busy?
	FileTextE105[3] = 0x00; //START_DATA_HEADER
	FileTextE105[4] = 0x00; //COMMAND
	FileTextE105[5] = 0x00; //Command parameter #1
	FileTextE105[6] = 0x00; //Command parameter #2
	FileTextE105[7] = 0x00; //Command parameter #3
	FileTextE105[8] = 0x00; //Command parameter #4
	FileTextE105[9] = END_DATA_HEADER; //END_DATA_HEADER
	FileTextE105[10] = END_DATA_HEADER; //END_DATA_HEADER
	FileTextE105[11] = 0x00; //'\n';
	FileTextE105[12] = NCK; //Smart Patch ACK response.

	/* PLEN */
	FileTextE106[0] = 0x00; /* HIGH BYTE */
	FileTextE106[1] = 0x1E; /* LOW BYTE */
	/* Proprietary Message */
	FileTextE106[2] = PATCH_NOT_BUSY;
	FileTextE106[3] = START_DATA_HEADER;
	FileTextE106[4] = TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_RESPONSE_COMMAND;
	FileTextE106[5] = 0x00;
	FileTextE106[6] = 0x00;
	FileTextE106[7] = TEMPERATURE_INTERFACE_ACQUIRE_TEMPERATURE_NOT_DONE;
	FileTextE106[8] = 0x00;
	FileTextE106[9] = 0x00;
	FileTextE106[10] = 0x00;

	/* PLEN */
	FileTextE107[0] = 0x00;
	FileTextE107[1] = 0x09; /* NLEN; NDEF length (3 byte long message) */
	/* Proprietary Message */
	FileTextE107[2] = PATCH_NOT_BUSY;
	FileTextE107[3] = START_DATA_HEADER;
	FileTextE107[4] = GSR_INTERFACE_ACQUIRE_GSR_RESPONSE_COMMAND;
	FileTextE107[5] = 0x00;
	FileTextE107[6] = 0x00;
	FileTextE107[7] = GSR_INTERFACE_ACQUIRE_GSR_NOT_DONE;
	FileTextE107[8] = 0x00;
	FileTextE107[9] = 0x00;
	FileTextE107[10] = 0x00;

	/* PLEN */
	FileTextE108[0] = 0x00;
	FileTextE108[1] = 0x09; /* NLEN; NDEF length (3 byte long message) */
	/* Proprietary Message */
	FileTextE108[2] = PATCH_NOT_BUSY;
	FileTextE108[3] = START_DATA_HEADER;
	FileTextE108[4] = ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_RESPONSE_COMMAND;
	FileTextE108[5] = 0x00;
	FileTextE108[6] = 0x00;
	FileTextE108[7] = ACCELERATION_INTERFACE_ACQUIRE_ACCELERATION_NOT_DONE;
	FileTextE108[8] = 0x00;
	FileTextE108[9] = 0x00;
	FileTextE108[10] = 0x00;



}


void ServiceInterrupt(unsigned int flags)
{
	unsigned int interrupt_serviced = 0;

	if(flags & GENERIC_ERROR_INT_FLAG) 									//check if the there was a reset on the RF430
	{
		interrupt_serviced |= GENERIC_ERROR_INT_FLAG;
		I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); 				//clear this flag
	}

	if(flags & FIELD_REMOVED_INT_FLAG) 									//check if the tag was removed
	{
		interrupt_serviced |= FIELD_REMOVED_INT_FLAG;					//clear this flag later
		I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); //ACK the flags to clear
	}

	if(flags & DATA_TRANSACTION_INT_FLAG) //check if the tag was read
	{
		unsigned int status;
		unsigned int ret;
		//2,8,18,24,34, 44
		status = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, STATUS_REG, 2, 2); //read the Transponder status register to determine the nature of the interrupt

		switch (status & APP_STATUS_REGS)
		{
			// NDEF File Select Request is coming from the mobile/reader - response to the request is determined here
			// based on whether the file exists in our file database
			case FILE_SELECT_STATUS:
			{
				unsigned int file_id;
				file_id = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_ID, 2 ,2);						// determine the file identifier of the select request
				ret = SearchForFile((unsigned char *)&file_id);				// determine if the file exists on the host controller
				interrupt_serviced |= DATA_TRANSACTION_INT_FLAG;			//clear this flag later
				if (ret == FileFound)
				{
					I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); 		//ACK the flags to clear
					I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, HOST_RESPONSE, INT_SERVICED_FIELD + FILE_EXISTS_FIELD, 2, 2);			//indicate to the RF430 that the file exist

					if(SelectedFile == 0)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_File_Selected);
					if(SelectedFile == 1)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_File_Selected);
					if(SelectedFile == 2)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_File_Selected);
					else if(SelectedFile == 3)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_File_Selected);
					else if(SelectedFile == 4)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_File_Selected);
					else if(SelectedFile == 5)
					StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_File_Selected);
				}
				else
				{
					I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); 		//ACK the flags to clear
					I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, HOST_RESPONSE, INT_SERVICED_FIELD + FILE_DOES_NOT_EXIST_FIELD, 2, 2);	// the file does not exist
				}
				break;
			}

			//NDEF ReadBinary request has been sent by the mobile / reader
			case FILE_REQUEST_STATUS:
			{
				unsigned int buffer_start;
				unsigned int file_offset;
				unsigned int file_length;

				buffer_start = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_BUFFER_START, 2, 2);		// where to start writing the file info in the RF430 buffer (0-2999)
				file_offset = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_OFFSET, 2, 2);			// what part of the file to start sending
				file_length = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_LENGTH, 2, 2);			// how much of the file starting at offset to send
																		// we can send more than requested, called caching
																		// as long as we write back into the length register how
																		// much we sent it
				interrupt_serviced |= DATA_TRANSACTION_INT_FLAG;					//clear this flag later
				//can have bounds check for the requested length

				if((SelectedFile == 0) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_Read_PLEN);
				else if((SelectedFile == 0) && (file_offset == 0) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_Read_Message);
				if((SelectedFile == 1) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_Read_PLEN);
				else if((SelectedFile == 1) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_Read_Message);
				if((SelectedFile == 2) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_Read_PLEN);
				else if((SelectedFile == 2) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_Read_Message);
				if((SelectedFile == 3) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_Read_PLEN);
				else if((SelectedFile == 3) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_Read_Message);
				if((SelectedFile == 4) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_Read_PLEN);
				else if((SelectedFile == 4) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_Read_Message);
				if((SelectedFile == 5) && (file_offset == 0) && (file_length == 2))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_Read_PLEN);
				else if((SelectedFile == 5) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_Read_Message);

				file_length = SendDataOnFile(SelectedFile, buffer_start, file_offset, file_length);//THis is a write data to FR430 buff command.				//13, 29, 39


				I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_LENGTH, file_length, 2, 2);  		// how much was actually written
				I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); 		// ACK the flags to clear
				I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, HOST_RESPONSE, INT_SERVICED_FIELD, 2, 2);		// indicate that we have service the request

				// custom response code
				// Write_Register(CUSTOM_RESPONSE_REG, 0x6491);  	//error code
				// Write_Register(HOST_RESPONSE, INT_SERVICED_FIELD + CUSTOM_RESPONSE_FIELD);
				break;
			}

			// NDEF UpdateBinary request
			case FILE_AVAILABLE_STATUS:
			{
				unsigned int buffer_start;
				unsigned int file_offset;
				unsigned int file_length;
				interrupt_serviced |= DATA_TRANSACTION_INT_FLAG;			// clear this flag later
				buffer_start = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_BUFFER_START, 2, 2);			// where to start in the RF430 buffer to read the file data (0-2999)
				file_offset = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_OFFSET, 2, 2);				// the file offset that the data begins at
				file_length = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_LENGTH, 2, 2);				// how much of the file is in the RF430 buffer


				//can have bounds check for the requested length
				ReadDataOnFile(SelectedFile, buffer_start, file_offset, file_length);

				if((SelectedFile == 0) && (file_offset == 0) && (file_length == 2) && (FileTextE105[0]== 0x00) && (FileTextE105[1] == 0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_Write_PLEN_Zero);
				else if((SelectedFile == 0) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_Write_Message);
				else if((SelectedFile == 0) && (file_offset == 0) && ((FileTextE105[0] != 0x00) || (FileTextE105[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E103_Write_PLEN);

				if((SelectedFile == 1) && (file_offset == 0) && (file_length == 2) && (FileTextE105[0]== 0x00) && (FileTextE105[1] == 0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_Write_PLEN_Zero);
				else if((SelectedFile == 1) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_Write_Message);
				else if((SelectedFile == 1) && (file_offset == 0) && ((FileTextE105[0] != 0x00) || (FileTextE105[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E104_Write_PLEN);

				if((SelectedFile == 2) && (file_offset == 0) && (file_length == 2) && (FileTextE105[0]== 0x00) && (FileTextE105[1] == 0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_Write_PLEN_Zero);
				else if((SelectedFile == 2) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_Write_Message);
				else if((SelectedFile == 2) && (file_offset == 0) && ((FileTextE105[0] != 0x00) || (FileTextE105[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E105_Write_PLEN);

				if((SelectedFile == 3) && (file_offset == 0) && (file_length == 2) && (FileTextE106[0]==0x00) && (FileTextE106[1]==0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_Write_PLEN_Zero);
				else if((SelectedFile == 3) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_Write_Message);
				else if((SelectedFile == 3) && (file_offset == 0) && ((FileTextE106[0] != 0x00) || (FileTextE106[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E106_Write_PLEN);

				if((SelectedFile == 4) && (file_offset == 0) && (file_length == 2) && (FileTextE107[0]==0x00) && (FileTextE107[1]==0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_Write_PLEN_Zero);
				else if((SelectedFile == 4) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_Write_Message);
				else if((SelectedFile == 4) && (file_offset == 0) && ((FileTextE107[0] != 0x00) || (FileTextE107[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E107_Write_PLEN);

				if((SelectedFile == 5) && (file_offset == 0) && (file_length == 2) && (FileTextE107[0]==0x00) && (FileTextE107[1]==0x00))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_Write_PLEN_Zero);
				else if((SelectedFile == 5) && (file_offset == 2) && (file_length >= 2))
		        StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_Write_Message);
				else if((SelectedFile == 5) && (file_offset == 0) && ((FileTextE107[0] != 0x00) || (FileTextE107[1] != 0x00)))
				StateMachine_processTracingWriteEventRxFifo(EVT_Process_Tracing_E108_Write_PLEN);

				I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2); 			// ACK the flags to clear
				I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, HOST_RESPONSE, INT_SERVICED_FIELD, 2, 2);			// the interrup has been serviced
				__no_operation();


				//custom response code
				// Write_Register(CUSTOM_RESPONSE_REG, 0x6491);  	//error code
				// Write_Register(HOST_RESPONSE, INT_SERVICED_FIELD + CUSTOM_RESPONSE_FIELD);
				break;
			}
		}
	}

	// This is the pre-fetch interrupt
	// If this is enabled with the interrupt enable, the RF430 will request more read data when it is sending the previous
	// ReadBinary request.  The goal is to have enough data in the RF430 buffer for the next ReadBinary request so that
	// it responds the ReadBinary request automatically, with no interrupt.
	else if(flags & EXTRA_DATA_IN_FLAG) //check if the tag was read
	{
		#define AMOUNT_DATA_TO_SENT_EARLY		255			// data to add to the buffer while transmit is happening
		#define MAX_TAG_BUFFER_SPACE			2998		// actually 3000 but to avoid any possibility of an issue

		unsigned int buffer_start;
		unsigned int file_offset;
		unsigned int file_length = 0;

		interrupt_serviced |= EXTRA_DATA_IN_FLAG;							// clear the interrupt flag

		if (SelectedFile == 1)   											// allows only prefetch on NDEF files, no Capability container ones
		{
			buffer_start = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_BUFFER_START, 2, 2);
			file_offset = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_OFFSET, 2, 2);
			//		file_length = Read_RF430_Register(NDEF_FILE_LENGTH);

			if ((buffer_start + AMOUNT_DATA_TO_SENT_EARLY) >= MAX_TAG_BUFFER_SPACE)
			{
				// can't fill the buffer anymore
				// do no fill.  New data request interrupt will come later.
				__no_operation();
			}
			else
			{
				//!!!!range check on file needs to be done here!!!
				//can have bounds check for the requested length
				file_length = SendDataOnFile(SelectedFile, buffer_start, file_offset, AMOUNT_DATA_TO_SENT_EARLY); //255 is enough for atleast one packet
			}
		}
		I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, NDEF_FILE_LENGTH, file_length, 2, 2);  						// how much was actually written
		I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, interrupt_serviced, 2, 2);						// ACK the flags to clear
		I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, HOST_RESPONSE, EXTRA_DATA_IN_SENT_FIELD, 2, 2);				// interrupt was serviced
	}
}

enum FileExistType SearchForFile(unsigned char *fileId)
{
	unsigned int i;
	enum FileExistType ret = FileNotFound; // 0 means not found, 1 mean found

	for (i = 0; i < NumberOfFiles; i++)
	{
		if (NdefFiles[i].FileID[0] == fileId[0] && NdefFiles[i].FileID[1] == fileId[1])
		{
			ret = FileFound;
			SelectedFile = i;  // the index of the selected file in the array
			break;
		}
	}
	return ret;
}

uint16_t SendDataOnFile(unsigned int selectedFile, unsigned int buffer_start, unsigned int file_offset, unsigned int length)
{
	unsigned int ret_length;

	I2CInterface_writeGenericData(IC_ADDRESS_RF430CL331_ADDRESS, buffer_start, (unsigned char *)&NdefFiles[selectedFile].FilePointer[file_offset], length);
	ret_length = length;

	return ret_length;

	//Regs[NDEF_FILE_LENGTH_INDEX]  = Regs[NDEF_FILE_LENGTH_INDEX]
	//do not change right now, we are only sending as much as has been requested
	//if we wanted to send data than requested, we would update the Regs[NDEF_FILE_LENGTH_INDEX] register to a higher value
}

void ReadDataOnFile(unsigned int selectedFile, unsigned int buffer_start, unsigned int file_offset, unsigned int length)
{
	unsigned int * e104_l = (unsigned int *)&E104_Length;

	I2CInterface_readGenericData(IC_ADDRESS_RF430CL331_ADDRESS, buffer_start, (unsigned char *)&NdefFiles[selectedFile].FilePointer[file_offset], length);
	if (NdefFiles[selectedFile].FileLength < (file_offset + length))
	{
		NdefFiles[selectedFile].FileLength = file_offset + length;
		*e104_l = file_offset + length;
	}
	//Regs[NDEF_FILE_LENGTH_INDEX]  = Regs[NDEF_FILE_LENGTH_INDEX]
	//do not change right now, we are only sending as much as has been requested
	//if we wanted to send data than requested, we would update the Regs[NDEF_FILE_LENGTH_INDEX] register to a higher value
//	NdefFiles[selectedFile].FileLength = offset + index;  //not safe here if random update binary writes are done by reader
}
