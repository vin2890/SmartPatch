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
//! \file   RF430CL331.h
//!
//! \brief  This is the IC driver file for the RF430CL331 IC. This IC driver
//! 		file contains function calls which are only found in the I2C
//!			Interface file. If the corresponding (I2C) Interface file is modified,
//!			then this IC driver file must also be modified. Although, in this case
//!			the file is set up to interface to the/a I2C peripheral interface
//!			file, it can be modified to interact with a SPI interface file, UART
//!			interface file, USB interface file, etc. The reason why this IC driver
//!			file is set up to interface with I2C peripheral interface file is
//!			because the RF430CL331 IC uses I2C communication protocol.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef RF430_COMM__H_
#define RF430_COMM__H_

//*****************************************************************************
// the includes
//*****************************************************************************
#include "types.h"

//! \brief This define contains the address of the RF430CL331 IC.
#define IC_ADDRESS_RF430CL331_ADDRESS 	0x0018

//! \brief INTO configuration.
#define PORT_INTO_IN	P4IN
#define PORT_INTO_OUT	P4OUT
#define PORT_INTO_DIR	P4DIR
#define PORT_INTO_SEL0	P4SEL0
#define PORT_INTO_SEL1	P4SEL1
#define PORT_INTO_REN	P4REN
#define PORT_INTO_IE	P4IE
#define PORT_INTO_IES	P4IES
#define PORT_INTO_IE	P4IE
#define PORT_INTO_IFG	P4IFG
#define INTO	BIT1

//! \brief RST configuration.
#define PORT_RST_OUT	P9OUT
#define PORT_RST_DIR	P9DIR
#define PORT_RST_SEL0	P9SEL0
#define PORT_RST_SEL1	P9SEL1
#define RST	BIT3

//! \brief I2C_READY.
#define I2C_READY_IN	P4IN
#define I2C_READY_OUT	P4OUT
#define I2C_READY_DIR	P4DIR
#define I2C_READY_SEL0	P4SEL0
#define I2C_READY_SEL1	P4SEL1
#define I2C_READY_REN	P4REN
#define I2C_READY_IE	P4IE
#define I2C_READY_IES	P4IES
#define I2C_READY_IFG	P4IFG
#define I2C_READY_PIN	BIT0

//! \brief define the values for Granite's registers we want to access.
#define CONTROL_REG 			0xFFFE
#define STATUS_REG				0xFFFC
#define INT_ENABLE_REG			0xFFFA
#define INT_FLAG_REG			0xFFF8
#define CRC_RESULT_REG			0xFFF6
#define CRC_LENGTH_REG			0xFFF4
#define CRC_START_ADDR_REG		0xFFF2
#define COMM_WD_CTRL_REG		0xFFF0
#define VERSION_REG				0xFFEE //contains the software version of the ROM
#define NDEF_FILE_ID			0xFFEC
#define HOST_RESPONSE			0xFFEA
#define NDEF_FILE_LENGTH		0xFFE8
#define NDEF_FILE_OFFSET		0xFFE6
#define NDEF_BUFFER_START		0xFFE4
#define TEST_FUNCTION_REG   	0xFFE2
#define TEST_MODE_REG			0xFFE0
#define SWTX_INDEX				0xFFDE
#define TIMER_DELAY				0xFFDC
#define CUSTOM_RESPONSE_REG		0xFFDA

//! \brief define the different virtual register bits

//! \brief CONTROL_REG bits
#define SW_RESET		BIT0
#define RF_ENABLE		BIT1
#define INT_ENABLE		BIT2
#define INTO_HIGH		BIT3
#define INTO_DRIVE		BIT4
#define BIP8_ENABLE		BIT5
#define STANDBY_ENABLE	BIT6
#define TEST_MODE		BIT7
#define DUPLEX_ON_WRITE	BIT8

//! \brief STATUS_REG bits
#define READY			BIT0
#define CRC_ACTIVE		BIT1
#define RF_BUSY			BIT2

//! \brief STATUS_REG bits
#define APP_STATUS_REGS			BIT4 + BIT5 + BIT6
#define FILE_SELECT_STATUS		BIT4
#define FILE_REQUEST_STATUS		BIT5
#define FILE_AVAILABLE_STATUS	BIT5 + BIT4

//! \brief INT_ENABLE_REG bits
#define EOR_INT_ENABLE				BIT1
#define EOW_INT_ENABLE				BIT2
#define CRC_INT_ENABLE				BIT3
#define BIP8_ERROR_INT_ENABLE		BIT4
#define DATA_TRANSACTION_INT_ENABLE	BIT5
#define FIELD_REMOVED_INT_ENABLE 	BIT6
#define GENERIC_ERROR_INT_ENABLE	BIT7
#define EXTRA_DATA_IN_INT_ENABLE	BIT8

//! \brief INT_FLAG_REG bits
#define EOR_INT_FLAG				BIT1
#define EOW_INT_FLAG				BIT2
#define CRC_INT_FLAG				BIT3
#define BIP8_ERROR_INT_FLAG			BIT4
#define DATA_TRANSACTION_INT_FLAG	BIT5
#define FIELD_REMOVED_INT_FLAG	 	BIT6
#define GENERIC_ERROR_INT_FLAG		BIT7
#define EXTRA_DATA_IN_FLAG			BIT8

//! \brief COMM_WD_CTRL_REG bits
#define WD_ENABLE	BIT0
#define TIMEOUT_PERIOD_2_SEC	0
#define TIMEOUT_PERIOD_32_SEC	BIT1
#define TIMEOUT_PERIOD_8_5_MIN	BIT2
#define TIMEOUT_PERIOD_MASK		BIT1 + BIT2 + BIT3

//! \brief Host response index
#define INT_SERVICED_FIELD 			BIT0
#define FILE_EXISTS_FIELD			BIT1
#define CUSTOM_RESPONSE_FIELD		BIT2
#define EXTRA_DATA_IN_SENT_FIELD	BIT3
#define FILE_DOES_NOT_EXIST_FIELD	0

#define TEST_MODE_KEY			0x004E
#define TEST430_ENABLE			0x0080

unsigned char into_fired;

//*****************************************************************************
//
//! \brief   This function initializes the RF430CL331 IC. Because the functions
//!			 call is remote, it has the possibility to hang. Caution should be
//!			 taken when calling this function.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_RegisterInit(void);

//*****************************************************************************
//
//! \brief   This function call initializes the GPIO port 4 as an input port
//!			 with a low to high trasition interrupt. This is needed by the
//!			 RF430CL331 INT pin.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_INT0Init(void);

//*****************************************************************************
//
//! \brief   This function call sets up the MSP430 port pin P9.3 as an output.
//!			 This port pin is tied to the RF430CL331 reset pin
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_ResetInit(void);

//*****************************************************************************
//
//! \brief   This function call sets up the MSP430 port pin P4.0 as an input.
//!			 This port pin is tied to the RF430CL331 ready pin
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_I2CReadyInit(void);

//*****************************************************************************
//
//! \brief   This function disables the RF430CL331 INT0 interrupt which is tied
//!			 to MSP430 port pin P4.1.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_dissableINT0Detect(void);

//*****************************************************************************
//
//! \brief   This function enables the RF430CL331 INT0 interrupt which is tied
//!			 to MSP430 port pin P4.1.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_enableINT0Detect(void);

//*****************************************************************************
//
//! \brief   This function call actually reset the RF430CL331 IC. Later change
//!			 will include a state machine to remove the delay_cycles.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_reset(void);


//*****************************************************************************
//
//! \brief   This functions will handle the interrupt received by the port bit
//!			 P4.1 caused by theRF430CL331 INT pin
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void RF430CL331_serviceInterrupt(void);

//*****************************************************************************
//
//! \brief   This function puts the NFC IC in Standby mode. According to
//!			 the datasheet the current consumption in this mode is 10uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL RF430CL331_remoteStandbyEnabled(void);

//*****************************************************************************
//
//! \brief   This function puts the NFC IC in Active mode. According to the
//!			 datasheet the current consumption in this mode is 40uA.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL RF430CL331_remoteStandbyDisabled(void);

//*****************************************************************************
//
//! \brief   This function enables the RF Interface. Please see the device
//!			 datasheet to see the difference between RF Interface and Standby mode.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL RF430CL331_remoteRFEnabled(void);

//*****************************************************************************
//
//! \brief  This function disables the RF Interface. Please see the device datasheet
//!			to see the difference between RF Interface and Standby mode.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
BOOL RF430CL331_remoteRFDisabled(void);


#endif /* RF430_COMM__H_ */
