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
//! \file   eUSCIB0_I2C.h
//!
//! \brief  This is the driver module for the eUSCIB0 peripheral. All of the
//!			functions found in this peripheral driver module are specific to the
//!			eUSCIB0 peripheral. If a peripheral module other than eUSCIB0 is used,
//!			then this module must be replaced. Most of the functions found in this
//!			module are considered remote and hence there is a real chance of it
//!			hanging. To avoid program hang, it is recommended to call the watchdog
//!			timer before entering this function. This module configures the eUSCIB0
//!			to act as a I2C module, transmitts data over the I2C bus by way of
//!			interrupt and reads data from the I2C bus by way of interrupt. This
//!			module also configures the eUSCIB0 clock source, eUSCIB0 frequency,
//!			peripheral address, initialize I2C ports. This module was lightly
//!			commented as the majority is explained in great detail using the
//!			eUSCIB0 datasheet. Module comments are by no means peripheral
//!			datasheet supplement.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef USCI_I2C_H
#define USCI_I2C_H

//*****************************************************************************
// the includes
//*****************************************************************************
#include <msp430.h>
#include <stdint.h>
#include "RF430CL331.h"


typedef unsigned char bool_t;

//! \brief This defines the I/O ports needed by the USCIBO peripheral. It makes
//!	sense to declare the port here at the top so in case they change, they are
//!edited only here.
#define PORT_I2C_OUT	P1OUT
#define PORT_I2C_DIR	P1DIR
#define PORT_I2C_SEL	P1SEL0
#define SDA	BIT6
#define SCL BIT7

//! \brief These defines are needed to keep track of data during read and write process.
#define START       	1
#define CONTINUE    	0
#define FAIL			0
#define PASS 			1
#define SET				1
#define CLEAR			0
#define I2C_NACK_RCVD	2
#define I2C_TRANSMIT	3
#define LPM_MODE		LPM0_bits
#define I2C_ALLOW		(I2C_READY_IN & I2C_READY_PIN)

uint8_t RF430_I2C_State;
uint8_t RF430_I2C_Start;			//AK 10-23-2013


//! \brief This declares both the register address and data.
//!
typedef struct
{
	unsigned char configReg;
	unsigned char data;
} i2cCmd_t;


//*****************************************************************************
//
//! \brief	This function is called only once and it is called when the MSP430
//!			is initializing. Furthermore, the function is called indirectly by
//!			the I2C peripheral interface module when it itself is initializing.
//!			This function configures the USCIBO peripheral to act as a I2C peripheral,
//!			set to master transmitting mode, 8 MHz clock frequency and configures
//!			its I2C address. All the instructions are local hence there is no
//!			chance of program hanging. All the instructions in this function
//!			apply specifically to the USCIBO peripheral. If another I2C peripheral
//!			is used, this this function must be completely modified.
//!
//! \param 	none
//
//! \return	None
//
//*****************************************************************************
void USCIBO_InitializeRegisters(void);

//*****************************************************************************
//
//! \brief	This function is called only once and it is called when the MSP430
//!			is initializing. Normally, the function gets called almost immediately
//!			after USCIBO_InitializeRegisters gets called. Local registers are
//!			configured here hence there is no chance of program hanging. The
//!			instructions executed in this function applies specifically to the
//!			MSP430. If another MCU is used, this this function must be completely
//!			modified. The function configures the port pins P1.6 and P1.7 for
//!			peripheral (specifically eUSCIB0 peripheral) use instead of GPIO use.
//!			Port pin P1.6 is set for SDA and port pin P1.7 is used for SCL.
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void USCIBO_InitializePorts(void);

//*****************************************************************************
//
//! \brief	This function gets called whenever a stop condition on the I2C bus
//!			is needed. Local registers are configured here hence there is no
//!			chance of program hanging. The instructions executed in this function
//!			applies specifically to the MSP430. If another MCU is used, then
//!			this function must be completely edited. To generate a stop condition
//!			simply set the UCTXSTP bit in the UCB0CTL1 register.
//
//! \param none     None
//
//! \return  None
//
//*****************************************************************************
void USCIBO_Stop(void);

//*****************************************************************************
//
//! \brief  This function is called whenever a REGISTER write operation on the
//!			I2C bus is desired. Remote registers are configured hence there is
//!			a real chance of it hanging. To avoid program hang, it is recommended
//!			to call the watchdog timer before entering this function. The parameters
//!			passed to this function are ic_address which is the destination
//!			address of the remote IC, reg_addr which is the address of the
//!			register within the remote IC which the write operation will take
//!			place, value which is data that that would like to be written on the
//!			remote register, bytesInAddr which states the size in bytes of the
//!			reg_addr, and bytesInValue which states the size in bytes of the value.
//
//
//! \param ic_address	Destination address of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the write operation will take place
//!	\param value		Data to write. Can be 1 or 2 bytes long
//! \param bytesInAddr	Size in bytes of the IC address
//! \param bytesInValue	Size in bytes for the data to write.
//
//! \return  None
//
//*****************************************************************************
void USCIBO_writeRegister(uint8_t ic_address, uint16_t reg_addr, uint16_t value, uint8_t bytesInAddr, uint8_t bytesInValue);

//*****************************************************************************
//
//! \brief  This function is called whenever a GENERIC write operation on the
//!			I2C bus is desired. Remote registers are configured hence there is
//!			a real chance of it hanging. To avoid program hang, it is recommended
//!			to call the watchdog timer before entering this function. the
//!			parameters passed to this function are ic_address which is the
//!			destination address of the remote IC, reg_addr which is the address
//!			of the register within the remote IC which the write operation will
//!			take place, write_data which is a pointer to a buffer which contains
//!			the data to be written, and data_length which is the length of the
//!			data bytes to write.
//
//! \param ic_address	Destination address of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the write operation will take place
//! \param write_data	Pointer to a buffer which contains the data to be written
//! \param data_length	Length of the data bytes to write
//
//! \return  None
//
//*****************************************************************************
void USCIBO_writeContinuous(uint8_t ic_address, uint16_t reg_addr, uint8_t  * write_data, uint16_t data_length);

//*****************************************************************************
//
//! \brief  This function gets called by both USCIBO_writeRegister and USCIBO_writeContinuous.
//!			This function transmits data over the I2C bust by way of interrupt. As a results
//!			caution must be taken that no other interrupt occurs except USCI_B0_ISR while this
//!			function is executing. To avoid conflicts, disable all interrupt except USCI_B0_ISR
//!			when executing this function.
//
//! \param data			Pointer to a buffer which contains the data to be written
//! \param length		Length of the data bytes to write
//! \param cont			Determines if a START or CONTINUE condition is needed.
//
//! \return  None
//
//*****************************************************************************
void RF430_I2C_Write(uint8_t  * data, uint16_t length, uint16_t cont);

//*****************************************************************************
//
//! \brief  This function is called whenever a REGISTER read operation operation
//!			on the I2C bus is desired. Remote registers are configured hence
//!			there is a real chance of it hanging. The parameters passed to this
//!			function are ic_address which is the destination address of the
//!			remote IC, reg_addr which is the address of the register within the
//!			remote IC which the read operation will take place, read_data which
//!			is a pointer to an buffer where the read data will be stored, and
//!			data_length which is the length of the data bytes to read.
//
//! \param ic_address	Destination address of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the read operation will take place
//! \param bytesInAddr	Size of reg_addr in bytes
//! \param bytesToRead	Bytes to read
//
//! \return		16 bit data
//
//*****************************************************************************
uint16_t USCIBO_readRegister(uint8_t ic_address, uint16_t reg_addr, uint8_t bytesInAddr, uint8_t bytesToRead);

//*****************************************************************************
//
//! \brief  This function is called whenever a GENERIC read operation operation
//!			on the I2C bus is desired. Remote registers are configured hence
//!			there is a real chance of it hanging. The parameters passed to this
//!			function are ic_address which is the destination address of the
//!			remote IC, reg_addr which is the address of the register within the
//!			remote IC which the read operation will take place, read_data which
//!			is a pointer to an buffer where the read data will be stored, and
//!			data_length which is the length of the data bytes to read.
//
//! \param ic_address	Destination address of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the read operation will take place
//! \param read_data	Pointer to an buffer where the read data will be stored
//! \param data_length	Length of the data bytes to read
//
//! \return		16 bit data
//
//*****************************************************************************
void USCIBO_readContinuous(uint8_t ic_address, uint16_t reg_addr, uint8_t* read_data, uint16_t data_length);

//*****************************************************************************
//
//! \brief  This function gets called by both USCIBO_readRegister and USCIBO_readContinuous.
//!			This function recieves data over the I2C bust by way of interrupt. As a results
//!			caution must be taken that no other interrupt occurs except USCI_B0_ISR while this
//!			function is executing. To avoid conflicts, disable all interrupt except USCI_B0_ISR
//!			when executing this function.
//
//! \param tx_data			Pointer to a buffer which contains the data to be written
//! \param tx_length		Length of the data bytes to write
//! \param rx_data			Determines if a START or CONTINUE condition is needed.
//! \param rx_length		Determines if a START or CONTINUE condition is needed.
//
//! \return  None
//
//*****************************************************************************
uint8_t RF430_I2C_Write_Restart_Read(uint8_t  * tx_data, uint16_t tx_length, uint8_t* rx_data, uint16_t rx_length);

#endif

