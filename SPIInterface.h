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
//! \file   SPIInterface.h
//!
//! \brief  This is the SPI peripheral interface file for the eUSCIA1
//!			peripheral driver module. This interface file provides functions common
//!			to all SPI peripherals. Most people familiar with SPI protocol should
//!			be familiar with the functions found in this module however will be not
//!			so familiar with the functions found in the eUSCIA1 driver module. Call
//!			the functions found on the eUSCIA1 peripheral driver file in order to
//!			initialize the eUSCIA1 peripheral module, initialize the ports, read
//!			data from SPI bus, write data to I2C bus, power down and power are the
//!			functions that should be found on this peripheral interface. Although in this case
//!     	most of the functions found in this peripheral interface module simply
//!			pipe out the data that is sent to them, that is not always the case
//!			with peripheral interface functions.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef SPIINTERFACE_H_
#define SPIINTERFACE_H_

#include <stdint.h>
//*****************************************************************************
//
//! \brief  This function is called only once and it is called when the MSP430
//! 		is initializing. All the function calls called by this function are
//! 		found only the eUSCIA1 peripheral driver module. Furthermore, all
//!			the function calls are local hence there is no chance of program
//!			hanging. First function configures the port pins P3.4, P3.5, P3.6
//!			and P3.7 for peripheral (specifically eUSCIA1 peripheral) use instead
//!			of GPIO use. Next the function calls a eUSCIA1 peripheral driver
//!			function which configures the eUSCIA1 registers (Set the eUSCIA1
//!			to act as an SPI peripheral, configures the peripheral clock source,
//!			and scacle down the clock source).
//!
//! \param none     None
//
//! \return		None
//
//*****************************************************************************
void SPIInterface_Initialize(void);

//*****************************************************************************
//
//! \brief  This function is called whenever a GENERIC read operation on the
//!			SPI bus is desired. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. Furthermore, this function call
//!			is remote and hence there is a real chance of it hanging. To avoid
//!			program hang, it is recommended to call the watchdog timer before
//!			entering this function. The parameters passed to this function
//!			are SPI_ic which is the chip select pin of the remote IC,
//!			reg_addr which is the address of the register within the remote
//!			IC which the read operation will take place, read_data which is a
//!			pointer to an buffer where the read data will be stored, and
//!			data_length which is the length of the  data bytes to read.
//
//! \param SPI_ic		Chip select pin of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the read operation will take place
//! \param read_data	A pointer to an buffer where the read data will be stored
//! \param data_length	Length of the data bytes to read
//
//! \return		None
//
//*****************************************************************************
void SPIInterface_readGenericData(uint8_t SPI_ic, uint16_t reg_addr, uint8_t* read_data, uint16_t data_length);

//*****************************************************************************
//
//! \brief  This function is called whenever a REGISTER read operation on the
//!			SPI bus is desired. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. Furthermore, this function call is
//!			remote and hence there is a real chance of it hanging. To avoid
//!			program hang, it is recommended to call the watchdog timer before
//!			entering this function. The parameters passed to this function are
//!			SPI_ic which is the chip select pin of the remote IC,
//!			reg_addr which is the address of the register within the remote
//!			IC which the read operation will take place, read_data which is a
//!			pointer to an buffer where the read data will be stored, and
//!			data_length which is the length of the data bytes to read.
//!
//! \param SPI_ic		Chip select pin of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the read operation will take place
//! \param read_data	A pointer to an buffer where the read data will be stored
//! \param data_length	Length of the data bytes to read
//
//! \return		None
//
//*****************************************************************************
void SPIInterface_readRegisterData(uint8_t SPI_ic, uint16_t reg_addr, uint8_t* read_data, uint16_t data_length);

//*****************************************************************************
//
//! \brief  This function is called whenever a GENERIC write operation on the
//!			SPI bus is desired. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. Furthermore, this function call is
//!			remote and hence there is a real chance of it hanging. To avoid
//!			program hang, it is recommended to call the watchdog timer before
//!			entering this function. The parameters passed to this function are
//!			SPI_ic which is the chip select pin of the remote IC,
//!			reg_addr which is the address of the register within the remote
//!			IC which the write operation will take place, write_data which is
//!			a pointer to a buffer which contains the data to be written, and
//!			data_length which is the length of the data bytes to write.
//
//! \param SPI_ic		Chip select pin of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the write operation will take place
//! \param write_data	Pointer to a buffer which contains the data to be written
//! \param data_length	Length of the data bytes to write
//
//! \return		None
//
//*****************************************************************************
void SPIInterface_writeGenericData(uint8_t SPI_ic, uint16_t reg_addr, uint8_t  * write_data, uint16_t data_length);

//*****************************************************************************
//
//! \brief  This function is called whenever a REGISTER write operation on the
//!			SPI bus is desired. Although, in this case the function simply
//!			pipes out the command sent to it, that is not always the case with
//!			peripheral interface functions. Furthermore, this function call is
//!			remote and hence there is a real chance of it hanging. To avoid
//!			program hang, it is recommended to call the watchdog timer before
//!			entering this function. The parameters passed to this function are
//!			SPI_ic which is the chip select pin of the remote IC,
//!			reg_addr which is the address of the register within the remote
//!			IC which the write operation will take place, write_data which is
//!			a pointer to a buffer which contains the data to be written, and
//!			data_length which is the length of the data bytes to write.
//
//! \param SPI_ic		Chip select pin of the remote IC
//! \param reg_addr		Address of the register within the remote IC which the write operation will take place
//! \param write_data	Pointer to a buffer which contains the data to be written
//! \param data_length	Length of the data bytes to write
//
//! \return		None
//
//*****************************************************************************
void SPIInterface_writeRegisterData(uint8_t SPI_ic, uint16_t reg_addr, uint16_t value, uint8_t bytesInAddr, uint8_t bytesInValue);



#endif /* SPIINTERFACE_H_ */
