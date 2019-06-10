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
//! \file   TMP112A.c
//!
//! \brief  Please see TMP112A.h
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
#include "TMP112A.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "I2CInterface.h"
#include "UART.h"
#include "RingBuffer.h"


uint16_t TMP112A_remoteRegisterInit(void)
{


	return 0;
}

uint16_t TMP112A_localGPIOInit(void)
{

    GPIO_setAsInputPinWithPullUpresistor(
            GPIO_PORT_P2,
            GPIO_PIN7
            );

    //P1.4 interrupt enabled
    GPIO_enableInterrupt(
            GPIO_PORT_P2,
            GPIO_PIN7
            );

    //P1.4 Hi/Lo edge
    GPIO_interruptEdgeSelect(
            GPIO_PORT_P2,
            GPIO_PIN7,
            GPIO_HIGH_TO_LOW_TRANSITION
            );

    //P1.4 IFG cleared
    GPIO_clearInterruptFlag(
            GPIO_PORT_P2,
            GPIO_PIN7
            );

	return 0;
}

void TMP112A_localEnableAlertDetect(void)
{

	GPIO_enableInterrupt(
			GPIO_PORT_P2,
            GPIO_PIN7
            );

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
            GPIO_PORT_P2,
            GPIO_PIN7
            );

}

void TMP112A_localDissableAlertDetect(void)
{

    GPIO_disableInterrupt(
    		GPIO_PORT_P2,
    		GPIO_PIN7
    		);

    GPIO_clearInterruptFlag(
    		GPIO_PORT_P2,
    		GPIO_PIN7
    		);
}

uint16_t TMP112A_remoteShutDownMode(void)
{
	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, 1, 2);
	expectedreturnValue = returnValue | SD;

	//I2CInterface_writeRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, expectedreturnValue, 1, 2);
	//char header = 0x01;
	//ring_buffer_put(_outputrb_d,header);
	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, 1, 2);

	if(returnValue == expectedreturnValue)
	{
	    //PUBLISH_EVENT(EV_UART_TX);
	    return true;
	}
	else
		return false;


}

uint16_t TMP112A_remoteActiveMode(void)
{

	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, 1, 2);
	expectedreturnValue = returnValue & ~SD;

	//I2CInterface_writeRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, expectedreturnValue, 1, 2);

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_TMP112_ADDRESS, CONFIGURATION_REGISTER, 1, 2);

	if(returnValue == expectedreturnValue)
		return true;
	else
		return false;

}

uint16_t TMP112A_remoteReadTemperature(void)
{
	int16_t  tmp112_Temperature = 0;
	int16_t  true_tmp112_Temperature = 0;

	tmp112_Temperature = I2CInterface_readRegisterData(IC_ADDRESS_TMP112_ADDRESS, TEMPERATURE_REGISTER, 2, 2);

	tmp112_Temperature = (tmp112_Temperature <<8) | (tmp112_Temperature & 0x00);

	tmp112_Temperature = tmp112_Temperature>>4;
	true_tmp112_Temperature = tmp112_Temperature * 0.0625;
	true_tmp112_Temperature = true_tmp112_Temperature*1.8+32;


	return true_tmp112_Temperature;

}

