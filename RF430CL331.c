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
//! \file   RF430CL331.c
//!
//! \brief  Please see RF430CL331.h
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
#include "msp430.h"
#include "RF430CL331.h"
#include <stdint.h>
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "I2CInterface.h"
#include "NFCForumType4TagType.h"
#include "driverlib.h"
#include "Executive.h"
#include "StateMachine.h"
#include "types.h"

uint16_t flags = 0;

void RF430CL331_RegisterInit(void)
{

	into_fired =0;

    __enable_interrupt();  // Enable interrupts globally
    while(!(I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, STATUS_REG,2 , 2) & READY)); //wait until READY bit has been set

	AppInit();

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_ENABLE_REG, DATA_TRANSACTION_INT_ENABLE + FIELD_REMOVED_INT_ENABLE + GENERIC_ERROR_INT_ENABLE, 2, 2);

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, TEST_MODE_REG, TEST_MODE_KEY, 2, 2);   //unlock test mode
	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, TEST430_ENABLE, 2, 2);    //enable test mode, now have
	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, 0x2A66, 0x0000, 2, 2);                   //the fix
	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, 0x27B8, 0, 2, 2);                      //exit test mode
	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, TEST_MODE_REG, 0, 2, 2);               //lock test reg

	RF430CL331_dissableINT0Detect();
    I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG,  INT_ENABLE + INTO_DRIVE + RF_ENABLE, 2, 2);

	// sets the command timeouts, generally should stay at these settings
    I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, SWTX_INDEX, 0x3B, 2, 2);
    I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, TIMER_DELAY, 300, 2, 2);
    RF430CL331_enableINT0Detect();

	MPY32_setOperandOne16Bit(MPY32_MULTIPLY_UNSIGNED, 0x1234);
}

void RF430CL331_INT0Init(void)
{

    //Enable P4.1 internal resistance as pull-Up resistance
    GPIO_setAsInputPinWithPullUpresistor(
            GPIO_PORT_P4,
            GPIO_PIN1
            );

    //P1.4 interrupt enabled
    GPIO_enableInterrupt(
            GPIO_PORT_P4,
            GPIO_PIN1
            );

    //P1.4 Hi/Lo edge
    GPIO_interruptEdgeSelect(
            GPIO_PORT_P4,
            GPIO_PIN1,
            GPIO_HIGH_TO_LOW_TRANSITION
            );


    //P1.4 IFG cleared
    GPIO_clearInterruptFlag(
            GPIO_PORT_P4,
            GPIO_PIN1
            );

}

void RF430CL331_ResetInit(void)
{

	GPIO_setAsOutputPin(
			GPIO_PORT_P9,
			GPIO_PIN3
			);//Reset signal to RF430

	GPIO_setOutputLowOnPin(
			GPIO_PORT_P9,
			GPIO_PIN3
			);//Deassert Reset signal

}

void RF430CL331_I2CReadyInit(void)
{
	//Set I2C_READY singal to input on MSP430
//    I2C_READY_DIR &= ~I2C_READY_PIN;

    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0);

}

void RF430CL331_dissableINT0Detect(void)
{

    GPIO_disableInterrupt(
    		GPIO_PORT_P4,
    		GPIO_PIN1
    		);

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
    		GPIO_PORT_P4,
    		GPIO_PIN1
    		);
}

void RF430CL331_enableINT0Detect(void)
{

	GPIO_enableInterrupt(
			GPIO_PORT_P4,
            GPIO_PIN1
            );

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
            GPIO_PORT_P4,
            GPIO_PIN1
            );

}

void RF430CL331_reset(void)
{
    __delay_cycles(1000000); 		//leave time for the RF430CL33H to see signal
    //GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);		//Reset signal to RF430
    PORT_RST_OUT &= ~RST;
    __delay_cycles(1000000); 		//leave time for the RF430CL33H to see signal
    //GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3);		//Reset signal to RF430
    PORT_RST_OUT |= RST;
    __delay_cycles(1000000); 		//leave time for the RF430CL33H to get itself initialized

}

void RF430CL331_serviceInterrupt(void)
{
	//device has woken up, check status
	if(into_fired)
	{
		flags = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, 2, 2); //read the flag register to check if a read or write occurred
		do
		{
			if (flags)
			{
				ServiceInterrupt(flags);
			}
			//interrupts may have occured while servicing a prior interrupt, check again to make sure all are serviced
			flags = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, INT_FLAG_REG, 2, 2); //read the flag register to check if a read or write occurred
		}while(flags);

		//re-enable INTO
		RF430CL331_enableINT0Detect();
		__no_operation();

    	flags = 0;
    	into_fired = 0; //we have serviced INTO

	}
	StateMachine_commandExecution();
}

BOOL RF430CL331_remoteStandbyEnabled(void)
{

	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);
	expectedreturnValue = returnValue | STANDBY_ENABLE;

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, expectedreturnValue, 2, 2);

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);

	if(returnValue == expectedreturnValue)
		return true;
	else
		return false;

}



BOOL RF430CL331_remoteStandbyDisabled(void)
{

	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);
	expectedreturnValue = returnValue & ~STANDBY_ENABLE;

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, expectedreturnValue, 2, 2);

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);

	if(returnValue == expectedreturnValue)
		return true;
	else
		return false;

}

BOOL RF430CL331_remoteRFEnabled(void)
{

	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);
	expectedreturnValue = returnValue | RF_ENABLE;

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, expectedreturnValue, 2, 2);

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);

	if(returnValue == expectedreturnValue)
		return true;
	else
		return false;

}



BOOL RF430CL331_remoteRFDisabled(void)
{

	uint16_t volatile expectedreturnValue, returnValue;

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);
	expectedreturnValue = returnValue & ~RF_ENABLE;

	I2CInterface_writeRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, expectedreturnValue, 2, 2);

	returnValue = I2CInterface_readRegisterData(IC_ADDRESS_RF430CL331_ADDRESS, CONTROL_REG, 2, 2);

	if(returnValue == expectedreturnValue)
		return true;
	else
		return false;

}


//This is the INTO ISR
#pragma vector=PORT4_VECTOR
__interrupt void PORT4_ISR(void)
{


//	//INTO interrupt fired
	if(PORT_INTO_IFG & INTO)
	{

		into_fired = 1;

		PUBLISH_FAST_EVENT(FEV_RF430CL331_INTERRUPT);
		//GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0 );//Deassert Reset signal
		RF430CL331_dissableINT0Detect();

		//__bic_SR_register_on_exit(LPM3_bits + GIE); //wake up to handle INTO
		__bic_SR_register_on_exit(LPM4_bits); //wake up to handle INTO
	}

}


