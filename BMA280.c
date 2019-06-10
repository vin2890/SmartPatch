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
//! \file   BMA280.c
//!
//! \brief  Please see BMA280.h
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
//TODO: Change Port 2.0 pins to port 2.5
#include "BMA280.h"
#include "types.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "I2CInterface.h"
#include "Executive.h"
#include "AccelerometerInterface.h"
#include "UART.h"

/*  Extern Variables */
EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingRxEventFifo[STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_SIZE];
unsigned char StateMachine_BMA280PowerTracingRxEventFifoHead=0;
unsigned char StateMachine_BMA280PowerTracingRxEventFifoTail=0;

//Only used for debugging purposes.
EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingTxEventFifo[STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_SIZE];
unsigned char StateMachine_BMA280PowerTracingTxEventFifoHead=0;
unsigned char StateMachine_BMA280PowerTracingTxEventFifoTail=0;

STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingRxStateFifo[STATE_MACHINE_BMA280_POWER_TRACING_RX_STATE_FIFO_SIZE];
unsigned char StateMachine_BMA280PowerTracingRxStateFifoHead=0;
unsigned char StateMachine_BMA280PowerTracingRxStateFifoTail=0;

//Only used for debugging purposes.
STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingTxStateFifo[STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_SIZE];
unsigned char StateMachine_BMA280PowerTracingTxStateFifoHead=0;
unsigned char StateMachine_BMA280PowerTracingTxStateFifoTail=0;


/*  Definitions */
#define IS_BMA280_POWER_TRACING_DATA_IN_TX_EVENT_FIFO() (!(StateMachine_BMA280PowerTracingTxEventFifoHead == StateMachine_BMA280PowerTracingTxEventFifoTail))
#define IS_BMA280_POWER_TRACING_DATA_IN_RX_EVENT_FIFO() (!(StateMachine_BMA280PowerTracingRxEventFifoHead == StateMachine_BMA280PowerTracingRxEventFifoTail))

#define IS_BMA280_POWER_TRACING_DATA_IN_TX_STATE_FIFO() (!(StateMachine_BMA280PowerTracingTxStateFifoHead == StateMachine_BMA280PowerTracingTxStateFifoTail))
#define IS_BMA280_POWER_TRACING_DATA_IN_RX_STATE_FIFO() (!(StateMachine_BMA280PowerTracingRxStateFifoHead == StateMachine_BMA280PowerTracingRxStateFifoTail))

STATES_BMA280_POWERMODE BMA280PowerModeCurrentState = St_Normal_Mode_Idle;

uint16_t BMA280_remoteRegisterInit(){


	/* Enable the accelerometer sensor. Take into consideration that current consumption will increase. */
	AccelerometerInterface_powerMode0();

	//Set interrupts to non-latched mode. This means that the interrupt tatus bit must
	//be read for the interrupt to be cleared.
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_RST_LATCH, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_RST_LATCH, 0x00, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_RST_LATCH, 1, 2);

	//select push-pull, and active high for INT1 and INT2 pin.
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_OUT_CTRL, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_OUT_CTRL, 0x0A, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_OUT_CTRL, 1, 2);

	//Map double tap interrupt to INT1 pin.
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_0, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_0, 0x10, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_0, 1, 2);

	//Filtered data only
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_SRC, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_SRC, 0x00, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_SRC, 1, 2);

	//700ms between taps
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_8, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_8, 0x87, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_8, 1, 2);

	//Number of samples that are processed after wake-up->16 sampels
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_9, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_9, 0xCA, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_9, 1, 2);

	//Double tap interrupt enable
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_0, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_0, 0x10, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_0, 1, 2);

	//set to 2g
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_RANGE, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_RANGE, 0x03, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_RANGE, 1, 2);

//	//set sampling rate to 62.5Hz.
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_BW, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_BW, 0x0A, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_BW, 1, 2);

	//Map data ready interrup to INT2- Enable
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_1, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_1, 0x80, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_MAP_1, 1, 2);

	//Data Readu interrupt enable
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 0x10, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);


	/* To conserver battery place the BMA280 in suspended mode. */
	AccelerometerInterface_powerMode4();

	return 0;
}

void BMA280_GPIOInit(void)
{


}

void BMA280_PeripheralInit(void)
{

//Enable P1.1 internal resistance as pull-Up resistance
	    GPIO_setAsInputPinWithPullUpresistor(
	            GPIO_PORT_P1,
	            GPIO_PIN1
	            );

	    //P1.1 interrupt enabled
	    GPIO_enableInterrupt(
	            GPIO_PORT_P1,
	            GPIO_PIN1
	            );

	    //P1.1 Hi/Lo edge
	    GPIO_interruptEdgeSelect(
	            GPIO_PORT_P1,
	            GPIO_PIN1,
	            GPIO_HIGH_TO_LOW_TRANSITION
	            );


	    //P1.1 IFG cleared
	    GPIO_clearInterruptFlag(
	            GPIO_PORT_P1,
	            GPIO_PIN1
	            );



//Enable P2.0 internal resistance as pull-Up resistance
		GPIO_setAsInputPinWithPullUpresistor(
				GPIO_PORT_P2,
				GPIO_PIN0
				//GPIO_PIN5
				);

		//P2.0 interrupt enabled
		GPIO_enableInterrupt(
				GPIO_PORT_P2,
				GPIO_PIN0
				//GPIO_PIN5
				);

		//P2.0 Hi/Lo edge
		GPIO_interruptEdgeSelect(
				GPIO_PORT_P2,
				GPIO_PIN0,
				//GPIO_PIN5,
				GPIO_LOW_TO_HIGH_TRANSITION
				);

		//P2.0 IFG cleared
		GPIO_clearInterruptFlag(
				GPIO_PORT_P2,
				GPIO_PIN0
				//GPIO_PIN5
				);



}

void BMA280_dissableINT1Detect(void)
{

    GPIO_disableInterrupt(
    		GPIO_PORT_P1,
    		GPIO_PIN1
    		);

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
    		GPIO_PORT_P1,
    		GPIO_PIN1
    		);

}

void BMA280_enableINT1Detect(void)
{

	GPIO_enableInterrupt(
			GPIO_PORT_P1,
            GPIO_PIN1
            );

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
            GPIO_PORT_P1,
            GPIO_PIN1
            );

}

void BMA280_dissableINT2Detect(void)
{

    GPIO_disableInterrupt(
    		GPIO_PORT_P2,
    		GPIO_PIN0
    		//GPIO_PIN5
    		);

    GPIO_clearInterruptFlag(
    		GPIO_PORT_P2,
    		GPIO_PIN0
    		//GPIO_PIN5
    		);

}

void BMA280_enableINT2Detect(void)
{

	GPIO_enableInterrupt(
			GPIO_PORT_P2,
            GPIO_PIN0
			//GPIO_PIN5
            );

    //clear P1.5 IFG flag
    GPIO_clearInterruptFlag(
            GPIO_PORT_P2,
            GPIO_PIN0
            //GPIO_PIN5
            );

}

void BMA280_disableDataReadyInterrupt(void)
{
	//Data Readu interrupt enable
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 0x00, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);
}

void BMA280_enableDataReadyInterrupt(void)
{
	//Data Readu interrupt enable
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 0x10, 1, 1);
	I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, INT_EN_1, 1, 2);
}

uint16_t BMA280_ReadXAcceleration(void)
{

	uint8_t RxBuffer[2] = {0,0};
	int16_t volatile value;

	RxBuffer[0] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_X_LSB, 1, 2);
	RxBuffer[1] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_X_MSB, 1, 2);

	value = RxBuffer[1];
	value = (value<<8 )|RxBuffer[0];
	value = value >> 2;
	value = value << 2;

	return value;

}

uint16_t BMA280_ReadYAcceleration(void)
{

	uint8_t RxBuffer[2] = {0,0};
	int16_t volatile value;

	RxBuffer[0] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_Y_LSB, 1, 2);
	RxBuffer[1] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_Y_MSB, 1, 2);

	value = RxBuffer[1];
	value = (value<<8 )|RxBuffer[0];
	value = value >> 2;
	value = value << 2;

	return value;

}

uint16_t BMA280_ReadZAcceleration(void)
{
	uint8_t RxBuffer[2] = {0,0};
	int16_t volatile value;

	RxBuffer[0] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_Z_LSB, 1, 2);
	RxBuffer[1] = I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, ACCD_Z_MSB, 1, 2);

	value = RxBuffer[1];
	value = (value<<8 )|RxBuffer[0];
	value = value >> 2;
	value = value << 2;

	return value;
}

void BMA280_softReset(void)
{
	I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, BGW_SOFTRESET, 0xB6, 1, 1);
}

BOOL BMA280_PowerModeStateMachine(void)
{
	/* Read the next incoming event. Usually this is a blocking function. */
	EVENTS_BMA280_POWERMODE changePowerModeevent;

	while(IS_BMA280_POWER_TRACING_DATA_IN_RX_EVENT_FIFO() == true)
	{
		changePowerModeevent = StateMachine_BMA280PowerTracingReadEventRxFifo();

		/* Switch the state and the event to execute the right transition. */
		switch(BMA280PowerModeCurrentState)
		{
			case St_Normal_Mode_Idle:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						//Already in this state
						break;
					case EVT_EnterDeepSuspendedMode:
						BMA280PowerModeCurrentState = St_Deep_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x20, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterSuspendedMode:
						BMA280PowerModeCurrentState = St_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Suspend mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x80, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterLowPowerMode1:
						BMA280PowerModeCurrentState = St_Low_Power_Mode_1;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterLowPowerMode2:
						BMA280PowerModeCurrentState = St_Low_Power_Mode_2;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterStandByMode:
						BMA280PowerModeCurrentState = St_Standby_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x80, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					default:
						return false;
				}
				break;

			case St_Low_Power_Mode_2:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						BMA280PowerModeCurrentState = St_Normal_Mode_Idle;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterDeepSuspendedMode:
						BMA280PowerModeCurrentState = St_Deep_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x20, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterSuspendedMode:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterSuspendedMode);
						break;
					case EVT_EnterLowPowerMode1:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode1);
						break;
					case EVT_EnterLowPowerMode2:
						//Already in this state
						//BMA280PowerModeCurrentState = St_Low_Power_Mode_2;

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterStandByMode:
						BMA280PowerModeCurrentState = St_Standby_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x80, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					default:
						return false;
				}
				break;

			case St_Standby_Mode:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						BMA280PowerModeCurrentState = St_Normal_Mode_Idle;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterDeepSuspendedMode:
						BMA280PowerModeCurrentState = St_Deep_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x20, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterSuspendedMode:
						//not allowed
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterSuspendedMode);

						break;
					case EVT_EnterLowPowerMode1:
						//not allowed
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode1);
						break;
					case EVT_EnterLowPowerMode2:
						BMA280PowerModeCurrentState = St_Low_Power_Mode_2;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterStandByMode:
						//Already in this state
						break;
					default:
						return false;
				}
				break;

			case St_Low_Power_Mode_1:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						BMA280PowerModeCurrentState = St_Normal_Mode_Idle;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Suspend mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterDeepSuspendedMode:
						BMA280PowerModeCurrentState = St_Deep_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x20, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterSuspendedMode:
						BMA280PowerModeCurrentState = St_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Suspend mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x80, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterLowPowerMode1:
						//Already in this state
						break;
					case EVT_EnterLowPowerMode2:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode2);
						break;
					case EVT_EnterStandByMode:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterStandByMode);
						break;
					default:
						return false;
				}
				break;

			case St_Suspended_Mode:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						BMA280PowerModeCurrentState = St_Normal_Mode_Idle;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterDeepSuspendedMode:
						BMA280PowerModeCurrentState = St_Deep_Suspended_Mode;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x20, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterSuspendedMode:
						//Already in this state
						break;
					case EVT_EnterLowPowerMode1:
						BMA280PowerModeCurrentState = St_Low_Power_Mode_1;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						//Standby mode is entered (left) by writing '1'(0)too the (0x11) suspend
						//bit after bit (0x12) lowpower_mode has been set to 0
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LOW_POWER, 1, 2);
//Add a delay
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x40, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterLowPowerMode2:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode2);
						break;
					case EVT_EnterStandByMode:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterStandByMode);
						return false;
						//break;
					default:
						return false;
				}
				break;


			case St_Deep_Suspended_Mode:
				switch(changePowerModeevent)
				{
					case EVT_EnterNormalMode:
						BMA280PowerModeCurrentState = St_Normal_Mode_Idle;
						StateMachine_BMA280PowerTracingWriteStateRxFifo(BMA280PowerModeCurrentState);

						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						I2CInterface_writeRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 0x00, 1, 1);
						I2CInterface_readRegisterData(IC_ADDRESS_BMA280_ADDRESS, PMU_LPW, 1, 2);
						break;
					case EVT_EnterDeepSuspendedMode:
						//Not allowed - Already in this state
						break;
					case EVT_EnterSuspendedMode:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterSuspendedMode);
						break;
					case EVT_EnterLowPowerMode1:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode1);
						break;
					case EVT_EnterLowPowerMode2:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode2);
						break;
					case EVT_EnterStandByMode:
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
						StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterStandByMode);
						break;
					default:
						return false;
				}
				//break;
		}//switch(NFCstate)
	}//while(IS_DATA_IN_RX_FIFO() == TRUE)
	return true;
}//void fuctionStateMachine(void)

void StateMachine_BMA280PowerTracingWriteEventRxFifo(EVENTS_BMA280_POWERMODE data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_BMA280PowerTracingRxEventFifo[StateMachine_BMA280PowerTracingRxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_BMA280PowerTracingRxEventFifoHead + 1) & (STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_MASK);
    StateMachine_BMA280PowerTracingRxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadEventRxFifo(void){

	unsigned char tmpTail;
	EVENTS_BMA280_POWERMODE dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_BMA280PowerTracingRxEventFifo[StateMachine_BMA280PowerTracingRxEventFifoTail];
	StateMachine_BMA280PowerTracingRxEventFifo[StateMachine_BMA280PowerTracingRxEventFifoTail] = EVT_EnterNormalMode;
	tmpTail = (StateMachine_BMA280PowerTracingRxEventFifoTail+1) & (STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_MASK);
	StateMachine_BMA280PowerTracingRxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadEventTxFifo(void){

	unsigned char tmpTail;
	EVENTS_BMA280_POWERMODE dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_BMA280PowerTracingTxEventFifo[StateMachine_BMA280PowerTracingTxEventFifoTail];
//	StateMachine_processTracingTxEventFifo[StateMachine_processTracingTxEventFifoTail] = EVT_Process_Tracing_Null;
	tmpTail = (StateMachine_BMA280PowerTracingTxEventFifoTail+1) & (STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_MASK);
	StateMachine_BMA280PowerTracingTxEventFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_BMA280PowerTracingWriteEventTxFifo(EVENTS_BMA280_POWERMODE data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_BMA280PowerTracingTxEventFifo[StateMachine_BMA280PowerTracingTxEventFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_BMA280PowerTracingTxEventFifoHead + 1) & (STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_MASK);
    StateMachine_BMA280PowerTracingTxEventFifoHead = tmpHead;
	ENABLE_INTS();

}

/******************************************************************************************
	Function Name: StateMachine_BMA280PowerTracingReadStateRxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: STATES_BMA280_POWERMODE
******************************************************************************************/
STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadStateRxFifo(void){

	unsigned char tmpTail;
	STATES_BMA280_POWERMODE dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_BMA280PowerTracingRxStateFifo[StateMachine_BMA280PowerTracingRxStateFifoTail];
	StateMachine_BMA280PowerTracingRxStateFifo[StateMachine_BMA280PowerTracingRxStateFifoTail] = St_Normal_Mode_Idle;
	tmpTail = (StateMachine_BMA280PowerTracingRxStateFifoTail+1) & (STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_MASK);
	StateMachine_BMA280PowerTracingRxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_BMA280PowerTracingWriteStateRxFifo(STATES_BMA280_POWERMODE data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_BMA280PowerTracingRxStateFifo[StateMachine_BMA280PowerTracingRxStateFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_BMA280PowerTracingRxStateFifoHead + 1) & (STATE_MACHINE_BMA280_POWER_TRACING_RX_STATE_FIFO_MASK);
    StateMachine_BMA280PowerTracingRxStateFifoHead = tmpHead;
	ENABLE_INTS();

}

/******************************************************************************************
	Function Name: StateMachine_BMA280PowerTracingReadStateTxFifo
	Function Description: This function is responsible for
	reading a single byte of data from the rx fifo, and
	updating the appropriate pointers.
	Inputs:  none
	Outputs: STATES_BMA280_POWERMODE
******************************************************************************************/
STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadStateTxFifo(void){

	unsigned char tmpTail;
	STATES_BMA280_POWERMODE dataByte;

	/* just return the current tail from the rx fifo */
	DISABLE_INTS();
	dataByte = StateMachine_BMA280PowerTracingTxStateFifo[StateMachine_BMA280PowerTracingTxStateFifoTail];
//	StateMachine_processTracingTxStateFifo[StateMachine_processTracingTxStateFifoTail] = St_Process_Tracing_Idle;
	tmpTail = (StateMachine_BMA280PowerTracingTxStateFifoTail+1) & (STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_MASK);
	StateMachine_BMA280PowerTracingTxStateFifoTail = tmpTail;
	ENABLE_INTS();

	return(dataByte);

}

void StateMachine_BMA280PowerTracingWriteStateTxFifo(STATES_BMA280_POWERMODE data)
{
	unsigned char tmpHead;

	DISABLE_INTS();
	StateMachine_BMA280PowerTracingTxStateFifo[StateMachine_BMA280PowerTracingTxStateFifoHead] = data;

    /* now move the head up */
    tmpHead = (StateMachine_BMA280PowerTracingTxStateFifoHead + 1) & (STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_MASK);
    StateMachine_BMA280PowerTracingTxStateFifoHead = tmpHead;
	ENABLE_INTS();

}





/**
* @brief  Reads acceleration data from MPU9150
* @details Calls HAL I2C.c functions for accessing I2C bus.
*
* @param  acc_data pointer to the buffer for storing acceleration data
*
* @return Acceleration data from 3 axis
*/
void MPU9150_Read_Acc(int *acc_data)
{
	unsigned char acc [6];

	//UCB0_I2C_read (MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, acc, 6);

	acc_data [0] = 2<<(((int) acc[0])<<8) | acc[1];
	acc_data [1] = 2<<(((int) acc[2])<<8) | acc[3];
	acc_data [2] = 2<<(((int) acc[4])<<8) | acc[5];
}

//
////This shoudl be for the double tap interrupt.
//#pragma vector=PORT1_VECTOR
//__interrupt void PORT1_ISR(void)
//{
//
//	if(PORT_INT1_IFG & INT1)
//	{
//
//		bma280INT1Flag = 1;
//
//		BMA280_dissableINT1Detect();
//		PUBLISH_EVENT(EV_BMA_WATERMARK_INTERRUPT);
//		__bic_SR_register_on_exit(LPM3_bits + GIE); //wake up to handle INTO
//	}
//
//}

//This is for the new BMA280 data ready pin
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{

	if(PORT_INT2_IFG & INT2)
	{

		bma280INT2Flag = 1;
		BMA280_dissableINT2Detect();
		PUBLISH_EVENT(EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE);

		/* Exit LPM */
		__no_operation();
		__bic_SR_register_on_exit(LPM3_bits); // Exit LPM
	}
//	__bic_SR_register_on_exit(GIE);

}




