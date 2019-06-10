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
//! \file   AccelerometerInterface.c
//!
//! \brief  Please see AccelerometerInterface.h
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
#include "types.h"
#include "BMA280.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "AccelerometerInterface.h"
#include "BMA280.h"
#include "NFCForumType4TagType.h"
#include "StateMachine.h"
#include "TimerInterface.h"
#include "Executive.h"
#include "pedometer/pedometer.h"
#include "Timer_A.h"
#include "NFCInterface.h"
#include "I2CInterface.h"
#include "DEBUG.h"
#include "UART.h"
#include "RingBuffer.h"

uint16_t desiredAccelerationSamples = 0, timeBetweenSamples, accelerationSampleCounter=0;
uint16_t E108PLEN = 0;
_Bool acquireAccelerationDataLogger = false;

unsigned short app_stepcount = 0;
unsigned char batStateRequested = 0;
unsigned char updateStepRequested = 0;
unsigned char disconnectRequested = 0;
unsigned char isConnected = 0;
unsigned char accDataIsReady = 0;
unsigned char resetStepRequested = 0;
unsigned char eventDetected = 0;

// the pedometer algorithm gets a signed short array
// acc union is defined as a bridge between the accelerometer data structure and the unsigned short array because casting does not work
union acc {
	signed short acc_array [10];
	acc_data_t acc_struct;
};

//! \brief	This Enum lists the possible states when in acceleration data logging mode. When
//!			in St_Pedometer_Initialize_Pedometer_Algorithm state basically the acceleration
//!			sensor is enabled, when in St_Pedometer_Data_Ready state basically the acceleration
//!			is being read from the acceleration sensor, and when in St_Pedometer_PowerDown_IC
//!			the acceleration sensor is once again powered down to conserve battery.
typedef enum {
	St_Pedometer_Initialize_Pedometer_Algorithm,
	St_Pedometer_Data_Ready,
	St_Pedometer_PowerDown_IC,
} STATES_PEDOMETER_DATA_LOGGING;

STATES_PEDOMETER_DATA_LOGGING static currentPedometerDataLoggingState = St_Pedometer_Initialize_Pedometer_Algorithm;

union acc acc_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#pragma PERSISTENT(array_x)
int16_t array_x[1000] = {0};

#pragma PERSISTENT(array_y)
int16_t array_y[1000] = {0};

#pragma PERSISTENT(array_z)
int16_t array_z[1000] = {0};

uint16_t AccelerometerInterface_Initialize(void)
{
    if(DEBUG)
        return 0;

	//The BMA280 is equipped with two programmable interrupt pins. Both pins were connected to theMSP430. They were configured as Data Ready Interrupt and Double tap Interrupt.
	BMA280_PeripheralInit();

	/* This is needed because the reset that follows will cause a false ready interrupt.  */
	BMA280_dissableINT1Detect();

	/* This is needed because the reset that follows will cause a false ready interrupt.  */
	BMA280_dissableINT2Detect();

	//The software reset is needed for if in case the patch did not shutdown properly and the BMA20 is still in active mode.
	BMA280_softReset();

	//To conserver battery place the BMA280 in suspended mode.
	AccelerometerInterface_powerMode4();

	//This is the register initialization. All the BMA280 registers are initialized by this function call.
	BMA280_remoteRegisterInit();

	//Enable the MSP40 GPIO Interrupt pins to. Interrupt 1 is double tap
	//	BMA280_enableINT1Detect();

	//Enable the MSP40 GPIO Interrupt pins to. Interrupt 2 is data ready
	//    BMA280_enableINT2Detect();

	return 0;
}



//This is a local function
void AccelerometerInterface_dataLoggerMode(void)
{
    // Debug
    //array_x[accelerationSampleCounter] = AccelerometerInterface_ReadXAcceleration();
    //array_y[accelerationSampleCounter] = AccelerometerInterface_readYAcceleration();
    //array_z[accelerationSampleCounter] = AccelerometerInterface_readZAcceleration();

    P3OUT ^= (1 << 0);
    P3OUT ^= (1 << 0);
    P3OUT ^= (1 << 0);
    P3OUT ^= (1 << 0);

    switch(currentPedometerDataLoggingState)
    {

        case (St_Pedometer_Initialize_Pedometer_Algorithm):

            /* From now on, service data ready interrupt sent by the motion sensor. */
            BMA280_enableINT2Detect();

            /* As can be guessed the Pedometer algorithm has been initialized. */
            ped_step_detect_init();

            /* This flag is used by the TimerInterface. This flag lets the timer know that Acceleration data tracking is desired. */
            acquireAccelerationDataLogger = true;

            /* The state to advance to. */
            currentPedometerDataLoggingState = St_Pedometer_Data_Ready;

            /* Enable timer. This will wake up the MCU when its goes into sleep mode */
            TimerInterface_powerMode0();

            /* In data logger since the NFC interface is not needed mode while the MPBSM system is logging in the data
             * values, it makes sense to disable the NFC IC. When the data values are gathered, re-enable the NFC interface.  */
            NFCInterface_powerMode1();

            /* Enable the accelerometer sensor. Take into consideration that current consumption will increase. */
            AccelerometerInterface_powerMode0();

            break;

        case (St_Pedometer_Data_Ready):

                /* Pause the timer here because a timer interrupt will cause a wake up in the I2C routine which will cause the firmware to hang.  */
                TimerInterface_powerMode1();

//              array_x[accelerationSampleCounter] = AccelerometerInterface_ReadXAcceleration();
//              array_y[accelerationSampleCounter] = AccelerometerInterface_readYAcceleration();
//              array_z[accelerationSampleCounter] = AccelerometerInterface_readZAcceleration();

                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);

                /* Read the x acceleration. */
                acc_data.acc_struct.x_raw = AccelerometerInterface_ReadXAcceleration();

                /* Read the x acceleration. */
                acc_data.acc_struct.y_raw = AccelerometerInterface_readYAcceleration();

                /* Read the x acceleration. */
                acc_data.acc_struct.z_raw = AccelerometerInterface_readZAcceleration();

                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);

                /* Acceleration reads have finished hence it is safe to re-enable the counter.  */
                TimerInterface_powerMode0();

    //          //MPU9150_Read_Acc (&acc_data.acc_struct.x_raw);
                acc_data.acc_struct.x = acc_data.acc_struct.x_raw;
                acc_data.acc_struct.y = acc_data.acc_struct.y_raw;
                acc_data.acc_struct.z = acc_data.acc_struct.z_raw;
                if(ped_update_sample((signed short *) acc_data.acc_array) == 1)
                {
                    app_stepcount = ped_step_detect();
                }

                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);
                P3OUT ^= (1 << 0);

                BMA280_enableINT2Detect();

            break;

    }

       char header = 0x20; //ACCEL HEADER
       char ACCELHeader = 0x2F;// ACCEL LIVE STREAMING
       unsigned char endDataHeader = 0x03;
       ring_buffer_put(_outputrb_d,&header);
       ring_buffer_put(_outputrb_d,&ACCELHeader);
       ring_buffer_put(_outputrb_d,&acc_data.acc_struct.x_raw);
       ring_buffer_put(_outputrb_d,&acc_data.acc_struct.y_raw);
       ring_buffer_put(_outputrb_d,&acc_data.acc_struct.z_raw);
       ring_buffer_put(_outputrb_d,&app_stepcount);
       ring_buffer_put(_outputrb_d,&endDataHeader);
       unsigned char newline = 0x0D;
       ring_buffer_put(_outputrb_d,&newline);
       PUBLISH_EVENT(EV_UART_TX);
}
void AccelerometerInterface_updateStepcountAndSampleNumber(void)
{
	P3OUT ^= (1 << 3);

	/* Update how many steps were taken in the time interval. */
	FileTextE108[accelerationSampleCounter+8] = app_stepcount;

	/* Update the sample counter */
	accelerationSampleCounter++;

	/* The acceleration Sample Counter variable is constantly updating the Cadence file so if incase the battery drains. High byte */
	FileTextE108[5] = accelerationSampleCounter>>8;

	/* The acceleration Sample Counter variable is constantly updating the Temperature file so if incase the battery drains. Low byte */
	FileTextE108[6] = accelerationSampleCounter & 0x00FF;

	P3OUT ^= (1 << 3);
	P3OUT ^= (1 << 3);

}

void AccelerometerInterface_setDataLoggerModeInitialize(void)
{

	/* This informs the app that the patch is in the process of acquiring the temperature */
	FileTextE105[2] = PATCH_BUSY;

	/*  */
	FileTextE105[12] = ACK;

	/* Calculate the time between samples. Calculate the low byte. */
	timeBetweenSamples = FileTextE105[5];

	/* Calculate the time between samples. Calculate the high byte. */
	timeBetweenSamples = ((timeBetweenSamples << 8)  | FileTextE105[6] );

	/* The bumber of samples variable is read from the command file which is provided by the Android app. */
	desiredAccelerationSamples = FileTextE105[7];

	/* The bumber of samples variable is read from the command file which is provided by the Android app. */
	desiredAccelerationSamples = ((desiredAccelerationSamples << 8)  | FileTextE105[8] );

	/* Set the Timer1 frequency howerver don't enable the timer.  */
	TimerInterface_timer1Frequency(timeBetweenSamples, ID_3, TAIDEX_7);

	/* Start the process to acquire desiredSamples number of temperature samples at timeBetweenSamples specified intervals. */
	PUBLISH_EVENT(EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE);

}



void AccelerometerInterface_disableDataReadyInterrupt(void)
{
	BMA280_disableDataReadyInterrupt();
}

void AccelerometerInterface_enableDataReadyInterrupt(void)
{
	//Data Readu interrupt enable
	BMA280_enableDataReadyInterrupt();
}

void AccelerometerInterface_powerMode0(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterNormalMode);
	BMA280_PowerModeStateMachine();
}

void AccelerometerInterface_powerMode1(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode2);
	BMA280_PowerModeStateMachine();
}

void AccelerometerInterface_powerMode2(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterStandByMode);
	BMA280_PowerModeStateMachine();
}

void AccelerometerInterface_powerMode3(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterLowPowerMode1);
	BMA280_PowerModeStateMachine();
}

void AccelerometerInterface_powerMode4(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterSuspendedMode);
	BMA280_PowerModeStateMachine();
}

void AccelerometerInterface_powerMode5(void)
{
	StateMachine_BMA280PowerTracingWriteEventRxFifo(EVT_EnterDeepSuspendedMode);
	BMA280_PowerModeStateMachine();
}

uint16_t AccelerometerInterface_ReadXAcceleration(void)
{
	return BMA280_ReadXAcceleration();
}

uint16_t AccelerometerInterface_readYAcceleration(void)
{
	return BMA280_ReadYAcceleration();
}

uint16_t AccelerometerInterface_readZAcceleration(void)
{
	return BMA280_ReadZAcceleration();
}

uint16_t AccelerometerInterface_readxyzAcceleration(void)
{
	return 0;
}
