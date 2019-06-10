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
//! \file   BMA280.h
//!
//! \brief  This is the IC driver module for the BMA280 IC. This IC driver
//!			file contains function calls which are only found in the I2C Interface
//!			file. If the corresponding (I2C)Interface file is modified, then
//!			this IC driver file must also be modified. Although, in this case the
//!			file is set up to interface to the/a I2C peripheral interface file,
//!			it can be modified to interact with a SPI interface file, UART
//!			interface file, USB interface file, etc. The reason why this IC driver
//!			file is set up to interface with I2C peripheral interface file is
//!			because the BMA280, RF430CL331, and TMP112A ICs use I2C communication
//!			protocole.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef BMA280_H_
#define BMA280_H_

//*****************************************************************************
// the includes
//*****************************************************************************
#include "types.h"
#include "driverlib/MSP430FR5xx_6xx/gpio.h"

//! \brief This define contains the address of the RF430CL331 IC.
#define IC_ADDRESS_BMA280_ADDRESS 	0x0019

//! \brief BMA280 register address
#define BGW_CHIPID 		0x00

#define ACCD_X_LSB		0x02
#define ACCD_X_MSB		0x03

#define ACCD_Y_LSB		0x04
#define ACCD_Y_MSB		0x05

#define ACCD_Z_LSB		0x06
#define ACCD_Z_MSB		0x07

#define ACCD_TEMP		0x08
#define INT_STATUS_0	0x09
#define INT_STATUS_1	0x0A
#define INT_STATUS_2	0x0B
#define INT_STATUS_3	0x0C
#define FIFO_STATUS		0x0E
#define	PMU_RANGE		0x0F
#define PMU_BW			0x10
#define PMU_LPW			0x11
#define PMU_LOW_POWER	0x12
#define ACCD_HBW		0x13
#define BGW_SOFTRESET	0x14
#define	INT_EN_0		0x16
#define	INT_EN_1		0x17
#define	INT_EN_2		0x18
#define	INT_MAP_0		0x19
#define	INT_MAP_1		0x1A
#define	INT_MAP_2		0x1B
#define	INT_SRC			0x1E
#define	INT_OUT_CTRL	0x20
#define	INT_RST_LATCH	0x21
#define	INT_0			0x22
#define	INT_1			0x23
#define	INT_2			0x24
#define	INT_3			0x25
#define	INT_4			0x26
#define	INT_5			0x27
#define	INT_6			0x28
#define	INT_7			0x29
#define	INT_8			0x2A
#define	INT_9			0x2B
#define	INT_A			0x2C
#define	INT_B			0x2D
#define	INT_C			0x2E
#define	INT_D			0x2F
#define	FIFO_CONFIG_0	0x30
#define	PMU_SELF_TEST	0x32
#define	TRIM_NVM_CTRL	0x33
#define	BGW_SPI3_WDT	0x34
#define	OFC_CTRL		0x36
#define	OFC_SETTING		0x37
#define	OFC_OFFSET_X	0x38
#define	OFC_OFFSET_Y	0x39
#define	OFC_OFFSET_Z	0x3A
#define	TRIM_GP0		0x3B
#define	TRIM_GP1		0x3C
#define	FIFO_CONFIG_1	0x3E
#define	FIFO_DATA		0x3F

//! \brief INT1
#define PORT_INT1_IN	P1IN
#define PORT_INT1_OUT	P1OUT
#define PORT_INT1_DIR	P1DIR
#define PORT_INT1_SEL0	P1SEL0
#define PORT_INT1_SEL1	P1SEL1
#define PORT_INT1_REN	P1REN
#define PORT_INT1_IE	P1IE
#define PORT_INT1_IES	P1IES
#define PORT_INT1_IE	P1IE
#define PORT_INT1_IFG	P1IFG
#define INT1			BIT1

//! \brief INT2
#define PORT_INT2_IN	P2IN
#define PORT_INT2_OUT	P2OUT
#define PORT_INT2_DIR	P2DIR
#define PORT_INT2_SEL0	P2SEL0
#define PORT_INT2_SEL1	P2SEL1
#define PORT_INT2_REN	P2REN
#define PORT_INT2_IE	P2IE
#define PORT_INT2_IES	P2IES
#define PORT_INT2_IE	P2IE
#define PORT_INT2_IFG	P2IFG
#define INT2			BIT0


//! \brief State machine defines
#define STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_MASK STATE_MACHINE_BMA280_POWER_TRACING_RX_EVENT_FIFO_SIZE-1
#define STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_SIZE 16
#define STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_MASK STATE_MACHINE_BMA280_POWER_TRACING_TX_EVENT_FIFO_SIZE-1

#define STATE_MACHINE_BMA280_POWER_TRACING_RX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_BMA280_POWER_TRACING_RX_STATE_FIFO_MASK STATE_MACHINE_BMA280_POWER_TRACING_RX_STATE_FIFO_SIZE-1
#define STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_SIZE 16
#define STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_MASK STATE_MACHINE_BMA280_POWER_TRACING_TX_STATE_FIFO_SIZE-1

//! \brief This enum defines all the power states that the BMA280 can be in
//!
typedef enum {
	St_Normal_Mode_Idle,
	St_Low_Power_Mode_2,
	St_Standby_Mode,
	St_Low_Power_Mode_1,
	St_Suspended_Mode,
	St_Deep_Suspended_Mode,

} STATES_BMA280_POWERMODE;

//! \brief This enum defines all the power state events that can be called
//!
typedef enum {
	EVT_EnterNormalMode,
	EVT_EnterDeepSuspendedMode,
	EVT_EnterSuspendedMode,
	EVT_EnterLowPowerMode1,
	EVT_EnterLowPowerMode2,
	EVT_EnterStandByMode

} EVENTS_BMA280_POWERMODE;


EVENTS_BMA280_POWERMODE bmaPowerMode;

uint8_t bma280INT1Flag;
uint8_t bma280INT2Flag;

//void MPU9150_Init();

//*****************************************************************************
//
//! \brief   This function initializes the BMA280 IC. The user should read the
//!			 BMA280 datasheet if deep understanding of the initialization procure
//!			 is desired.Because the functions call is remote, it has the
//!			 possibility to hang. Caution should be taken when calling this function.
//
//! \param none     None
//
//! \return  \b NO_ERROR, or \b ERROR
//
//*****************************************************************************
uint16_t BMA280_remoteRegisterInit();

//*****************************************************************************
//
//! \brief   The BMA280 IC contains two programmable I/O interrupt pins. In this
//!			 case port pin P2.0 is configured as the data ready pin. Whenever the
//!			 BMA280 has new accelerometer data for the MSP430, it interrupts the
//!			 MSP430 by pulling the line low. By calling this function, an interrupt
//!			 will not occur if the line is pulled low.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_disableDataReadyInterrupt(void);

//*****************************************************************************
//
//! \brief   The BMA280 IC contains two programmable I/O interrupt pins. In this
//!			 case port pin P2.0 is configured as the data ready pin. Whenever the
//!			 BMA280 has new accelerometer data for the MSP430, it interrupts the
//!			 MSP430 by pulling the line low. By calling this function, an interrupt
//!			 will occur if the line is pulled low.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_enableDataReadyInterrupt(void);

//*****************************************************************************
//
//! \brief   This function reads the X acceleration value from the BMA280. It
//!			 does so by enabling the interrupt and going to sleep.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
uint16_t BMA280_ReadXAcceleration(void);

//*****************************************************************************
//
//! \brief   This function reads the Y acceleration value from the BMA280. It
//!			 does so by enabling the interrupt and going to sleep.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
uint16_t BMA280_ReadYAcceleration(void);

//*****************************************************************************
//
//! \brief   This function reads the Z acceleration value from the BMA280. It
//!			 does so by enabling the interrupt and going to sleep.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
uint16_t BMA280_ReadZAcceleration(void);

//*****************************************************************************
//
//! \brief   This function configures the port pin needed for the BMA280 interrupts.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_GPIOInit(void);

//*****************************************************************************
//
//! \brief   The BMA280 IC contains two programmable I/O interrupt pins. In this
//!			 case data ready and double tap are configured as interrupts.  This
//!			 function call configures port pins P1.1 and P2.0 as inputs which will
//!			 generate an interrupt in a high to low transition of the line. The
//!			 BMA280 INT1 pin which is tied to Port pin P1.1 is set for double
//!			 tap interrupt and BMA280 INT2 pin which is tied to port pin 2 is
//!			 set for data ready interrupt.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_PeripheralInit(void);

//*****************************************************************************
//
//! \brief   This function disables the BMA280 INT1 interrupt which is tied
//!			 to MSP430 port pin P1.1.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_dissableINT1Detect(void);

//*****************************************************************************
//
//! \brief   This function enables the BMA280 INT1 interrupt which is tied to
//!			 MSP430 port pin P1.1.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_enableINT1Detect(void);

//*****************************************************************************
//
//! \brief   This function disables the BMA280 INT2 interrupt which is tied
//!			 to MSP430 port pin P2.0.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_dissableINT2Detect(void);

//*****************************************************************************
//
//! \brief   This function enables the BMA280 INT2 interrupt which is tied to
//!			 MSP430 port pin P2.0.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_enableINT2Detect(void);

//*****************************************************************************
//
//! \brief   This functions does a software reset on the BMA280. All the BMA280
//!			 registers should be reconfigured after a call to this function.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void BMA280_softReset(void);

//*****************************************************************************
//
//! \brief   This function takes care of transition from different power states.
//!			 The BMA280 contains six power states. Transition from one power
//!			 state to another is not completely supported. Sometimes an in between
//!			 state must be entered before the final sate can be entered. This
//!			 function takes care of the transitioning between states.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
BOOL BMA280_PowerModeStateMachine(void);

//*****************************************************************************
//
//! \brief   This function reads the events which are written by the
//!			 StateMachine_BMA280PowerTracingWriteEventRxFifo() function
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadEventRxFifo(void);

//*****************************************************************************
//
//! \brief   This function set the desired power mode. The
//!			 StateMachine_BMA280PowerTracingReadEventRxFifo function will read
//!			 the event which has been written.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************

void StateMachine_BMA280PowerTracingWriteEventRxFifo(EVENTS_BMA280_POWERMODE data);

//*****************************************************************************
//
//! \brief   This function is for debugging purposes only.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
EVENTS_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadEventTxFifo(void);

//*****************************************************************************
//
//! \brief   This function is for debugging purposes only.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void StateMachine_BMA280PowerTracingWriteEventTxFifo(EVENTS_BMA280_POWERMODE data);

//*****************************************************************************
//
//! \brief   This function determines what the next state will be.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadStateRxFifo(void);

//*****************************************************************************
//
//! \brief   This function determines what the next state will be.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void StateMachine_BMA280PowerTracingWriteStateRxFifo(STATES_BMA280_POWERMODE data);

//*****************************************************************************
//
//! \brief   This function is for debugging purposes only.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
STATES_BMA280_POWERMODE StateMachine_BMA280PowerTracingReadStateTxFifo(void);

//*****************************************************************************
//
//! \brief   This function is for debugging purposes only.
//
//! \param none     None
//
//! \return  none
//
//*****************************************************************************
void StateMachine_BMA280PowerTracingWriteStateTxFifo(STATES_BMA280_POWERMODE data);

void MPU9150_Read_Acc(int *acc_data);



#endif /* BMA280_H_ */
