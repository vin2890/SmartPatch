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
//! \file   Timer_A.c
//!
//! \brief  Please see Timer_A.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

#include "Timer_A.h"
#include <msp430.h>
#include <stdint.h>
#include "Executive.h"
#include "Events.h"
#include "TimerInterface.h"
#include "GSRInterface.h"
#include "DEBUG.h"


extern _Bool acquireTemperatureDataLogger;
extern _Bool acquireTemperatureStreaming;
extern _Bool acquireAccelerationDataLogger;
extern _Bool acquireGSRDataLogger;
extern _Bool acquireGSRStreaming;

void TimerA_ConfigTimer(void)
{

	/*Timer_A0 Control Register
		TASSEL__ACLK - Timer_A clock source select - (ACLK)
		ID_3 --------- Input divider. These bits along with the TAIDEX
					   bits select the divider for the input clock - (/8)
		TACLR -------- Timer_A clear. (Setting this bit resets TAxR, the timer
				  	   clock divider logic, and the count direction. The TACLR
				  	   bit is automatically reset and is always read as zero.)
		TAIE --------- Timer_A interrupt enable. - (Overflow Interrupt enabled)
		MC_1 --------- Timer only counts up to CCR0 */
    if(!DEBUG){
        TA0CTL |= TASSEL__ACLK | ID_3 | TACLR;
        //Input divider expansion - Divide by 8
        TA0EX0 |= TAIDEX_7;
    }else{
        TA0CTL |= TASSEL__ACLK | ID_0 | MC_1 | TACLR;
        //Input divider expansion - Divide by 1
        TA0EX0 |= TAIDEX_0;
    }

}

void TimerA_StopMode(void)
{
	//Timer_A0 Control Register
		//TA0CTL - Mode control (Up mode: Timer counts up to TAxCCR0)
	TA0CTL &= ~MC_3;
	TA0CTL &= ~MC_1;

}

void TimerA_UpMode(void)
{
	//Timer_A0 Control Register
		//TA0CTL - Mode control (Up mode: Timer counts up to TAxCCR0)
	TA0CTL |= MC_1;

}

void TimerA_clearTimer(void)
{
	//Timer_A0 Control Register
	TA0CTL |= TACLR;

}

void TimerA_overflowInterruptEnabled(void)
{
	//Timer_A0 Control Register
		//TAIE - Timer_A interrupt enable(Interrupt enabled)
	TA0CTL |=  TAIE;

}

void TimerA_overflowinterruptDisabled(void)
{
	//Timer_A0 Control Register
		//TAIE - Timer_A interrupt enable(Interrupt disabled)
	TA0CTL &= ~TAIE;

}

void TimerA_CCR0InterruptEnable(void)
{

	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL0 |= CCIE;

}

void TimerA_CCR0InterruptDisabled(void)
{
	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL0 &= ~CCIE;

}

void TimerA_ConfigCCR0(uint16_t compareValue)
{
	//Timer_A Capture/Compare 0 Register
		//Compare mode: TA0CCR0 holds the data for the comparison to the timer value
		//in the Timer_A Register, TAR.
	TA0CCR0 = compareValue;

}

void TimerA_CCR1InterruptEnable(void)
{
	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL1 |= CCIE;

}

void TimerA_CCR1InterruptDisabled(void)
{
	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL1 &= ~CCIE;

}

void TimerA_ConfigCCR1(uint16_t compareValue)
{
	//Timer_A Capture/Compare 1 Register
		//Compare mode: TA0CCR1 holds the data for the comparison to the timer value
		//in the Timer_A Register, TAR.
	TA0CCR1 = compareValue;

}

void TimerA_ADC12BTriggerSource(void)
{
	//Timer_A Capture/Compare 1 Register
		//OUTMOD_3 - Output mode (Set/reset)
	TA0CCTL1 &= ~OUTMOD_3;
	TA0CCTL1 |= OUTMOD_3;

}

void TimerA_CCR2InterruptEnable(void)
{

	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL2 |= CCIE;

}

void TimerA_CCR2InterruptDisabled(void)
{

	/*Timer_A0 Capture/Compare Control 0 Register
		CCIE --------- Capture/compare interrupt enable (Interrupt enabled) */
	TA0CCTL2 &= ~CCIE;

}

void TimerA_ConfigCCR2(unsigned int compareValue)
{
	//Timer_A Capture/Compare 2 Register
		//Compare mode: TA0CCR2 holds the data for the comparison to the timer value
		//in the Timer_A Register, TAR.
	TA0CCR2 = compareValue;
}

void TimerA_clockDivider(uint16_t IDClockDivider, uint16_t EX0ClockDivider)
{

	/* First clear the ID bit within the TA0CTL register. */
	TA0CTL &= ~ID_3;

	/* Now set the ID divider. */
	TA0CTL |= IDClockDivider;

	/* First clear the TAIDEX bit within the TA0EX0 register. */
	TA0EX0 &= ~TAIDEX_7;

	/* Now set the TAIDEX divider. */
	TA0EX0 |= EX0ClockDivider;

}

/***********************************************************
	Function Name: Timer0_A0_ISR ISR
	Function Description: This is the CCR0 interrupt service routine.
	The only interrupt that will cause this interrupt to fire is the
	TA0CCTL0. If the overflow interrupt (TAIE) is also enabled, then
 	back to back interrupts will occur.

	Inputs:  none
	Outputs: none
	NOTES: none
***********************************************************/
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
{

	  switch(__even_in_range(TA0IV, TA0IV_TAIFG))
	  {
	    case TA0IV_NONE:
	    	__no_operation();
	    	break;               // No interrupt
	    case TA0IV_TACCR1:
	    	__no_operation();
	    	break;
	    case TA0IV_TACCR2:
	    	__no_operation();
	    	break;
	    case TA0IV_3:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_4:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_5:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_6:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_TAIFG:                       // overflow
	    	 TA0CTL &= ~TAIFG;
	    	__no_operation();
	    	break;
	    default:
	    	__no_operation();
	    	break;
	  }
}


/***********************************************************
	Function Name: Timer0_A1_ISR ISR
	Function Description: This is the Timer general interrupt service
	routine. All of the interrupts that are possible by the Timer_A
	with the exception of the CCR0 interrupt, are service inside this
	interrupt service routine. CCR1, CCR2, CCR3 and Timer overflow can
	cause this interrupt to fire.

	Inputs:  none
	Outputs: none
	NOTES: This function could probrably be improved. If
	improvements are detected, please contact the developer
	and suggest the improvments.
***********************************************************/
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{

	  switch(__even_in_range(TA0IV, TA0IV_TAIFG))
	  {
	    case TA0IV_NONE:
	    	__no_operation();
	    	break;               // No interrupt
	    case TA0IV_TACCR1:
	        if(!DEBUG){
                if(acquireTemperatureDataLogger)
                {

                    /* Do not turn off the timer. Simply pause it */
                    TimerInterface_powerMode1();

                    /* Advance to the next state in acquiring temperature. */
                    PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE);

                    /* Exit LPM */
                    __no_operation();
                    __bic_SR_register_on_exit(LPM3_bits); // Exit LPM

                }
	        }else{
	                PUBLISH_EVENT(EV_ECG_DATA_READY);
	        }
	    	break;
	    case TA0IV_TACCR2:
	    	if(acquireTemperatureDataLogger)
	    	{

				/* Do not turn off the timer. Simply pause it */
	    		TimerInterface_powerMode1();

	    		/* Advance to the next state in acquiring temperature. */
	    		PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE);

	    		/* Exit LPM */
	    		__no_operation();
				__bic_SR_register_on_exit(LPM3_bits);
	    	}

	    	break;
	    case TA0IV_3:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_4:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_5:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_6:
	    	__no_operation();
	    	break;               // reserved
	    case TA0IV_TAIFG:        // overflow
	    	 TA0CTL &= ~TAIFG;
			if(acquireTemperatureDataLogger)
			{

				/* Do not turn off the timer. Simply pause it */
				TimerInterface_powerMode1();

				/* Advance to the next state in acquiring temperature. */
				PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE);

				/* Exit LPM */
				__no_operation();
				__bic_SR_register_on_exit(LPM3_bits); // Exit LPM
			}

			else if(acquireTemperatureStreaming)
			{

				/* Do not turn off the timer. Simply pause it */
				TimerInterface_powerMode1();

				/* Advance to the next state in acquiring temperature. */
				PUBLISH_EVENT(EV_ACQUIRE_TEMPERATURE_STREAMING_STOP);

				/* Exit LPM */
				__no_operation();
				__bic_SR_register_on_exit(LPM3_bits); // Exit LPM
			}

			else if(acquireGSRStreaming)
			{

				/* Do not turn off the timer. Simply pause it */
				TimerInterface_powerMode1();

				/* Advance to the next state in acquiring temperature. */
				PUBLISH_EVENT(EV_ACQUIRE_GSR_STREAMING);

				/* Exit LPM */
				__no_operation();
				__bic_SR_register_on_exit(LPM3_bits); // Exit LPM
			}

			else if(acquireAccelerationDataLogger)
			{
				/* Do not turn off the timer. Simply pause it */
				TimerInterface_powerMode1();

				/* Advance to the next state in acquiring temperature. */
				PUBLISH_EVENT(EV_ACQUIRE_PEDOMETER_DATA_LOG_MODE_UPDATE_STEPCOUNT_AND_SAMPLE_NUMBER);

				P3OUT ^= (1 << 3);


				/* Exit LPM */
				__no_operation();
				__bic_SR_register_on_exit(LPM3_bits); // Exit LPM

			}



			break;
	    default:
	    	__no_operation();
	    	break;
	  }

}
