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
//! \file   ADC12_B.c
//!
//! \brief  Please see ADC12_B.h
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
#include "ADC12_B.h"
#include "Executive.h"
#include "TimerInterface.h"

int16_t  ADCValues;
extern _Bool acquireGSRDataLogger;

void ADC12B_init(void)
{


	//ADC12_B Control 0 Register
		//ADC12ENC - ADC12_B enable conversion (ADC12_B disabled)
	ADC12CTL0 &= ~ADC12ENC;

	//ADC12_B Control 1 Register
		//ADC12PDIV_0 - ADC12_B predivider (Predivide by 1)
		//ADC12SHS_1 - ADC12_B sample-and-hold source select (Timer_A TA0 CCR1 output)
		//ADC12DIV_0 - ADC12_B clock divider(/1)
		//ADC12SSEL_1 - ADC12_B clock source select (ACLK)
		//ADC12CONSEQ_2 - ADC12_B conversion sequence mode select (Repeat-single-channel)
	ADC12CTL1 = ADC12PDIV_0 | ADC12SHS_1 | ADC12DIV_0 | ADC12SSEL_1 | ADC12CONSEQ_2;

	//ADC12_B Control 2 Register
		//ADC12RES_2 - ADC12_B resolution (12 bit (14 clock cycle conversion time))
	ADC12CTL2 = ADC12RES_2;

	//ADC12_B Control 3 Register
		//ADC12CSTARTADD_0 - ADC12_B conversion start address.(ADC12MEM0/ADC12MCTL0)
	ADC12CTL3 = ADC12CSTARTADD_0;

	//ADC12_B Conversion Memory Control 0 Register
		//ADC12VRSEL_0 - Selects combinations of VR+ and VR- sources (0000b = VR+ = AVCC, VR- = AVSS)
		//ADC12INCH_2 - Input channel select (A2)
		//ADC12EOS - End of sequence (End of sequence)
	ADC12MCTL0 = ADC12VRSEL_0 | ADC12INCH_2 | ADC12EOS;

	//ADC12_B Interrupt Enable 0 Register
		//ADC12IE0 - Interrupt enable (Interrupt enabled)
	ADC12IER0 |= ADC12IE0;

	//ADC12_B Interrupt Enable 2 Register
		//ADC12TOVIE - ADC12_B conversion-time-overflow interrupt enable (Interrupt enabled)
		//ADC12OVIE - ADC12MEM0 overflow-interrupt enable (Interrupt enabled)
	ADC12IER2 |= ADC12TOVIE | ADC12OVIE;

	//ADC12_B Control 0 Register
		//ADC12ENC - ADC12_B enable conversion (ADC12_B enable)
	ADC12CTL0 |= ADC12ENC;

}

void ADC12_PortsInit(void)
{

	/* Set Port pin P1.2 as intput. */
	PORT_ADC_DIR &= ~A2;

	/* configure the port pins P1.2 for peripheral. */
	PORT_ADC_SEL |= A2;

}

BOOL ADC12_busy(void)
{

	/* Isolate the ADC12BUSY bit in the ADC12_B Control 1 Register. Next, check only that bit to see if it is set.  */
	if(ADC12CTL1 & ADC12BUSY)

	/* It is set hence return true. */
	return true;

	/* It is not set hence return false. */
	else return false;

}

void ADC12_enableADC_B(void)
{

	/* ADC12_B Control 0 Register.*/
		/* ADC12ON - The ADC12ON bit enables the core. */
	ADC12CTL0 |= ADC12ON;

}

void ADC12_disableADC_B(void)
{

	/* ADC12_B Control 0 Register. */
		/* ADC12ON - The ADC12ON bit disables the core. */
	ADC12CTL0 &= ~ADC12ON;

}

void ADC12_enableConversion(void)
{

	/* ADC12_B Control 0 Register. */
		/* ADC12_B enable conversion(ADC12_B enabled). */
	ADC12CTL0 |= ADC12ENC;

}

void ADC12_disableConversion(void)
{
	/* ADC12_B Control 0 Register. */
		/* ADC12ENC - ADC12_B enable conversion(ADC12_B disabled). */
	ADC12CTL0 &= ~ADC12ENC;

}

void ADC12_SoftwareStartConversion(void)
{

	/* ADC12_B start conversion. Software-controlled sample-and-conversion start.
	   ADC12SC and ADC12ENC may be set together with one instruction. ADC12SC
	   is reset automatically.*
		/* ADC12SC - Start sample-and-conversion. */
	ADC12CTL0 |= ADC12SC;

}

void ADC12_SHISourceSC(void)
{
	/* ADC12_B Control 1 Register. */
		/* ADC12SHS_0 - Set the trigger source for ADC12SC. */
	ADC12CTL1 &= ~ADC12SHS_0;
}

void ADC12_SHISourcenCCR1(void)
{

	/* ADC12_B Control 1 Register. */
		/* ADC12SHS_0 - Clear the ADC12SHS bits*/
	ADC12CTL1 &= ~ADC12SHS_0;

	/* ADC12_B Control 1 Register. */
		/* ADC12SHS_0 - Set the trigger source to be "Timer_A TA0 CCR1 output". */
	ADC12CTL1 |= ADC12SHS_1;

}

void ADC12_lowPowerModeEnable(void)
{
	/* ADC12_B Control 1 Register. */
		/*	ADC12PWRMD - Low power mode enable, ADC12CLK can not be greater than 1/4 the
			device-specific data sheet specified maximum for ADC12PWRMD = 0 */
	ADC12CTL2 |= ADC12PWRMD;
}

void ADC12_lowPowerModeDisable(void)
{
	/* ADC12_B Control 1 Register. */
		/* Regular power mode where sample rate is not restricted.*/
	ADC12CTL2 &= ~ADC12PWRMD;
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
  {
    case ADC12IV_NONE:
    	__no_operation();
    	break;        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG:
    	__no_operation();
    	break;        // Vector  2:  ADC12MEMx Overflow
    case ADC12IV_ADC12TOVIFG:
    	__no_operation();
    	break;        // Vector  4:  Conversion time overflow
    case ADC12IV_ADC12HIIFG:
    	__no_operation();
    	break;        // Vector  6:  ADC12BHI
    case ADC12IV_ADC12LOIFG:
    	__no_operation();
    	break;        // Vector  8:  ADC12BLO
    case ADC12IV_ADC12INIFG:
    	__no_operation();
    	break;        // Vector 10:  ADC12BIN
    case ADC12IV_ADC12IFG0:
    				 // Vector 12:  ADC12MEM0 Interrupt
    	ADCValues = ADC12MEM0;
		if(acquireGSRDataLogger)
		{
			/* Do not turn off the timer. Simply pause it */
			TimerInterface_powerMode1();

			/* Advance to the next state in acquiring ADC. */
			PUBLISH_EVENT(EV_ACQUIRE_GSR_DATA_LOGGER);

			/* Exit LPM */
			__no_operation();
			__bic_SR_register_on_exit(LPM3_bits); // Exit LPM

		}

    	__no_operation();
      break;
    case ADC12IV_ADC12IFG1:
    	__no_operation();
    	break;        // Vector 14:  ADC12MEM1
    case ADC12IV_ADC12IFG2:
    	__no_operation();
    	break;        // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3:
    	__no_operation();
    	break;        // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4:
    	_no_operation();
    	break;        // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5:
    	__no_operation();
    	break;        // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6:
    	__no_operation();
    	break;        // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7:
    	__no_operation();
    	break;        // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8:
    	__no_operation();
    	break;        // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9:
    	__no_operation();
    	break;        // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10:
    	__no_operation();
    	break;        // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11:
    	__no_operation();
    	break;        // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12:
    	__no_operation();
    	break;        // Vector 36:  ADC12MEM12
    case ADC12IV_ADC12IFG13:
    	__no_operation();
    	break;        // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14:
    	__no_operation();
    	break;        // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15:
    	__no_operation();
    	break;        // Vector 42:  ADC12MEM15
    case ADC12IV_ADC12IFG16:
    	__no_operation();
    	break;        // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17:
    	__no_operation();
    	break;        // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18:
    	__no_operation();
    	break;        // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19:
    	__no_operation();
    	break;        // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20:
    	__no_operation();
    	break;        // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21:
    	__no_operation();
    	break;        // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22:
    	__no_operation();
    	break;        // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23:
    	__no_operation();
    	break;        // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24:
    	__no_operation();
    	break;        // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25:
    	__no_operation();
    	break;        // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26:
    	__no_operation();
    	break;        // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27:
    	__no_operation();
    	break;        // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28:
    	__no_operation();
    	break;        // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29:
    	__no_operation();
    	break;        // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30:
    	__no_operation();
    	break;        // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31:
    	__no_operation();
    	break;        // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG:
    	__no_operation();
    	break;        // Vector 76:  ADC12RDY
    default:
    	__no_operation();
    	break;
  }
}
