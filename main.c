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

#include <msp430.h>
#include "driverlib/MSP430FR5xx_6xx/pmm.h"

#include "RF430CL331.h"
#include "NFCForumType4TagType.h"
#include "GPIOInterface.h"
#include "types.h"
#include "I2CInterface.h"
#include "ADS1x9x_USB_Communication.h"
#include "ADS1x9x.h"
//#include "ADS1x9x_Nand_Flash.h"
#include "types.h"
#include "StateMachine.h"
#include "AccelerometerInterface.h"
#include "NFCInterface.h"
#include "Executive.h"
#include "TimerInterface.h"
#include "ADCInterface.h"
#include "TemperatureInterface.h"
#include "ECGInterface.h"
#include "SPIInterface.h"
#include "GSRInterface.h"
#include "UART.h"
#include "DEBUG.h"  //change the DEBUG define in this file to 0 to re-enable peripherals,
                    //this was used because it stalled execution without actual hardware to initialize -Kyle
//

#include "string.h"
#include "GPIO.h"
#include "driverlib.h"
//
//extern unsigned Live_Streaming_flag;
int main(void)
{
	uint16_t volatile xACC = 0;
	WDTCTL = WDTPW + WDTHOLD;		// Halt the dog

	GPIOInterface_initClocks();   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
	PMM_unlockLPM5();

    /*Initialize Peripherals drivers only*/
    GPIOInterface_initGPIO();
    TimerInterface_Init();
    I2CInterface_Initialize();
    SPIInterface_Initialize();
   // if(!DEBUG)
    ADCInterface_Init();
    UART0_Initialize();
    /*Initialize Interfaces only*/

    //NFCInterface_Init();      // I think we can get away with not initializing this as NFC will not be used...
                                //will cut out this code later after it's functionality is duplicated over UART or Bluetooth - Kyle
    TemperatureInterface_Initialize();
    GSRInterface_Initialize();
    ECGInterface_Init();
    AccelerometerInterface_Initialize();

	/* the rest of the application will be under the
	control of the Executive.  */

    Exec_run();

	/* this should never be reached */
	return(0);

}




