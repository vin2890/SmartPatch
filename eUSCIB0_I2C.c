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
//! \file   eUSCIB0_I2C.c
//!
//! \brief  This is the eUSCIB0 peripheral driver file. All of the functions
//!			found in this peripheral driver file are specific to the eUSCIB0 peripheral.
//!			If a peripheral module other than eUSCIB0 is used, then this file must be
//!			replaced. Configure the eUSCIB0 to act as a I2C peripheral, configure the
//!			peripheral clock source, peripheral frequency, peripheral address, initialize
//!			peripheral ports and interrupts, store the returned data and transfer data
//!			are the functions that are found on this peripheral driver file.
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
#include "eUSCIB0_I2C.h"
#include <stdint.h>
#include "driverlib/MSP430FR5xx_6xx/gpio.h"
#include "UART.h"
#include "RingBuffer.h"
#include "Executive.h"
#include "Events.h"


/*  Definitions */
/* Bit definitions for the tw_status register */
#define MAX_TWI_RETRIES 2
#define BUSY 7

uint8_t  *PTxData;                     // Pointer to TX data
uint8_t *PRxData;                     // Pointer to RX data
uint16_t TXByteCtr;
uint16_t RXByteCtr;

#define I2C_DELAY 			0				//AK 10-23-2013 changed from 50000

/*  Local Function Prototypes */
void I2CInit_ports(void);

void USCIBO_InitializeRegisters(void)
{

	UCB0CTL1 = UCSWRST;	            			// Software reset enabled
	UCB0CTLW0 = UCMODE_3  + UCMST + UCSYNC;		// I2C mode, Master mode, sync, transmitter
	UCB0CTL1 = UCSSEL_2 + UCSWRST;              // SMCLK = 8MHz
	UCB0BRW = 80; 								// Baudrate = SMLK/20 = 400kHz
	UCB0I2CSA  = 0x0018;						// slave address - determined by pins E0, E1, and E2 on the RF430CL331H
	UCB0CTL1  &= ~UCSWRST;
	UCB0IE |= UCTXIE + UCRXIE + UCSTPIE + UCNACKIE;

}

void USCIBO_InitializePorts(void)
{

	PORT_I2C_SEL |= (SCL + SDA);

}

void USCIBO_Stop(void)
{
    __delay_cycles(100);		//AK 10-23-2013
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    __enable_interrupt();
    while (!(UCB0CTL1 & UCTXSTP));             // Ensure stop condition got sent
    __disable_interrupt();
}

void USCIBO_writeRegister(uint8_t ic_address, uint16_t reg_addr, uint16_t value, uint8_t bytesInAddr, uint8_t bytesInValue)
{
    uint8_t TxData[4] = {0,0,0,0};
    uint8_t totalBytes = 0;

    if((bytesInAddr == 1) && (bytesInValue == 1 ))
    {
    	TxData[0] = reg_addr; 		// MSB of address
    	TxData[1] = value; 	// LSB of address
    	totalBytes = 2;
    }
    else if((bytesInAddr == 1) && (bytesInValue == 2))
    {
    	TxData[0] = reg_addr; 		// MSB of address
    	TxData[1] = value & 0xFF;
    	TxData[2] = value >> 8;
    	totalBytes = 3;
    }
    else if((bytesInAddr == 2) && (bytesInValue == 1))
    {
    	TxData[0] = reg_addr >> 8; 		// MSB of address
    	TxData[1] = reg_addr & 0xFF; 	// LSB of address
    	TxData[2] = value;
    	totalBytes = 3;
    }
    else if((bytesInAddr == 2) && (bytesInValue == 2))
    {
    	TxData[0] = reg_addr >> 8; 		// MSB of address
    	TxData[1] = reg_addr & 0xFF; 	// LSB of address
    	TxData[2] = value & 0xFF;
    	TxData[3] = value >> 8;
    	totalBytes = 4;

    }

	UCB0I2CSA  = ic_address;

    //while(!I2C_ALLOW);		//wait until it is safe to transmit

    RF430_I2C_Write((uint8_t  *)TxData, totalBytes, START);

    USCIBO_Stop();

    {
    	uint16_t delay = 10;
		while ((UCB0CTL1 & UCTXSTP) && delay)             // Ensure stop condition got sent, added timeout due to infinite loop when it should not be  AK 10-27-2013
		{
			__delay_cycles(10);
			delay--;
		}
    }
}


void USCIBO_writeContinuous(uint8_t ic_address, uint16_t reg_addr, uint8_t  * write_data, uint16_t data_length)
{
    uint8_t TxAddr[2] = {0,0};

	UCB0I2CSA  = ic_address;

    //while(!I2C_ALLOW);		//wait until it is safe to transmit

    TxAddr[0] = reg_addr >> 8; 		//MSB of address
	TxAddr[1] = reg_addr & 0xFF; 	//LSB of address
	//First send the addres over I2C
	RF430_I2C_Write((uint8_t *)TxAddr, 2, START);//if this instruction is executed, then the data line gets pulled low.
	//Then send its data.
    RF430_I2C_Write((uint8_t  *)write_data, data_length, CONTINUE);

    USCIBO_Stop();

    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
}

void RF430_I2C_Write(uint8_t  * data, uint16_t length, uint16_t cont)
{
//	__delay_cycles(I2C_DELAY);                   // Delay required between transaction

    PTxData = (uint8_t  *)data;      // TX array start address
    TXByteCtr = length;

    if (cont == START) {
        RF430_I2C_Start = SET;					// need a way of knowing when a start condition has been set in the ISR AK 10-23-2013
        UCB0CTL1 |= UCTR + UCTXSTT;             // I2C Master Transmitter Mode, Generate START Condition
    }
    else {
        UCB0CTL1 |= UCTR;                       // I2C TX
        UCB0IFG |= UCTXIFG;                     // Kickoff IFG since START not sent
    }

    //__bis_SR_register(LPM_MODE + GIE);     // Enter LPM, enable interrupts
    //__no_operation();                       // Remain in LPM until all data
                                            // is TX'd
    RF430_I2C_Start = CLEAR;				// this should be cleared in the interrupt, but to be sure it is also cleared here, AK-1023-2013

    //if (!(RF430_I2C_State == I2C_NACK_RCVD))
    //{
//    	WDTCTL = WDTPW + WDTIS_1;
        //while (TXByteCtr != 0);          // Ensure stop condition got sent
//        WDTCTL = WDTPW + WDTHOLD;
    //}

}

uint16_t USCIBO_readRegister(uint8_t ic_address, uint16_t reg_addr, uint8_t bytesInAddr, uint8_t bytesToRead)
{
    uint8_t TxAddr[2] = {0,0};
    uint8_t RxBuffer[2] = {0,0};

    if(bytesInAddr==1)
    {
    	 TxAddr[0] = reg_addr; 		// MSB of address
    }

    else if(bytesInAddr == 2)
    {
    	TxAddr[0] = reg_addr >> 8; 		// MSB of address
    	TxAddr[1] = reg_addr & 0xFF; 	// LSB of address
    }

	UCB0I2CSA  = ic_address;

    //while(!I2C_ALLOW);		//wait until it is safe to transmit

    if(RF430_I2C_Write_Restart_Read((uint8_t *)TxAddr, bytesInAddr, (uint8_t *)RxBuffer, bytesToRead) != FAIL)
    ///    return RxBuffer[0];
     return RxBuffer[1] << 8 | RxBuffer[0];
    //else
    __no_operation();
        //return FAIL;

    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    return RxBuffer[1] << 8 | RxBuffer[0];
}

void USCIBO_readContinuous(uint8_t ic_address, uint16_t reg_addr, uint8_t* read_data, uint16_t data_length)
{
    uint8_t TxAddr[2] = {0,0};

	TxAddr[0] = reg_addr >> 8; 		// MSB of address
	TxAddr[1] = reg_addr & 0xFF; 	// LSB of address

	UCB0I2CSA  = ic_address;

    //while(!I2C_ALLOW);		//wait until it is safe to transmit

    //RF430_I2C_Write_Restart_Read((uint8_t  *)TxAddr, 2, (uint8_t *)read_data, data_length);

    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

}

uint8_t RF430_I2C_Write_Restart_Read(uint8_t  * tx_data, uint16_t tx_length, uint8_t* rx_data, uint16_t rx_length)
{
    RF430_I2C_State = I2C_TRANSMIT;

    PTxData = (uint8_t  *)tx_data;     // TX array start address
    TXByteCtr = tx_length;

    RF430_I2C_Start = SET;					// need a way of knowing when a start condition has been set in the ISR AK 10-23-2013
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C Master Transmitter Mode, Generate START Condition

    //if(RF430_I2C_State == I2C_NACK_RCVD)    // If RF430 NACKed
    //{
        //Debug_Print("RF430 NACKed!!!!!!!\n");
        //return FAIL;
    //}

        // Enter LPM, enable interrupts
    //__no_operation();                       // Remain in LPM until all data

    RF430_I2C_Start = CLEAR;				// this should be cleared in the interrupt, but to be sure it is also cleared here, AK-1023-2013

    //if(RF430_I2C_State == I2C_NACK_RCVD)    // If RF430 NACKed
    //{
        //Debug_Print("RF430 NACKed!!!!!!!\n");
        //return FAIL;
    //}
    __disable_interrupt();
    PRxData = (uint8_t *)rx_data;           // Start of RX buffer
    RXByteCtr = rx_length;                          // Load RX byte counter

    UCB0CTL1 &= ~UCTR;                      // I2C Read
    RF430_I2C_Start = CLEAR;				// only needed on transmit and start condition --AK 10-23-2013
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
    while (UCB0CTL1 & UCTXSTT);
       // Enter LPM, enable interrupts
                                            // Remain in LPM until all data
                                            // is RX'd
    //kick(); //this will halt the mcu then send a message to the ble to respond with a uart event to kick the mcu out of low power mode

    UCB0CTL1 |= UCTXSTP;
    while (UCB0CTL1 & UCTXSTP);
    RF430_I2C_Start = CLEAR;				// this should be cleared in the interrupt, but to be sure it is also cleared here, AK-1023-2013
                     // Set breakpoint >>here<< and
                                            // read out the RxBuffer buffer
    __enable_interrupt();
    return PASS;
}



/***********************************************************
	Function Name: <interrupt handler for I2C>
	Function Description: This function is responsible for
	implementing the control logic needed to perform a
	read or write operation with an I2C slave.
	Inputs:  none
	Outputs: none
***********************************************************/
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{

  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: //break;                         // Vector  4: NACKIFG
    RF430_I2C_State = I2C_NACK_RCVD;
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    UCB0IFG = 0;                            // Clear All USCI_B0 flags
    __bic_SR_register_on_exit(LPM_MODE);    // Exit LPM
    break;
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case 0x16:
     // Vector 10: RXIFG
    RXByteCtr--;                            // Decrement RX byte counter
    if (RXByteCtr)
    {

      *PRxData++ = UCB0RXBUF;               // Move RX data to address PRxData
      if (RXByteCtr == 1)
        UCB0CTL1 |= UCTXSTP;                // Generate I2C stop condition on next RX
    }
    else
    {
      *PRxData = UCB0RXBUF;                 // Move final RX data to PRxData
      __bic_SR_register_on_exit(LPM_MODE); // Exit active CPU
    }
    break;
  case 0x18:                                  // Vector 12: TXIFG
    if (TXByteCtr)                          // Check TX byte counter
    {
      unsigned int delay = 0;
      UCB0TXBUF = *PTxData++;               // Load TX buffer
//      TXByteCtr--;                          // Decrement TX byte counter
      if(TXByteCtr)
      {
          __bic_SR_register_on_exit(LPM_MODE);
    	  while((delay < 9000) && (UCB0CTL1 & UCTXSTT) && (RF430_I2C_Start == SET)) //only go into this loop when a start condition has been requested AK 10-23-2013
    	  {
    		  __delay_cycles(40);
    		  delay += 40;
    	  }
    	  //This just checks to see if a ACK was recieved.

		  if((UCB0CTL1 & UCTXSTT) && (RF430_I2C_Start == SET))            // Still Waiting?  --only when start condition is expected, AK 10-23-2013
		  {
			  RF430_I2C_State = I2C_NACK_RCVD;
			  __bic_SR_register_on_exit(LPM_MODE); // Exit LPM
			  break;
		  }
      }

    }
    else
    {
      //UCB0CTL1 |= UCTXSTP;                  // I2C stop condition
      UCB0IFG &= ~UCTXIFG;                  // Clear USCI_B0 TX int flag
      __bic_SR_register_on_exit(LPM_MODE); // Exit LPM
      break;
    }
    TXByteCtr--;
    RF430_I2C_Start = CLEAR;			//Clear after the first TX, start has been passed		//AK 10-23-2013
    break;
  default:
    __no_operation();
    break;
  }
}







