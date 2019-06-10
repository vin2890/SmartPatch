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
//*****************************************************************************
//
//! \addtogroup crc32_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_CRC32__
#include "crc32.h"

#include <assert.h>

//*****************************************************************************
//
//! Sets the seed for the CRC.
//!
//! \param seed is the seed for the CRC to start generating a signature from.
//!        Modified bits are \b CRC16INIRESL0 of \b CRC16INIRESL0 register.
//!            \b CRC32INIRESL0 of \b CRC32INIRESL0 register
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the seed for the CRC to begin generating a signature with
//! the given seed and all passed data. Using this function resets the CRC32
//! signature.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_setSeed(uint32_t seed, uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    HWREG16(CRC32_BASE + OFS_CRC16INIRESW0) = seed;
    else
    {
        HWREG16(CRC32_BASE + OFS_CRC32INIRESW1) = ((seed & 0xFFFF0000)
                >> 16);
        HWREG16(CRC32_BASE + OFS_CRC32INIRESW0) = (seed & 0xFFFF);
    }
}

//*****************************************************************************
//
//! Sets the 8 Bit data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIB0 of \b CRC16DIB0 register.
//!                            \b CRC32DIB0 of \b CRC32DIB0 register.
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set8BitData(uint8_t dataIn, uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    HWREG8(CRC32_BASE + OFS_CRC16DIW0_L) = dataIn;
    else
    HWREG8(CRC32_BASE + OFS_CRC32DIW0_L) = dataIn;
}

//*****************************************************************************
//
//! Sets the 16 Bit data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIW0 of \b CRC16DIW0 register.
//!                          \b CRC32DIW0 of \b CRC32DIW0 register.
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set16BitData(uint16_t dataIn, uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    HWREG16(CRC32_BASE + OFS_CRC16DIW0) = dataIn;
    else
    HWREG16(CRC32_BASE + OFS_CRC32DIW0) = dataIn;

}
//*****************************************************************************
//
//! Sets the 32 Bit data to add into the CRC module to generate a new signature.
//!    Available only for CRC32_MODE and not for CRC16_MODE
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC32DIL0 of \b CRC32DIL0 register.
//!
//! This function sets the given data into the CRC module to generate the new
//! signature from the current signature and new data. Bit 0 is
//!    treated as LSB
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set32BitData(uint32_t dataIn)
{

    HWREG16(CRC32_BASE + OFS_CRC32DIW0) = dataIn & 0xFFFF;
    HWREG16(CRC32_BASE + OFS_CRC32DIW1) = (uint16_t) ((dataIn & 0xFFFF0000)
            >> 16);
}

//*****************************************************************************
//
//! Translates the data by reversing the bits in each 8 bit data and then sets this
//! data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIRBB0 of \b CRC16DIRBB0 register.
//!                             \b CRC32DIRBB0 of \b CRC32DIRBB0 register.
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data. Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set8BitDataReversed(uint8_t dataIn, uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    HWREG8(CRC32_BASE + OFS_CRC16DIRBW1_L) = dataIn;
    else
    HWREG8(CRC32_BASE + OFS_CRC32DIRBW1_L) = dataIn;
}

//*****************************************************************************
//
//! Translates the data by reversing the bits in each 16 bit data and then
//!    sets this data to add into the CRC module to generate a new signature.
//!
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC16DIRBW0 of \b CRC16DIRBW0 register.
//!                             \b CRC32DIRBW0 of \b CRC32DIRBW0 register.
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data.  Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set16BitDataReversed(uint16_t dataIn, uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    HWREG16(CRC32_BASE + OFS_CRC16DIRBW1) = dataIn;
    else
    HWREG16(CRC32_BASE + OFS_CRC32DIRBW1) = dataIn;
}

//*****************************************************************************
//
//! Translates the data by reversing the bits in each 32 Bit Data and then
//!    sets this data to add into the CRC module to generate a new signature.
//!    Available only for CRC32 mode and not for CRC16 mode
//! \param dataIn is the data to be added, through the CRC module, to the
//!       signature.
//!        Modified bits are \b CRC32DIRBL0 of \b CRC32DIRBL0 register.
//!
//! This function first reverses the bits in each byte of the data and then
//! generates the new signature from the current signature and new translated
//! data.  Bit 0 is treated as MSB.
//!
//! \return NONE
//
//*****************************************************************************
void CRC32_set32BitDataReversed(uint32_t dataIn)
{
    HWREG16(CRC32_BASE + OFS_CRC32DIRBW1) = dataIn & 0xFFFF;
    HWREG16(CRC32_BASE + OFS_CRC32DIRBW0) = (uint16_t) ((dataIn & 0xFFFF0000)
            >> 16);

}
//*****************************************************************************
//
//! Returns the value of CRC Signature Result.
//!
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function returns the value of the signature result generated by the CRC.
//! Bit 0 is treated as LSB.
//! \return uint32_t Result
//
//*****************************************************************************
uint32_t CRC32_getResult(uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    return (HWREG16(CRC32_BASE + OFS_CRC16INIRESW0) );
    else
    {
        uint32_t result = 0;
        result = HWREG16(CRC32_BASE + OFS_CRC32INIRESW1);
        result = (result << 16);
        result |= HWREG16(CRC32_BASE + OFS_CRC32INIRESW0);
        return (result);
    }
}
//*****************************************************************************
//
//! Returns the bit-wise reversed format of the 32 bit Signature Result.
//!
//! \param crcMode selects between CRC32 and CRC16
//!            Valid values are \b CRC16_MODE and \b CRC32_MODE
//!
//! This function returns the bit-wise reversed format of the Signature Result.
//! Bit 0 is treated as MSB.
//!
//! \return uint32_t Result
//
//*****************************************************************************
uint32_t CRC32_getResultReversed(uint8_t crcMode)
{
    assert((CRC16_MODE == crcMode) || (CRC32_MODE == crcMode));

    if (CRC16_MODE == crcMode)
    return (HWREG16(CRC32_BASE + OFS_CRC16RESRW0) );
    else
    {
        uint32_t result = 0;
        result = HWREG16(CRC32_BASE + OFS_CRC32RESRW0);
        result = (result << 16);
        result |= HWREG16(CRC32_BASE + OFS_CRC32RESRW1);
        return (result);
    }
}
#endif
