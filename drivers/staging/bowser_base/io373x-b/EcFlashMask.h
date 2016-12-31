// EcFlashMask.c/.h:
//   Implement struct EcFlash interface through mask rom, for EC supporting
//     embedded flash update through mask ROM (such as 3730).
// Other files required:
//    Ec.c
//    MaskRom.c
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#ifndef _EC_FLASH_MASK_H_
#define _EC_FLASH_MASK_H_

#include "EcFlash.h"
#include "MaskRom.h"

typedef struct EcFlashMask EcFlashMask;

// Protected: -----------------------------------------------------------------

struct EcFlashMask
{
// Inherit:
    EcFlash _ecFlash;

// Protected:
    MaskRom *_mask;
    BOOL _bIsCodeInRom;
};

// friend EcFlashInit()
EcFlashMask *_EcFlashMask_Ctor(EcFlashMask *fromDerive, Ec *ec);
VOID _EcFlashMask_Dtor(EcFlashMask *ecFlashMask);

#endif // _EC_FLASH_MASK_H_
