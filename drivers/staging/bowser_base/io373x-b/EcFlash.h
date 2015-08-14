// EcFlash.h/.c:
//   1. Define uniform interface EcFlash::ChipErase()/SectorErase()/Read(), etc.
//   2. Create proper EcFlashXxx object according to external or embeded flash,
//      or mask-rom used.
//   3. Provide default behaviors.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#ifndef _EC_FLASH_H_
#define _EC_FLASH_H_

#include "Ec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct EcFlash EcFlash;
struct FlashInfo;

EcFlash *EcFlashInit(Ec *ec, struct FlashInfo *optionalFlashList);

VOID EcFlashExit(EcFlash *ecFlash);

BOOL EcFlashChipErase(EcFlash *ecFlash);

BOOL EcFlashSectorErase(EcFlash *ecFlash, DWORD dwStartSec, DWORD dwSecCnt);

BOOL EcFlashRead(EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen);

BOOL EcFlashWrite(EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen);

DWORD EcFlashGetSize(EcFlash *ecFlash);

DWORD EcFlashGetSectorSize(EcFlash *ecFlash);

#ifdef __cplusplus
}
#endif

////////////////////////////// Internal use ///////////////////////////////////
//
typedef VOID (*PFN_ECFLASH_DTOR) (EcFlash *ecFlash);
typedef BOOL (*PFN_ECFLASH_CHIP_ERASE) (EcFlash *ecFlash);
typedef BOOL (*PFN_ECFLASH_SECTOR_ERASE) (EcFlash *ecFlash, DWORD dwStartSec, DWORD dwSecCnt);
typedef BOOL (*PFN_ECFLASH_READ) (EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen);
typedef BOOL (*PFN_ECFLASH_WRITRE) (EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen);
typedef DWORD (*PFN_ECFLASH_GET_SIZE) (EcFlash *ecFlash);
typedef DWORD (*PFN_ECFLASH_GET_SECTOR_SIZE) (EcFlash *ecFlash);

struct EcFlash
{
// Public, virtual:
    PFN_ECFLASH_DTOR                    Dtor;
    PFN_ECFLASH_CHIP_ERASE              ChipErase;           //=0
    PFN_ECFLASH_SECTOR_ERASE            SectorErase;         //=0
    PFN_ECFLASH_READ                    Read;                //=0
    PFN_ECFLASH_WRITRE                  Write;               //=0
    PFN_ECFLASH_GET_SIZE                GetSize;             //=0
    PFN_ECFLASH_GET_SECTOR_SIZE         GetSectorSize;       //=0

// Protected:
    Ec *_ec;
};

// Protected: (for derive only)
EcFlash *_EcFlash_Ctor(EcFlash *fromDerive, Ec *ec);
VOID _EcFlash_Dtor(EcFlash *ecFlash);

#endif // _EC_FLASH_H_
