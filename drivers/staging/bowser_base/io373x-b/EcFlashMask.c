// EcFlashMask.c/.h:
//   Implement struct EcFlash interface through mask rom, for EC supporting
//     embedded flash update through mask ROM (such as 3730).
// Other files required:
//    Ec.c
//    MaskRom.c
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#include "MyTypeDefs.h"
#include "EcFlashMask.h"
#include "MaskRom.h"

//------------------------ Static data -----------------------------------

static EcFlashMask s_ecFlashMask; // at most one of this structure working.

//------------------------ Private functions -----------------------------------

static BOOL __EcFlashMask_SectorErase(EcFlashMask *ecFlashMask, DWORD dwStartSec, DWORD dwSecCnt);
static BOOL __EcFlashMask_ChipErase(EcFlashMask *ecFlashMask);
static BOOL __EcFlashMask_Read(EcFlashMask *ecFlashMask, DWORD addr, BYTE *buf, DWORD buflen);
static BOOL __EcFlashMask_Write(EcFlashMask *ecFlashMask, DWORD addr, BYTE *buf, DWORD buflen);
static DWORD __EcFlashMask_GetSize(EcFlashMask *ecFlashMask);
static DWORD __EcFlashMask_GetSectorSize(EcFlashMask *ecFlashMask);

//------------------------- Protected functions ----------------------------------

EcFlashMask *_EcFlashMask_Ctor(EcFlashMask *fromDerive, Ec *ec)
{
    EcFlashMask *ecFlashMask = fromDerive ? fromDerive : &s_ecFlashMask;
    BOOL bSkipRam1 = (EcIs373x(ec) && (EcGetRevId(ec) <= 0x0F)) || // 373x-Ax
                        (EcGetConnType(ec) == CONN_BRIDGEEC);      // bridge has only 1 ram for flash-update.
    ASSERT(ec);
    ASSERT(ecFlashMask->_mask == NULL); // at most one of this structure working.

    // Call base ctor.
    if(_EcFlash_Ctor((EcFlash *) ecFlashMask, ec) == NULL)
        return NULL;

    // Init the mask rom.
    ecFlashMask->_mask = MaskRomInit(EcGetConn(ec), bSkipRam1); // MaskRom needs Conn, not Ec or Ts.
    if(!ecFlashMask->_mask)
        goto _ERROR_EXIT;

    // Enter code-in-rom before we can talk to mask ROM.
    ecFlashMask->_bIsCodeInRom = MaskRomEnterCodeInRom(ecFlashMask->_mask);
    if(!ecFlashMask->_bIsCodeInRom)
        goto _ERROR_EXIT;

    // Different EC or TS chips have different embedded flash size.
    MaskRomFlashSetSize(ecFlashMask->_mask, EcGetEbdFlashSize(ec));
    MaskRomFlashSetPageSize(ecFlashMask->_mask, EcGetEbdFlashSectorSize(ec));

    ecFlashMask->_ecFlash.Dtor          = (PFN_ECFLASH_DTOR)            _EcFlashMask_Dtor;
    ecFlashMask->_ecFlash.ChipErase     = (PFN_ECFLASH_CHIP_ERASE)      __EcFlashMask_ChipErase;
    ecFlashMask->_ecFlash.SectorErase   = (PFN_ECFLASH_SECTOR_ERASE)    __EcFlashMask_SectorErase;
    ecFlashMask->_ecFlash.Read          = (PFN_ECFLASH_READ)            __EcFlashMask_Read;
    ecFlashMask->_ecFlash.Write         = (PFN_ECFLASH_WRITRE)          __EcFlashMask_Write;
    ecFlashMask->_ecFlash.GetSize       = (PFN_ECFLASH_GET_SIZE)        __EcFlashMask_GetSize;
    ecFlashMask->_ecFlash.GetSectorSize = (PFN_ECFLASH_GET_SECTOR_SIZE) __EcFlashMask_GetSectorSize;

    return ecFlashMask;

_ERROR_EXIT:

    if(ecFlashMask)
        _EcFlashMask_Dtor(ecFlashMask);

    return NULL;
}

VOID _EcFlashMask_Dtor(EcFlashMask *ecFlashMask)
{
    ASSERT(ecFlashMask);

    if(ecFlashMask->_mask)
    {
        if(ecFlashMask->_bIsCodeInRom)
        {
            MaskRomExitCodeInRom(ecFlashMask->_mask);
            ecFlashMask->_bIsCodeInRom = FALSE;
        }
        MaskRomExit(ecFlashMask->_mask);
        ecFlashMask->_mask = NULL;
    }

    // Call base dtor.
    _EcFlash_Dtor((EcFlash *) ecFlashMask);
}

//------------------------ Private functions -----------------------------------

BOOL __EcFlashMask_SectorErase(EcFlashMask *ecFlashMask, DWORD dwStartSec, DWORD dwSecCnt)
{
    return MaskRomFlashPageErase(ecFlashMask->_mask, dwStartSec, dwSecCnt);
}

BOOL __EcFlashMask_ChipErase(EcFlashMask *ecFlashMask)
{
    return MaskRomFlashChipErase(ecFlashMask->_mask);
}

BOOL __EcFlashMask_Read(EcFlashMask *ecFlashMask, DWORD addr, BYTE *buf, DWORD buflen)
{
    return MaskRomFlashRead(ecFlashMask->_mask, addr, buf, buflen);
}

BOOL __EcFlashMask_Write(EcFlashMask *ecFlashMask, DWORD addr, BYTE *buf, DWORD buflen)
{
    return MaskRomFlashWrite(ecFlashMask->_mask, addr, buf, buflen);
}

DWORD __EcFlashMask_GetSize(EcFlashMask *ecFlashMask)
{
    return MaskRomFlashGetSize(ecFlashMask->_mask);
}

DWORD __EcFlashMask_GetSectorSize(EcFlashMask *ecFlashMask)
{
    return MaskRomFlashGetPageSize(ecFlashMask->_mask);
}
