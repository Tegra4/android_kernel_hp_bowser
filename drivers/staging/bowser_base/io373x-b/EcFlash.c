// EcFlash.h/.c:
//   1. Define uniform interface EcFlash::ChipErase()/SectorErase()/Read(), etc.
//   2. Create proper EcFlashXxx object according to external or embeded flash,
//      or mask-rom used.
//   3. Provide default behaviors.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#include "MyTypeDefs.h"
#include "EcFlash.h"

#if defined(_EC_FLASH_EXT) || defined(_EC_FLASH_EBD) || defined(_EC_FLASH_MASK)
  #ifdef _EC_FLASH_EXT
    #include "EcFlashExt.h"
  #endif
  #ifdef _EC_FLASH_EBD
    #include "EcFlashEbd.h"
  #endif
  #ifdef _EC_FLASH_MASK
    #include "EcFlashMask.h"
  #endif
#else
  #error "Please define one or more desired _EC_FLASH_XXX !"
#endif

//
// Define _EC_FLASH_EXT for 926D, 930 external SPI flash-update.
// Define _EC_FLASH_EBD for 9010 embedded-flash-update through XBI.
// Define _EC_FLASH_MASK for 373X embedded-flash-update through maskrom.
// Define some of them to support different flash-update in the program.
// Define both _EC_FLASH_EXT and _EC_FLASH_EBD in a EDI debug AP.
//

enum { EXT, EBD, MASK };

// If EDI or non-373X, use XBI(external or embedded) flash-update
// else, use maskrom.
static INT FlashUpdateWay(Ec *ec)
{
    if(EcGetConnType(ec) == CONN_FTEDI || // always use XBI if connected through EDI.
       !EcIs373x(ec))                     // non-373X has only XBI.
    {
        if(EcIsEbdFlash(ec))
            return EBD; // ebd flash through XBI.
        else
            return EXT; // extern flash through XBI.
    }

    // !EDI && 373X
    return MASK;
}

// struct EcFlashXxx "Object factory"
EcFlash *EcFlashInit(Ec *ec, struct FlashInfo *optionalFlashList)
{
    EcFlash *ecFlash = NULL;

    // Disable WDT so chip won't suddenly reset when having access to flash.
    EcDisableWdt(ec);

    // Make "connection" or bridge prepare for flash access.
    if(!EcConnEnterCodeInRam(ec))
        return NULL;

    switch(FlashUpdateWay(ec))
    {
#ifdef _EC_FLASH_EXT
    case EXT:
        ecFlash = (EcFlash *) _EcFlashExt_Ctor(NULL, ec, optionalFlashList);
        break;
#endif

#ifdef _EC_FLASH_EBD
    case EBD:
        ecFlash = (EcFlash *) _EcFlashEbd_Ctor(NULL, ec);
        break;
#endif

#ifdef _EC_FLASH_MASK
    case MASK:
        ecFlash = (EcFlash *) _EcFlashMask_Ctor(NULL, ec);
        break;
#endif

    default:
        ENETRACE("The _EC_FLASH_XXX defined is not proper for current chip ID and connection !\n");
        ASSERT(0);
    }

    if(ecFlash == NULL)
    {
        EcConnExitCodeInRam(ec);
        return NULL;
    }

    // Do common init here for _all_ kinds of flash.
    // ...

    return ecFlash;
}

VOID EcFlashExit(EcFlash *ecFlash)
{
    Ec *ec = ecFlash->_ec;

    ecFlash->Dtor(ecFlash);
    EcConnExitCodeInRam(ec);
}

EcFlash *_EcFlash_Ctor(EcFlash *fromDerive, Ec *ec)
{
    EcFlash *ecFlash = fromDerive;

    ASSERT(fromDerive); // can't instantiate EcFlash directly; only derives can.

    ecFlash->_ec = ec;

    // These "virtual functions" have default behaviors.
    ecFlash->Dtor                   = _EcFlash_Dtor;

    return ecFlash;
}

// ---- The default behaviors ---------------------------------------------------------------------

VOID _EcFlash_Dtor(EcFlash *ecFlash)
{
    ecFlash->_ec = NULL;
}

// ---- "Virtual functions" (dispatch to derives) -------------------------------------------------

BOOL EcFlashChipErase(EcFlash *ecFlash)
{
    return ecFlash->ChipErase(ecFlash);
}

BOOL EcFlashSectorErase(EcFlash *ecFlash, DWORD dwStartSec, DWORD dwSecCnt)
{
    return ecFlash->SectorErase(ecFlash, dwStartSec, dwSecCnt);
}

BOOL EcFlashRead(EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen)
{
    return ecFlash->Read(ecFlash, addr, buf, buflen);
}

BOOL EcFlashWrite(EcFlash *ecFlash, DWORD addr, BYTE *buf, DWORD buflen)
{
    return ecFlash->Write(ecFlash, addr, buf, buflen);
}

DWORD EcFlashGetSize(EcFlash *ecFlash)
{
    return ecFlash->GetSize(ecFlash);
}

DWORD EcFlashGetSectorSize(EcFlash *ecFlash)
{
    return ecFlash->GetSectorSize(ecFlash);
}
