// MaskRom.h/.c:
//   Implement EC or TS embedded flash update through handshaking with mask ROM.
// Other files required:
//   One of ConnXxx.c.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#ifndef _MASK_ROM_H_6DBFDFBB_4312_40EE_AB26_BFC02B5ABC62
#define _MASK_ROM_H_6DBFDFBB_4312_40EE_AB26_BFC02B5ABC62

#include "Conn.h"
typedef struct MaskRom MaskRom;

MaskRom *MaskRomInit(Conn *conn, BOOL bSkipRam1);

VOID MaskRomExit(MaskRom *mask);

// Call it before flash access.
BOOL MaskRomEnterCodeInRom(MaskRom *mask);

// Call it after flash access.
BOOL MaskRomExitCodeInRom(MaskRom *mask);

BOOL MaskRomFlashChipErase(MaskRom *mask);

// Erase a page. addr = N*m_dwPageSize.
BOOL MaskRomFlashPageErase(MaskRom *mask, DWORD dwStartPage, DWORD dwPageCnt);

// Flash size and page size should be inited by client Ec or Ts.
DWORD MaskRomFlashGetSize(MaskRom *mask);
VOID MaskRomFlashSetSize(MaskRom *mask, DWORD siz);

DWORD MaskRomFlashGetPageSize(MaskRom *mask);
VOID MaskRomFlashSetPageSize(MaskRom *mask, DWORD siz);

BOOL MaskRomFlashWrite(MaskRom *mask, DWORD addr, BYTE *buf, DWORD buflen);

BOOL MaskRomFlashRead(MaskRom *mask, DWORD addr, BYTE *buf, DWORD buflen);

#endif // _MASK_ROM_H_6DBFDFBB_4312_40EE_AB26_BFC02B5ABC62
