//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#ifndef _EC_H_85796CFA_2552_4082_AB42_52D4B4556DD1
#define _EC_H_85796CFA_2552_4082_AB42_52D4B4556DD1

#include "MyTypeDefs.h"
#include "Conn.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Ec Ec;


//
// EcXxx() functions
//

// Call this before calling other EcXxx().
// Return a pointer to Ec if success.
Ec *EcInit(enum CONN connType, ConnParam param);

// Call this to undo EcInit() if EcInit() was ever called and succeeded.
VOID EcExit(Ec *ec);

BOOL EcReadRegs(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes);

BOOL EcWriteRegs(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes);

BOOL EcReadReg(Ec *ec, WORD wReg, BYTE *p1Byte);

BOOL EcWriteReg(Ec *ec, WORD wReg, BYTE val);

WORD EcGetChipId(Ec *ec);

BYTE EcGetRevId(Ec *ec);

BOOL EcIsEbdFlash(Ec *ec);

BOOL EcIs373x(Ec *ec);

BOOL EcIs901x(Ec *ec);

BOOL EcIs902x(Ec *ec);

BOOL EcIs394x(Ec *ec);

BOOL EcIs1000(Ec *ec);

BOOL EcSupportNewMemLayout(Ec *ec);

BOOL EcHasBurstWrite(Ec *ec);

BOOL EcHasPageWrite(Ec *ec);

BOOL EcStop(Ec *ec);

BOOL EcRun(Ec *ec);

BOOL EcDisableWdt(Ec *ec);

BOOL EcIsStopped(Ec *ec);

BOOL EcConnEnterCodeInRam(Ec *ec);

BOOL EcConnExitCodeInRam(Ec *ec);

Conn *EcGetConn(Ec *ec);

enum CONN EcGetConnType(Ec *ec);

DWORD EcGetEbdFlashSize(Ec *ec);

DWORD EcGetEbdFlashSectorSize(Ec *ec);

BOOL EcLoadTrimData(Ec *ec, INT type);

// Special purpose.
BOOL EcReadEbdFlash(Ec *ec, DWORD dwAdr, BYTE *buf, INT bufLen);
BOOL EcReadSpecialRow(Ec *ec, BYTE Ra/*row*/, BYTE Ca/*col*/, BYTE *buf, INT bufLen);

#ifdef LOAD_TRIM_373X // Test purpose.
BOOL EcLoadTrimData373x(Ec *ec);
#endif

// Command sequence related.
BOOL EcSupportCmdSeq(Ec *ec);
VOID EcBeginCmdSeq(Ec *ec);
BOOL EcEndCmdSeq(Ec *ec, BYTE *buf, INT nBufLen);
VOID EcReadRegsSeq(Ec *ec, WORD wStartReg, INT nBytes);
VOID EcReadRegSeq(Ec *ec, WORD w1Reg);
VOID EcWriteRegsSeq(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes);
VOID EcWriteRegSeq(Ec *ec, WORD w1Reg, BYTE _1ByteVal);
VOID EcDelaySeq(Ec *ec, INT us);

///////////////////////////////////////////////////////////////////////////////////////////////////
// EC registers, register bits, defined as GLOBAL variables.
// These global varibles are inited for 39XX in the beginning, and modified during EcInit()
//   373X values if 373X chip detected.
// EC registers are accessed through EcRead/WriteReg() after EcInit(); So these
//   global values are assured to init properly before Read/WriteReg() are called.

extern WORD REG_ECSTS;
extern WORD REG_DEVIDH;
extern WORD REG_DEVIDL;
extern WORD REG_REVID;


extern WORD REG_SPIIN;
extern WORD REG_SPIOUT;
extern WORD REG_SPICFG;

extern WORD REG_EFCFG;
extern WORD REG_EFCMD;
extern WORD REG_EFA0;
extern WORD REG_EFA1;
extern WORD REG_EFA2;
extern WORD REG_EFDOUT;
extern WORD REG_EFDIN;

extern BYTE BIT_EFCFG_ENA;
extern BYTE BIT_EFCFG_BSY;
extern BYTE BIT_EFCFG_RDY;

extern WORD REG_CLKCFG;
extern WORD REG_CLKCFG2;

extern WORD REG_EDIID;

extern BYTE BIT_SPICFG_CSLOW;
extern BYTE BIT_SPICFG_ENA;
extern BYTE BIT_SPICFG_FASTRDDMY;
extern BYTE BIT_SPICFG_FLHBSY;
extern BYTE BIT_SPICFG_SPIBSY;
extern BYTE BIT_SPICFG_SPIRDY;

extern BYTE BIT_SPICFG_FLHBSYAUTO;
extern BYTE BITS_SPICFG_DEF;

extern DWORD EBDFLH_PAGESIZE;
extern DWORD EBDFLH_SIZE;

#ifdef __cplusplus
}
#endif

// Embedded flash commands (no difference between chips)
enum
{
    CMD_BYTE_PROGRAM        = 0x02,
    CMD_LATCH               = 0x02,
    CMD_READ                = 0x03,
    CMD_FAST_READ           = 0x0B,
    CMD_PAGE_ERASE          = 0x20,
    CMD_CHIP_ERASE          = 0x60,
    CMD_PAGE_PROGRAM        = 0x70,
    CMD_CLEAR_HVPL          = 0x80,
    CMD_TRIM_DATA           = 0x90,
    CMD_BURST_WRITE         = 0x46,
};

// Internal use.
struct Ec
{
    Conn *conn;
    WORD wDevId;
    BYTE byRevId;
};

#endif // _EC_H_85796CFA_2552_4082_AB42_52D4B4556DD1
