// Ec.c:
//      Implement EcXxx() common utilities (not including EcFlashXxx()).
// Other files required:
//      One of ConnXxx.c.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#include "MyTypeDefs.h"
#include "Ec.h"
#include "Conn.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// EC registers, register bits, defined as GLOBAL variables.
// These global varibles are inited for 39XX in the beginning, and modified during init of
//   CEc-derives objects to 373X values if 373X chip detected.
// EC registers are accessed through EcRead/WriteRegs(Ec *ec) through Ec*; So these
//   global values are assured to init properly before Read/WriteReg() are called.

WORD REG_ECSTS              = 0xFF1D;
WORD REG_DEVIDH             = 0xFF1E;
WORD REG_DEVIDL             = 0xFF1F;
WORD REG_REVID              = 0xFF00;

// Byte mode (s/w mode) regs. (this lib does not use h/w mode for external flash)
WORD REG_SPIIN              = 0xFEAB; // for data in
WORD REG_SPIOUT             = 0xFEAC; // for cmd, A2~A0, data out, dummy out
WORD REG_SPICFG             = 0xFEAD;

// Embedded flash (of course, h/w mode) regs.
WORD REG_EFCFG              = 0xFEAD;
WORD REG_EFCMD              = 0xFEAC; // for cmd
WORD REG_EFA0               = 0xFEA8; // for A0
WORD REG_EFA1               = 0xFEA9; // for A1
WORD REG_EFA2               = 0xFEAA; // for A2
WORD REG_EFDOUT             = 0xFEAB; // for data out. // 39XX h/w mode uses the same reg
WORD REG_EFDIN              = 0xFEAB; // for data in   //   for both in and out.

BYTE BIT_EFCFG_ENA          = 0x08;
BYTE BIT_EFCFG_BSY          = 0x02;
BYTE BIT_EFCFG_RDY          = 0x80; // 373X only

WORD REG_CLKCFG             = 0xFF0D;
WORD REG_CLKCFG2            = 0xFF1E;

WORD REG_EDIID              = 0xFF24;

BYTE BIT_SPICFG_CSLOW       = 0x10; // SPICS# output low (s/w mode only)
BYTE BIT_SPICFG_ENA         = 0x08; // h/w people: must set it for both byte mode and command mode.
BYTE BIT_SPICFG_FASTRDDMY   = 0x04; // Dummy byte for fast read command enable (h/w mode only)
BYTE BIT_SPICFG_FLHBSY      = 0x02; // for external flash, s/w mode: 1=spi busy;
BYTE BIT_SPICFG_SPIBSY      = 0x02; //                     h/w mode: 1=spi busy;
                                    //                     h/w mode and bit0=1: 1=flash busy
                                    // for embedded flash(must be h/w mode), 1=flash busy;
BYTE BIT_SPICFG_FLHBSYAUTO  = 0x01; // enable auto flash busy check(cmd 05) (for h/w mode and external flash)

// Embedded flash parameters (3730 uses the same flash as 9010B, except the capacity)
// Here are params for 39XX so they are zero.
DWORD EBDFLH_PAGESIZE       = 0; // 9010B, 373 have 128 bytes page write.
DWORD EBDFLH_SIZE           = 0; // 9010A/B has 128K, 373 has only 32K(for non-x86)

#define REG_XBITRIM         0xFEA6
#define REG_E51RST          0xF010

union FLASHADR
{
    DWORD dwAdr;
    struct
    {
        BYTE A0;
        BYTE A1;
        BYTE A2;
    };
};

// Private members ----------------------------------------------------------------------------------------------------

static Ec ecs[2]; // allow at most 2 of this working structure.
static INT ecCount;

// Private function declarations --------------------------------------------------------------------------------------

static VOID InitReg(Ec *ec);
static BOOL GetChipIdFromDevice(Ec *ec);
static BOOL LoadTrimData90xx_100x(Ec *ec, INT type);
static BOOL LoadTrimData902x(Ec *ec, INT type);

// Private function implementation ------------------------------------------------------------------------------------

void InitReg(Ec *ec)
{
    if(EcIs373x(ec))
    {
        REG_DEVIDH          = 0xF01C;
        REG_DEVIDL          = 0xF01D;
        REG_REVID           = 0xF019;

        REG_CLKCFG          = 0xF00B;
        REG_CLKCFG2         = 0xF01F;

        REG_EFCFG           = 0xFEA0;

        // Command mode (h/w mode) regs.
        REG_EFCMD           = 0xFEA7;
        REG_EFA0            = 0xFEA8;
        REG_EFA1            = 0xFEA9; // 373X has no A2
        REG_EFDOUT          = 0xFEAA;
        REG_EFDIN           = 0xFEAB;

        EBDFLH_PAGESIZE     = 128;
        EBDFLH_SIZE         = 1024 * 32;

        return;
    }
    else if(EcIs901x(ec) || EcIs902x(ec))
    {
        if(ec->wDevId == 0x9010 && ec->byRevId == 0xA0) // 9010A (in fact, we abort support for 9010A)
        {
            EBDFLH_PAGESIZE     = 0; // no page write (note: there is no so-called page read; all flash read is byte-addressable)
            EBDFLH_SIZE         = 1024 * 128;
        }
        else // rev 9010B/C..., 9011..., 902x
        {
            EBDFLH_PAGESIZE      = 128;
            EBDFLH_SIZE          = 1024 * 128; // for 9010A/B
        }
    }
    else if(EcIs1000(ec))
    {
        EBDFLH_PAGESIZE      = 256;
        EBDFLH_SIZE          = 1024 * 256;
    }
}

BOOL GetChipIdFromDevice(Ec *ec)
{
    /* When 930 or 9010 on i2c bus, the only registers we can access are XBI (through f/w's help)
       and must after code-in-ram. So need to get chip id without access to EC.
       We can hardcoding chip id for these chips here.
    ec->wDevId = 0x3930;
    ec->byRevId = 0xA0;
    return TRUE;
    */

    //
    // Get chip id from device.
    //

    BYTE ecSts = 0;

    // 39X and 373X have different reg offsets defines.
    // Assume 373X first.
    const WORD REG373_DEVIDL = 0xF01D;
    const WORD REG373_DEVIDH = 0xF01C;
    const WORD REG373_REVID  = 0xF019;

#if 0 // remove this when having KB1000 board.
#pragma message("warning: KB1000 test code enabled !!!")
    ec->wDevId = 0x1000;
    ec->byRevId = 0;
    return TRUE;
#endif
    //
    // Assume 373X first, whose reg offsets are different from 39XX
    EcReadReg(ec, REG373_DEVIDH, ((BYTE *) &ec->wDevId) + 1);     // Read twice to workaround: 3730+FT2232D+SMB: run program 1st time OK, then need to press reset button
    if(!EcReadReg(ec, REG373_DEVIDH, ((BYTE *) &ec->wDevId) + 1)) //   for the other times. "USB2TS_kit v16 chip DVT" AP OK. Don't know why.
        return FALSE;
    if((ec->wDevId >> 8) == 0x37)
    {
        EcReadReg(ec, REG373_DEVIDL, (BYTE *) &ec->wDevId);
        EcReadReg(ec, REG373_REVID, &ec->byRevId);
        if((ec->wDevId & 0xFFF0) == 0x3730) // _possibly_ 373x
        {
            // If device is 39xx, the reg reading above read flash code, which could be "luckily" 0x373x,
            //   so h/w people suggested checking these regs:
            BYTE byte = 0;
            EcReadReg(ec, REG_DEVIDH, &byte);
            if(byte == 0xFF) // definitely 373x
            {
                byte = 0;
                EcReadReg(ec, REG_DEVIDL, &byte);
                if(byte == 0xFF)
                    return TRUE;
            }
        }
    }

    //
    // 39XX ...

    if(!EcReadReg(ec, REG_ECSTS, &ecSts))
        return FALSE;

    if(ecSts & 0x04) // turn off bit2 before we can read chip id.
        EcWriteReg(ec, REG_ECSTS, (BYTE) (ecSts & ~0x04));

    EcReadReg(ec, REG_DEVIDH, ((BYTE *) &ec->wDevId) + 1);
    EcReadReg(ec, REG_DEVIDL, (BYTE *) &ec->wDevId);

    if(ecSts & 0x04) // restore bit2
        EcWriteReg(ec, REG_ECSTS, ecSts);

    EcReadReg(ec, REG_REVID, &ec->byRevId);

    return TRUE;
}

// Public functions ---------------------------------------------------------------------------------------------------

Ec *EcInit(enum CONN connType, ConnParam param)
{
    Ec *ec = ecs + ecCount++;

    ASSERT(ecCount <= ARRAY_SIZE(ecs));
    ASSERT(ec->conn == NULL); // forget EcExit() for previous EcInit()?

    param.flags |= CPFLAGS_EC; // indicate to conn that we are EC.
    ec->conn = ConnCreate(connType, param);

    if(!ec->conn)
        goto _ERROR_EXIT;

    if(!GetChipIdFromDevice(ec))
        goto _ERROR_EXIT;

    InitReg(ec); // change REG_XXX and BIT_XXX values according to chip id.

    if(param.flags & CPFLAGS_STOP_ON_CONNECT)
        EcStop(ec);

    if(connType != CONN_WINIDXIO) // win indexI/O, let f/w do it.
        EcLoadTrimData(ec, 0); // 0=basic data.

    return ec;

_ERROR_EXIT:

    if(ec)
        EcExit(ec);

    return NULL;
}

VOID EcExit(Ec *ec)
{
    ASSERT(ec);

    if(ec->conn)
    {
        ec->conn->Dtor(ec->conn);
        ec->conn = NULL;
    }
    ecCount--;
}

WORD EcGetChipId(Ec *ec)
{
    return ec->wDevId;
}

BYTE EcGetRevId(Ec *ec)
{
    return ec->byRevId;
}

BOOL EcIsEbdFlash(Ec *ec)
{
    return (EcIs901x(ec) || EcIs902x(ec) || EcIs373x(ec) || EcIs1000(ec));
}

BOOL EcSupportNewMemLayout(Ec *ec)
{
    return (EcIs394x(ec) || EcIs902x(ec) || EcIs1000(ec));
}

BOOL EcIs373x(Ec *ec)
{
    return (ec->wDevId >> 4) == 0x373;
}

BOOL EcIs901x(Ec *ec)
{
    return ((ec->wDevId & 0xFFF0) == 0x9010);
}

BOOL EcIs902x(Ec *ec)
{
    return ((ec->wDevId & 0xFFF0) == 0x9020);
}

BOOL EcIs394x(Ec *ec)
{
    return
        ((ec->wDevId & 0xFFF0) == 0x3940) || // 394x
        ((ec->wDevId & 0xFFF0) == 0x0940) || // 094x
        (ec->wDevId == 0x3313);              // 3313
}

BOOL EcIs1000(Ec *ec)
{
    return ((ec->wDevId & 0xFFF0) == 0x1000);
}

// Support embedded flash burst write: EDI and (9010C || 9011 || 9012 || 1000...)
BOOL EcHasBurstWrite(Ec *ec)
{
    if(EcGetConnType(ec) == CONN_FTEDI)
    {
        if(EcIs901x(ec) || EcIs902x(ec) || EcIs1000(ec))
        {
            if(ec->wDevId == 0x9010 && ec->byRevId < 0xC0) // exclude 9010A/B
                return FALSE;
            return TRUE;
        }
    }
    return FALSE;
}

// Support embedded flash page write: 9010B/C... 9011, 9012... 373x
BOOL EcHasPageWrite(Ec *ec)
{
    if(EcIsEbdFlash(ec))
    {
        if(ec->wDevId == 9010 && ec->byRevId == 0xA0) // exclude 9010A
            return FALSE;
        return TRUE;
    }
    return FALSE;
}

BOOL EcConnEnterCodeInRam(Ec *ec)
{
    return ec->conn->EnterCodeInRam(ec->conn);
}

BOOL EcConnExitCodeInRam(Ec *ec)
{
    return ec->conn->ExitCodeInRam(ec->conn);
}

Conn *EcGetConn(Ec *ec)
{
    return ec->conn;
}

enum CONN EcGetConnType(Ec *ec)
{
    return ec->conn->connType;
}

DWORD EcGetEbdFlashSize(Ec *ec)
{
    return EcIsEbdFlash(ec) ? EBDFLH_SIZE : 0;
}

DWORD EcGetEbdFlashSectorSize(Ec *ec) // or say page size
{
    return EBDFLH_PAGESIZE;
}

BOOL EcReadRegs(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes)
{
    return ec->conn->ReadRegs(ec->conn, wStartReg, pBytes, nBytes);
}

BOOL EcWriteRegs(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes)
{
    return ec->conn->WriteRegs(ec->conn, wStartReg, pBytes, nBytes);
}

// Read one byte.
BOOL EcReadReg(Ec *ec, WORD wReg, BYTE *p1Byte)
{
    return EcReadRegs(ec, wReg, p1Byte, 1);
}

// Write one byte.
BOOL EcWriteReg(Ec *ec, WORD wReg, BYTE val)
{
    return EcWriteRegs(ec, wReg, &val, 1);
}

BOOL EcSupportCmdSeq(Ec *ec)
{
    return (EcGetConnType(ec) == CONN_FTEDI);
}

VOID EcBeginCmdSeq(Ec *ec)
{
    ((ConnSeq *) ec->conn)->BeginCmdSeq((ConnSeq *) ec->conn);
}

BOOL EcEndCmdSeq(Ec *ec, BYTE *buf, INT nBufLen)
{
    return ((ConnSeq *) ec->conn)->EndCmdSeq((ConnSeq *) ec->conn, buf, nBufLen);
}

VOID EcReadRegsSeq(Ec *ec, WORD wStartReg, INT nBytes)
{
    ((ConnSeq *) ec->conn)->ReadRegsSeq((ConnSeq *) ec->conn, wStartReg, nBytes);
}

VOID EcReadRegSeq(Ec *ec, WORD w1Reg)
{
    ((ConnSeq *) ec->conn)->ReadRegsSeq((ConnSeq *) ec->conn, w1Reg, 1);
}

VOID EcWriteRegsSeq(Ec *ec, WORD wStartReg, BYTE *pBytes, INT nBytes)
{
    ((ConnSeq *) ec->conn)->WriteRegsSeq((ConnSeq *) ec->conn, wStartReg, pBytes, nBytes);
}

VOID EcWriteRegSeq(Ec *ec, WORD w1Reg, BYTE _1ByteVal)
{
    ((ConnSeq *) ec->conn)->WriteRegsSeq((ConnSeq *) ec->conn, w1Reg, &_1ByteVal, 1);
}

VOID EcDelaySeq(Ec *ec, INT us)
{
    ((ConnSeq *) ec->conn)->DelaySeq((ConnSeq *) ec->conn, us);
}

BOOL EcStop(Ec *ec)
{
    BYTE byte;

    if(EcIs373x(ec))
    {
        EcReadReg(ec, REG_E51RST, &byte);
        return EcWriteReg(ec, REG_E51RST, (BYTE) (byte | 0x01));
    }
    else
    {
        EcReadReg(ec, 0xFF14, &byte);
        return EcWriteReg(ec, 0xFF14, (BYTE) (byte | 0x01));
    }

    return FALSE;
}

BOOL EcRun(Ec *ec)
{
    BYTE byte;

    if(EcIs373x(ec))
    {
        EcReadReg(ec, REG_E51RST, &byte);
        return EcWriteReg(ec, REG_E51RST, (BYTE) (byte & ~0x01));
    }
    else
    {
        EcReadReg(ec, 0xFF14, &byte);
        return EcWriteReg(ec, 0xFF14, (BYTE) (byte & ~0x01));
    }

    return FALSE;
}

BOOL EcDisableWdt(Ec *ec)
{
    return EcWriteReg(ec, 0xFE80, 0x48);
}

BOOL EcIsStopped(Ec *ec)
{
    BYTE byte;

    if(EcIs373x(ec))
    {
        EcReadReg(ec, REG_E51RST, &byte);
        return (byte & 1);
    }
    else
    {
        EcReadReg(ec, 0xFF14, &byte);
        return (byte & 1);
    }

    return FALSE;
}

BOOL EcLoadTrimData(Ec *ec, INT type)
{
    if(EcIs901x(ec) || EcIs1000(ec))
        return LoadTrimData90xx_100x(ec, type);
    else if(EcIs902x(ec))
        return LoadTrimData902x(ec, type);

    return TRUE; // don't care others.
}

// ----------------- Trim flash related ---------------------------------------

//
// Get number of continuous bits in a value.
// For example, 0x1F ==> return 5
static INT GetBitsCount(ULONG val)
{
    INT nBit = 0;
    while(val)
    {
        if(val & 1)
            nBit++;
        else
            ASSERT(nBit == 0); // not continuous bits
        val >>= 1;
    }
    return nBit;
}

static INT GetFirstNonZeroBit(BYTE val)
{
    INT n = 0;
    while(val)
    {
        if(val & 1)
            return n;
        n++;
        val >>= 1;
    }
    return -1;
}

//
// For example, dstBits=0xF0(bit4~7), srcBits=0x0F(bit0~3) then
// copy bit0~3 of src to bit4~7 of dst.
static VOID BitsCopy(BYTE *dst, BYTE dstBits, BYTE src, BYTE srcBits)
{
    INT nShift = GetFirstNonZeroBit(dstBits) - GetFirstNonZeroBit(srcBits);

    ASSERT(GetBitsCount(dstBits) == GetBitsCount(srcBits));

    *dst &= ~dstBits;
    src &= srcBits;
    if(nShift >= 0)
        *dst |= src << nShift;
    else
        *dst |= src >> -nShift;
}

static BOOL WaitEbdFlashReady(Ec *ec, INT nMiliSec)
{
    BYTE cfg = 0;
    INT nThisWait;

    while(nMiliSec)
    {
        if(!EcReadReg(ec, REG_EFCFG, &cfg))
            return FALSE;

        if(EcIs373x(ec))
        {
            if(cfg & BIT_EFCFG_RDY)
                return TRUE;
        }
        else
        {
            if(!(cfg & BIT_EFCFG_BSY))
                return TRUE;
        }

        nThisWait = MIN(20, nMiliSec);
        Sleep(nThisWait);

        nMiliSec -= nThisWait;
    }

    return FALSE;
}

// Read embedded flash or special row without using EcFlashXxx moudle.
// (Similar to Read() in EcFlashEbd.c except the command code)
static BOOL _EcReadEbdFlash(Ec *ec, DWORD dwAdr, BYTE *buf, INT bufLen, BOOL bSpecialRow)
{
    BYTE cmd = bSpecialRow ? CMD_TRIM_DATA : CMD_READ;
    INT i;
#ifdef _DEBUG
    BYTE byte = 0;
#endif
    union FLASHADR adr = {0}; // avoid DMC error: constant initializer expected
    adr.dwAdr = dwAdr;        //

    ASSERT(EcIsEbdFlash(ec));

#ifdef _DEBUG
    if(!EcIs373x(ec))
    {
        if(!EcReadReg(ec, REG_EFCFG, &byte))
            return FALSE;
        ASSERT(byte & BIT_EFCFG_ENA); // make sure caller set this bit. (373x has no this bit)
    }
#endif

    if(!EcWriteReg(ec, REG_EFA0, adr.A0) || // the adr order doesn't matter for h/w mode.
       !EcWriteReg(ec, REG_EFA1, adr.A1))
       return FALSE;
    if(!EcIs373x(ec)) // 373X has no A2
        EcWriteReg(ec, REG_EFA2, adr.A2);

    for(i = 0; i < bufLen; i++)
    {
        if(!EcWriteReg(ec, REG_EFCMD, cmd))
            return FALSE;
        if(!WaitEbdFlashReady(ec, 1))
            return FALSE;
        EcReadReg(ec, REG_EFDIN, buf++);

        if(i < bufLen - 1) // not final byte
        {
            adr.dwAdr++;
            EcWriteReg(ec, REG_EFA0, adr.A0);
            if(adr.A0 == 0) // "carry" flag into A1
            {
                EcWriteReg(ec, REG_EFA1, adr.A1);
                if(adr.A1 == 0 && !EcIs373x(ec)) // "carry" flag into A2
                    EcWriteReg(ec, REG_EFA2, adr.A2);
            }
        }
    }

    return TRUE;
}

BOOL EcReadEbdFlash(Ec *ec, DWORD dwAdr, BYTE *buf, INT bufLen)
{
    return _EcReadEbdFlash(ec, dwAdr, buf, bufLen, FALSE/*not special row*/);
}

BOOL EcReadSpecialRow(Ec *ec, BYTE Ra/*row*/, BYTE Ca/*col*/, BYTE *buf, INT bufLen)
{
    DWORD dwAdr;

    if(EcIs901x(ec) || EcIs902x(ec)) // a row has 128 bytes (Ca=7bits)
        dwAdr = (Ra << 7) + Ca;
    else if(EcIs1000(ec))            // a row has 256 bytes (Ca=8bits)
        dwAdr = (Ra << 8) + Ca;
    else { ASSERT(!"Unsupported chip to read special row\n"); }

    return _EcReadEbdFlash(ec, dwAdr, buf, bufLen, TRUE/*special row*/);
}

// Copy special row to register (copy only some bits of a byte).
static BOOL SrowToReg(Ec *ec, WORD whichReg, BYTE regBits, BYTE Ra, BYTE Ca, BYTE srBits)
{
    BYTE reg, sr;

    if (!EcReadReg(ec, whichReg, &reg)   ||
        !EcReadSpecialRow(ec, Ra, Ca, &sr, 1))
        return FALSE;

    BitsCopy(&reg, regBits, sr, srBits);

    return EcWriteReg(ec, whichReg, reg);
}

typedef struct TRIM_OP
{
    BYTE SpecialRawA1;     // addr15:8 of flash's special row, containing flash vendor's parameters.
    BYTE SpecialRawA0;     // addr7:0 of flash's special row, containing flash vendor's parameters.
    BYTE SpecialRawBits;   // only some bits of the byte are valid.
    WORD TrimReg;          // we'll write the bits of the byte to some bits of one of the trim registers.
    BYTE TrimRegBits;      //
} TRIM_OP;

BOOL LoadTrimData90xx_100x(Ec *ec, INT type)
{
    TRIM_OP TrimOp_901C[] =
    { // SpecialRawA1 SpecialRawA0 SpecialRawBits TrimReg TrimRegBits
        {0x00,        0xF0,        0x1F,          0xFEB9, 0x1F},
        {0x00,        0xF1,        0x0F,          0xFEB6, 0xF0},
        {0x00,        0xF2,        0x0F,          0xFEB6, 0x0F},
        {0x00,        0xF4,        0x0F,          0xFEB7, 0x0F},
        {0x00,        0xF5,        0x07,          0xFEB8, 0xE0},
        {0x00,        0xF6,        0x1F,          0xFEB8, 0x1F},
        {0x00,        0xF3,        0x0F,          0xFEB7, 0xF0}, // (should be the last one to update)
        {0, 0, 0, 0} // EOT
    };

    TRIM_OP TrimOp_100X[] =
    { // SpecialRawA1 SpecialRawA0 SpecialRawBits TrimReg TrimRegBits
        {0x03,        0x71,        0x0F,          0xFEB6, 0xFF},  //PDACP, PDACE
        {0x03,        0x72,        0xFF,          0xFEB7, 0xFF},  //NDACP, NDACE
        {0x03,        0x74,        0x07,          0xFEB9, 0x07},  //TCTRIM
        {0x03,        0x76,        0x1F,          0xFEBA, 0x1F},  //ABSTRIM
        {0x03,        0x73,        0x3F,          0xFEB8, 0x3F},  //ITIM (should be the last one to update)
        {0, 0, 0, 0} // EOT
    };

    INT i;
    BYTE cfgOrg = 0;
    BYTE sr; // data from one byte of special raw
    BYTE tr; // trim data
    BYTE byte;
    BOOL bOK = FALSE;
    TRIM_OP *trimOps;

    if(ec->wDevId == 0x9010 && ec->byRevId >= 0xC0)
        trimOps = TrimOp_901C;
    else if(EcIs1000(ec))
        trimOps = TrimOp_100X;
    else
    {
        ENETRACE("FIXME: Loading trim data for unsupported chip\n");
        return FALSE;
    }

    EcReadReg(ec, REG_EFCFG, &cfgOrg);
    EcWriteReg(ec, REG_EFCFG, BIT_EFCFG_ENA);

    if(type == 0)
    {
        EcReadSpecialRow(ec, 1, 0, &byte, 1);
        if(byte == 0x5A)
        {
            BYTE s81, feb9;
            EcReadSpecialRow(ec, 1, 1, &s81, 1);
            EcReadReg(ec, 0xFEB9, &feb9);
            BitsCopy(&feb9, 0xC0, s81, 0x03);
            EcWriteReg(ec, 0xFEB9, feb9);
        }
    }
    else if(type == 1)
    {
        //Remark Reset&Run: Otherwise our loading trim-data does nothing because f/w loads again in its own (wrong/old) way.
        //                  Let EDI AP stop/run cpu.
        //BOOL wasRunning = !EcIsStopped(ec);
        //EcStop(ec);

        EcReadSpecialRow(ec, 3, 0x7F, &byte, 1);
        if(byte != 0)
        {
            // Addressing A2 of special row
            if (!EcWriteReg(ec, REG_EFA2, 0x00))
                goto _EXIT;

            i = 0;
            while(trimOps[i].TrimReg != 0) // not EOT
            {
                // Read the byte of special row at (A2,A1,A0)
                if (!EcWriteReg(ec, REG_EFA1, trimOps[i].SpecialRawA1)  ||
                    !EcWriteReg(ec, REG_EFA0, trimOps[i].SpecialRawA0)  ||
                    !EcWriteReg(ec, REG_EFCMD, CMD_TRIM_DATA)           ||
                    !WaitEbdFlashReady(ec, 10)                          ||
                    !EcReadReg(ec, REG_EFDIN, &sr))
                    goto _EXIT;

                // Read one of the trim register
                if(!EcReadReg(ec, trimOps[i].TrimReg, &tr))
                    goto _EXIT;

                // Copy bits of the special byte to bits of trim reg.
                BitsCopy(&tr, trimOps[i].TrimRegBits, sr, trimOps[i].SpecialRawBits);
                if (!EcWriteReg(ec, trimOps[i].TrimReg, tr)    ||
                    !WaitEbdFlashReady(ec, 10))
                    goto _EXIT;

                i++;
            }
            EcWriteReg(ec, 0xFEA5, 0x01); // flash clock timing from loose to normal.
        }

        EcReadSpecialRow(ec, 1, 0, &byte, 1);
        if(byte == 0x5A)
        {
            BYTE s82, s83; // value from special row.
            BYTE pllCfg, pllCfg2; // store desired value.
            BYTE ecsts;

            EcReadReg(ec, REG_ECSTS, &ecsts);                 // switch to PLLCFG2 so it
            EcWriteReg(ec, REG_ECSTS, (BYTE) (ecsts | 0x04)); //   is meaningful.

            EcReadReg(ec, 0xFF0F, &pllCfg);
            EcReadReg(ec, 0xFF1F, &pllCfg2);
            EcReadSpecialRow(ec, 1, 2, &s82, 1);
            EcReadSpecialRow(ec, 1, 3, &s83, 1);

            BitsCopy(&pllCfg, 0xF0, s82, 0x0F);
            BitsCopy(&pllCfg, 0x0F, s83, 0xF0);
            BitsCopy(&pllCfg2, 0xC0, s82, 0x30);

            EcWriteReg(ec, 0xFF0F, pllCfg);
            EcWriteReg(ec, 0xFF1F, pllCfg2);

            EcWriteReg(ec, REG_ECSTS, ecsts);                 // switch back.
        }

        //if(wasRunning)  See above.
        //    EcRun(ec);
    }

    bOK = TRUE;

_EXIT:

    EcWriteReg(ec, REG_EFCFG, cfgOrg);

    return bOK;
}

BOOL LoadTrimData902x(Ec *ec, INT type)
{
    BYTE cfgOrg = 0;
    BYTE byte;
    BOOL bOK = FALSE;

    if (!EcReadReg(ec, REG_EFCFG, &cfgOrg) ||
        !EcWriteReg(ec, REG_EFCFG, BIT_EFCFG_ENA))
        return FALSE;

    if(type == 0)
    {
        if(!EcReadSpecialRow(ec, 1, 0, &byte, 1))
            goto _EXIT;
        if(byte == 0x5A)
        {
            if (!SrowToReg(ec, 0x103C, 0x0F, 1, 1, 0x0F) ||
                !SrowToReg(ec, 0x103D, 0xFF, 1, 6, 0xFF))
                goto _EXIT;
        }
    }
    else if(type == 1)
    {
        if(!EcReadSpecialRow(ec, 3, 0x7F, &byte, 1))
            goto _EXIT;
        if(byte != 0)
        {
            if (!SrowToReg(ec, 0x1036, 0xFF, 3, 0x71, 0xFF) || //PDACP, PDACE
                !SrowToReg(ec, 0x1037, 0xFF, 3, 0x72, 0xFF) || //NDACP, NDACE
              //!SrowToReg(ec, 0x1038, 0x3F, 3, 0x73, 0x3F) || //ITIM (should be the last one to update)
                !SrowToReg(ec, 0x1039, 0x07, 3, 0x74, 0x07) || //TCTRIM
                !SrowToReg(ec, 0x103A, 0x1F, 3, 0x76, 0x1F) || //ABSTRIM
                !SrowToReg(ec, 0x1038, 0x3F, 3, 0x73, 0x3F))   //ITIM (should be the last one to update)
                goto _EXIT;

            if(!EcWriteReg(ec, 0x1025, 0x01)) // flash clock timing from loose to normal.
                goto _EXIT;
        }

        if(!EcReadSpecialRow(ec, 1, 0, &byte, 1))
            goto _EXIT;
        if(byte == 0x5A)
        {
            BYTE ecsts;
            EcReadReg(ec, 0x141D, &ecsts);                 // switch to PLLCFG2 so it
            EcWriteReg(ec, 0x141D, (BYTE) (ecsts | 0x04)); //   is meaningful.

            if (!SrowToReg(ec, 0x140F, 0xFF, 1, 3, 0xFF) ||
                !SrowToReg(ec, 0x141F, 0x0F, 1, 2, 0x0F) ||
                !SrowToReg(ec, 0x0E1C, 0x01, 1, 4, 0x01) ||
                !SrowToReg(ec, 0x0E0B, 0xFF, 1, 5, 0xFF))
                goto _EXIT;

            EcWriteReg(ec, 0x141D, ecsts);                 // switch back.
        }
    }

    bOK = TRUE;

_EXIT:

    if(!EcWriteReg(ec, REG_EFCFG, cfgOrg))
        return FALSE;

    return bOK;
}

#ifdef LOAD_TRIM_373X // This is for special AP's use. Normally, 373x maskROM load trim.

typedef struct TRIM_OP_373X
{
    BYTE SpecialRawA0;     // low addr of a byte in flash's special row, containing flash vendor's parameters.
    BYTE SpecialRawBits;   // only some bits of the byte are valid.
    WORD TrimReg;          // we'll write the bits of the byte to some bits of one of the trim registers.
    BYTE TrimRegBits;      //
} TRIM_OP_373X;

BOOL EcLoadTrimData373x(Ec *ec)
{
    TRIM_OP_373X TrimOp_A[] =
    { // SpecialRawA0 SpecialRawBits TrimReg TrimRegBits
        {0xF0,        0x1F,          0xFEA6, 0x1F}, // S[4:0]
        {0xF1,        0x0F,          0xFEA3, 0xF0}, // Pdac[3:0]
        {0xF2,        0x0F,          0xFEA3, 0x0F}, // Ndac[3:0]
        {0xF5,        0x0F,          0xFEA5, 0xF0}, // Tctrim[3:0]
        {0xF6,        0x0F,          0xFEA5, 0x0F}, // Abstrim[3:0]
        {0xF4,        0x0F,          0xFEA4, 0x0F}, // Bdac[3:0]
        {0xF3,        0x0F,          0xFEA4, 0xF0}, // Itim[3:0] (should be the last one to update)
        {0, 0, 0, 0} // EOT
    };
    TRIM_OP_373X TrimOp_B[] =
    { // SpecialRawA0 SpecialRawBits TrimReg TrimRegBits
        {0xF0,        0x1F,          0xFEA6, 0x1F}, // S[4:0]
        {0xF1,        0x0F,          0xFEA3, 0xF0}, // Pdac[3:0]
        {0xF2,        0x0F,          0xFEA3, 0x0F}, // Ndac[3:0]
        {0xF5,        0x07,          0xFEA5, 0xE0}, // Tctrim[2:0]  <--+- difference from 373xA
        {0xF6,        0x1F,          0xFEA5, 0x1F}, // Abstrim[4:0] <-/
        {0xF4,        0x0F,          0xFEA4, 0x0F}, // Bdac[3:0]
        {0xF3,        0x0F,          0xFEA4, 0xF0}, // Itim[3:0] (should be the last one to update)
        {0, 0, 0, 0} // EOT
    };

    INT i;
    BYTE sr; // data from one byte of special raw
    BYTE tr; // trim data
    BYTE byte;//, orgE51RST;
    BOOL bOK = FALSE;
    TRIM_OP_373X *pTrimOp = (ec->byRevId >= 0x10/*rev B0*/) ? TrimOp_B : TrimOp_A;

    // BIT_EFCFG_ENA is not needed for 373X

    //Remark Reset&Run: Otherwise our loading trim-data does nothing because maskROM loads again in its own (wrong/old) way.
    //                  Let EDI AP stop/run cpu.
    //EcReadReg(ec, REG_E51RST, &orgE51RST);
    //if(!EcWriteReg(ec, REG_E51RST, 1)) // stop cpu.
    //    goto _EXIT;

    if(!EcReadReg(ec, REG_XBITRIM, &byte) || (byte & 0x80) == 0)
    {
        ENETRACE("CRDY fail\n");
        goto _EXIT;
    }

    // Set A1 of special row (373X has no A2)
    if(!EcWriteReg(ec, REG_EFA1, 0x01))
        goto _EXIT;

    i = 0;
    while(pTrimOp[i].TrimReg != 0) // not EOT
    {
        // Read the byte of special row at (A1,A0)
        if (!EcWriteReg(ec, REG_EFA0, pTrimOp[i].SpecialRawA0)   ||
            !EcWriteReg(ec, REG_EFCMD, CMD_TRIM_DATA)            ||
            !WaitEbdFlashReady(ec, 10)                           ||
            !EcReadReg(ec, REG_EFDIN, &sr))
            goto _EXIT;

        // Read one of the trim register
        if(!EcReadReg(ec, pTrimOp[i].TrimReg, &tr))
            goto _EXIT;

        // Copy bits of the special byte to bits of trim reg.
        BitsCopy(&tr, pTrimOp[i].TrimRegBits, sr, pTrimOp[i].SpecialRawBits);
        if (!EcWriteReg(ec, pTrimOp[i].TrimReg, tr)    ||
            !WaitEbdFlashReady(ec, 10))
            goto _EXIT;

        i++;
    }

    bOK = TRUE;

_EXIT:

    EcWriteReg(ec, REG_EFCFG, 0x10); // bit4~6=cycle_count, 1=trimed
    //EcWriteReg(ec, REG_E51RST, orgE51RST); See above.

    return bOK;
}
#endif
