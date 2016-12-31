// MaskRom.h/.c:
//   Implement EC or TS embedded flash update through handshaking with mask ROM.
// Other files required:
//   One of ConnXxx.c.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#include "MyTypeDefs.h"
#include "Conn.h"
#include "MaskRom.h"

/*------------------- skipRam1 or not ----------------------------------------
  bin file 22KB
    I2C-clock   RAM         erase/prog/verify
                            time (sec)
    40K         1(skip)     15.1
    40K         2           14.2
    200K        1           4.7
    200K        2           3.7
    1M          1           2.1
    1M          2           2.8
*/
//-------------------- Struct, constants  -------------------------------------

struct MaskRom
{
    Conn *conn;
    DWORD dwFlashSize;
    DWORD dwPageSize;
    INT ramToggle; // see _GetReadyRam()
    BOOL skipRam1;
};

enum
{
    REG_E51CFG          = 0xF010, // E51 config
    REG_F011            = 0xF011, // E51_efen, real h/w reg
    REG_F012            = 0xF012, // E51_status, real h/w reg
    REG_BUF0            = 0x8000, // SRAM
    REG_BUF0_PARAMS     = 0x80F0,
    REG_BUF0STS         = 0x80F6,
    REG_BUF1            = 0x8100, // SRAM
    REG_BUF1_PARAMS     = 0x81F0,
    REG_BUF1STS         = 0x81F6,
};

enum
{
    BIT_BUFRDY          = 0x02,
    BIT_UPDATE_REQ      = 0x01,
    BIT_CODE_IN_FLH_REQ = 0x01,
    BIT_ROM_OWNERSHIP   = 0x02,
};

enum
{
    //EBD_FLH_SIZE        = 0x3FF0, // replaced by dwFlashSize
    //EBD_FLH_PAGE_SIZE   = 512,    //             dwPageSize
    RAM_SIZE            = 128,
    CMD_CHIP_ERASE      = 0x10,
    CMD_PAGE_ERASE      = 0x11,
    CMD_PROGRAM         = 0x20,
    CMD_FLH_READ        = 0x30,
    CMD_FLH_FINISH      = 0x80,
};

typedef struct
{
    BYTE parsing : 1; // f/w is parsing/processing the command.
    BYTE ready : 1;   // the buffer is ready for use.
    BYTE rev : 2;
    BYTE err : 4;     // ERR_XXX below
} BufSts;

enum {
    ERR_OK          = 0,
    ERR_INV_ADR     = 1,
    ERR_INV_SIZ     = 2,
    ERR_INV_ADR_SIZ = 3,
    ERR_WP          = 4,
    ERR_RP          = 5,
    ERR_INV_CMD     = 0x0F,
    /* my error codes, not from MaskRom spec. */
    ERR_TIMEOUT     = 0x10,
    ERR_IO          = 0x11,
};

#pragma pack(1)
typedef struct
{
    BYTE A2;
    BYTE A1;
    BYTE A0;
    BYTE len2;
    BYTE len1;
    BYTE len0;
    BYTE zeros; // it is in fact buffer status; must zero it when sending cmd/params.
    BYTE cmd;
} CmdParams;
#pragma pack()

//------------------------- Static var ----------------------------------------------------

static MaskRom s_mask; // at most one of this working structure allowed.

//-------------------------- Static functions ---------------------------------------------------

static BOOL ReadRegs(MaskRom *mask, WORD wReg, BYTE *pBytes, INT nBytes)
                { return mask->conn->ReadRegs(mask->conn, wReg, pBytes, nBytes); }
static BOOL ReadReg(MaskRom *mask, WORD wReg, BYTE *p1Byte)
                { return ReadRegs(mask, wReg, p1Byte, 1); }
static BOOL WriteRegs(MaskRom *mask, WORD wReg, BYTE *pBytes, INT nBytes)
                { return mask->conn->WriteRegs(mask->conn, wReg, pBytes, nBytes); }
static BOOL WriteReg(MaskRom *mask, WORD wReg, BYTE val)
                { return WriteRegs(mask, wReg, &val, 1); }

static CHAR *ErrString(INT err)
{
    switch(err)
    {
    case ERR_OK:
        return "No error";
    case ERR_INV_ADR:
        return "Invalid address";
    case ERR_INV_SIZ:
        return "Invalid size";
    case ERR_INV_ADR_SIZ:
        return "Invalid address size";
    case ERR_WP:
        return "Write protected";
    case ERR_RP:
        return "Read protected";
    case ERR_INV_CMD:
        return "Invalid command";
    case ERR_TIMEOUT:
        return "RAM timeout";
    case ERR_IO:
        return "Read/write reg error";
    default:
        return "Unknown error";
    };
}

static BOOL ReadRamBuffer(MaskRom *mask, INT iRam, BYTE *buf, DWORD len)
{
    WORD regBuf = (iRam == 0) ? REG_BUF0 : REG_BUF1;
    return ReadRegs(mask, regBuf, buf, len);
}

#define RAM_TOGGLE  2

// If whichRam=RAM_TOGGLE, we try one ram then another ("toggle"), UNTIL we get any ready ram.
//   This is useful when reading flash: when both rams have data ready from flash, if we don't toggle,
//   we always get ram0 so moves only ram0 data to app.
// If whichRam=0 or 1, we wait UNTIL whichRam is ready.
// Return ready ram 0 or 1, or return negative error code.
static INT _GetReadyRam(MaskRom *mask, INT whichRam)
{
    BufSts sts;
    WORD regSts;
    INT ram = 0;
    DWORD dt = 0;
#ifdef GetTickCount
    DWORD t0 = GetTickCount();
#endif

    if(whichRam == 0 || whichRam == 1)
        ram = whichRam;

    while(dt < 2000) // retry 2 sec.
    {
        if(whichRam == RAM_TOGGLE) // try one ram than another...
        {
            mask->ramToggle = (mask->ramToggle == 0) ? 1 : 0;
            ram = mask->ramToggle;
        }

        regSts = (ram == 0) ? REG_BUF0STS : REG_BUF1STS;

        if (!ReadReg(mask, regSts, (BYTE *) &sts))
        {
            ENETRACE("_GetReadyRam() failed: ReadReg(REG_BUF%dSTS) fail.\n", ram);
            return -ERR_IO;
        }

        if(sts.ready)
        {
            if(sts.err)
            {
                ENETRACE("_GetReadyRam() RAM%d ERR: %s\n", ram, ErrString(sts.err));
                return -sts.err;
            }
            return ram;
        }

#ifdef GetTickCount
        dt = GetTickCount() - t0; // wrapping doesn't matter.
#else
        Sleep(10);
        dt += 10;
#endif
    }

    ENETRACE("_GetReadyRam() timed out\n");
    return -ERR_TIMEOUT;
}

static INT GetReadyRam(MaskRom *mask)
{
    return _GetReadyRam(mask, mask->skipRam1 ? 0 : RAM_TOGGLE);
}

static BOOL WaitRamReady(MaskRom *mask, INT iRam)
{
    ASSERT(!(mask->skipRam1 && (iRam == 1)));
    return (_GetReadyRam(mask, iRam) == iRam);
}

/*
static BOOL WaitBothRamReady(MaskRom *mask)
{
    if(WaitRamReady(mask, 0))
    {
        if(!mask->skipRam1)
            return WaitRamReady(mask, 1);
        else
            return TRUE;
    }
    else
        return FALSE;
}*/

static BOOL StartReadFlashToRam(MaskRom *mask, INT ram, DWORD adr, DWORD len)
{
    WORD regParams;
    CmdParams params = {0};

    ASSERT(len <= RAM_SIZE);

    regParams = (ram == 0) ? REG_BUF0_PARAMS : REG_BUF1_PARAMS;

    params.A0 = (BYTE) adr;
    params.A1 = (BYTE) (adr >> 8);
    params.A2 = (BYTE) (adr >> 16);
    params.len0 = (BYTE) len;
    params.len1 = (BYTE) (len >> 8);
    params.len2 = (BYTE) (len >> 16);
    params.cmd = CMD_FLH_READ;

    if(!WriteRegs(mask, regParams, (BYTE *) &params, sizeof(params)))
        return FALSE;

    return TRUE;
}

static BOOL WriteDataToRam(MaskRom *mask, INT ram, DWORD adr, BYTE *buf, DWORD len)
{
    WORD regBuf;
    WORD regParams;
    CmdParams params = {0};

    ASSERT(len <= RAM_SIZE);

    regBuf = (ram == 0) ? REG_BUF0 : REG_BUF1;
    regParams = (ram == 0) ? REG_BUF0_PARAMS : REG_BUF1_PARAMS;

    params.A0 = (BYTE) adr;
    params.A1 = (BYTE) (adr >> 8);
    params.A2 = (BYTE) (adr >> 16);
    params.len0 = (BYTE) len;
    params.len1 = (BYTE) (len >> 8);
    params.len2 = (BYTE) (len >> 16);
    params.cmd = CMD_PROGRAM;

    return WriteRegs(mask, regBuf, buf, len) &&
            WriteRegs(mask, regParams, (BYTE *) &params, sizeof(params));
}

//-------------------- Public functions ---------------------------------------------------------

MaskRom *MaskRomInit(Conn *conn, BOOL bSkipRam1)
{
    MaskRom *mask = &s_mask;

    ASSERT(mask->conn == NULL); // at most one of this working structure allowed.

    mask->conn = conn;
    mask->ramToggle = 0; // 0 or 1 initially doesn't matter.
    mask->skipRam1 = bSkipRam1;

    return mask;
}

VOID MaskRomExit(MaskRom *mask)
{
    ASSERT(mask);

    mask->conn = NULL;
}

BOOL MaskRomEnterCodeInRom(MaskRom *mask)
{
    INT i;
    BYTE flg12 = 0, flg11 = 0, rst;

    if (!ReadReg(mask, REG_F012, &flg12) || !ReadReg(mask, REG_F011, &flg11) || !ReadReg(mask, REG_E51CFG, &rst))
        return FALSE;

    flg12 |= BIT_UPDATE_REQ;          // update request
    flg11 &= ~BIT_CODE_IN_FLH_REQ;    // code-in-rom request

    if (!WriteReg(mask, REG_E51CFG, (BYTE) (rst | 1)) || // reset&stop 8051 to make maskrom totally restart!
        !WriteReg(mask, 0xFE80, 0x48)                 || // disable WDT.
        !WriteReg(mask, REG_F012, flg12)              || // set the requests.
        !WriteReg(mask, REG_F011, flg11))                //
        return FALSE;

    if(!WriteReg(mask, REG_E51CFG, (BYTE) (rst & ~1))) // run 8051
        return FALSE;

    for(i = 0; i < 200; i++) // retry 2 sec
    {
        if (!ReadReg(mask, REG_F012, &flg12) || !ReadReg(mask, REG_F011, &flg11))
            return FALSE;

        if ((flg12 & BIT_ROM_OWNERSHIP)    &&
            (flg12 & BIT_UPDATE_REQ)       &&
            !(flg11 & BIT_CODE_IN_FLH_REQ))
            return TRUE;

        Sleep(10);
    }

    ENETRACE("MaskRomEnterCodeInRom() failed\n");
    return FALSE;
}

BOOL MaskRomExitCodeInRom(MaskRom *mask)
{
    WORD regParams;
    CmdParams params = {0};
    INT i, ram;

    ram = GetReadyRam(mask);
    if(ram < 0) // error
        return FALSE;

    regParams = (ram == 0) ? REG_BUF0_PARAMS : REG_BUF1_PARAMS;
    params.cmd = CMD_FLH_FINISH; // tell rom to end parsing cmds.

    if(!WriteRegs(mask, regParams, (BYTE *) &params, sizeof(params)))
        return FALSE;

    for(i = 0; i < 200; i++) // retry 2 sec
    {
        BYTE flg12 = 0;
        if(!ReadReg(mask, REG_F012, &flg12))
            return FALSE;
        if (!(flg12 & BIT_UPDATE_REQ)) // &&
            //!(flg12 & BIT_ROM_OWNERSHIP) == 0)  This stands only if flash code is checksum good.
        {
            // Seems required. The ROM checks checksum by reading flash, if good, gives ownership to flash code...
            // This delay especially required when there is checksum-good f/w in flash.
            // If no delay, the next loop of enter-code-in-rom->erase->program, is easy to fail.
            // Not clear about why ... ???
            Sleep(500);

            return TRUE;
        }

        Sleep(10);
    }

    return FALSE;
}

BOOL MaskRomFlashChipErase(MaskRom *mask)
{
    DWORD len;
    WORD regParams;
    CmdParams params = {0};
    INT ram;

    ram = GetReadyRam(mask);
    if(ram < 0) // error
        return FALSE;

    len = mask->dwFlashSize;
    regParams = (ram == 0) ? REG_BUF0_PARAMS : REG_BUF1_PARAMS;

    params.len0 = (BYTE) (len);
    params.len1 = (BYTE) (len >> 8);
    params.len2 = (BYTE) (len >> 16);
    params.cmd = CMD_CHIP_ERASE;

    if(!WriteRegs(mask, regParams, (BYTE *) &params, sizeof(params)))
        return FALSE;

    // Doesn't help. but spec. says it is needed.
    if(!WaitRamReady(mask, ram))
        return FALSE;

    Sleep(200);  // seems required for waiting erase done, otherwise subsequent flash-write
                 //   causes compare error. (any better way like status checking ?)
    return TRUE;
}

// Not easy to perform checksum if using page-erase ...
// Just for testing purpose.
BOOL MaskRomFlashPageErase(MaskRom *mask, DWORD dwStartPage, DWORD dwPageCnt)
{
    DWORD adr = dwStartPage * mask->dwPageSize;
    INT ram = -1;
    CONST DWORD SIZ = mask->dwPageSize; // it should always be page_size.
    WORD regParams;
    CmdParams params = {0};

    while(dwPageCnt)
    {
        ram = GetReadyRam(mask);
        if(ram < 0) // error
            return FALSE;

        regParams = (ram == 0) ? REG_BUF0_PARAMS : REG_BUF1_PARAMS;

        params.A0 = (BYTE) adr;
        params.A1 = (BYTE) (adr >> 8);
        params.A2 = (BYTE) (adr >> 16);
        params.len0 = (BYTE) SIZ;
        params.len1 = (BYTE) (SIZ >> 8);
        params.len2 = (BYTE) (SIZ >> 16);
        params.cmd = CMD_PAGE_ERASE;

        if(!WriteRegs(mask, regParams, (BYTE *) &params, sizeof(params)))
            return FALSE;

        // No need to wait ram ready, because we'll do it in next GetReadyRam();

        adr += mask->dwPageSize;
        dwPageCnt--;
    }

    return WaitRamReady(mask, ram); // wait final page erase done.
}

BOOL MaskRomFlashWrite(MaskRom *mask, DWORD adr, BYTE *buf, DWORD len)
{
    DWORD dwBytesLeft = len;
    DWORD dwBytesThisWrite;
    INT ram = 0;

    while(dwBytesLeft)
    {
        ram = GetReadyRam(mask);
        if(ram < 0) // error
            break;

        dwBytesThisWrite = MIN(dwBytesLeft, (DWORD) RAM_SIZE);

        if(WriteDataToRam(mask, ram, adr, buf, dwBytesThisWrite))
        {
            dwBytesLeft -= dwBytesThisWrite;
            adr += dwBytesThisWrite;
            buf += dwBytesThisWrite;
        }
        else
            break;
    }

    if(dwBytesLeft) // not all done.
        return FALSE;

    //Sleep(100); // seems not required.

    return WaitRamReady(mask, ram); // wait final write done.
}

BOOL MaskRomFlashRead(MaskRom *mask, DWORD adr, BYTE *buf, DWORD len)
{
    struct RamDesc
    {
        BYTE *callerBuf;
        DWORD flashDataLen;
    } RamDescs[2] = {{0}};

    DWORD dwBytesNotToCaller = len; // bytes not copied to caller yet. 0=completely done.
    DWORD dwBytesNotToRam = len;    // bytes not moved from flash to EC ram yet.
    INT ram;

    while(dwBytesNotToCaller)
    {
        ram = GetReadyRam(mask);
        if(ram < 0) // error
            break;

        // If this ready ram HAD flash data in it, move data {from ram to host}.
        if(RamDescs[ram].flashDataLen)
        {
            if(ReadRamBuffer(mask, ram, RamDescs[ram].callerBuf, RamDescs[ram].flashDataLen))
            {
                dwBytesNotToCaller -= RamDescs[ram].flashDataLen;
                RamDescs[ram].callerBuf = NULL;
                RamDescs[ram].flashDataLen = 0;
            }
            else // error
                break;
        }

        // Now the ready ram is empty, move data {from flash to ram}.
        if(dwBytesNotToRam)
        {
            DWORD dwBytesThisRead = MIN(dwBytesNotToRam, (DWORD) RAM_SIZE);

            if(StartReadFlashToRam(mask, ram, adr, dwBytesThisRead))
            {
                RamDescs[ram].callerBuf = buf;
                RamDescs[ram].flashDataLen = dwBytesThisRead;

                adr += dwBytesThisRead;
                buf += dwBytesThisRead;
                dwBytesNotToRam -= dwBytesThisRead;
            }
            else // error
                break;
        }
    }

    return (dwBytesNotToCaller == 0);
}

VOID MaskRomFlashSetSize(MaskRom *mask, DWORD siz)
{
    mask->dwFlashSize = siz;
}

DWORD MaskRomFlashGetSize(MaskRom *mask)
{
    return mask->dwFlashSize;
}

VOID MaskRomFlashSetPageSize(MaskRom *mask, DWORD siz)
{
    mask->dwPageSize = siz;
}

DWORD MaskRomFlashGetPageSize(MaskRom *mask)
{
    return mask->dwPageSize;
}
