// Conn.h:
//   Define the uniform interface Conn::Read/WriteRegs(), and
//     ConnSeq::Read/WriteRegsSeq(), ... etc, for access to EC or TS
//     internal register/SRAM.
//   Representing "connections" such as FTDI<-->EC/TS_SPID, or FTDI<-->EC/TS_SMB,
//     LPC<-->EC_index_I/O.
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#ifndef _CONN_H_A7795606_4BC1_4AB6_8F24_AFD6A8D94C8F
#define _CONN_H_A7795606_4BC1_4AB6_8F24_AFD6A8D94C8F

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Conn Conn;
typedef struct ConnSeq ConnSeq;

enum CONN
{
    CONN_DOSIDXIO = 1,
    CONN_WINIDXIO,
    CONN_FTEDI,
    CONN_FTSPID,
    CONN_FTSMB,
    CONN_LNECDRV,
    CONN_LNTSDRV,
    CONN_WINHID,
    CONN_FTSPID360X,
    CONN_LNI2CDEVSMB,
    CONN_WOA373XRW,
    CONN_SPBTEST,
    CONN_IO373B_SMBD,
    CONN_BRIDGEEC,
    CONN_ECSMB,
    CONN_FAKEEC,
    CONN_INVALID = -1
};

typedef struct ConnParam
{
    union
    {
        WORD idxIoBase;
        BYTE i2cAdr;
        struct
        {
            struct Ec *ecMaster;
            INT busNo;
            BYTE slaveAdr;
        } ecSmb;
        struct
        {
            INT iAdapter;
            BYTE i2cAdr;
        } i2cDev;
        VOID *p;
    } res;
    enum CONN connBrg;    // CONN_XXX connection to the EC bridge; this selects which field to use in res.
    WORD brgBase;         // SRAM base for cmd/params inside the EC bridge.
    DWORD flags;

#ifdef __cplusplus
    // ConnParam() : idxIoBase(0), i2cAdr(0) { } // not allowed
    ConnParam() { res.idxIoBase = 0; res.i2cAdr = 0; res.i2cDev.iAdapter = -1, res.i2cDev.i2cAdr = 0; res.p = NULL; flags = 0; connBrg = CONN_INVALID, brgBase = 0; }
#endif
} ConnParam;

#ifdef __cplusplus
#define CONN_NO_PARAM   ConnParam() // for dmc to pass
#endif

// Public functions
//
Conn *ConnCreate(enum CONN connType, ConnParam param);

typedef VOID (*PFN_CONN_DTOR) (Conn *conn);
typedef BOOL (*PFN_CONN_READ_REGS) (Conn *conn, WORD wStartReg, BYTE *pBytes, INT nBytes);
typedef BOOL (*PFN_CONN_WRITE_REGS) (Conn *conn, WORD wStartReg, BYTE *pBytes, INT nBytes);
typedef BOOL (*PFN_CONN_ENTER_CODE_IN_RAM) (Conn *conn);
typedef BOOL (*PFN_CONN_EXIT_CODE_IN_RAM) (Conn *conn);

typedef VOID (*PFN_CONNSEQ_BEGIN_CMD_SEQ) (ConnSeq *conn);
typedef BOOL (*PFN_CONNSEQ_END_CMD_SEQ) (ConnSeq *conn, BYTE *buf, INT nBufLen);
typedef VOID (*PFN_CONNSEQ_READ_REGS_SEQ) (ConnSeq *conn, WORD wStartReg, INT nBytes);
typedef VOID (*PFN_CONNSEQ_WRITE_REGS_SEQ) (ConnSeq *conn, WORD wStartReg, BYTE *pBytes, INT nBytes);
typedef VOID (*PFN_CONNSEQ_DELAY_SEQ) (ConnSeq *conn, INT us);
typedef BOOL (*PFN_CONNSEQ_READ_ADR) (ConnSeq *conn, DWORD dwStartAdr, BYTE *pBytes, INT nBytes);
typedef BOOL (*PFN_CONNSEQ_WRITE_ADR) (ConnSeq *conn, DWORD dwStartAdr, BYTE *pBytes, INT nBytes);

struct Conn // acts as an "ABC".
{
// Public, virtual:
    PFN_CONN_DTOR               Dtor;
    PFN_CONN_READ_REGS          ReadRegs;
    PFN_CONN_WRITE_REGS         WriteRegs;
    PFN_CONN_ENTER_CODE_IN_RAM  EnterCodeInRam;
    PFN_CONN_EXIT_CODE_IN_RAM   ExitCodeInRam;

    enum CONN connType;
};

// Protected: (for derive only)
Conn *_Conn_Ctor(Conn *conn);
VOID _Conn_Dtor(Conn *conn);

struct ConnSeq // acts as an "ABC".
{
    Conn _conn;

// Public, virtual:
    PFN_CONNSEQ_BEGIN_CMD_SEQ      BeginCmdSeq;
    PFN_CONNSEQ_END_CMD_SEQ        EndCmdSeq;
    PFN_CONNSEQ_READ_REGS_SEQ      ReadRegsSeq;
    PFN_CONNSEQ_WRITE_REGS_SEQ     WriteRegsSeq;
    PFN_CONNSEQ_DELAY_SEQ          DelaySeq;
};

// Protected: (for derive only)
ConnSeq *_ConnSeq_Ctor(ConnSeq *connSeq);
VOID _ConnSeq_Dtor(ConnSeq *connSeq);

// For special purpose to access underlying SPI, I2C or EDI class.
#ifdef __cplusplus // a client .cpp file can just get and call these underlying C++ classes.

class CMyFtSpi;
class CMyFtI2c;
class CEcFtEdi;

CMyFtSpi *ConnFtSpid_GetFtSpi(struct ConnFtSpid *connFtSpid);
CMyFtI2c *ConnFtSmb_GetFtI2c(struct ConnFtSmb *connFtSmb);
CEcFtEdi *ConnFtEdi_GetFtEdi(struct ConnFtEdi *connFtEdi);
CMyFtSpi *ConnFtSpid360x_GetFtSpi(struct ConnFtSpid360x *connFtSpid360x);

#endif

struct ConnFtSpid;
struct ConnFtSmb;
struct ConnFtEdi;
struct ConnFtSpid360x;
struct ConnWinIdxIo;

// For a client .c file to use these special functions, we declare the functions here, and implement them in right place.
BOOL ConnFtSpid_SetSerialClock(struct ConnFtSpid *connFtSpid, DWORD dwKHz);
BOOL ConnFtSpid360x_SetSerialClock(struct ConnFtSpid360x *connFtSpid360x, DWORD dwKHz);
BOOL ConnFtSmb_SetSerialClock(struct ConnFtSmb *connFtSmb, DWORD dwKHz);
BOOL ConnFtEdi_SetSerialClock(struct ConnFtEdi *connFtEdi, DWORD dwKHz);
BOOL ConnFtEdi_BurstWrite(struct ConnFtEdi *connFtEdi, BYTE *pBytes, INT nBytes);
BOOL ConnFtSmb_SwitchToHwMode(struct ConnFtSmb *connFtSmb);
BYTE ConnFtSmb_DetectI2cAdr(struct ConnFtSmb *connFtSmb);
BYTE ConnFtSmb_GetI2cAdr(struct ConnFtSmb *connFtSmb);
HANDLE ConnWinIdxIo_GetMyio(struct ConnWinIdxIo *connWinIdxIo);

#ifdef __cplusplus
}
#endif

#define CPFLAGS_STOP_ON_CONNECT     0x00000001
#define CPFLAGS_EC                  0x80000000

#endif // _CONN_H_A7795606_4BC1_4AB6_8F24_AFD6A8D94C8F
