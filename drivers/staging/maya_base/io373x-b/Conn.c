// Conn.c:
//   Dispatch ConnCreate() call to proper ConnXxx_Ctor().
//
// Copyright (C) ENE TECHNOLOGY INC. 2010

#include "MyTypeDefs.h"
#include "Conn.h"

struct io373b;

struct ConnDosIdxIo     *ConnDosIdxIo_Ctor(WORD idxIoBase);
struct ConnWinIdxIo     *ConnWinIdxIo_Ctor(WORD idxIoBase);
struct ConnFtEdi        *ConnFtEdi_Ctor(VOID);
struct ConnFtSpid       *ConnFtSpid_Ctor(VOID);
struct ConnFtSmb        *ConnFtSmb_Ctor(BYTE i2cAdr, BOOL bEc);
struct ConnLnEcDrv      *ConnLnEcDrv_Ctor(VOID);
struct ConnLnTsDrv      *ConnLnTsDrv_Ctor(VOID);
struct ConnWinHid       *ConnWinHid_Ctor(VOID);
struct ConnFtSpid360x   *ConnFtSpid360x_Ctor(VOID);
struct ConnLnI2cDevSmb  *ConnLnI2cDevSmb_Ctor(INT iAdapter, BYTE i2cAdr);
struct ConnWoa373xRw    *ConnWoa373xRw_Ctor(VOID);
struct ConnSpbTest      *ConnSpbTest_Ctor(VOID);
struct conn_io373b_smbd *conn_io373b_smbd_ctor(struct io373b *io373b);
struct ConnEcSmb        *ConnEcSmb_Ctor(ConnParam param);
struct ConnBrgEc        *ConnBridgeEc_Ctor(ConnParam param);
struct ConnFakeEc       *ConnFakeEc_Ctor(ConnParam param);

#if defined(_CONN_DOSIDXIO)     || \
    defined(_CONN_WINIDXIO)     || \
    defined(_CONN_FTEDI)        || \
    defined(_CONN_FTSPID)       || \
    defined(_CONN_FTSMB)        || \
    defined(_CONN_LNECDRV)      || \
    defined(_CONN_LNTSDRV)      || \
    defined(_CONN_FTSPID360X)   || \
    defined(_CONN_WINHID)       || \
    defined(_CONN_LNI2CDEVSMB)  || \
    defined(_CONN_WOA373XRW)    || \
    defined(_CONN_SPBTEST)      || \
    defined(_CONN_IO373B_SMBD)  || \
    defined(_CONN_ECSMB)        || \
    defined(_CONN_BRIDGEEC)     || \
    defined(_CONN_FAKEEC)

Conn *ConnCreate(enum CONN connType, ConnParam param)
{
    switch(connType)
    {
#ifdef _CONN_DOSIDXIO
    case CONN_DOSIDXIO:
        return (Conn *) ConnDosIdxIo_Ctor(param.res.idxIoBase);
#endif

#ifdef _CONN_WINIDXIO
    case CONN_WINIDXIO:
        return (Conn *) ConnWinIdxIo_Ctor(param.res.idxIoBase);
#endif

#ifdef _CONN_FTEDI
    case CONN_FTEDI:
        return (Conn *) ConnFtEdi_Ctor();
#endif

#ifdef _CONN_FTSPID
    case CONN_FTSPID:
        return (Conn *) ConnFtSpid_Ctor();
#endif

#ifdef _CONN_FTSMB
    case CONN_FTSMB:
        return (Conn *) ConnFtSmb_Ctor(param.res.i2cAdr, param.flags & CPFLAGS_EC);
#endif

#ifdef _CONN_LNECDRV
    case CONN_LNECDRV:
        return (Conn *) ConnLnEcDrv_Ctor();
#endif

#ifdef _CONN_LNTSDRV
    case CONN_LNTSDRV:
        return (Conn *) ConnLnTsDrv_Ctor();
#endif

#ifdef _CONN_WINHID
    case CONN_WINHID:
        return (Conn *) ConnWinHid_Ctor();
#endif

#ifdef _CONN_FTSPID360X
    case CONN_FTSPID360X:
        return (Conn *) ConnFtSpid360x_Ctor();
#endif

#ifdef _CONN_LNI2CDEVSMB
    case CONN_LNI2CDEVSMB:
        return (Conn *) ConnLnI2cDevSmb_Ctor(param.res.i2cDev.iAdapter, param.res.i2cDev.i2cAdr);
#endif

#ifdef _CONN_WOA373XRW
    case CONN_WOA373XRW:
        return (Conn *) ConnWoa373xRw_Ctor();
#endif

#ifdef _CONN_SPBTEST
    case CONN_SPBTEST:
        return (Conn *) ConnSpbTest_Ctor();
#endif

#ifdef _CONN_IO373B_SMBD
    case CONN_IO373B_SMBD:
        return (Conn *) conn_io373b_smbd_ctor(param.res.p);
#endif

#ifdef _CONN_ECSMB
    case CONN_ECSMB:
        return (Conn *) ConnEcSmb_Ctor(param);
#endif

#ifdef _CONN_BRIDGEEC
    case CONN_BRIDGEEC:
        return (Conn *) ConnBridgeEc_Ctor(param);
#endif

#ifdef _CONN_FAKEEC
    case CONN_FAKEEC:
        return (Conn *) ConnFakeEc_Ctor(param);
#endif

    default:
        ENETRACE("Invalid connType. Be sure you've defined desired _CONN_XXX !\n");
        ASSERT(0);
        return NULL;
    }
}

#else // no any _CONN_XXX defined !

#error "Please define one or more desired _CONN_XXX !"

#endif

static BOOL EnterCodeInRam(Conn *conn) { return TRUE; }
static BOOL ExitCodeInRam(Conn *conn) { return TRUE; }

Conn *_Conn_Ctor(Conn *conn)
{
    ASSERT(conn);

    conn->connType = CONN_INVALID;
    conn->EnterCodeInRam = EnterCodeInRam;
    conn->ExitCodeInRam = ExitCodeInRam;
    return conn;
}

VOID _Conn_Dtor(Conn *conn)
{
}

ConnSeq *_ConnSeq_Ctor(ConnSeq *connSeq)
{
    return (ConnSeq *) _Conn_Ctor((Conn *) connSeq);
}

VOID _ConnSeq_Dtor(ConnSeq *connSeq)
{
    _Conn_Dtor((Conn *) connSeq);
}
