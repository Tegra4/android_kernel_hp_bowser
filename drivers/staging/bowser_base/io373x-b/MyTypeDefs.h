///////////////////////////////////////////////////////////////////////////////////////
// Modify this file to have proper #include, #define, typedef for your platform.
// Most files in this folder #include this header.

#ifndef _MY_TYPE_DEFS_H_
#define _MY_TYPE_DEFS_H_

#include <linux/module.h>
#include <linux/delay.h>

// TRUE, FALSE
#ifndef TRUE
#define TRUE      1
#endif
#ifndef FALSE
#define FALSE     0
#endif

// Sleep()
#define Sleep(ms)       msleep(ms)

// ASSERT()
#if defined(DEBUG) || defined(_DEBUG)
#define ASSERT(expr)     if (!(expr)) { \
			     printk("assertion failed! %s[%d]: %s\n", \
				    __func__, __LINE__, #expr); \
			     panic("%s", __func__); \
			 }
#else
#define ASSERT(expr)     do {} while (0)
#endif

// GetTickCount()
#define GetTickCount()            jiffies
#define TicksToMsec(ticks)        jiffies_to_msecs((ticks))

// it gets called only on error
#define ENETRACE pr_err

  typedef char CHAR;
  typedef short SHORT;
  typedef long LONG;
  typedef int INT;
  typedef unsigned char UCHAR;
  typedef unsigned short USHORT;
  typedef unsigned long ULONG;
  typedef unsigned int UINT;
  typedef unsigned char BYTE;
  typedef unsigned short WORD;
  typedef unsigned long DWORD;
  typedef int BOOL;
  typedef void VOID;
  typedef char * PCHAR;
  typedef short * PSHORT;
  typedef long * PLONG;
  typedef unsigned char * PUCHAR;
  typedef unsigned short * PUSHORT;
  typedef unsigned long * PULONG;
  typedef void * PVOID;
  typedef void * HANDLE;
  typedef const char * LPCSTR;

  #define CONST const

  #ifndef NULL
    #define NULL 0
  #endif

// MAX()
#ifndef MAX
  #define MAX(a,b)        (((a) > (b)) ? (a) : (b))
#endif

// MIN()
#ifndef MIN
  #define MIN(a,b)        (((a) < (b)) ? (a) : (b))
#endif

#endif // _MY_TYPE_DEFS_H_
