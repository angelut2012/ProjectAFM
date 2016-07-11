

/* this ALWAYS GENERATED file contains the IIDs and CLSIDs */

/* link this file in with the server and any clients */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Jul 10 23:39:40 2016
 */
/* Compiler settings for F:\google_drive\project_STM32\VS2012\AFM_GUI_solution_integration\AFM_Project\MKernel\src\MKernel_idl.idl:
    Oicf, W1, Zp8, env=Win64 (32b run), target_arch=IA64 8.00.0595 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


#ifdef __cplusplus
extern "C"{
#endif 


#include <rpc.h>
#include <rpcndr.h>

#ifdef _MIDL_USE_GUIDDEF_

#ifndef INITGUID
#define INITGUID
#include <guiddef.h>
#undef INITGUID
#else
#include <guiddef.h>
#endif

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8)

#else // !_MIDL_USE_GUIDDEF_

#ifndef __IID_DEFINED__
#define __IID_DEFINED__

typedef struct _IID
{
    unsigned long x;
    unsigned short s1;
    unsigned short s2;
    unsigned char  c[8];
} IID;

#endif // __IID_DEFINED__

#ifndef CLSID_DEFINED
#define CLSID_DEFINED
typedef IID CLSID;
#endif // CLSID_DEFINED

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        const type name = {l,w1,w2,{b1,b2,b3,b4,b5,b6,b7,b8}}

#endif !_MIDL_USE_GUIDDEF_

MIDL_DEFINE_GUID(IID, IID_IKernel,0x4ACCBAD1,0xEC3C,0x4E9E,0x90,0xC6,0xB7,0x83,0x52,0xA5,0xEF,0xAB);


MIDL_DEFINE_GUID(IID, LIBID_MKernel,0x3E411F7C,0xF5DD,0x48CC,0x87,0xA0,0x97,0x12,0xCB,0x3C,0x01,0x9B);


MIDL_DEFINE_GUID(CLSID, CLSID_Kernel,0xA910B6A2,0x579B,0x40E4,0xAB,0xF2,0x77,0x26,0x03,0xF4,0xE6,0x03);

#undef MIDL_DEFINE_GUID

#ifdef __cplusplus
}
#endif



