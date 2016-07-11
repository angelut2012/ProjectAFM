

/* this ALWAYS GENERATED file contains the proxy stub code */


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

#if defined(_M_IA64) 


#pragma warning( disable: 4049 )  /* more than 64k source lines */
#if _MSC_VER >= 1200
#pragma warning(push)
#endif

#pragma warning( disable: 4211 )  /* redefine extern to static */
#pragma warning( disable: 4232 )  /* dllimport identity*/
#pragma warning( disable: 4024 )  /* array to pointer mapping*/
#pragma warning( disable: 4152 )  /* function/data pointer conversion in expression */

#define USE_STUBLESS_PROXY


/* verify that the <rpcproxy.h> version is high enough to compile this file*/
#ifndef __REDQ_RPCPROXY_H_VERSION__
#define __REQUIRED_RPCPROXY_H_VERSION__ 475
#endif


#include "rpcproxy.h"
#ifndef __RPCPROXY_H_VERSION__
#error this stub requires an updated version of <rpcproxy.h>
#endif /* __RPCPROXY_H_VERSION__ */


#include "MKernel_idl.h"

#define TYPE_FORMAT_STRING_SIZE   1025                              
#define PROC_FORMAT_STRING_SIZE   389                               
#define EXPR_FORMAT_STRING_SIZE   1                                 
#define TRANSMIT_AS_TABLE_SIZE    0            
#define WIRE_MARSHAL_TABLE_SIZE   1            

typedef struct _MKernel_idl_MIDL_TYPE_FORMAT_STRING
    {
    short          Pad;
    unsigned char  Format[ TYPE_FORMAT_STRING_SIZE ];
    } MKernel_idl_MIDL_TYPE_FORMAT_STRING;

typedef struct _MKernel_idl_MIDL_PROC_FORMAT_STRING
    {
    short          Pad;
    unsigned char  Format[ PROC_FORMAT_STRING_SIZE ];
    } MKernel_idl_MIDL_PROC_FORMAT_STRING;

typedef struct _MKernel_idl_MIDL_EXPR_FORMAT_STRING
    {
    long          Pad;
    unsigned char  Format[ EXPR_FORMAT_STRING_SIZE ];
    } MKernel_idl_MIDL_EXPR_FORMAT_STRING;


static const RPC_SYNTAX_IDENTIFIER  _RpcTransferSyntax = 
{{0x8A885D04,0x1CEB,0x11C9,{0x9F,0xE8,0x08,0x00,0x2B,0x10,0x48,0x60}},{2,0}};


extern const MKernel_idl_MIDL_TYPE_FORMAT_STRING MKernel_idl__MIDL_TypeFormatString;
extern const MKernel_idl_MIDL_PROC_FORMAT_STRING MKernel_idl__MIDL_ProcFormatString;
extern const MKernel_idl_MIDL_EXPR_FORMAT_STRING MKernel_idl__MIDL_ExprFormatString;


extern const MIDL_STUB_DESC Object_StubDesc;


extern const MIDL_SERVER_INFO IKernel_ServerInfo;
extern const MIDL_STUBLESS_PROXY_INFO IKernel_ProxyInfo;


extern const USER_MARSHAL_ROUTINE_QUADRUPLE UserMarshalRoutines[ WIRE_MARSHAL_TABLE_SIZE ];

#if !defined(__RPC_WIN64__)
#error  Invalid build platform for this stub.
#endif

static const MKernel_idl_MIDL_PROC_FORMAT_STRING MKernel_idl__MIDL_ProcFormatString =
    {
        0,
        {

	/* Procedure get_MWFlags */

			0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/*  2 */	NdrFcLong( 0x0 ),	/* 0 */
/*  6 */	NdrFcShort( 0x7 ),	/* 7 */
/*  8 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 10 */	NdrFcShort( 0x0 ),	/* 0 */
/* 12 */	NdrFcShort( 0x8 ),	/* 8 */
/* 14 */	0x45,		/* Oi2 Flags:  srv must size, has return, has ext, */
			0x2,		/* 2 */
/* 16 */	0xa,		/* 10 */
			0x1,		/* Ext Flags:  new corr desc, */
/* 18 */	NdrFcShort( 0x0 ),	/* 0 */
/* 20 */	NdrFcShort( 0x0 ),	/* 0 */
/* 22 */	NdrFcShort( 0x0 ),	/* 0 */
/* 24 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter ppvFlags */

/* 26 */	NdrFcShort( 0x13 ),	/* Flags:  must size, must free, out, */
/* 28 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 30 */	NdrFcShort( 0x2 ),	/* Type Offset=2 */

	/* Return value */

/* 32 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 34 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 36 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure put_MWFlags */

/* 38 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 40 */	NdrFcLong( 0x0 ),	/* 0 */
/* 44 */	NdrFcShort( 0x8 ),	/* 8 */
/* 46 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 48 */	NdrFcShort( 0x0 ),	/* 0 */
/* 50 */	NdrFcShort( 0x8 ),	/* 8 */
/* 52 */	0x46,		/* Oi2 Flags:  clt must size, has return, has ext, */
			0x2,		/* 2 */
/* 54 */	0xa,		/* 10 */
			0x1,		/* Ext Flags:  new corr desc, */
/* 56 */	NdrFcShort( 0x0 ),	/* 0 */
/* 58 */	NdrFcShort( 0x0 ),	/* 0 */
/* 60 */	NdrFcShort( 0x0 ),	/* 0 */
/* 62 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter pvFlags */

/* 64 */	NdrFcShort( 0xb ),	/* Flags:  must size, must free, in, */
/* 66 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 68 */	NdrFcShort( 0x6 ),	/* Type Offset=6 */

	/* Return value */

/* 70 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 72 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 74 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure AFM_convert_height2RGB */

/* 76 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 78 */	NdrFcLong( 0x0 ),	/* 0 */
/* 82 */	NdrFcShort( 0x9 ),	/* 9 */
/* 84 */	NdrFcShort( 0x60 ),	/* ia64 Stack size/offset = 96 */
/* 86 */	NdrFcShort( 0x8 ),	/* 8 */
/* 88 */	NdrFcShort( 0x8 ),	/* 8 */
/* 90 */	0x47,		/* Oi2 Flags:  srv must size, clt must size, has return, has ext, */
			0x7,		/* 7 */
/* 92 */	0xa,		/* 10 */
			0x7,		/* Ext Flags:  new corr desc, clt corr check, srv corr check, */
/* 94 */	NdrFcShort( 0x1 ),	/* 1 */
/* 96 */	NdrFcShort( 0x1 ),	/* 1 */
/* 98 */	NdrFcShort( 0x0 ),	/* 0 */
/* 100 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter nargout */

/* 102 */	NdrFcShort( 0x48 ),	/* Flags:  in, base type, */
/* 104 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 106 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Parameter r */

/* 108 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 110 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 112 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter g */

/* 114 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 116 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 118 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter b */

/* 120 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 122 */	NdrFcShort( 0x20 ),	/* ia64 Stack size/offset = 32 */
/* 124 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter im_height */

/* 126 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 128 */	NdrFcShort( 0x28 ),	/* ia64 Stack size/offset = 40 */
/* 130 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Parameter parameter */

/* 132 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 134 */	NdrFcShort( 0x40 ),	/* ia64 Stack size/offset = 64 */
/* 136 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Return value */

/* 138 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 140 */	NdrFcShort( 0x58 ),	/* ia64 Stack size/offset = 88 */
/* 142 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure AFM_line_for_show */

/* 144 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 146 */	NdrFcLong( 0x0 ),	/* 0 */
/* 150 */	NdrFcShort( 0xa ),	/* 10 */
/* 152 */	NdrFcShort( 0x68 ),	/* ia64 Stack size/offset = 104 */
/* 154 */	NdrFcShort( 0x8 ),	/* 8 */
/* 156 */	NdrFcShort( 0x8 ),	/* 8 */
/* 158 */	0x47,		/* Oi2 Flags:  srv must size, clt must size, has return, has ext, */
			0x6,		/* 6 */
/* 160 */	0xa,		/* 10 */
			0x7,		/* Ext Flags:  new corr desc, clt corr check, srv corr check, */
/* 162 */	NdrFcShort( 0x1 ),	/* 1 */
/* 164 */	NdrFcShort( 0x1 ),	/* 1 */
/* 166 */	NdrFcShort( 0x0 ),	/* 0 */
/* 168 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter nargout */

/* 170 */	NdrFcShort( 0x48 ),	/* Flags:  in, base type, */
/* 172 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 174 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Parameter line_show */

/* 176 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 178 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 180 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter line_in */

/* 182 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 184 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 186 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Parameter fit_order */

/* 188 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 190 */	NdrFcShort( 0x30 ),	/* ia64 Stack size/offset = 48 */
/* 192 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Parameter index_base_point */

/* 194 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 196 */	NdrFcShort( 0x48 ),	/* ia64 Stack size/offset = 72 */
/* 198 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Return value */

/* 200 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 202 */	NdrFcShort( 0x60 ),	/* ia64 Stack size/offset = 96 */
/* 204 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure AFM_scan_set_ROI */

/* 206 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 208 */	NdrFcLong( 0x0 ),	/* 0 */
/* 212 */	NdrFcShort( 0xb ),	/* 11 */
/* 214 */	NdrFcShort( 0x50 ),	/* ia64 Stack size/offset = 80 */
/* 216 */	NdrFcShort( 0x8 ),	/* 8 */
/* 218 */	NdrFcShort( 0x8 ),	/* 8 */
/* 220 */	0x47,		/* Oi2 Flags:  srv must size, clt must size, has return, has ext, */
			0x5,		/* 5 */
/* 222 */	0xa,		/* 10 */
			0x7,		/* Ext Flags:  new corr desc, clt corr check, srv corr check, */
/* 224 */	NdrFcShort( 0x1 ),	/* 1 */
/* 226 */	NdrFcShort( 0x1 ),	/* 1 */
/* 228 */	NdrFcShort( 0x0 ),	/* 0 */
/* 230 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter nargout */

/* 232 */	NdrFcShort( 0x48 ),	/* Flags:  in, base type, */
/* 234 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 236 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Parameter position */

/* 238 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 240 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 242 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter im_buffer */

/* 244 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 246 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 248 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Parameter position_in1 */

/* 250 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 252 */	NdrFcShort( 0x30 ),	/* ia64 Stack size/offset = 48 */
/* 254 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Return value */

/* 256 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 258 */	NdrFcShort( 0x48 ),	/* ia64 Stack size/offset = 72 */
/* 260 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure AFM_show_indent_data */

/* 262 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 264 */	NdrFcLong( 0x0 ),	/* 0 */
/* 268 */	NdrFcShort( 0xc ),	/* 12 */
/* 270 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 272 */	NdrFcShort( 0x0 ),	/* 0 */
/* 274 */	NdrFcShort( 0x8 ),	/* 8 */
/* 276 */	0x44,		/* Oi2 Flags:  has return, has ext, */
			0x1,		/* 1 */
/* 278 */	0xa,		/* 10 */
			0x1,		/* Ext Flags:  new corr desc, */
/* 280 */	NdrFcShort( 0x0 ),	/* 0 */
/* 282 */	NdrFcShort( 0x0 ),	/* 0 */
/* 284 */	NdrFcShort( 0x0 ),	/* 0 */
/* 286 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Return value */

/* 288 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 290 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 292 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure StringEval */

/* 294 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 296 */	NdrFcLong( 0x0 ),	/* 0 */
/* 300 */	NdrFcShort( 0xd ),	/* 13 */
/* 302 */	NdrFcShort( 0x58 ),	/* ia64 Stack size/offset = 88 */
/* 304 */	NdrFcShort( 0x8 ),	/* 8 */
/* 306 */	NdrFcShort( 0x8 ),	/* 8 */
/* 308 */	0x47,		/* Oi2 Flags:  srv must size, clt must size, has return, has ext, */
			0x6,		/* 6 */
/* 310 */	0xa,		/* 10 */
			0x7,		/* Ext Flags:  new corr desc, clt corr check, srv corr check, */
/* 312 */	NdrFcShort( 0x1 ),	/* 1 */
/* 314 */	NdrFcShort( 0x1 ),	/* 1 */
/* 316 */	NdrFcShort( 0x0 ),	/* 0 */
/* 318 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Parameter nargout */

/* 320 */	NdrFcShort( 0x48 ),	/* Flags:  in, base type, */
/* 322 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 324 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Parameter out_str */

/* 326 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 328 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 330 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter out_data */

/* 332 */	NdrFcShort( 0x11b ),	/* Flags:  must size, must free, in, out, simple ref, */
/* 334 */	NdrFcShort( 0x18 ),	/* ia64 Stack size/offset = 24 */
/* 336 */	NdrFcShort( 0x3e8 ),	/* Type Offset=1000 */

	/* Parameter in_str */

/* 338 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 340 */	NdrFcShort( 0x20 ),	/* ia64 Stack size/offset = 32 */
/* 342 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Parameter in_data */

/* 344 */	NdrFcShort( 0x8b ),	/* Flags:  must size, must free, in, by val, */
/* 346 */	NdrFcShort( 0x38 ),	/* ia64 Stack size/offset = 56 */
/* 348 */	NdrFcShort( 0x3f6 ),	/* Type Offset=1014 */

	/* Return value */

/* 350 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 352 */	NdrFcShort( 0x50 ),	/* ia64 Stack size/offset = 80 */
/* 354 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

	/* Procedure MCRWaitForFigures */

/* 356 */	0x33,		/* FC_AUTO_HANDLE */
			0x6c,		/* Old Flags:  object, Oi2 */
/* 358 */	NdrFcLong( 0x0 ),	/* 0 */
/* 362 */	NdrFcShort( 0xe ),	/* 14 */
/* 364 */	NdrFcShort( 0x10 ),	/* ia64 Stack size/offset = 16 */
/* 366 */	NdrFcShort( 0x0 ),	/* 0 */
/* 368 */	NdrFcShort( 0x8 ),	/* 8 */
/* 370 */	0x44,		/* Oi2 Flags:  has return, has ext, */
			0x1,		/* 1 */
/* 372 */	0xa,		/* 10 */
			0x1,		/* Ext Flags:  new corr desc, */
/* 374 */	NdrFcShort( 0x0 ),	/* 0 */
/* 376 */	NdrFcShort( 0x0 ),	/* 0 */
/* 378 */	NdrFcShort( 0x0 ),	/* 0 */
/* 380 */	NdrFcShort( 0x0 ),	/* 0 */

	/* Return value */

/* 382 */	NdrFcShort( 0x70 ),	/* Flags:  out, return, base type, */
/* 384 */	NdrFcShort( 0x8 ),	/* ia64 Stack size/offset = 8 */
/* 386 */	0x8,		/* FC_LONG */
			0x0,		/* 0 */

			0x0
        }
    };

static const MKernel_idl_MIDL_TYPE_FORMAT_STRING MKernel_idl__MIDL_TypeFormatString =
    {
        0,
        {
			NdrFcShort( 0x0 ),	/* 0 */
/*  2 */	
			0x11, 0x10,	/* FC_RP [pointer_deref] */
/*  4 */	NdrFcShort( 0x2 ),	/* Offset= 2 (6) */
/*  6 */	
			0x2f,		/* FC_IP */
			0x5a,		/* FC_CONSTANT_IID */
/*  8 */	NdrFcLong( 0xa295776 ),	/* 170481526 */
/* 12 */	NdrFcShort( 0x23a1 ),	/* 9121 */
/* 14 */	NdrFcShort( 0x410a ),	/* 16650 */
/* 16 */	0x94,		/* 148 */
			0xbd,		/* 189 */
/* 18 */	0xc,		/* 12 */
			0x6c,		/* 108 */
/* 20 */	0x61,		/* 97 */
			0xb8,		/* 184 */
/* 22 */	0x91,		/* 145 */
			0xb7,		/* 183 */
/* 24 */	
			0x11, 0x0,	/* FC_RP */
/* 26 */	NdrFcShort( 0x3ce ),	/* Offset= 974 (1000) */
/* 28 */	
			0x13, 0x0,	/* FC_OP */
/* 30 */	NdrFcShort( 0x3b6 ),	/* Offset= 950 (980) */
/* 32 */	
			0x2b,		/* FC_NON_ENCAPSULATED_UNION */
			0x9,		/* FC_ULONG */
/* 34 */	0x7,		/* Corr desc: FC_USHORT */
			0x0,		/*  */
/* 36 */	NdrFcShort( 0xfff8 ),	/* -8 */
/* 38 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 40 */	NdrFcShort( 0x2 ),	/* Offset= 2 (42) */
/* 42 */	NdrFcShort( 0x10 ),	/* 16 */
/* 44 */	NdrFcShort( 0x2f ),	/* 47 */
/* 46 */	NdrFcLong( 0x14 ),	/* 20 */
/* 50 */	NdrFcShort( 0x800b ),	/* Simple arm type: FC_HYPER */
/* 52 */	NdrFcLong( 0x3 ),	/* 3 */
/* 56 */	NdrFcShort( 0x8008 ),	/* Simple arm type: FC_LONG */
/* 58 */	NdrFcLong( 0x11 ),	/* 17 */
/* 62 */	NdrFcShort( 0x8001 ),	/* Simple arm type: FC_BYTE */
/* 64 */	NdrFcLong( 0x2 ),	/* 2 */
/* 68 */	NdrFcShort( 0x8006 ),	/* Simple arm type: FC_SHORT */
/* 70 */	NdrFcLong( 0x4 ),	/* 4 */
/* 74 */	NdrFcShort( 0x800a ),	/* Simple arm type: FC_FLOAT */
/* 76 */	NdrFcLong( 0x5 ),	/* 5 */
/* 80 */	NdrFcShort( 0x800c ),	/* Simple arm type: FC_DOUBLE */
/* 82 */	NdrFcLong( 0xb ),	/* 11 */
/* 86 */	NdrFcShort( 0x8006 ),	/* Simple arm type: FC_SHORT */
/* 88 */	NdrFcLong( 0xa ),	/* 10 */
/* 92 */	NdrFcShort( 0x8008 ),	/* Simple arm type: FC_LONG */
/* 94 */	NdrFcLong( 0x6 ),	/* 6 */
/* 98 */	NdrFcShort( 0xe8 ),	/* Offset= 232 (330) */
/* 100 */	NdrFcLong( 0x7 ),	/* 7 */
/* 104 */	NdrFcShort( 0x800c ),	/* Simple arm type: FC_DOUBLE */
/* 106 */	NdrFcLong( 0x8 ),	/* 8 */
/* 110 */	NdrFcShort( 0xe2 ),	/* Offset= 226 (336) */
/* 112 */	NdrFcLong( 0xd ),	/* 13 */
/* 116 */	NdrFcShort( 0xf6 ),	/* Offset= 246 (362) */
/* 118 */	NdrFcLong( 0x9 ),	/* 9 */
/* 122 */	NdrFcShort( 0x102 ),	/* Offset= 258 (380) */
/* 124 */	NdrFcLong( 0x2000 ),	/* 8192 */
/* 128 */	NdrFcShort( 0x10e ),	/* Offset= 270 (398) */
/* 130 */	NdrFcLong( 0x24 ),	/* 36 */
/* 134 */	NdrFcShort( 0x304 ),	/* Offset= 772 (906) */
/* 136 */	NdrFcLong( 0x4024 ),	/* 16420 */
/* 140 */	NdrFcShort( 0x2fe ),	/* Offset= 766 (906) */
/* 142 */	NdrFcLong( 0x4011 ),	/* 16401 */
/* 146 */	NdrFcShort( 0x2fc ),	/* Offset= 764 (910) */
/* 148 */	NdrFcLong( 0x4002 ),	/* 16386 */
/* 152 */	NdrFcShort( 0x2fa ),	/* Offset= 762 (914) */
/* 154 */	NdrFcLong( 0x4003 ),	/* 16387 */
/* 158 */	NdrFcShort( 0x2f8 ),	/* Offset= 760 (918) */
/* 160 */	NdrFcLong( 0x4014 ),	/* 16404 */
/* 164 */	NdrFcShort( 0x2f6 ),	/* Offset= 758 (922) */
/* 166 */	NdrFcLong( 0x4004 ),	/* 16388 */
/* 170 */	NdrFcShort( 0x2f4 ),	/* Offset= 756 (926) */
/* 172 */	NdrFcLong( 0x4005 ),	/* 16389 */
/* 176 */	NdrFcShort( 0x2f2 ),	/* Offset= 754 (930) */
/* 178 */	NdrFcLong( 0x400b ),	/* 16395 */
/* 182 */	NdrFcShort( 0x2dc ),	/* Offset= 732 (914) */
/* 184 */	NdrFcLong( 0x400a ),	/* 16394 */
/* 188 */	NdrFcShort( 0x2da ),	/* Offset= 730 (918) */
/* 190 */	NdrFcLong( 0x4006 ),	/* 16390 */
/* 194 */	NdrFcShort( 0x2e4 ),	/* Offset= 740 (934) */
/* 196 */	NdrFcLong( 0x4007 ),	/* 16391 */
/* 200 */	NdrFcShort( 0x2da ),	/* Offset= 730 (930) */
/* 202 */	NdrFcLong( 0x4008 ),	/* 16392 */
/* 206 */	NdrFcShort( 0x2dc ),	/* Offset= 732 (938) */
/* 208 */	NdrFcLong( 0x400d ),	/* 16397 */
/* 212 */	NdrFcShort( 0x2da ),	/* Offset= 730 (942) */
/* 214 */	NdrFcLong( 0x4009 ),	/* 16393 */
/* 218 */	NdrFcShort( 0x2d8 ),	/* Offset= 728 (946) */
/* 220 */	NdrFcLong( 0x6000 ),	/* 24576 */
/* 224 */	NdrFcShort( 0x2d6 ),	/* Offset= 726 (950) */
/* 226 */	NdrFcLong( 0x400c ),	/* 16396 */
/* 230 */	NdrFcShort( 0x2d4 ),	/* Offset= 724 (954) */
/* 232 */	NdrFcLong( 0x10 ),	/* 16 */
/* 236 */	NdrFcShort( 0x8002 ),	/* Simple arm type: FC_CHAR */
/* 238 */	NdrFcLong( 0x12 ),	/* 18 */
/* 242 */	NdrFcShort( 0x8006 ),	/* Simple arm type: FC_SHORT */
/* 244 */	NdrFcLong( 0x13 ),	/* 19 */
/* 248 */	NdrFcShort( 0x8008 ),	/* Simple arm type: FC_LONG */
/* 250 */	NdrFcLong( 0x15 ),	/* 21 */
/* 254 */	NdrFcShort( 0x800b ),	/* Simple arm type: FC_HYPER */
/* 256 */	NdrFcLong( 0x16 ),	/* 22 */
/* 260 */	NdrFcShort( 0x8008 ),	/* Simple arm type: FC_LONG */
/* 262 */	NdrFcLong( 0x17 ),	/* 23 */
/* 266 */	NdrFcShort( 0x8008 ),	/* Simple arm type: FC_LONG */
/* 268 */	NdrFcLong( 0xe ),	/* 14 */
/* 272 */	NdrFcShort( 0x2b2 ),	/* Offset= 690 (962) */
/* 274 */	NdrFcLong( 0x400e ),	/* 16398 */
/* 278 */	NdrFcShort( 0x2b6 ),	/* Offset= 694 (972) */
/* 280 */	NdrFcLong( 0x4010 ),	/* 16400 */
/* 284 */	NdrFcShort( 0x2b4 ),	/* Offset= 692 (976) */
/* 286 */	NdrFcLong( 0x4012 ),	/* 16402 */
/* 290 */	NdrFcShort( 0x270 ),	/* Offset= 624 (914) */
/* 292 */	NdrFcLong( 0x4013 ),	/* 16403 */
/* 296 */	NdrFcShort( 0x26e ),	/* Offset= 622 (918) */
/* 298 */	NdrFcLong( 0x4015 ),	/* 16405 */
/* 302 */	NdrFcShort( 0x26c ),	/* Offset= 620 (922) */
/* 304 */	NdrFcLong( 0x4016 ),	/* 16406 */
/* 308 */	NdrFcShort( 0x262 ),	/* Offset= 610 (918) */
/* 310 */	NdrFcLong( 0x4017 ),	/* 16407 */
/* 314 */	NdrFcShort( 0x25c ),	/* Offset= 604 (918) */
/* 316 */	NdrFcLong( 0x0 ),	/* 0 */
/* 320 */	NdrFcShort( 0x0 ),	/* Offset= 0 (320) */
/* 322 */	NdrFcLong( 0x1 ),	/* 1 */
/* 326 */	NdrFcShort( 0x0 ),	/* Offset= 0 (326) */
/* 328 */	NdrFcShort( 0xffff ),	/* Offset= -1 (327) */
/* 330 */	
			0x15,		/* FC_STRUCT */
			0x7,		/* 7 */
/* 332 */	NdrFcShort( 0x8 ),	/* 8 */
/* 334 */	0xb,		/* FC_HYPER */
			0x5b,		/* FC_END */
/* 336 */	
			0x13, 0x0,	/* FC_OP */
/* 338 */	NdrFcShort( 0xe ),	/* Offset= 14 (352) */
/* 340 */	
			0x1b,		/* FC_CARRAY */
			0x1,		/* 1 */
/* 342 */	NdrFcShort( 0x2 ),	/* 2 */
/* 344 */	0x9,		/* Corr desc: FC_ULONG */
			0x0,		/*  */
/* 346 */	NdrFcShort( 0xfffc ),	/* -4 */
/* 348 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 350 */	0x6,		/* FC_SHORT */
			0x5b,		/* FC_END */
/* 352 */	
			0x17,		/* FC_CSTRUCT */
			0x3,		/* 3 */
/* 354 */	NdrFcShort( 0x8 ),	/* 8 */
/* 356 */	NdrFcShort( 0xfff0 ),	/* Offset= -16 (340) */
/* 358 */	0x8,		/* FC_LONG */
			0x8,		/* FC_LONG */
/* 360 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 362 */	
			0x2f,		/* FC_IP */
			0x5a,		/* FC_CONSTANT_IID */
/* 364 */	NdrFcLong( 0x0 ),	/* 0 */
/* 368 */	NdrFcShort( 0x0 ),	/* 0 */
/* 370 */	NdrFcShort( 0x0 ),	/* 0 */
/* 372 */	0xc0,		/* 192 */
			0x0,		/* 0 */
/* 374 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 376 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 378 */	0x0,		/* 0 */
			0x46,		/* 70 */
/* 380 */	
			0x2f,		/* FC_IP */
			0x5a,		/* FC_CONSTANT_IID */
/* 382 */	NdrFcLong( 0x20400 ),	/* 132096 */
/* 386 */	NdrFcShort( 0x0 ),	/* 0 */
/* 388 */	NdrFcShort( 0x0 ),	/* 0 */
/* 390 */	0xc0,		/* 192 */
			0x0,		/* 0 */
/* 392 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 394 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 396 */	0x0,		/* 0 */
			0x46,		/* 70 */
/* 398 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 400 */	NdrFcShort( 0x2 ),	/* Offset= 2 (402) */
/* 402 */	
			0x13, 0x0,	/* FC_OP */
/* 404 */	NdrFcShort( 0x1e4 ),	/* Offset= 484 (888) */
/* 406 */	
			0x2a,		/* FC_ENCAPSULATED_UNION */
			0x89,		/* 137 */
/* 408 */	NdrFcShort( 0x20 ),	/* 32 */
/* 410 */	NdrFcShort( 0xa ),	/* 10 */
/* 412 */	NdrFcLong( 0x8 ),	/* 8 */
/* 416 */	NdrFcShort( 0x50 ),	/* Offset= 80 (496) */
/* 418 */	NdrFcLong( 0xd ),	/* 13 */
/* 422 */	NdrFcShort( 0x70 ),	/* Offset= 112 (534) */
/* 424 */	NdrFcLong( 0x9 ),	/* 9 */
/* 428 */	NdrFcShort( 0x90 ),	/* Offset= 144 (572) */
/* 430 */	NdrFcLong( 0xc ),	/* 12 */
/* 434 */	NdrFcShort( 0xb0 ),	/* Offset= 176 (610) */
/* 436 */	NdrFcLong( 0x24 ),	/* 36 */
/* 440 */	NdrFcShort( 0x102 ),	/* Offset= 258 (698) */
/* 442 */	NdrFcLong( 0x800d ),	/* 32781 */
/* 446 */	NdrFcShort( 0x11e ),	/* Offset= 286 (732) */
/* 448 */	NdrFcLong( 0x10 ),	/* 16 */
/* 452 */	NdrFcShort( 0x138 ),	/* Offset= 312 (764) */
/* 454 */	NdrFcLong( 0x2 ),	/* 2 */
/* 458 */	NdrFcShort( 0x14e ),	/* Offset= 334 (792) */
/* 460 */	NdrFcLong( 0x3 ),	/* 3 */
/* 464 */	NdrFcShort( 0x164 ),	/* Offset= 356 (820) */
/* 466 */	NdrFcLong( 0x14 ),	/* 20 */
/* 470 */	NdrFcShort( 0x17a ),	/* Offset= 378 (848) */
/* 472 */	NdrFcShort( 0xffff ),	/* Offset= -1 (471) */
/* 474 */	
			0x21,		/* FC_BOGUS_ARRAY */
			0x3,		/* 3 */
/* 476 */	NdrFcShort( 0x0 ),	/* 0 */
/* 478 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 480 */	NdrFcShort( 0x0 ),	/* 0 */
/* 482 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 484 */	NdrFcLong( 0xffffffff ),	/* -1 */
/* 488 */	NdrFcShort( 0x0 ),	/* Corr flags:  */
/* 490 */	
			0x13, 0x0,	/* FC_OP */
/* 492 */	NdrFcShort( 0xff74 ),	/* Offset= -140 (352) */
/* 494 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 496 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 498 */	NdrFcShort( 0x10 ),	/* 16 */
/* 500 */	NdrFcShort( 0x0 ),	/* 0 */
/* 502 */	NdrFcShort( 0x6 ),	/* Offset= 6 (508) */
/* 504 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 506 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 508 */	
			0x11, 0x0,	/* FC_RP */
/* 510 */	NdrFcShort( 0xffdc ),	/* Offset= -36 (474) */
/* 512 */	
			0x21,		/* FC_BOGUS_ARRAY */
			0x3,		/* 3 */
/* 514 */	NdrFcShort( 0x0 ),	/* 0 */
/* 516 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 518 */	NdrFcShort( 0x0 ),	/* 0 */
/* 520 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 522 */	NdrFcLong( 0xffffffff ),	/* -1 */
/* 526 */	NdrFcShort( 0x0 ),	/* Corr flags:  */
/* 528 */	0x4c,		/* FC_EMBEDDED_COMPLEX */
			0x0,		/* 0 */
/* 530 */	NdrFcShort( 0xff58 ),	/* Offset= -168 (362) */
/* 532 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 534 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 536 */	NdrFcShort( 0x10 ),	/* 16 */
/* 538 */	NdrFcShort( 0x0 ),	/* 0 */
/* 540 */	NdrFcShort( 0x6 ),	/* Offset= 6 (546) */
/* 542 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 544 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 546 */	
			0x11, 0x0,	/* FC_RP */
/* 548 */	NdrFcShort( 0xffdc ),	/* Offset= -36 (512) */
/* 550 */	
			0x21,		/* FC_BOGUS_ARRAY */
			0x3,		/* 3 */
/* 552 */	NdrFcShort( 0x0 ),	/* 0 */
/* 554 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 556 */	NdrFcShort( 0x0 ),	/* 0 */
/* 558 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 560 */	NdrFcLong( 0xffffffff ),	/* -1 */
/* 564 */	NdrFcShort( 0x0 ),	/* Corr flags:  */
/* 566 */	0x4c,		/* FC_EMBEDDED_COMPLEX */
			0x0,		/* 0 */
/* 568 */	NdrFcShort( 0xff44 ),	/* Offset= -188 (380) */
/* 570 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 572 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 574 */	NdrFcShort( 0x10 ),	/* 16 */
/* 576 */	NdrFcShort( 0x0 ),	/* 0 */
/* 578 */	NdrFcShort( 0x6 ),	/* Offset= 6 (584) */
/* 580 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 582 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 584 */	
			0x11, 0x0,	/* FC_RP */
/* 586 */	NdrFcShort( 0xffdc ),	/* Offset= -36 (550) */
/* 588 */	
			0x21,		/* FC_BOGUS_ARRAY */
			0x3,		/* 3 */
/* 590 */	NdrFcShort( 0x0 ),	/* 0 */
/* 592 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 594 */	NdrFcShort( 0x0 ),	/* 0 */
/* 596 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 598 */	NdrFcLong( 0xffffffff ),	/* -1 */
/* 602 */	NdrFcShort( 0x0 ),	/* Corr flags:  */
/* 604 */	
			0x13, 0x0,	/* FC_OP */
/* 606 */	NdrFcShort( 0x176 ),	/* Offset= 374 (980) */
/* 608 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 610 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 612 */	NdrFcShort( 0x10 ),	/* 16 */
/* 614 */	NdrFcShort( 0x0 ),	/* 0 */
/* 616 */	NdrFcShort( 0x6 ),	/* Offset= 6 (622) */
/* 618 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 620 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 622 */	
			0x11, 0x0,	/* FC_RP */
/* 624 */	NdrFcShort( 0xffdc ),	/* Offset= -36 (588) */
/* 626 */	
			0x2f,		/* FC_IP */
			0x5a,		/* FC_CONSTANT_IID */
/* 628 */	NdrFcLong( 0x2f ),	/* 47 */
/* 632 */	NdrFcShort( 0x0 ),	/* 0 */
/* 634 */	NdrFcShort( 0x0 ),	/* 0 */
/* 636 */	0xc0,		/* 192 */
			0x0,		/* 0 */
/* 638 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 640 */	0x0,		/* 0 */
			0x0,		/* 0 */
/* 642 */	0x0,		/* 0 */
			0x46,		/* 70 */
/* 644 */	
			0x1b,		/* FC_CARRAY */
			0x0,		/* 0 */
/* 646 */	NdrFcShort( 0x1 ),	/* 1 */
/* 648 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 650 */	NdrFcShort( 0x4 ),	/* 4 */
/* 652 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 654 */	0x1,		/* FC_BYTE */
			0x5b,		/* FC_END */
/* 656 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 658 */	NdrFcShort( 0x18 ),	/* 24 */
/* 660 */	NdrFcShort( 0x0 ),	/* 0 */
/* 662 */	NdrFcShort( 0xa ),	/* Offset= 10 (672) */
/* 664 */	0x8,		/* FC_LONG */
			0x8,		/* FC_LONG */
/* 666 */	0x4c,		/* FC_EMBEDDED_COMPLEX */
			0x0,		/* 0 */
/* 668 */	NdrFcShort( 0xffd6 ),	/* Offset= -42 (626) */
/* 670 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 672 */	
			0x13, 0x0,	/* FC_OP */
/* 674 */	NdrFcShort( 0xffe2 ),	/* Offset= -30 (644) */
/* 676 */	
			0x21,		/* FC_BOGUS_ARRAY */
			0x3,		/* 3 */
/* 678 */	NdrFcShort( 0x0 ),	/* 0 */
/* 680 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 682 */	NdrFcShort( 0x0 ),	/* 0 */
/* 684 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 686 */	NdrFcLong( 0xffffffff ),	/* -1 */
/* 690 */	NdrFcShort( 0x0 ),	/* Corr flags:  */
/* 692 */	
			0x13, 0x0,	/* FC_OP */
/* 694 */	NdrFcShort( 0xffda ),	/* Offset= -38 (656) */
/* 696 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 698 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 700 */	NdrFcShort( 0x10 ),	/* 16 */
/* 702 */	NdrFcShort( 0x0 ),	/* 0 */
/* 704 */	NdrFcShort( 0x6 ),	/* Offset= 6 (710) */
/* 706 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 708 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 710 */	
			0x11, 0x0,	/* FC_RP */
/* 712 */	NdrFcShort( 0xffdc ),	/* Offset= -36 (676) */
/* 714 */	
			0x1d,		/* FC_SMFARRAY */
			0x0,		/* 0 */
/* 716 */	NdrFcShort( 0x8 ),	/* 8 */
/* 718 */	0x1,		/* FC_BYTE */
			0x5b,		/* FC_END */
/* 720 */	
			0x15,		/* FC_STRUCT */
			0x3,		/* 3 */
/* 722 */	NdrFcShort( 0x10 ),	/* 16 */
/* 724 */	0x8,		/* FC_LONG */
			0x6,		/* FC_SHORT */
/* 726 */	0x6,		/* FC_SHORT */
			0x4c,		/* FC_EMBEDDED_COMPLEX */
/* 728 */	0x0,		/* 0 */
			NdrFcShort( 0xfff1 ),	/* Offset= -15 (714) */
			0x5b,		/* FC_END */
/* 732 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 734 */	NdrFcShort( 0x20 ),	/* 32 */
/* 736 */	NdrFcShort( 0x0 ),	/* 0 */
/* 738 */	NdrFcShort( 0xa ),	/* Offset= 10 (748) */
/* 740 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 742 */	0x36,		/* FC_POINTER */
			0x4c,		/* FC_EMBEDDED_COMPLEX */
/* 744 */	0x0,		/* 0 */
			NdrFcShort( 0xffe7 ),	/* Offset= -25 (720) */
			0x5b,		/* FC_END */
/* 748 */	
			0x11, 0x0,	/* FC_RP */
/* 750 */	NdrFcShort( 0xff12 ),	/* Offset= -238 (512) */
/* 752 */	
			0x1b,		/* FC_CARRAY */
			0x0,		/* 0 */
/* 754 */	NdrFcShort( 0x1 ),	/* 1 */
/* 756 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 758 */	NdrFcShort( 0x0 ),	/* 0 */
/* 760 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 762 */	0x1,		/* FC_BYTE */
			0x5b,		/* FC_END */
/* 764 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 766 */	NdrFcShort( 0x10 ),	/* 16 */
/* 768 */	NdrFcShort( 0x0 ),	/* 0 */
/* 770 */	NdrFcShort( 0x6 ),	/* Offset= 6 (776) */
/* 772 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 774 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 776 */	
			0x13, 0x0,	/* FC_OP */
/* 778 */	NdrFcShort( 0xffe6 ),	/* Offset= -26 (752) */
/* 780 */	
			0x1b,		/* FC_CARRAY */
			0x1,		/* 1 */
/* 782 */	NdrFcShort( 0x2 ),	/* 2 */
/* 784 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 786 */	NdrFcShort( 0x0 ),	/* 0 */
/* 788 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 790 */	0x6,		/* FC_SHORT */
			0x5b,		/* FC_END */
/* 792 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 794 */	NdrFcShort( 0x10 ),	/* 16 */
/* 796 */	NdrFcShort( 0x0 ),	/* 0 */
/* 798 */	NdrFcShort( 0x6 ),	/* Offset= 6 (804) */
/* 800 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 802 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 804 */	
			0x13, 0x0,	/* FC_OP */
/* 806 */	NdrFcShort( 0xffe6 ),	/* Offset= -26 (780) */
/* 808 */	
			0x1b,		/* FC_CARRAY */
			0x3,		/* 3 */
/* 810 */	NdrFcShort( 0x4 ),	/* 4 */
/* 812 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 814 */	NdrFcShort( 0x0 ),	/* 0 */
/* 816 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 818 */	0x8,		/* FC_LONG */
			0x5b,		/* FC_END */
/* 820 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 822 */	NdrFcShort( 0x10 ),	/* 16 */
/* 824 */	NdrFcShort( 0x0 ),	/* 0 */
/* 826 */	NdrFcShort( 0x6 ),	/* Offset= 6 (832) */
/* 828 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 830 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 832 */	
			0x13, 0x0,	/* FC_OP */
/* 834 */	NdrFcShort( 0xffe6 ),	/* Offset= -26 (808) */
/* 836 */	
			0x1b,		/* FC_CARRAY */
			0x7,		/* 7 */
/* 838 */	NdrFcShort( 0x8 ),	/* 8 */
/* 840 */	0x19,		/* Corr desc:  field pointer, FC_ULONG */
			0x0,		/*  */
/* 842 */	NdrFcShort( 0x0 ),	/* 0 */
/* 844 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 846 */	0xb,		/* FC_HYPER */
			0x5b,		/* FC_END */
/* 848 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 850 */	NdrFcShort( 0x10 ),	/* 16 */
/* 852 */	NdrFcShort( 0x0 ),	/* 0 */
/* 854 */	NdrFcShort( 0x6 ),	/* Offset= 6 (860) */
/* 856 */	0x8,		/* FC_LONG */
			0x40,		/* FC_STRUCTPAD4 */
/* 858 */	0x36,		/* FC_POINTER */
			0x5b,		/* FC_END */
/* 860 */	
			0x13, 0x0,	/* FC_OP */
/* 862 */	NdrFcShort( 0xffe6 ),	/* Offset= -26 (836) */
/* 864 */	
			0x15,		/* FC_STRUCT */
			0x3,		/* 3 */
/* 866 */	NdrFcShort( 0x8 ),	/* 8 */
/* 868 */	0x8,		/* FC_LONG */
			0x8,		/* FC_LONG */
/* 870 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 872 */	
			0x1b,		/* FC_CARRAY */
			0x3,		/* 3 */
/* 874 */	NdrFcShort( 0x8 ),	/* 8 */
/* 876 */	0x7,		/* Corr desc: FC_USHORT */
			0x0,		/*  */
/* 878 */	NdrFcShort( 0xffc8 ),	/* -56 */
/* 880 */	NdrFcShort( 0x1 ),	/* Corr flags:  early, */
/* 882 */	0x4c,		/* FC_EMBEDDED_COMPLEX */
			0x0,		/* 0 */
/* 884 */	NdrFcShort( 0xffec ),	/* Offset= -20 (864) */
/* 886 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 888 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x3,		/* 3 */
/* 890 */	NdrFcShort( 0x38 ),	/* 56 */
/* 892 */	NdrFcShort( 0xffec ),	/* Offset= -20 (872) */
/* 894 */	NdrFcShort( 0x0 ),	/* Offset= 0 (894) */
/* 896 */	0x6,		/* FC_SHORT */
			0x6,		/* FC_SHORT */
/* 898 */	0x8,		/* FC_LONG */
			0x8,		/* FC_LONG */
/* 900 */	0x40,		/* FC_STRUCTPAD4 */
			0x4c,		/* FC_EMBEDDED_COMPLEX */
/* 902 */	0x0,		/* 0 */
			NdrFcShort( 0xfe0f ),	/* Offset= -497 (406) */
			0x5b,		/* FC_END */
/* 906 */	
			0x13, 0x0,	/* FC_OP */
/* 908 */	NdrFcShort( 0xff04 ),	/* Offset= -252 (656) */
/* 910 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 912 */	0x1,		/* FC_BYTE */
			0x5c,		/* FC_PAD */
/* 914 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 916 */	0x6,		/* FC_SHORT */
			0x5c,		/* FC_PAD */
/* 918 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 920 */	0x8,		/* FC_LONG */
			0x5c,		/* FC_PAD */
/* 922 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 924 */	0xb,		/* FC_HYPER */
			0x5c,		/* FC_PAD */
/* 926 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 928 */	0xa,		/* FC_FLOAT */
			0x5c,		/* FC_PAD */
/* 930 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 932 */	0xc,		/* FC_DOUBLE */
			0x5c,		/* FC_PAD */
/* 934 */	
			0x13, 0x0,	/* FC_OP */
/* 936 */	NdrFcShort( 0xfda2 ),	/* Offset= -606 (330) */
/* 938 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 940 */	NdrFcShort( 0xfda4 ),	/* Offset= -604 (336) */
/* 942 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 944 */	NdrFcShort( 0xfdba ),	/* Offset= -582 (362) */
/* 946 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 948 */	NdrFcShort( 0xfdc8 ),	/* Offset= -568 (380) */
/* 950 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 952 */	NdrFcShort( 0xfdd6 ),	/* Offset= -554 (398) */
/* 954 */	
			0x13, 0x10,	/* FC_OP [pointer_deref] */
/* 956 */	NdrFcShort( 0x2 ),	/* Offset= 2 (958) */
/* 958 */	
			0x13, 0x0,	/* FC_OP */
/* 960 */	NdrFcShort( 0x14 ),	/* Offset= 20 (980) */
/* 962 */	
			0x15,		/* FC_STRUCT */
			0x7,		/* 7 */
/* 964 */	NdrFcShort( 0x10 ),	/* 16 */
/* 966 */	0x6,		/* FC_SHORT */
			0x1,		/* FC_BYTE */
/* 968 */	0x1,		/* FC_BYTE */
			0x8,		/* FC_LONG */
/* 970 */	0xb,		/* FC_HYPER */
			0x5b,		/* FC_END */
/* 972 */	
			0x13, 0x0,	/* FC_OP */
/* 974 */	NdrFcShort( 0xfff4 ),	/* Offset= -12 (962) */
/* 976 */	
			0x13, 0x8,	/* FC_OP [simple_pointer] */
/* 978 */	0x2,		/* FC_CHAR */
			0x5c,		/* FC_PAD */
/* 980 */	
			0x1a,		/* FC_BOGUS_STRUCT */
			0x7,		/* 7 */
/* 982 */	NdrFcShort( 0x20 ),	/* 32 */
/* 984 */	NdrFcShort( 0x0 ),	/* 0 */
/* 986 */	NdrFcShort( 0x0 ),	/* Offset= 0 (986) */
/* 988 */	0x8,		/* FC_LONG */
			0x8,		/* FC_LONG */
/* 990 */	0x6,		/* FC_SHORT */
			0x6,		/* FC_SHORT */
/* 992 */	0x6,		/* FC_SHORT */
			0x6,		/* FC_SHORT */
/* 994 */	0x4c,		/* FC_EMBEDDED_COMPLEX */
			0x0,		/* 0 */
/* 996 */	NdrFcShort( 0xfc3c ),	/* Offset= -964 (32) */
/* 998 */	0x5c,		/* FC_PAD */
			0x5b,		/* FC_END */
/* 1000 */	0xb4,		/* FC_USER_MARSHAL */
			0x83,		/* 131 */
/* 1002 */	NdrFcShort( 0x0 ),	/* 0 */
/* 1004 */	NdrFcShort( 0x18 ),	/* 24 */
/* 1006 */	NdrFcShort( 0x0 ),	/* 0 */
/* 1008 */	NdrFcShort( 0xfc2c ),	/* Offset= -980 (28) */
/* 1010 */	
			0x12, 0x0,	/* FC_UP */
/* 1012 */	NdrFcShort( 0xffe0 ),	/* Offset= -32 (980) */
/* 1014 */	0xb4,		/* FC_USER_MARSHAL */
			0x83,		/* 131 */
/* 1016 */	NdrFcShort( 0x0 ),	/* 0 */
/* 1018 */	NdrFcShort( 0x18 ),	/* 24 */
/* 1020 */	NdrFcShort( 0x0 ),	/* 0 */
/* 1022 */	NdrFcShort( 0xfff4 ),	/* Offset= -12 (1010) */

			0x0
        }
    };

static const USER_MARSHAL_ROUTINE_QUADRUPLE UserMarshalRoutines[ WIRE_MARSHAL_TABLE_SIZE ] = 
        {
            
            {
            VARIANT_UserSize
            ,VARIANT_UserMarshal
            ,VARIANT_UserUnmarshal
            ,VARIANT_UserFree
            }

        };



/* Object interface: IUnknown, ver. 0.0,
   GUID={0x00000000,0x0000,0x0000,{0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x46}} */


/* Object interface: IDispatch, ver. 0.0,
   GUID={0x00020400,0x0000,0x0000,{0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x46}} */


/* Object interface: IKernel, ver. 0.0,
   GUID={0x4ACCBAD1,0xEC3C,0x4E9E,{0x90,0xC6,0xB7,0x83,0x52,0xA5,0xEF,0xAB}} */

#pragma code_seg(".orpc")
static const unsigned short IKernel_FormatStringOffsetTable[] =
    {
    (unsigned short) -1,
    (unsigned short) -1,
    (unsigned short) -1,
    (unsigned short) -1,
    0,
    38,
    76,
    144,
    206,
    262,
    294,
    356
    };

static const MIDL_STUBLESS_PROXY_INFO IKernel_ProxyInfo =
    {
    &Object_StubDesc,
    MKernel_idl__MIDL_ProcFormatString.Format,
    &IKernel_FormatStringOffsetTable[-3],
    0,
    0,
    0
    };


static const MIDL_SERVER_INFO IKernel_ServerInfo = 
    {
    &Object_StubDesc,
    0,
    MKernel_idl__MIDL_ProcFormatString.Format,
    &IKernel_FormatStringOffsetTable[-3],
    0,
    0,
    0,
    0};
CINTERFACE_PROXY_VTABLE(15) _IKernelProxyVtbl = 
{
    &IKernel_ProxyInfo,
    &IID_IKernel,
    IUnknown_QueryInterface_Proxy,
    IUnknown_AddRef_Proxy,
    IUnknown_Release_Proxy ,
    0 /* IDispatch::GetTypeInfoCount */ ,
    0 /* IDispatch::GetTypeInfo */ ,
    0 /* IDispatch::GetIDsOfNames */ ,
    0 /* IDispatch_Invoke_Proxy */ ,
    (void *) (INT_PTR) -1 /* IKernel::get_MWFlags */ ,
    (void *) (INT_PTR) -1 /* IKernel::put_MWFlags */ ,
    (void *) (INT_PTR) -1 /* IKernel::AFM_convert_height2RGB */ ,
    (void *) (INT_PTR) -1 /* IKernel::AFM_line_for_show */ ,
    (void *) (INT_PTR) -1 /* IKernel::AFM_scan_set_ROI */ ,
    (void *) (INT_PTR) -1 /* IKernel::AFM_show_indent_data */ ,
    (void *) (INT_PTR) -1 /* IKernel::StringEval */ ,
    (void *) (INT_PTR) -1 /* IKernel::MCRWaitForFigures */
};


static const PRPC_STUB_FUNCTION IKernel_table[] =
{
    STUB_FORWARDING_FUNCTION,
    STUB_FORWARDING_FUNCTION,
    STUB_FORWARDING_FUNCTION,
    STUB_FORWARDING_FUNCTION,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2,
    NdrStubCall2
};

CInterfaceStubVtbl _IKernelStubVtbl =
{
    &IID_IKernel,
    &IKernel_ServerInfo,
    15,
    &IKernel_table[-3],
    CStdStubBuffer_DELEGATING_METHODS
};

static const MIDL_STUB_DESC Object_StubDesc = 
    {
    0,
    NdrOleAllocate,
    NdrOleFree,
    0,
    0,
    0,
    0,
    0,
    MKernel_idl__MIDL_TypeFormatString.Format,
    1, /* -error bounds_check flag */
    0x50002, /* Ndr library version */
    0,
    0x8000253, /* MIDL Version 8.0.595 */
    0,
    UserMarshalRoutines,
    0,  /* notify & notify_flag routine table */
    0x1, /* MIDL flag */
    0, /* cs routines */
    0,   /* proxy/server info */
    0
    };

const CInterfaceProxyVtbl * const _MKernel_idl_ProxyVtblList[] = 
{
    ( CInterfaceProxyVtbl *) &_IKernelProxyVtbl,
    0
};

const CInterfaceStubVtbl * const _MKernel_idl_StubVtblList[] = 
{
    ( CInterfaceStubVtbl *) &_IKernelStubVtbl,
    0
};

PCInterfaceName const _MKernel_idl_InterfaceNamesList[] = 
{
    "IKernel",
    0
};

const IID *  const _MKernel_idl_BaseIIDList[] = 
{
    &IID_IDispatch,
    0
};


#define _MKernel_idl_CHECK_IID(n)	IID_GENERIC_CHECK_IID( _MKernel_idl, pIID, n)

int __stdcall _MKernel_idl_IID_Lookup( const IID * pIID, int * pIndex )
{
    
    if(!_MKernel_idl_CHECK_IID(0))
        {
        *pIndex = 0;
        return 1;
        }

    return 0;
}

const ExtendedProxyFileInfo MKernel_idl_ProxyFileInfo = 
{
    (PCInterfaceProxyVtblList *) & _MKernel_idl_ProxyVtblList,
    (PCInterfaceStubVtblList *) & _MKernel_idl_StubVtblList,
    (const PCInterfaceName * ) & _MKernel_idl_InterfaceNamesList,
    (const IID ** ) & _MKernel_idl_BaseIIDList,
    & _MKernel_idl_IID_Lookup, 
    1,
    2,
    0, /* table of [async_uuid] interfaces */
    0, /* Filler1 */
    0, /* Filler2 */
    0  /* Filler3 */
};
#if _MSC_VER >= 1200
#pragma warning(pop)
#endif


#endif /* defined(_M_IA64) */

