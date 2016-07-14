

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Tue Jul 12 00:37:18 2016
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


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 475
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__

#ifndef COM_NO_WINDOWS_H
#include "windows.h"
#include "ole2.h"
#endif /*COM_NO_WINDOWS_H*/

#ifndef __MKernel_idl_h__
#define __MKernel_idl_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __IKernel_FWD_DEFINED__
#define __IKernel_FWD_DEFINED__
typedef interface IKernel IKernel;

#endif 	/* __IKernel_FWD_DEFINED__ */


#ifndef __Kernel_FWD_DEFINED__
#define __Kernel_FWD_DEFINED__

#ifdef __cplusplus
typedef class Kernel Kernel;
#else
typedef struct Kernel Kernel;
#endif /* __cplusplus */

#endif 	/* __Kernel_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"
#include "mwcomtypes.h"

#ifdef __cplusplus
extern "C"{
#endif 


#ifndef __IKernel_INTERFACE_DEFINED__
#define __IKernel_INTERFACE_DEFINED__

/* interface IKernel */
/* [unique][helpstring][dual][uuid][object] */ 


EXTERN_C const IID IID_IKernel;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("41C05E9C-8408-44A7-B3DE-9A86A53B1557")
    IKernel : public IDispatch
    {
    public:
        virtual /* [helpstring][id][propget] */ HRESULT STDMETHODCALLTYPE get_MWFlags( 
            /* [retval][out] */ IMWFlags **ppvFlags) = 0;
        
        virtual /* [helpstring][id][propput] */ HRESULT STDMETHODCALLTYPE put_MWFlags( 
            /* [in] */ IMWFlags *pvFlags) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE AFM_convert_height2RGB( 
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *r,
            /* [out][in] */ VARIANT *g,
            /* [out][in] */ VARIANT *b,
            /* [in] */ VARIANT im_height,
            /* [in] */ VARIANT parameter) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE AFM_line_for_show( 
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *line_show,
            /* [in] */ VARIANT line_in,
            /* [in] */ VARIANT fit_order,
            /* [in] */ VARIANT index_base_point) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE AFM_scan_set_ROI( 
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *position,
            /* [in] */ VARIANT im_buffer,
            /* [in] */ VARIANT position_in1) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE AFM_show_indent_data( void) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE StringEval( 
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *out_str,
            /* [out][in] */ VARIANT *out_data,
            /* [in] */ VARIANT in_str,
            /* [in] */ VARIANT in_data) = 0;
        
        virtual /* [helpstring][id] */ HRESULT STDMETHODCALLTYPE MCRWaitForFigures( void) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IKernelVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IKernel * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IKernel * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IKernel * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetTypeInfoCount )( 
            IKernel * This,
            /* [out] */ UINT *pctinfo);
        
        HRESULT ( STDMETHODCALLTYPE *GetTypeInfo )( 
            IKernel * This,
            /* [in] */ UINT iTInfo,
            /* [in] */ LCID lcid,
            /* [out] */ ITypeInfo **ppTInfo);
        
        HRESULT ( STDMETHODCALLTYPE *GetIDsOfNames )( 
            IKernel * This,
            /* [in] */ REFIID riid,
            /* [size_is][in] */ LPOLESTR *rgszNames,
            /* [range][in] */ UINT cNames,
            /* [in] */ LCID lcid,
            /* [size_is][out] */ DISPID *rgDispId);
        
        /* [local] */ HRESULT ( STDMETHODCALLTYPE *Invoke )( 
            IKernel * This,
            /* [annotation][in] */ 
            _In_  DISPID dispIdMember,
            /* [annotation][in] */ 
            _In_  REFIID riid,
            /* [annotation][in] */ 
            _In_  LCID lcid,
            /* [annotation][in] */ 
            _In_  WORD wFlags,
            /* [annotation][out][in] */ 
            _In_  DISPPARAMS *pDispParams,
            /* [annotation][out] */ 
            _Out_opt_  VARIANT *pVarResult,
            /* [annotation][out] */ 
            _Out_opt_  EXCEPINFO *pExcepInfo,
            /* [annotation][out] */ 
            _Out_opt_  UINT *puArgErr);
        
        /* [helpstring][id][propget] */ HRESULT ( STDMETHODCALLTYPE *get_MWFlags )( 
            IKernel * This,
            /* [retval][out] */ IMWFlags **ppvFlags);
        
        /* [helpstring][id][propput] */ HRESULT ( STDMETHODCALLTYPE *put_MWFlags )( 
            IKernel * This,
            /* [in] */ IMWFlags *pvFlags);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *AFM_convert_height2RGB )( 
            IKernel * This,
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *r,
            /* [out][in] */ VARIANT *g,
            /* [out][in] */ VARIANT *b,
            /* [in] */ VARIANT im_height,
            /* [in] */ VARIANT parameter);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *AFM_line_for_show )( 
            IKernel * This,
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *line_show,
            /* [in] */ VARIANT line_in,
            /* [in] */ VARIANT fit_order,
            /* [in] */ VARIANT index_base_point);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *AFM_scan_set_ROI )( 
            IKernel * This,
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *position,
            /* [in] */ VARIANT im_buffer,
            /* [in] */ VARIANT position_in1);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *AFM_show_indent_data )( 
            IKernel * This);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *StringEval )( 
            IKernel * This,
            /* [in] */ long nargout,
            /* [out][in] */ VARIANT *out_str,
            /* [out][in] */ VARIANT *out_data,
            /* [in] */ VARIANT in_str,
            /* [in] */ VARIANT in_data);
        
        /* [helpstring][id] */ HRESULT ( STDMETHODCALLTYPE *MCRWaitForFigures )( 
            IKernel * This);
        
        END_INTERFACE
    } IKernelVtbl;

    interface IKernel
    {
        CONST_VTBL struct IKernelVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IKernel_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IKernel_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IKernel_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IKernel_GetTypeInfoCount(This,pctinfo)	\
    ( (This)->lpVtbl -> GetTypeInfoCount(This,pctinfo) ) 

#define IKernel_GetTypeInfo(This,iTInfo,lcid,ppTInfo)	\
    ( (This)->lpVtbl -> GetTypeInfo(This,iTInfo,lcid,ppTInfo) ) 

#define IKernel_GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId)	\
    ( (This)->lpVtbl -> GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId) ) 

#define IKernel_Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr)	\
    ( (This)->lpVtbl -> Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr) ) 


#define IKernel_get_MWFlags(This,ppvFlags)	\
    ( (This)->lpVtbl -> get_MWFlags(This,ppvFlags) ) 

#define IKernel_put_MWFlags(This,pvFlags)	\
    ( (This)->lpVtbl -> put_MWFlags(This,pvFlags) ) 

#define IKernel_AFM_convert_height2RGB(This,nargout,r,g,b,im_height,parameter)	\
    ( (This)->lpVtbl -> AFM_convert_height2RGB(This,nargout,r,g,b,im_height,parameter) ) 

#define IKernel_AFM_line_for_show(This,nargout,line_show,line_in,fit_order,index_base_point)	\
    ( (This)->lpVtbl -> AFM_line_for_show(This,nargout,line_show,line_in,fit_order,index_base_point) ) 

#define IKernel_AFM_scan_set_ROI(This,nargout,position,im_buffer,position_in1)	\
    ( (This)->lpVtbl -> AFM_scan_set_ROI(This,nargout,position,im_buffer,position_in1) ) 

#define IKernel_AFM_show_indent_data(This)	\
    ( (This)->lpVtbl -> AFM_show_indent_data(This) ) 

#define IKernel_StringEval(This,nargout,out_str,out_data,in_str,in_data)	\
    ( (This)->lpVtbl -> StringEval(This,nargout,out_str,out_data,in_str,in_data) ) 

#define IKernel_MCRWaitForFigures(This)	\
    ( (This)->lpVtbl -> MCRWaitForFigures(This) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IKernel_INTERFACE_DEFINED__ */



#ifndef __MKernel_LIBRARY_DEFINED__
#define __MKernel_LIBRARY_DEFINED__

/* library MKernel */
/* [helpstring][version][uuid] */ 


EXTERN_C const IID LIBID_MKernel;

EXTERN_C const CLSID CLSID_Kernel;

#ifdef __cplusplus

class DECLSPEC_UUID("7F637BB4-6D20-443C-9CD1-C463BBE1F0A4")
Kernel;
#endif
#endif /* __MKernel_LIBRARY_DEFINED__ */

/* Additional Prototypes for ALL interfaces */

unsigned long             __RPC_USER  VARIANT_UserSize(     unsigned long *, unsigned long            , VARIANT * ); 
unsigned char * __RPC_USER  VARIANT_UserMarshal(  unsigned long *, unsigned char *, VARIANT * ); 
unsigned char * __RPC_USER  VARIANT_UserUnmarshal(unsigned long *, unsigned char *, VARIANT * ); 
void                      __RPC_USER  VARIANT_UserFree(     unsigned long *, VARIANT * ); 

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


