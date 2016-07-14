#ifndef _MKernel_Kernel_com_HPP
#define _MKernel_Kernel_com_HPP 1

#include <windows.h>
#include "MKernel_idl.h"
#include "mclmcrrt.h"
#include "mclcom.h"
#include "mclcommain.h"
#include "mclcomclass.h"

class CKernel : public CMCLClassImpl<IKernel, &IID_IKernel, CKernel, &CLSID_Kernel>
{
public:
  CKernel();
  ~CKernel();

  HRESULT __stdcall AFM_convert_height2RGB(/*[in]*/long nargout, /*[in,out]*/VARIANT* r, 
                                           /*[in,out]*/VARIANT* g, /*[in,out]*/VARIANT* 
                                           b, /*[in]*/VARIANT im_height, /*[in]*/VARIANT 
                                           parameter); 

  HRESULT __stdcall AFM_line_for_show(/*[in]*/long nargout, /*[in,out]*/VARIANT* 
                                      line_show, /*[in]*/VARIANT line_in, /*[in]*/VARIANT 
                                      fit_order, /*[in]*/VARIANT index_base_point); 

  HRESULT __stdcall AFM_scan_set_ROI(/*[in]*/long nargout, /*[in,out]*/VARIANT* position, 
                                     /*[in]*/VARIANT im_buffer, /*[in]*/VARIANT 
                                     position_in1); 

  HRESULT __stdcall AFM_show_indent_data(); 

  HRESULT __stdcall StringEval(/*[in]*/long nargout, /*[in,out]*/VARIANT* out_str, 
                               /*[in,out]*/VARIANT* out_data, /*[in]*/VARIANT in_str, 
                               /*[in]*/VARIANT in_data); 

  HRESULT __stdcall MCRWaitForFigures();

};
#endif
