#include "Kernel_com.hpp"


CKernel::CKernel()
{
  g_pModule->InitializeComponentInstanceEx(&m_hinst);
}
CKernel::~CKernel()
{
  if (m_hinst)
    g_pModule->TerminateInstance(&m_hinst);
}


HRESULT __stdcall CKernel::AFM_convert_height2RGB(/*[in]*/long nargout, 
                                                  /*[in,out]*/VARIANT* r, 
                                                  /*[in,out]*/VARIANT* g, 
                                                  /*[in,out]*/VARIANT* b, /*[in]*/VARIANT 
                                                  im_height, /*[in]*/VARIANT parameter) {
  return( CallComFcn( "AFM_convert_height2RGB", (int) nargout, 3, 2, r, g, b, &im_height, 
                      &parameter ) );
}


HRESULT __stdcall CKernel::AFM_line_for_show(/*[in]*/long nargout, /*[in,out]*/VARIANT* 
                                             line_show, /*[in]*/VARIANT line_in, 
                                             /*[in]*/VARIANT fit_order, /*[in]*/VARIANT 
                                             index_base_point) {
  return( CallComFcn( "AFM_line_for_show", (int) nargout, 1, 3, line_show, &line_in, 
                      &fit_order, &index_base_point ) );
}


HRESULT __stdcall CKernel::AFM_scan_set_ROI(/*[in]*/long nargout, /*[in,out]*/VARIANT* 
                                            position, /*[in]*/VARIANT im_buffer, 
                                            /*[in]*/VARIANT position_in1) {
  return( CallComFcn( "AFM_scan_set_ROI", (int) nargout, 1, 2, position, &im_buffer, 
                      &position_in1 ) );
}


HRESULT __stdcall CKernel::AFM_show_indent_data() {
  return( CallComFcn( "AFM_show_indent_data", 0, 0, 0 ) );
}


HRESULT __stdcall CKernel::StringEval(/*[in]*/long nargout, /*[in,out]*/VARIANT* out_str, 
                                      /*[in,out]*/VARIANT* out_data, /*[in]*/VARIANT 
                                      in_str, /*[in]*/VARIANT in_data) {
  return( CallComFcn( "StringEval", (int) nargout, 2, 2, out_str, out_data, &in_str, 
                      &in_data ) );
}

HRESULT __stdcall CKernel::MCRWaitForFigures()
{
  mclWaitForFiguresToDie(m_hinst);
  return(S_OK);
}
