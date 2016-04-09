#ifndef __MAIN_INCLUDE__
#define __MAIN_INCLUDE__

#define __AFM_USB_DEBUG__

#include "defines.h"
#include "constant_define.h"

#include "define_AFM_platform.h"

#include "mbed.h"
#include "USBSerial.h"


#include "tm_stm32_adc.h"
#include "ADC_Temperature.h"

#include "define_AFM_hardware.h"
#include "SWSPI.h"


//#include "tm_stm32_spi.h"
#include "DAC.h"
#include "ADC_Chain.h"


//#define  __AFM_test


#include "function_tests.h"
#ifdef __AFM_test


#else 


#include "AFM_functions.h"
#endif


//extern  USBSerial mUSerial;

#endif // !__MAIN_INCLUDE__
///////#include "main_include.h"

