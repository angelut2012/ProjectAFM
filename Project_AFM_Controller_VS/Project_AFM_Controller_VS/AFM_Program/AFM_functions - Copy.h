//AFM_functions.h
#ifndef __AFM_FUNCTIONS__
#define  __AFM_FUNCTIONS__

#include "mbed.h"
#include "SWSPI.h"
#include "DAC.h"
#include "ADC_Chain.h"


#define byte uint8_t



//#include "SPI_DAC.h"
#include "constant_define.h"
#include "unviersial_function.h"
//#include "RunningMedian.h"
//#include "SPI_ADC_DAC.h"
#include "tic_toc.h"
#include "PID_v1.h"
//#include "I2C_DRpot.h"USB_Debug_Value_LN
#include "console_serial.h"
#include "piezo_feed_forward.h"
//#include "IIR_filter.h"
//#include <DueFlashStorage.h>
//_ALWAYS_INLINE_ //__attribute__((always_inline))

/////////////////////////////////////////AFM global members///////////////////////////////////////////////////////////////////////

class AFM_Core
{
	 AFM_Core(){	};
	 ~AFM_Core(){	};
void console_StartIndentation();

void send_IndentData_Package_To_PC(uint32_t v1, uint32_t v2, uint32_t v3);

void process_Indent_First_SendDataThen();

int rtos_send_image_frame_to_PC();

int rtos_buffer_in_index=0;
void Serial_write_reset_input_output();

void Serial_write(byte *d, int L);

inline void Serial_write(byte d);
/////////////////////////////////////////////////////////////////////
void calculate_scan_parameter();
//for akiyama probe  

int y_reverse(int u);

int XYscanning();
void XYscanning_Initialize();
void process_XYscanningReset();


void process_ScanRealTimeLoop();
float convert_byte4_to_float(byte* pb);
byte console_Parameter(byte* com_buffer_frame);
byte console_DRpot(byte *com_buffer_frame);

void console_PiezoFeedforward_output(byte* com_buffer_frame);
void console_DAC_output(byte* com_buffer_frame);
uint32_t ADC_read_MedianFilter(byte port, int num, int delay_time_us);
uint32_t ADC_read_average(byte port, int num, int delay_time_us);
 void console_ResetScannerModel(int axis);

 void console_WithDrawZScanner_SetSystemIdle();
void console_StartApproach();
void console_CancelApproach();
void console_StartZScannerEngage();

inline void console_YScan_Enable();//{y_enable = 1;}
inline void console_YScan_Disable();//{y_enable = 0;}

inline void  console_XYScanReset();
void console_XYScanStart();
inline void console_XYScanPause();//{mode_pause0_scan1_pending2=0;}

void console_GetData(byte* com);
inline void console_ReadStrainGaugeData(int value);//{switch_read_SG=value;}
void console_Control(byte* com);
int command_console(bool echo_b);
void read_SG_data(int* pSwitch_read_SG);
void receiveEvent(int howMany);
void convert_uint32_to_byte4(uint32_t x, byte*b);
void convert_uint32_to_byte3(uint32_t x, byte*b);
void convert_uint32_to_byte2(uint32_t x, byte*b);
void send_back_approach_heart_beat(uint32_t v, uint32_t step, byte done);
void send_system_package16_char_to_PC(const char*inf);
double some_function(double x, double y);
//////////////////////-- process-------------------
void process_ZScannerEngage();
void process_Approach();
void prepare_engaged_package_to_PC(int indx, int indy, double vHeight, double vError);
void prepare_image_package_to_PC(int indx, int indy, double vHeight, double vError);
void prepare_image_package_to_PC_sub(int indx, int indy, double vHeight, double vError);
void send_back_debug_infomation();

void process_ScanRealTimeLoop_Initialize(double position_01);
void process_Idle();
void prepare_system_package_to_PC(uint32_t v1, uint32_t v2, uint32_t v3);

int test(int x);

void AFM_main_setup();
void read_SG_data_temp();
void AFM_main_loop();
void AFM_Realtime_loop();
#endif // !__AFM_FUNCTIONS__
