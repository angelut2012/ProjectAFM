//AFM_functions.h
#ifndef __AFM_FUNCTIONS__
#define  __AFM_FUNCTIONS__


#include "defines.h"
#include "constant_define.h"

//#include "define_AFM_platform.h"

#include "mbed.h"
#include "USBSerial.h"


//#include "tm_stm32_adc.h"
#include "ADC_Temperature.h"
#include "define_AFM_hardware.h"
#include "SWSPI.h"


 
 

#include "DAC.h"
#include "ADC_Chain.h"

//#include "function_tests.h"

typedef  uint8_t byte;



//#include "SPI_DAC.h"
//#include "constant_define.h"
#include "unviersial_function.h"
#include "RunningMedian.h"
//#include "SPI_ADC_DAC.h"
//#include "tic_toc.h"
#include "PID_v1.h"
//#include "I2C_DRpot.h"USB_Debug_Value_LN
#include "console_serial.h"


#include "Scanner.h"

#include "AFM_ClockDefine.h"

/////////////////////////////////////////AFM global members///////////////////////////////////////////////////////////////////////
extern DigitalOut* p_LED;
extern DigitalOut *p_LED;// = new DigitalOut(PORT_LED);
extern DigitalOut *p_Tdio1;// = new DigitalOut(Tdio1);
extern DigitalOut *p_Tdio2;// = new DigitalOut(Tdio2);
extern DigitalOut *p_Tdio3;// = new DigitalOut(Tdio3);
extern DigitalOut *p_Tdio4;// = new DigitalOut(Tdio4);
extern DigitalOut *p_Tdio5;// = new DigitalOut(Tdio5);

extern CDAC mAFM_DAC;	
extern CSEM mAFM_SEM;

extern CTickTimer mCTickTimer_RealTime;
extern  USBSerial mUSerial;
	

extern CScanner mCScanner[3];

extern void toggle_pin(PinName port);
extern void toggle_pin_led();
extern void toggle_pin_p(DigitalOut* p);
	//----------------------tic toc .h------------------------------------------------------------





class AFM_Core
{
public:
	
	
	void toggle_pin(PinName port)
	{
		static bool v = 0;
		v = !v;
		DigitalOut p(port);
		p = v;
	}
	;
	void toggle_pin_led(){toggle_pin(PORT_LED);}
	;
	void toggle_pin_p(DigitalOut* p)
	{
		static bool v = 0;
		v = !v;
		//DigitalOut p(port);
		*p = v;
	}
	;
	
	float  wave_triangle_0ToMax(float delta, float value_max, bool reset)
	{
		static float v = 0;
		static bool direction = true;
		if (reset == true) {v = 0;direction = true;}
		if (direction == true)
			v += delta;
		else
			v -= delta;
		if (v >= value_max | v <= 0)
			direction = !direction;
		v = LIMIT_MAX_MIN(v, value_max,0);
		return v;
	}
	;
	
	
#define _MICRO_TIMER_ENABLE_
#ifndef _MICRO_TIMER_ENABLE_
#define TIC()
#define TOC() 0
#define TIC_P()
#define TOC_P() 0

#define TICX()
#define TOCX()
#define TICX_P() 0
#define TOCX_P() 0
#define PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(dt) 0//period_check_time_us_due(4,dt)

#define PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(dt) 0//period_check_time_us_due(3,dt)

#define PERIOD_CHECK_TIME_US_DUE_Z_REAL_TIME_SERVO_LOOP(dt) 0//period_check_time_us_due(2,dt)
#define PERIOD_CHECK_TIME_US_DUE_ZSCANNERENGAGE(dt) 0//period_check_time_us_due(1,dt)
#define PERIOD_CHECK_TIME_US_DUE_APPROACH(dt) 0//period_check_time_us_due(0,dt)
#else
#define TIC() tic_toc(0)
#define TOC() tic_toc(1)
#define TIC_P()	{mUSerial.println("TIC");TIC();}
//#define TIC_P	do{mUSerial.println("TIC");TIC;}while(0)
#define TOC_P()	{long t=TOC();mUSerial.print("TOC");mUSerial.println(t,DEC);} 
#define micros() mCTickTimer_RealTime.read_us()	//2^31/1e6/3600=0.6 hour
#define micros_reset() mCTickTimer_RealTime.reset()	//2^31/1e6/3600=0.6 hour
	

#define CHECK_COUNT_DUE(x) 		static uint32_t count = 0;count++;if (count <(x)) return;count = 0;
#define CHECK_COUNT_DUE2(x) 		static uint32_t count2 = 0;count2++;if (count2 <(x)) return;count2 = 0;
	
//inline long micros()
//{
//	return mCTickTimer_RealTime.read_us();	//2^31/1e6/3600=0.6 hour
//}
	int tic_toc(int start0_end1)
	{
//		static long t = 0;
		if (start0_end1 == 0)
			return micros();
		else
		{
			micros_reset();
			//t = micros() - t;
			return 0;
		}
				
	}
	long tic_toc_OLD(int start0_end1)
	{
		static long t = 0;
		if (start0_end1 == 0)
			t = micros();
		else
			t = micros() - t;
		return t;
	}

					////////////
#define TICX(n) tic_tocX(0,(n))
#define TOCX(n) tic_tocX(1,(n))
#define TICX_P(n)	{mUSerial.println("TIC");TICX((n));}
													//#define TIC_P	do{mUSerial.println("TIC");TIC;}while(0)
#define TOCX_P(n)	{long t=TOCX((n));mUSerial.print("TOC");mUSerial.println(t,DEC);} 
#define MAX_NUMBER_OF_MICRO_TIMER (10)
	long tic_tocX(int start0_end1, int n)
	{
		static long t[MAX_NUMBER_OF_MICRO_TIMER] = {0};
		if (start0_end1 == 0)
			t[n] = micros();
		else
			t[n] = micros() - t[n];
		return t[n];
	}
	//long evaluate_function_time(void (*pfun)())
	//{
	//	TIC;
	//	pfun();
	//	long t= TOC;
	//	mUSerial.println(t,DEC);
	//}

#define PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(dt) period_check_time_us_due(4,dt)

#define PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(dt) period_check_time_us_due(3,dt)

#define PERIOD_CHECK_TIME_US_DUE_Z_REAL_TIME_SERVO_LOOP(dt) period_check_time_us_due(2,dt)
#define PERIOD_CHECK_TIME_US_DUE_ZSCANNERENGAGE(dt) period_check_time_us_due(1,dt)
#define PERIOD_CHECK_TIME_US_DUE_APPROACH(dt) period_check_time_us_due(0,dt)
	bool period_check_time_us_due(int index, unsigned long dt)
		// do not use this function directly, to avoid multi use at different places
	{
		static unsigned long time_store[5] = {0};	
		unsigned long time_now = micros();	
		if ((time_now - time_store[index]) < (dt))
			return false;
		else 
		{
			time_store[index] = time_now;
			return true;
		}
	}

	void Software_Reset() 
	{
		NVIC_SystemReset();
		//	const int RSTC_KEY = 0xA5;
		//	//RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
		//	while (true);
	}

#endif 	//_MICRO_TIMER_ENABLE_
	
		///-------------------------end of -tic toc .h-----------------------------------------------------------
	
	
	
	AFM_Core(void) 
	{
		;
	}
	;
	~AFM_Core(void) {}
	;
	//
//	Ticker mTicker_AFM_Realtime;	
//	Ticker mTicker_AFM_Communication;//.attach_us(&AFM_Communication_Process(), mPeriod_Communication_us);
//	Timer mCTickTimer_RealTime;
//	CTickTimer mCTickTimer_RealTime = mCTickTimer_RealTime;
//	CTickTimer mCTickTimer_RealTime = mCTickTimer_RealTime;
//	mCTickTimer_RealTime
	
//	static 
	void AFM_ProcessScheduler_Realtime();
	//	static 
	void AFM_ProcessScheduler_NonRealtime();
	//////////////////////////////////////hardware test functions


		/////////////////////////////////////////AFM global members///////////////////////////////////////////////////////////////////////

			//the port use for PIEZO_Z and PIEZO_T(tuning fork)
			//	byte Z_scanner_port;// SCANNER_Z_ONLY;
#define PIEZO_T_CenterBit18 (140581.0) //80.441/150*2^18
#define PIEZO_T_Center01 (PIEZO_T_CenterBit18/BIT18MAX) //80.441/150*2^18

				/////////////////////----------------------------------
				// send package16 to PC via RTOS
				//	CPackageToPC* mCPackageToPC_SystemPackage;// new CPackageToPC(2);
					////////////////////////// median filter

					//	int rtos_buffer_in_index;
						// strain gauge global switch ON_OFF
	int switch_read_SG;// -1;// >0: read; ==1, read only once; ==2 continuously read
	///////////////// approaching
	int mPeriod_Realtime_us;
	int mPeriod_RealtimePID_us;
	float mSamplingFrequency_Realtime_MHz;
	int mPeriod_Communication_us;
	uint32_t step_counter_Approach;// 0;
	uint32_t step_size_increament_Appraoch;// 1;
	uint32_t Z_position_DAC_Approach;// 0;
	uint32_t ADC_sensor_buffer;// 0;


	uint32_t Z_position_DAC_ZScannerEngage;// 0;

		//int counter_large_step_approaching;//0;
		//DueTimer //mTimer_ZLoop ;// DueTimer(1);
		//DueTimer mTimer_Approach ;// DueTimer(2);
		///////////////////////
		//typedef 
	enum Sys_State {SS_Idle = 0, SS_WaveTest, SS_Scan, SS_XYScanReset, SS_Engage, SS_Approach, SS_ApproachWait, SS_Indent};//,SS_DataCapture

	Sys_State sys_state;// SS_Idle;
	uint32_t V18_Adc[NUM_OF_ADC];// {0};//
	uint32_t V18_Dac[NUM_OF_DAC];// {0};//
	float   position_feedforward_output_01[NUM_OF_DAC];// {0};//

	int Vdf_infinite;// BIT18MAX_HALF;


	float threshold_approach_delta_B18;

	
#define  sampling_period_us_of_Approach_Process (250.0)//old;//2000.0
	//#endif
#define  sampling_frequency_of_Approach_Process ( 1000000.0/ sampling_period_us_of_Approach_Process)
		// z scanner engage
#define  sampling_period_us_of_ZScannerEngage_Process (1000.0)
#define  sampling_frequency_of_ZScannerEngage_Process ( 1000000.0/ sampling_period_us_of_ZScannerEngage_Process)	
	//// Zloop
#define SamplingTime_ZLoop_us (350) //200(500)// measured by experiments

#define SamplingFrequency_ZLoop_Hz  (1000000.0/SamplingTime_ZLoop_us)// Hz

	////////////////////////// input for  xy scan
	int N_FramesToScan;// 1;
	int XL_NM;// 
	int DX_NM;// 1000, 
	int YL_NM;// 0, 
	int DY_NM;// 1000;

	int XL;// 0, 
	int YL;// 0, 
	int DX;// 0, 
	int DY;// 0; //to be calculated in calculate_scan_parameter()
	// XL_NM * DAC_PER_NM_X, DX ;// DX_NM * DAC_PER_NM_X, 
	// YL_NM * DAC_PER_NM_Y, DY ;// DY_NM * DAC_PER_NM_Y;

	
	float scan_rate;// 0.5;//line per second 
	int N_x;// 128, 
	int N_y;// 128;// image size
	int diry_input;// 0;



	int mode_pause0_scan1_pending2;// 0;
	int y_enable;// 1;
	int dds_reset;// 0;
	int enable;// 1;
	//output for xy scan
	int indx;// 0, 
	int indy;// 0;// x y index for image
	int VDACx;// 0, 
	int VDACy;// 0, 
	int VDACz;// 0;//value output to DAC 
	int SX;// 0, 
	int SY;// 0;
	//typedef int 
	enum DDS_XY_Scanner_State {DDS_XY_Reset, DDS_XY_Scan, DDS_XY_Idle};
	// sys_ini;//0;// 0:initial, power on
	//const int DDS_XY_Reset ;// 0; // slow move;
	//const int DDS_XY_Scan ;// 1; // scan move;
	//const int DDS_XY_Idle ;// 2; //  idle

	DDS_XY_Scanner_State mDDS_XY_Scanner_State;// DDS_XY_Reset; // the mDDS_XY_Scanner_State of the system
	   /////////////////////////////////////////////

	   	/////////////////////// image
#define SIZE_IMAGE_BUFFER_LINES (5)
#define SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE (512)
#define SIZE_IMAGE_BUFFER_FORWARD0_BACKWORD1	(2)
#define SIZE_IMAGE_BUFFER_BIT24	(3)
	   								//-------------------------- indentation------------------------------------------------------------------//
	//int mI_MaxStep;// 4096;//6000;
	// indentation parameters, input from GUI
	float mI_MaxDepth;// 1;//SCANNER_RANGE_Z_NM for savety reason only
	float mI_StiffnessPRC_nN_per_nm;// 40.0;//40N/m, 40 nN/nm
	float mI_TriggerForce_nN;// mI_StiffnessPRC_nN_per_nm * 37.5;// 100nm
	float mI_PRC_ADCValue_per_nm;// 86.9;//
	float mI_LoopDelay_uS;// 500;//
	int	 mI_HalfNumberOfSamplingPoints;// mI_MaxStep / 2;// number of points to sample and store and send back
	// paramerter to be calculated
	float mI_PRC_ADC_ValueInitial;// 0;// initial value of ADC PRC value;
	float mI_PRC_Force_nN_per_ADCValue;// mI_StiffnessPRC_nN_per_nm / mI_PRC_ADCValue_per_nm;
	float mI_PRC_Force_nN_Now;// 0;
	float mI_step_size_nm;// 1;//Math_Min;//0.0816
	float mI_step_size_01;// mI_step_size_nm / SCANNER_RANGE_Z_NM;// 4.675524715375971e-05
	bool mI_direction_indent_True_withdraw_False;// true;
#define mI_Withdraw_Done_Threshold_nm (1)// should not be smaller than noise level
	float mI_Withdraw_Done_Threshold_ADC;// mI_PRC_ADCValue_per_nm*mI_Withdraw_Done_Threshold_nm;

#define mI_MaxStep (4096)//(25*1024) is max
	uint32_t mIndentData[3][mI_MaxStep];// {0};

	uint32_t mZLoopPID_WorkingDistance_nm;// 500 / 4;// set working voltage;// receive from rs232
	float mZLoopPID_WorkingDistance_Threshold_01;// hold XY scanner if dts error is too high
	//bool  mHoldXYScanner ;// hold XY scanner if dts error is too high
	float DTS_Sensitivity_B18_per_nm;//
	int32_t VWset_deltaV_ADC_b18;// 0;// use in engage//CONV_DELTA_WORKING_VOLTAGE_MV_TO_ADC(mZLoopPID_WorkingDistance_nm);// update VWset_deltaV_ADC_b18 each time when set mZLoopPID_WorkingDistance_nm
//	float mTF_DC_Gain;// 1;// gain for tf DC actuation

//	float TF_SensorRange;// nm, assumed range, or Zmax

	//
	//float DSet_01=(-(float)mZLoopPID_WorkingDistance_nm/1000.0/4.0+2.5)// voltage 0~5 //2.375V
	//	*BIT18MAX/5// adc value 0~2^18
	//	*pid_input_Gain_adjust;// pid range 1000 nm


	//////////// send image package back tp PC///////////////////
	///////send one byte each time
//	byte com_image_frame_buffer[LENGTH_IMAGE_FRAME_BUFFER];// {0};// 
//	int pointer_out_frame_buffer;// LENGTH_IMAGE_FRAME_BUFFER;

	CPID *mPID_ZLOOP;// new PID(DInput_01, DOutput_01, DSet_01, 0.05, 0.02, 0, true);// use true for PRC imagining scanning

	//float z_output_01;// 0;
#define LENGTH_COM_BUFFER_PC2MCU_CONSOLE (LENGTH_COM_FRAME_PC2MCU * 4)
	byte com_buffer[LENGTH_COM_BUFFER_PC2MCU_CONSOLE];// {0};
	byte com_buffer_frame[LENGTH_COM_FRAME_PC2MCU - 4];// {0};

	float measured_sampling_frequency_of_system;// 0;
	
	float mStepSize_Appoach;
	byte sys_idle_package_index;
	
	int mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2;// = 0;
	
	//-----------------------------------------------------------------------------------------
	
	
	
//#define MY_Debug_LN(x) //{mUSerial.println(x);}
//#define MY_Debug_LN2(x,y) //mUSerial.println(x,y)
//#define MY_Debug(x) //{mUSerial.print(x);}
////#define MY_Debug2(x,y) mUSerial.print(x,y)
//#define MY_Debug_StringValue_LN(str,value) // do{mUSerial.print(str);mUSerial.println(value);}while(0)	
	
#define MY_Debug_LN(x) {mUSerial.println(x);}
#define MY_Debug_LN2(x,y) mUSerial.println(x,y)
#define MY_Debug(x) {mUSerial.print(x);}
//#define MY_Debug2(x,y) mUSerial.print(x,y)
#define MY_Debug_StringValue_LN(str,value)  do{mUSerial.print(str);mUSerial.println(value);}while(0)
#define MY_Debug_StringValue_T(str,value)  do{mUSerial.print(str);mUSerial.print(value);mUSerial.print("\t");}while(0)

#define TIMES_PID_LOOP (4)	// 5~10 period
	void Initial_parameters()
	{		
//#define  sampling_period_us_of_Approach_Process (250.0)//old;//2000.0
//	//#endif
//#define  sampling_frequency_of_Approach_Process ( 1000000.0/ sampling_period_us_of_Approach_Process)
//		// z scanner engage
//#define  sampling_period_us_of_ZScannerEngage_Process (1000.0)
//#define  sampling_frequency_of_ZScannerEngage_Process ( 1000000.0/ sampling_period_us_of_ZScannerEngage_Process)	
//	//// Zloop
//#define SamplingTime_us_Zloop  (400)// (330)//1000 
//#define SamplingFrequency_ZLoop_Hz  (1000000.0/SamplingTime_us_Zloop)// Hz
		mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 0;
		sys_idle_package_index = 0;
		mPeriod_RealtimePID_us = SamplingTime_ZLoop_us;//__TICK_PERIOD_US__ * 0.9;//200;
		mPeriod_Realtime_us = SamplingTime_ZLoop_us;// mPeriod_RealtimePID_us * TIMES_PID_LOOP;//501;//200;

		mPeriod_Communication_us = 1007;	
		
		float mSamplingFrequency_ApproachProcess_Hz = 385;// measured
		
#if (ADC_PORT_ZlOOP_SENSOR==ADC_CHANNEL_PRC)
#define TIME_APPROACHING_COARSE_STEP (2)//(2.5)//Second
#else
#define TIME_APPROACHING_COARSE_STEP (5)//5 Second, tuning fork
#endif

		mStepSize_Appoach = (BIT18MAX_0D9 / (TIME_APPROACHING_COARSE_STEP*mSamplingFrequency_ApproachProcess_Hz));
		
		

			//-------------------------------------------------------------------		
//		rtos_buffer_in_index = 0;
//		TF_SensorRange = 1000;
//the port use for PIEZO_Z and PIEZO_T(tuning fork)
//		Z_scanner_port = SCANNER_Z_ONLY;
	   /////////////////////----------------------------------
	   // send package16 to PC via RTOS
//		mCPackageToPC_SystemPackage = new CPackageToPC(2);
			////////////////////////// median filter


				// strain gauge global switch ON_OFF
		switch_read_SG = -1;// >0: read; ==1, read only once; ==2 continuously read
	   ///////////////// approaching

		step_counter_Approach = 0;
		step_size_increament_Appraoch = 1;
		Z_position_DAC_Approach = 0;
		ADC_sensor_buffer = 0;

			// z scanner engage

		Z_position_DAC_ZScannerEngage = 0;

			// counter_large_step_approaching=0;
			//DueTimer //mTimer_ZLoop = DueTimer(1);


		sys_state = SS_Idle;
		//V18_Adc[4] = {0};//
		//V18_Dac[4] = {0};//
		//position_feedforward_output_01[4] = {0};//


					////////////////////////// input for  xy scan
		N_FramesToScan = 1;
		XL_NM = 0;
		DX_NM = 1000;
		YL_NM = 0;
		DY_NM = 1000;

		XL = 0;
		YL = 0;
		DX = 0;
		DY = 0; //to be calculated in calculate_scan_parameter()
	   // XL_NM * DAC_PER_NM_X, DX = DX_NM * DAC_PER_NM_X, 
	   // YL_NM * DAC_PER_NM_Y, DY = DY_NM * DAC_PER_NM_Y;

	   	// scan_rate=100;// 100 is 1 line/s
		scan_rate = 0.5;//line per second 
		N_x = 128;
		N_y = 128;// image size
		diry_input = 0;

			//// Zloop


		mode_pause0_scan1_pending2 = 0;
		y_enable = 1;
		dds_reset = 0;
		enable = 1;
	   //output for xy scan
		indx = 0;
		indy = 0;// x y index for image
		VDACx = 0;
		VDACy = 0;
		VDACz = 0;//value output to DAC 
		SX = 0;
		SY = 0;

		mDDS_XY_Scanner_State = DDS_XY_Reset; // the 
		  /////////////////////// image
		  //-------------------------- indentation------------------------------------------------------------------//
		
		// indentation parameters, input from GUI
		mI_MaxDepth = 1;//SCANNER_RANGE_Z_NM for savety reason only
		mI_StiffnessPRC_nN_per_nm = 40.0;//40N/m, 40 nN/nm
		mI_TriggerForce_nN = mI_StiffnessPRC_nN_per_nm * 37.5;// 100nm
		mI_PRC_ADCValue_per_nm = 86.9;//
		mI_LoopDelay_uS = 500;//
		mI_HalfNumberOfSamplingPoints = mI_MaxStep / 2;// number of points to sample and store and send back
   // paramerter to be calculated
		mI_PRC_ADC_ValueInitial = 0;// initial value of ADC PRC value;
		mI_PRC_Force_nN_per_ADCValue = mI_StiffnessPRC_nN_per_nm / mI_PRC_ADCValue_per_nm;
		mI_PRC_Force_nN_Now = 0;
		mI_step_size_nm = 1;//Math_Min=0.0816
		mI_step_size_01 = mI_step_size_nm / SCANNER_RANGE_Z_NM;// 4.675524715375971e-05
		mI_direction_indent_True_withdraw_False = true;

		mI_Withdraw_Done_Threshold_ADC = mI_PRC_ADCValue_per_nm*mI_Withdraw_Done_Threshold_nm;

		//mIndentData[3][mI_MaxStep] = {0};

		mZLoopPID_WorkingDistance_nm = 5;// set working voltage;// receive from rs232
		DTS_Sensitivity_B18_per_nm = 40; //192/2;// nm/V   // receive from rs232
		VWset_deltaV_ADC_b18 = 0;// use in engage//CONV_DELTA_WORKING_VOLTAGE_MV_TO_ADC(mZLoopPID_WorkingDistance_nm);// update VWset_deltaV_ADC_b18 each time when set mZLoopPID_WorkingDistance_nm
//		mTF_DC_Gain = 1;// gain for tf DC actuation

//		TF_SensorRange = 1000;// nm, assumed range, or Zmax
	   // pid_input_Gain_adjust=0;//5.0*DTS_Sensitivity_B18_per_nm/BIT18MAX/TF_SensorRange;

	   	//
	   	// DSet_01=(-()mZLoopPID_WorkingDistance_nm/1000.0/4.0+2.5)// voltage 0~5 //2.375V
	   	//	*BIT18MAX/5// adc value 0~2^18
	   	//	*pid_input_Gain_adjust;// pid range 1000 nm
//		DSet_01 = 0;
//		DInput_01 = 0;//% input from ADC, voltage for delta frequency
//		DOutput_01 = 0;

			//////////// send image package back tp PC///////////////////
			///////send one  each time
		//com_image_frame_buffer[LENGTH_IMAGE_FRAME_BUFFER] = {0};// 
//		pointer_out_frame_buffer = LENGTH_IMAGE_FRAME_BUFFER;

		mPID_ZLOOP = new CPID(true);//DInput_01, DOutput_01, DSet_01, 0.05, 0.02, 0, use true for PRC imagining scanning
		//z_output_01 = 0;
		//com_buffer[LENGTH_COM_FRAME_PC2MCU * 2] = {0};
		//com_buffer_frame[LENGTH_COM_FRAME_PC2MCU - 4] = {0};

		measured_sampling_frequency_of_system = 0;
		
		calculate_scan_parameter();

	}
	;
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//------console 

	void console_ResetScannerModel(int axis)
	{
		position_feedforward_output_01[axis] = 0;
		//V18_Dac[axis]=
		piezo_predict_Position01_To_Voltage_DAC18(axis, position_feedforward_output_01[axis]);
	}
	void console_WithDrawZScanner_SetSystemIdle()
	{
		console_ResetScannerModel(PIEZO_Z);
		//mTimer_ZLoop.stop();
		//mTimer_Approach.stop();
		V18_Adc[ADC_PORT_ZlOOP_SENSOR] = 0;

			//// tuning fork dc drive
			//V18_Dac[PIEZO_T]=PIEZO_T_CenterBit18;
			//mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_T,V18_Dac[PIEZO_T]);
			//console_TF_Scan_Disable();
			////
		sys_state = SS_Idle;
		mCScanner[PIEZO_Z].MoveToPosition01(0);
		mCScanner[PIEZO_Z].mPID_Scanner->Reset();
		
	}


	inline void console_StartIndentation()
	{
		//parameters initialize
		mI_PRC_Force_nN_per_ADCValue = mI_StiffnessPRC_nN_per_nm / mI_PRC_ADCValue_per_nm;
		mI_PRC_Force_nN_Now = 0;

		mI_step_size_01 = mI_step_size_nm / SCANNER_RANGE_Z_NM;// 4.675524715375971e-05
		mI_direction_indent_True_withdraw_False = true;
		mI_Withdraw_Done_Threshold_ADC = mI_PRC_ADCValue_per_nm*mI_Withdraw_Done_Threshold_nm;
		memset(&(mIndentData[0][0]), 0, 3*mI_MaxStep*sizeof(mIndentData[0][0]));

		mI_HalfNumberOfSamplingPoints = LIMIT_MAX_MIN(mI_HalfNumberOfSamplingPoints, mI_MaxStep / 2, 0);// to avoid usefull data to be covered

			//-------------------------
		sys_state = SS_Indent;
		
		console_ResetScannerModel(PIEZO_Z);  // for vibration test, comment
		
		V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	
		mI_PRC_ADC_ValueInitial = mAFM_SEM.ADC_Read_N_Average(ADC_PORT_ZlOOP_SENSOR, 20, 10);//1.97 kHz
		//ADC_read_average(ADC_PORT_ZlOOP_SENSOR,50,100);
		//USB_Debug_LN("start indent");
	}

	void send_IndentData_Package_To_PC(uint32_t v1, uint32_t v2, uint32_t v3)
	{	
//		byte com[LENGTH_COM_FRAME_MCU2PC] = {0};
//		com[0] = COM_HEADER1;
//		com[1] = COM_HEADER2,
//			com[LENGTH_COM_FRAME_MCU2PC - 2] = COM_TAIL1;
//		com[LENGTH_COM_FRAME_MCU2PC - 1] = COM_TAIL2;
//
//		com[0 + 2] = 'I';
//		com[1 + 2] = 'D';
//		convert_uint32_to_byte4(v1, &com[2 + 2]);// byte4
//		convert_uint32_to_byte3(v2, &com[2 + 4 + 2]);// byte3
//		convert_uint32_to_byte3(v3, &com[2 + 4 + 3 + 2]);//byte3
//		mUSerial.write(com, LENGTH_COM_FRAME_MCU2PC); 		
//		
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 'I';
		com[1] = 'D';
		//com[2]=0;
		convert_uint32_to_byte4(v1, &com[2]);// byte4
		convert_uint32_to_byte3(v2, &com[2 + 4]);// byte3
		convert_uint32_to_byte3(v3, &com[2 + 4 + 3]);//byte3
//		mCPackageToPC_SystemPackage->prepare_package_Byte12_to_PC(com);
		send_Data_In_Frame_To_PC(com);
		
	}
	//
	//void process_Indent_First_SendDataThen_temp()
	//{
	//
	//	int dt=200;
	//	while(1)
	//	{
	//		mAFM_DAC.DAC_write(PIEZO_X,0);
	//		mAFM_DAC.DAC_write(PIEZO_Y,0);
	//	
	//		wait_ms(dt);
	//		mAFM_DAC.DAC_write(PIEZO_X,BIT18MAX);
	//		mAFM_DAC.DAC_write(PIEZO_Y,0);
	//		wait_ms(dt);
	//		mAFM_DAC.DAC_write(PIEZO_X,BIT18MAX);
	//		mAFM_DAC.DAC_write(PIEZO_Y,BIT18MAX);
	//		wait_ms(dt);
	//		mAFM_DAC.DAC_write(PIEZO_X,0);
	//		mAFM_DAC.DAC_write(PIEZO_Y,BIT18MAX);
	//		wait_ms(dt);
	//	}
	//
	//
	//	uint32_t* pData=&(mIndentData[0][0]);
	//	int data_length=mI_MaxStep*3;
	//	int t=0;
	//	int dtime=100;
	//	//int v1=BIT18MAX/4.0;
	//	//int v2=BIT18MAX_0D75;
	//	int v1=0;
	//	int v2=BIT18MAX*(130.0/150.0);//
	//	// pre
	//	//for (int k=0;k<300;k++)
	//	//{
	//	//	mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//	//}
	//	mAFM_DAC.DAC_write(PIEZO_Z,0);
	//	wait_ms(10);
	//	pData[0]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//	//
	//	//mAFM_DAC.DAC_write(PIEZO_Z,BIT18MAX_HALF);
	//	////piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,0.5);
	//	mAFM_DAC.DAC_write(PIEZO_Z,v1);
	//	int st=1;
	//	int re=100;
	//	int len=data_length/re/2;
	//	// sin
	//	float N=100;
	//	float T=10;
	//	float v=0,V=0;
	//
	//
	//	for (int k=0;k<1e6;k++)
	//	{
	//		V+=T;
	//		v=wave_triangle(V,BIT18MAX);
	//		mAFM_DAC.DAC_write(PIEZO_Y,v*2);
	//		//wait_us(dtime);
	//			//pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//			//wait_us(dtime);
	//	}
	//
	//		for (int m=0;m<4;m++)// send finish signal multi times, to make sure PC receive it
	//	{
	//		wait_ms(100);
	//		send_IndentData_Package_To_PC(BIT24MAX,BIT24MAX,BIT24MAX);// finished
	//	}
	//
	//		return;
	//
	//	//---------------------------------------------------
	////#include"table.h"
	//	for (int k=0;k<data_length;k++)
	//	{
	//		V+=T;
	//		//v=MOD_range(V,BIT18MAX*2);
	//		//v-=BIT18MAX;
	//		//v=Math_Abs(v);
	//		v=wave_triangle(V,BIT18MAX);
	//
	//		mAFM_DAC.DAC_write(PIEZO_Z,v*13/15);
	//		wait_us(dtime);
	//			pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//			wait_us(dtime);
	//	}
	//	//for (int q=0;q<re;q++)
	//	//{
	//	//	for (int k=st;k<(st+len);k++)
	//	//	{
	//	//		pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//	//		wait_us(dtime);
	//	//	}
	//	//	st+=len;
	//	//	mAFM_DAC.DAC_write(PIEZO_Z,v2);
	//	//	for (int k=st;k<(st+len);k++)
	//	//	{
	//	//		pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//	//		wait_us(dtime);
	//	//	}
	//	//	st+=len;
	//	//	mAFM_DAC.DAC_write(PIEZO_Z,v1);
	//	//}
	//
	//	//for (int k=0;k<data_length;k++)
	//	//{
	//	//	fastDigitalWrite(23,true);
	//	//	//for noise rms test
	//	//	pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
	//	//	wait_us(dtime);
	//
	//	//		t=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,25);
	//	//wait_us(25);
	//	//		t+=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,25);
	//	//wait_us(25);
	//	//t>>=1;
	//	//pData[k]=t;
	//
	//	//use for indent
	//	//pData[k]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,3,10);//20_10-->1.97 kHz, noise rms=   11.9427ADC18, 0.311 nm;//40_10-->1.13 kHz
	//	//--------------------judge status----------
	//	//fastDigitalWrite(23,false);
	//	// store data
	//	//index=MOD_range(k,mI_MaxStep); //here % operator has problem with int
	//	//mIndentData[0][index]=(uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX);
	//	//mIndentData[1][k]=V18_Adc[ADC_PORT_ZlOOP_SENSOR];
	//	//mIndentData[2][index]=V18_Dac[PIEZO_Z];
	//
	//	//}// fore loop
	//
	//	//------------------------------------------------------------------
	//	// finished indentation, send data back to PC now
	//	console_WithDrawZScanner_SetSystemIdle();
	//
	//	//for (int k=0;k<mI_MaxStep;k++)
	//	for (int k=0;k<mI_MaxStep;k++)	
	//	{
	//		send_IndentData_Package_To_PC(mIndentData[0][k],mIndentData[1][k],mIndentData[2][k]);
	//		wait_ms(7);//old delay=4 , but lose point sometimes
	//	}
	//	for (int m=0;m<4;m++)// send finish signal multi times, to make sure PC receive it
	//	{
	//		wait_ms(100);
	//		send_IndentData_Package_To_PC(BIT24MAX,BIT24MAX,BIT24MAX);// finished
	//	}
	//	// tuning fork indent
	//	//uint32_t amp_ad0=analogRead(A0);
	//	//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
	//}

	void process_Indent_First_SendDataThen_vibration_test()
	{
		//   //if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(1300)==false) return;
		int index = 0;
		int withdraw_delay_steps = mI_HalfNumberOfSamplingPoints;// add more step while withdrawing
		int index_store_at_max_point = 0;
		float V18_Adc_Diff = 0;
		//while(1)
#define MAX_INDENT_STEPS (SCANNER_RANGE_Z_NM*2000)// 0.05 nm/ step

#define PRE_ACC_DISTANCE (0.2)// take 4 um/20um
		float mI_step_size_pre_acceleration = mI_step_size_01 / 50000;
		float mI_step_size_pre_speed = 0;
		//float mI_step_size_pre_step=0;
		position_feedforward_output_01[PIEZO_Z] = 10.0 / SCANNER_RANGE_Z_NM;

		for (int k = 0;k < MAX_INDENT_STEPS;k++)
		{
			if (sys_state != SS_Indent) return;// to interrupt during indentation
				

			//*p_LED=1;
			toggle_pin_led();
			//while(PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(mI_LoopDelay_uS)==false)
			//	;
			//-----------motion direction 
			if (mI_direction_indent_True_withdraw_False == true)
			{
				if (position_feedforward_output_01[PIEZO_Z] > PRE_ACC_DISTANCE)
					position_feedforward_output_01[PIEZO_Z] += mI_step_size_01;
				else// acceleration, use 1000 steps to go to position PRE_ACC_DISTANCE
				{
					if (mI_step_size_pre_speed < mI_step_size_01)
						mI_step_size_pre_speed += mI_step_size_pre_acceleration;
		//mI_step_size_pre_speed=Max(mI_step_size_pre_speed,0.00001);// make sure at lease a little step
					position_feedforward_output_01[PIEZO_Z] += mI_step_size_pre_speed;
				}
			}
			else
				position_feedforward_output_01[PIEZO_Z] -= mI_step_size_01;
		//------------------------------------------- move and sensing
			piezo_predict_Position01_To_Voltage_DAC18
			(PIEZO_Z, position_feedforward_output_01[PIEZO_Z]);//,&V18_Adc[ADC_PORT_ZlOOP_SENSOR]);// time=80 uS
			wait_us(mI_LoopDelay_uS);

					//for noise rms test
			V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true);
			//use for indent
			//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,25,10);//20_10-->1.97 kHz, noise rms=   11.9427ADC18, 0.311 nm;//40_10-->1.13 kHz


					//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,5,10);// good for capture vibration noise

							//MY_Debug_StringValue_LN("v now: ",V18_Adc[ADC_PORT_ZlOOP_SENSOR]);
							//MY_Debug_StringValue_LN("v inf: ",mI_PRC_ADC_ValueInitial);
							//MY_Debug_StringValue_LN("v ff:",position_feedforward_output_01[PIEZO_Z]);
							//wait_ms(200);


									//--------------------judge status----------
			toggle_pin_led();
			// 85 us for DAC and ADC part delay_1us
			// 15 us for the following part
			V18_Adc_Diff = Math_Abs((float)V18_Adc[ADC_PORT_ZlOOP_SENSOR] - (float)mI_PRC_ADC_ValueInitial);// use Math_Abs for both direction
			mI_PRC_Force_nN_Now = V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;


			if (mI_direction_indent_True_withdraw_False == true)// judge while indenting
			{
				if (mI_PRC_Force_nN_Now >= mI_TriggerForce_nN)// reach trigger force
				{
					// old: trigger 
					//mI_direction_indent_True_withdraw_False=false;// start to withdraw
					//index_store_at_max_point=index;
					//new:  read adc again to avoid sudden noise
					wait_us(10);
					V18_Adc[ADC_PORT_ZlOOP_SENSOR] = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 20, 10);
					V18_Adc_Diff = Math_Abs((float)V18_Adc[ADC_PORT_ZlOOP_SENSOR] - (float)mI_PRC_ADC_ValueInitial);// use Math_Abs for both direction
					mI_PRC_Force_nN_Now = V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;

					if (mI_PRC_Force_nN_Now >= mI_TriggerForce_nN)
					{
						mI_direction_indent_True_withdraw_False = false;// start to withdraw
						index_store_at_max_point = index;
						
						/////////////////////////////// vibration test, stay here and record data
						for (int index = 0;index < mI_MaxStep * 3;index++)
						//mIndentData[0][index] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);//(uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX);
						{
							mIndentData[0][index] =  mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true);
							mIndentData[1][index] =  mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true);
							mIndentData[2][index] =  mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true);
						
							wait_us(mI_LoopDelay_uS);
						}
						
						break;
						//mIndentData[2][index] = (int)(position_feedforward_output_01[PIEZO_Z]*BIT18MAX);// V18_Dac[PIEZO_Z];
						
					}
				}
				if (position_feedforward_output_01[PIEZO_Z] >= mI_MaxDepth)// Math_Max indent depth without touching, withdraw to 0 directly
					break;	

			}		
			else//if	(mI_direction_indent_True_withdraw_False==false)// judge while withdraw
			{	
				withdraw_delay_steps--;// start delay counter
				if (withdraw_delay_steps <= 0) break;
				if (position_feedforward_output_01[PIEZO_Z] <= 0)// Z piezo to end		
					break;		// finished
		//// previous indent data not to be covered
		//int dis_in_circle_clockwise=index_store_at_max_point-index;
		//if (dis_in_circle_clockwise<0) dis_in_circle_clockwise+=mI_MaxStep;
		//if (dis_in_circle_clockwise<mI_HalfNumberOfSamplingPoints)
		//	break;// withdraw data are too much and write over indent part.
			}
		}// fore loop

			//------------------------------------------------------------------
			// finished indentation, send data back to PC now
		// withdraw Z piezo, and continue to send data in SS_Indent state
		console_ResetScannerModel(PIEZO_Z);//console_WithDrawZScanner_SetSystemIdle();
		int index_send = 0;
		for (int k = 0;k < mI_MaxStep;k++)
		//for (int k = (index_store_at_max_point - mI_HalfNumberOfSamplingPoints);k < (index_store_at_max_point + mI_HalfNumberOfSamplingPoints);k++)	
		{
			if (sys_state != SS_Indent) return;// to interrupt during indentation
			// keep the order
			//index_send=k%mI_MaxStep; has error
			index_send = MOD_range(k, mI_MaxStep); //here % operator has problem with int
			send_IndentData_Package_To_PC(mIndentData[0][index_send], mIndentData[1][index_send], mIndentData[2][index_send]);
			wait_ms(7);//old delay=4 , but lose point sometimes
		}
		for (int m = 0;m < 4;m++)// send finish signal multi times, to make sure PC receive it
		{
			wait_ms(100);
			send_IndentData_Package_To_PC(BIT24MAX, BIT24MAX, BIT24MAX);// finished
		}
		// tuning fork indent
		//uint32_t amp_ad0=analogRead(A0);
		//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
	}

	void process_Indent_First_SendDataThen()
	{	

		//   //if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(1300)==false) return;
		int index = 0;
		int withdraw_delay_steps = mI_HalfNumberOfSamplingPoints;// add more step while withdrawing
		int index_store_at_max_point = 0;
		float V18_Adc_Diff = 0;
		//while(1)
#define MAX_INDENT_STEPS (SCANNER_RANGE_Z_NM*2000)// 0.05 nm/ step

#define PRE_ACC_DISTANCE (0.2)// take 4 um/20um
		float mI_step_size_pre_acceleration = mI_step_size_01 / 50000;
		float mI_step_size_pre_speed = 0;
		//float mI_step_size_pre_step=0;
		position_feedforward_output_01[PIEZO_Z] = 10.0 / SCANNER_RANGE_Z_NM;

		for (int k = 0;k < MAX_INDENT_STEPS;k++)
		{
			if (sys_state != SS_Indent) return;// to interrupt during indentation
				
			*p_Tdio4 = 1;

			//while(PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(mI_LoopDelay_uS)==false)
			//	;
			//-----------motion direction 
			if (mI_direction_indent_True_withdraw_False == true)
			{
				if (position_feedforward_output_01[PIEZO_Z] > PRE_ACC_DISTANCE)
					position_feedforward_output_01[PIEZO_Z] += mI_step_size_01;
				else// acceleration, use 1000 steps to go to position PRE_ACC_DISTANCE
				{
					if (mI_step_size_pre_speed < mI_step_size_01)
						mI_step_size_pre_speed += mI_step_size_pre_acceleration;
		//mI_step_size_pre_speed=Max(mI_step_size_pre_speed,0.00001);// make sure at lease a little step
					position_feedforward_output_01[PIEZO_Z] += mI_step_size_pre_speed;
				}
			}
			else
				position_feedforward_output_01[PIEZO_Z] -= mI_step_size_01;
		//------------------------------------------- move and sensing
		//V18_Dac[PIEZO_Z]=
			piezo_predict_Position01_To_Voltage_DAC18
			(PIEZO_Z, position_feedforward_output_01[PIEZO_Z]);//,&V18_Adc[ADC_PORT_ZlOOP_SENSOR]);// time=80 uS
			wait_us(mI_LoopDelay_uS);

					//for noise rms test
			V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true);
			//use for indent
			//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,25,10);//20_10-->1.97 kHz, noise rms=   11.9427ADC18, 0.311 nm;//40_10-->1.13 kHz


					//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,5,10);// good for capture vibration noise

							//MY_Debug_StringValue_LN("v now: ",V18_Adc[ADC_PORT_ZlOOP_SENSOR]);
							//MY_Debug_StringValue_LN("v inf: ",mI_PRC_ADC_ValueInitial);
							//MY_Debug_StringValue_LN("v ff:",position_feedforward_output_01[PIEZO_Z]);
							//wait_ms(200);


									//--------------------judge status----------
			
			// 85 us for DAC and ADC part delay_1us
			// 15 us for the following part
			V18_Adc_Diff = Math_Abs((float)V18_Adc[ADC_PORT_ZlOOP_SENSOR] - (float)mI_PRC_ADC_ValueInitial);// use Math_Abs for both direction
			mI_PRC_Force_nN_Now = V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;

					// store data
					//index=k%mI_MaxStep;
			index = MOD_range(k, mI_MaxStep); //here % operator has problem with int
			mIndentData[0][index] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);//(uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX);
			mIndentData[1][index] = V18_Adc[ADC_PORT_ZlOOP_SENSOR];
			mIndentData[2][index] = V18_Dac[PIEZO_Z];

			if (mI_direction_indent_True_withdraw_False == true)// judge while indenting
			{
				if (mI_PRC_Force_nN_Now >= mI_TriggerForce_nN)// reach trigger force
				{
					// old: trigger 
					//mI_direction_indent_True_withdraw_False=false;// start to withdraw
					//index_store_at_max_point=index;
					//new:  read adc again to avoid sudden noise
					wait_us(10);
					V18_Adc[ADC_PORT_ZlOOP_SENSOR] = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 20, 10);
					V18_Adc_Diff = Math_Abs((float)V18_Adc[ADC_PORT_ZlOOP_SENSOR] - (float)mI_PRC_ADC_ValueInitial);// use Math_Abs for both direction
					mI_PRC_Force_nN_Now = V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;

					if (mI_PRC_Force_nN_Now >= mI_TriggerForce_nN)
					{
						mI_direction_indent_True_withdraw_False = false;// start to withdraw
						index_store_at_max_point = index;
					}
				}
				if (position_feedforward_output_01[PIEZO_Z] >= mI_MaxDepth)// Math_Max indent depth without touching, withdraw to 0 directly
					break;	

			}		
			else//if	(mI_direction_indent_True_withdraw_False==false)// judge while withdraw
			{	
				withdraw_delay_steps--;// start delay counter
				if (withdraw_delay_steps <= 0) break;
				if (position_feedforward_output_01[PIEZO_Z] <= 0)// Z piezo to end		
					break;		// finished
		//// previous indent data not to be covered
		//int dis_in_circle_clockwise=index_store_at_max_point-index;
		//if (dis_in_circle_clockwise<0) dis_in_circle_clockwise+=mI_MaxStep;
		//if (dis_in_circle_clockwise<mI_HalfNumberOfSamplingPoints)
		//	break;// withdraw data are too much and write over indent part.
			}
			
			*p_Tdio4 = 0;
		}// fore loop

			//------------------------------------------------------------------
			// finished indentation, send data back to PC now
		// withdraw Z piezo, and continue to send data in SS_Indent state
		console_ResetScannerModel(PIEZO_Z);//console_WithDrawZScanner_SetSystemIdle();
		int index_send = 0;
		//for (int k=0;k<mI_MaxStep;k++)
		for (int k = (index_store_at_max_point - mI_HalfNumberOfSamplingPoints);k < (index_store_at_max_point + mI_HalfNumberOfSamplingPoints);k++)	
		{
			*p_Tdio5 = 1;
			if (sys_state != SS_Indent) return;// to interrupt during indentation
			// keep the order
			//index_send=k%mI_MaxStep; has error
			index_send = MOD_range(k, mI_MaxStep); //here % operator has problem with int
			send_IndentData_Package_To_PC(mIndentData[0][index_send], mIndentData[1][index_send], mIndentData[2][index_send]);
			wait_ms(7);//old delay=4 , but lose point sometimes
			*p_Tdio5 = 0;
		}
		for (int m = 0;m < 4;m++)// send finish signal multi times, to make sure PC receive it
		{
			wait_ms(100);
			send_IndentData_Package_To_PC(BIT24MAX, BIT24MAX, BIT24MAX);// finished
		}
		// tuning fork indent
		//uint32_t amp_ad0=analogRead(A0);
		//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
	}
	
	

	
		
	void process_data_capture_blocking(float time_delay)
	{
		for (int k = 0;k < mI_MaxStep;k++)
		{
			wait_us(time_delay);
			toggle_pin_p(p_Tdio4);
			send_system_package_to_PC(20,
				k,
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, false),
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, true),
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false),
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false),
				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));			
		}
	}
	
	
	
	
	void process_data_capture_blocking_old(float time_delay)
	{
		for (int k = 0;k < mI_MaxStep;k++)
		//int k=0;
		//while(1)
		{
			//if (sys_state != SS_DataCapture) return;// to interrupt during indentation
			//*p_Tdio4 = 1;
			wait_us(time_delay);
			toggle_pin_p(p_Tdio4);
			mIndentData[0][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);// 7.84 kHz-->6.076kHz
			mIndentData[1][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false);
			mIndentData[2][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);	

//			mIndentData[0][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, true);//4.78 kHz
//			mIndentData[1][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false);
//			mIndentData[2][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false);	
			
//			mIndentData[0][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, true);//4.78 kHz-->11.934kHz
//			mIndentData[1][k] = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);
//			mIndentData[2][k] = k;//mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false);			 
			//*p_Tdio4 = 0;
			
		}
		int index_send = 0;
		for (int k = 0;k < mI_MaxStep;k++)
		{
			*p_Tdio5 = 1;
			//if (sys_state != SS_DataCapture) return;// to interrupt during indentation
			index_send = MOD_range(k, mI_MaxStep); //here % operator has problem with int
			send_IndentData_Package_To_PC(mIndentData[0][index_send], mIndentData[1][index_send], mIndentData[2][index_send]);
			wait_us(1000);//7ms old delay=4 , but lose point sometimes
			*p_Tdio5 = 0;
		}
		for (int m = 0;m < 4;m++)// send finish signal multi times, to make sure PC receive it
		{
			wait_ms(100);
			send_IndentData_Package_To_PC(BIT24MAX, BIT24MAX, BIT24MAX);// finished
		}
		// tuning fork indent
		//uint32_t amp_ad0=analogRead(A0);
		//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
	}
	//void process_Indent()
	//{
	//	if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(1300)==false) return;
	//
	//	if (mI_direction_indent_True_withdraw_False==true)
	//		position_feedforward_output_01[PIEZO_Z]+=mI_step_size_01;
	//	else
	//		position_feedforward_output_01[PIEZO_Z]-=mI_step_size_01;
	//
	//	V18_Dac[PIEZO_Z]=piezo_predict_Position01_To_Voltage_DAC18
	//		(PIEZO_Z,position_feedforward_output_01[PIEZO_Z]);
	//	V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//	mI_PRC_Force_nN_Now= (V18_Adc[ADC_PORT_ZlOOP_SENSOR]-mI_PRC_ADC_ValueInitial)*mI_PRC_Force_nN_per_ADCValue;
	//
	//	if (
	//		(mI_PRC_Force_nN_Now>=mI_TriggerForce_nN)// assume adc value increase while indentation
	//		||
	//		(position_feedforward_output_01[PIEZO_Z]>=mI_MaxDepth)
	//		)
	//		mI_direction_indent_True_withdraw_False=false;// start to withdraw
	//
	//	if	(// finished
	//		// adc value return to initial value
	//			(V18_Adc[ADC_PORT_ZlOOP_SENSOR]<(mI_PRC_ADC_ValueInitial-mI_PRC_ADCValue_per_nm*0.1))
	//			||// logic or
	//			(position_feedforward_output_01[PIEZO_Z]<=0)// Z piezo to end		
	//			)		
	//	{
	//		send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX)
	//			,V18_Adc[ADC_PORT_ZlOOP_SENSOR],BIT24MAX);// finished
	//		console_WithDrawZScanner_SetSystemIdle();
	//	}
	//
	//	send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX)
	//		,V18_Adc[ADC_PORT_ZlOOP_SENSOR],V18_Dac[PIEZO_Z]);
	//
	//
	//	// tuning fork indent
	//	//uint32_t amp_ad0=analogRead(A0);
	//	//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
	//
	//}

//	int rtos_send_image_frame_to_PC()
//	{
//		if (pointer_out_frame_buffer < LENGTH_IMAGE_FRAME_BUFFER)
//		{
//			mUSerial.write(com_image_frame_buffer[pointer_out_frame_buffer]);
//			pointer_out_frame_buffer++;
//			
//		}
//		return pointer_out_frame_buffer;
//	}
//
//	void Serial_write_reset_input_output()
//	{rtos_buffer_in_index = 0;pointer_out_frame_buffer = 0;}
//
//	void Serial_write(byte *d, int L)
//	{
//		for (int k = 0;k < L;k++)
//			Serial_write(d[k]);
//	}
//	inline void Serial_write(byte d)
//	{
//		com_image_frame_buffer[rtos_buffer_in_index] = d;
//		rtos_buffer_in_index++;
//	}
	/////////////////////////////////////////////////////////////////////
	void calculate_scan_parameter()
	{
		XL = XL_NM * DAC_PER_NM_X, DX = DX_NM * DAC_PER_NM_X, 
		YL = YL_NM * DAC_PER_NM_Y, DY = DY_NM * DAC_PER_NM_Y;
	
		const float VADC_Ref_V = 5.0;	

//		VWset_deltaV_ADC_b18 = (float)(mZLoopPID_WorkingDistance_nm) / 1000.0 / VADC_Ref_V*BIT18MAX;
		VWset_deltaV_ADC_b18 = (float)(mZLoopPID_WorkingDistance_nm) *DTS_Sensitivity_B18_per_nm;

		float VdeltaF_FarAway_01 = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 30, 100) / BIT18MAX;
		

		
		Vdf_infinite = BIT18MAX_HALF;
		
#if (ADC_PORT_ZlOOP_SENSOR==ADC_CHANNEL_PRC)
//#define PRC_sensitivity_ADC18_per_nm (41.2)
		//threshold_approach_delta_B18 = PRC_sensitivity_ADC18_per_nm * mZLoopPID_WorkingDistance_nm;
		threshold_approach_delta_B18 = DTS_Sensitivity_B18_per_nm * mZLoopPID_WorkingDistance_nm;//7nm  threashold to avoid peak-to-peak noise //112*5;// vpp*5 102*4;//87*11;//;3nm112*3;
#else
		threshold_approach_delta_B18 = (150.0 / SCANNER_RANGE_Z_NM*BIT18MAX);//1638;//0.025V/4*bit18=5nm;
#endif		
		
#if(ADC_PORT_ZlOOP_SENSOR==ADC_CHANNEL_PRC)
			// for PRC, use mZLoopPID_WorkingDistance_nm as nm,
		float mZLoopPID_WorkingDistance_01 = (float)(mZLoopPID_WorkingDistance_nm) * DTS_Sensitivity_B18_per_nm / BIT18MAX;
		mZLoopPID_WorkingDistance_Threshold_01 = mZLoopPID_WorkingDistance_01 * 10;
		float DSet_01 = VdeltaF_FarAway_01 - mZLoopPID_WorkingDistance_01;
#else	// tuning fork
		float DSet_01 = VdeltaF_FarAway_01 - (float)(mZLoopPID_WorkingDistance_nm) / 1000.0 / VADC_Ref_V;
#endif	
		mPID_ZLOOP->SetReferenceValue(DSet_01);
		mPID_ZLOOP->SetSampleTime(mPeriod_Realtime_us);		
		float zpid_limit = MAX_STEP_SIZE_PIEZO_MODEL_01;
		mPID_ZLOOP->SetStepSize(zpid_limit);// limit the change in each period
		mPID_ZLOOP->SetOutputLimits(0, 1); 
//		mPID_ZLOOP->SetPID_P(0.004);//(0.001
//		mPID_ZLOOP->SetPID_I(0.001);//(0.0001
		
// 0.2~0.5 line/s		
		mPID_ZLOOP->SetPID_P(0.01);//use P=0.01, I=0.002 OK, 20160416
		mPID_ZLOOP->SetPID_I(0.002);//(0.0001
		mPID_ZLOOP->SetPID_D(0);
		
//// 1 line/s		
//		mPID_ZLOOP->SetPID_P(0.05);//use P=0.01, I=0.002 OK, 20160416
//		mPID_ZLOOP->SetPID_I(0.009);//(0.0001
//		mPID_ZLOOP->SetPID_D(0);
// 2 line/s		
//		mPID_ZLOOP->SetPID_P(0.09);//use P=0.01, I=0.002 OK, 20160416
//		mPID_ZLOOP->SetPID_I(0.02);//(0.0001
//		mPID_ZLOOP->SetPID_D(0);	
		
	}

	int XYScanning_y_reverse(int u)
	{
		static int t = 0, yt = -1;
		if (u == 0 && t == 1)
		{
			if (yt == 1)
				yt = -1;
			else
				yt = 1;
		}
		t = u;
		//y=yt;
		return yt;
	}

	int XYscanning()
	{
		diry_input =  XYScanning_y_reverse(diry_input);
		//  static int px = XL, py = YL;
		//	static int sawx = XL, sawy = YL;
		static int frame_counter = 0;

			// output
		VDACx = 0;
		VDACy = 0;
		indx = 0;
		indy = 0;
		int Nx = (int)((float)SamplingFrequency_ZLoop_Hz / scan_rate);

			//mUSerial.print("Nx"); 
			//mUSerial.println(Nx,DEC);
			//int Nx = SamplingFrequency_ZLoop_Hz * scan_second_per_line;
		int Ny = N_y;
		diry_input = -diry_input;
		static int sx = -XL * Nx / DX, sy = -YL * Ny / DY, dirx = 1, diry_core = diry_input; // for index kernel
		//  static int dds_reset_old = 0;

			//// sys_ini=0;// 0:initial, power on
			//const int DDS_XY_Reset = 0; // slow move;
			//const int DDS_XY_Scan = 1; // scan move;
			//const int DDS_XY_Idle = 2; //  idle

				//static int mDDS_XY_Scanner_State = DDS_XY_Reset; // the mDDS_XY_Scanner_State of the system

					////////////////////////////////////////DDS_XY_Idle
		if (mDDS_XY_Scanner_State == DDS_XY_Idle)
		{
			if (mode_pause0_scan1_pending2 == 1 && dds_reset == 0)
				mDDS_XY_Scanner_State = DDS_XY_Scan;

			if (dds_reset == 1)
				mDDS_XY_Scanner_State = DDS_XY_Reset;
		}
		//////////////////////////////// DDS_XY_Scan
#pragma region   DDS_XY_Scan

		if (mDDS_XY_Scanner_State == DDS_XY_Scan)
		{
			if (mode_pause0_scan1_pending2 == 0)
				mDDS_XY_Scanner_State = DDS_XY_Idle;

			if (dds_reset == 1)
				mDDS_XY_Scanner_State = DDS_XY_Reset;

						// do scan
			if (frame_counter < N_FramesToScan && enable == 1)
				sx += dirx; // kernel tick

			if (sx == 0 && dirx == -1 && y_enable == 1)
				sy += diry_core * diry_input;

			if (sx == 0)
				dirx = 1;

			if (sx == Nx)
				dirx = -1;

			if (sy == 0)
				diry_core = diry_input;

			if (sy == Ny)// eg. 128
			{
				if (diry_core == diry_input) // to avoid continue adding
				{
					frame_counter = frame_counter + 1;
					if (frame_counter >= N_FramesToScan)// finished predefined frames, stop scanning
						console_XYScanReset();
				}
				diry_core = -diry_input;
			}

		} // scan
#pragma endregion
		////////////////////////////////////////////// DDS_XY_Reset
#pragma region// DDS_XY_Reset
		if (mDDS_XY_Scanner_State == DDS_XY_Reset)
		{ 
			if (sx == 0 && sy == 0)
			{
				mDDS_XY_Scanner_State = DDS_XY_Idle;
				dds_reset = 0;
			}

					// reset  slow move
			frame_counter = 0;
			dirx = 1;
			diry_core = diry_input;

					//mUSerial.println("sxsy:");
					//mUSerial.println(sx,DEC);
					//mUSerial.println(sy,DEC);
			int  stepx = Math_Max(1, Math_Abs(sx) >> 12)*SIGN(sx); //sx/3000
			int  stepy = Math_Max(1, Math_Abs(sy) >> 12)*SIGN(sy); // xy/3000
			if (enable == 1)
			{
				if (sx != 0)
					sx = sx - stepx;

				if (sy != 0)
					sy = sy - stepy;

			}
			if (Math_Abs(sx) < 1) sx = 0;
			if (Math_Abs(sy) < 1) sy = 0;
		}// reset
#pragma endregion
		///////// DAC output
		VDACx = XL + DX * sx / Nx;
		VDACy = YL + DY * sy / Ny;

			//if (diry_input == 1)
			//{
			//if (dirx == 1)
			//indsx = sx >> 1;
			//else
			//indsx = ((Nx << 1) - sx) >> 1;
			//}
			//else
			//{
			//if (dirx == -1)
			//indsx = sx >> 1;
			//else
			//indsx = ((Nx << 1) - sx) >> 1;
			//}

				////    indsx=(sx*float((dirx==-1))+(2*Nx-sx)*float((dirx==1)))/2;

					//VDACy = YL + DY * (sy * Nx + indsx) / Ny / Nx; // smooth Y axis
					//

						//     px=VDACx;
						//     py=VDACy;
						// sawx sawy has sign to show its direction
		indx = sx * N_x / Nx * dirx; // sawx has floating point number
		indy = sy * diry_core;
		//sawx=round(sawx);
		//sawy=round(sawy);
		//sys_state_out = mDDS_XY_Scanner_State;
		//SX = sx;
		//SY = sy;


//		piezo_predict_Position01_To_Voltage_DAC18(PIEZO_X, (float)VDACx / (float)BIT18MAX);	
//		piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Y, (float)VDACy / (float)BIT18MAX);	
		
		mCScanner[PIEZO_X].MoveToPosition01((_Float_)VDACy / (_Float_)BIT18MAX);	// only set position reference, real move be be done in AFM_ProcessScheduler_Realtime while calculate PID
		mCScanner[PIEZO_Y].MoveToPosition01((_Float_)VDACx / (_Float_)BIT18MAX);	

		// pid update and DAC move for XY scanner
		
		{
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, mCScanner[PIEZO_Y].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true)));// 3.745 kHz
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false)));
		}	
		
		return mDDS_XY_Scanner_State;
	}
	void XYscanning_Initialize()
		// also call initialize when change dds parameters XL YL DX DY
	{
		dds_reset = 1;
		mode_pause0_scan1_pending2 = 0;

		sys_state = SS_XYScanReset;
		// put this into main loop, execute once each loop
		//for (int k=0;k<50;k++)
		//{XYscanning();wait_us(500);}
	}
	void process_XYscanningReset()
	{
		*p_Tdio3 = 1;
		int state = XYscanning();
		wait_ms(1);// otherwise, it is too fast
		if (state == DDS_XY_Idle)
			sys_state = SS_Idle;
		*p_Tdio3 = 0;

	}
	//  int indx = 0, indy = 0, VDACx = 0, VDACy = 0, sys_state_out = 0, SX = 0, SY = 0;
	////float DInput_01=-1000;//% input from ADC, voltage for delta frequency
	////float DSet_01=0;// set working voltage
	////float DOutput_01=0;

	void process_ScanRealTimeLoop()
	{
		*p_Tdio2 = 1;


//		V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR,true);
//		
//		DInput_01 = (float)V18_Adc[ADC_PORT_ZlOOP_SENSOR]*BIT18MAX_RECIPROCAL;// normalize
//		DOutput_01 = mPID_ZLOOP->ComputePI(DInput_01);
//		piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z, z_output_01);
		static bool mHoldXYScanner = true;
		float distance_tip_sample_01 = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR, true)*BIT18MAX_RECIPROCAL;
		float z_actuation_position_01 = mPID_ZLOOP->ComputePI(distance_tip_sample_01);
		mCScanner[PIEZO_Z].MoveToPosition01(z_actuation_position_01);
		
		piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z, z_actuation_position_01);
		
		float distance_tip_sample_error = mPID_ZLOOP->GetError();
//		mHoldXYScanner = true;// for test only  to test zero scan
		mHoldXYScanner = Math_Abs(distance_tip_sample_error) > mZLoopPID_WorkingDistance_Threshold_01;
		int	xy_state = 0;
		if (mHoldXYScanner == false)// 2.8 kHz, if do not move XY, 10.78 kHz
			xy_state = XYscanning();

		float Z_sensor_height01 = mCScanner[PIEZO_Z].GetSensorPosition01(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));		
		
//		if (xy_state == (int)DDS_XY_Scan)
		//send_image_package_to_PC(indx, indy, Z_sensor_height01, distance_tip_sample_error);//z_output_01
		send_image_package_to_PC_dac(indx, indy, Z_sensor_height01, distance_tip_sample_error, mAFM_DAC.ReadDACValue(PIEZO_Z));

		//send_image_package_to_PC_direct(indx, indy, Z_sensor_height, mPID_ZLOOP->GetError());//z_output_01
		//		else// after engage send out (0,0) point continuously
		//			send_engaged_package_to_PC(0, 0,Z_sensor_height01,z_actuation_position_01); // mPID_ZLOOP->GetError()

		*p_Tdio2 = 0;		
	}
			//mUSerial.print("_zoutput_01 accumu ");
			//mUSerial.println(z_output_01,DEC);
			//V18_Dac[PIEZO_Z]=(DOutput_01/(float)MAX_RANGE_Z_PM)*(float)BIT18MAX;
			//piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,(float)V18_Dac[PIEZO_Z]/(float)BIT18MAX);
			//float z_output_01=Z_position_pm/(float)MAX_RANGE_Z_PM;	

				//// for piezo_z and tuning fork switch
				//if (Z_scanner_port==PIEZO_Z)
				//	V18_Dac[PIEZO_Z]=piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,z_output_01);
				//else
				//{
				//	V18_Dac[Z_scanner_port]=(z_output_01-PIEZO_T_Center01)*BIT18MAX+PIEZO_T_CenterBit18;
				//	mAFM_DAC.DAC_write(PIEZO_T,V18_Dac[Z_scanner_port]);
				//}

		
////		if (Z_scanner_port == SCANNER_Z_ONLY)// Z piezo only with 
////			//V18_Dac[PIEZO_Z]=
////			piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z, z_output_01);
////		else
////			if (Z_scanner_port == SCANNER_Z_LPF)// Z piezo only with LPF
////		{
////			float z_output_01_lpf = IIR_filter_LPF(z_output_01);
////			//V18_Dac[PIEZO_Z]=
////			piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z, z_output_01_lpf);
////		}
////		else
////			if (Z_scanner_port == SCANNER_T_ONLY)// switched
////		{	
////			V18_Dac[PIEZO_T] = (z_output_01*mTF_DC_Gain + PIEZO_T_Center01)*BIT18MAX;
////		}
////		else
////			if (Z_scanner_port == SCANNER_ZT)// switched
////		{
////			float z_output_01_hpf = IIR_filter_HPF(z_output_01);
////			V18_Dac[PIEZO_T] = (z_output_01_hpf*mTF_DC_Gain + PIEZO_T_Center01)*BIT18MAX;
////			V18_Dac[PIEZO_T] = LIMIT_MAX_MIN(V18_Dac[PIEZO_T], 1, BIT18MAX);
////			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_T, V18_Dac[PIEZO_T]);
////
////			float z_output_01_lpf = IIR_filter_LPF(z_output_01);
////			//V18_Dac[PIEZO_Z]=
////			piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z, z_output_01_lpf);
////		}
		//static int c=0;
		//c=LIMIT_MAX_MIN(c++,100000,0);

						//if (c==0)
						//	{
						//		MY_Debug("L: ");
						//MY_Debug_LN(z_output_01_lpf);
						//MY_Debug("H: ");
						//MY_Debug_LN(z_output_01_hpf);
						//}

										//

														//V18_Dac[Z_scanner_port]=piezo_predict_Position01_To_Voltage_DAC18(Z_scanner_port,z_output_01);


	
						////TICX(2);
						////store image
						//int current_image_line=indy;
						//current_image_line%=SIZE_IMAGE_BUFFER_LINES;
						////byte f0b1=0;
						////if (indx>=0) f0b1=0;
						////if (indx<0)  f0b1=1;
						//byte valueH3b[SIZE_IMAGE_BUFFER_BIT24]={0};
						//uint32_t vH24=(uint32_t)(z_output_01*(float)BIT24MAX);
						//convert_uint32_to_byte3(vH24,valueH3b);

										//byte valueE3b[SIZE_IMAGE_BUFFER_BIT24]={0};
										//uint32_t vE24=(uint32_t)(mPID_ZLOOP->GetError()*(float)BIT24MAX);
										//convert_uint32_to_byte3(vE24,valueE3b);
										////6 us
										//for (int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
										//{
										//	if (indx>=0) 
										//	{
										//		pImageHF[k][current_image_line][Math_Abs(indx)]=valueH3b[k];
										//		pImageEF[k][current_image_line][Math_Abs(indx)]=valueE3b[k];
										//	}
										//	else
										//	{
										//		pImageHB[k][current_image_line][Math_Abs(indx)]=valueH3b[k];
										//		pImageEB[k][current_image_line][Math_Abs(indx)]=valueE3b[k];
										//	}

														//}// 7us
														////TOCX_P(2);
	



	float convert_byte4_to_float(byte* pb)
		// accuracy 10^-4
	{
		float* pf = (float*)(pb);
		float fx = *pf;
		return fx;
		//float x=4.5f;
		//byte b[4]={102,64,156,197};
		//float xx=10000.565556546551f;//convert result--->10000.5654296875, accuracy 10^-4
		//byte *pb=(byte*)&xx;
		//float x=convert_byte4_to_float(pb);
		//mUSerial.println(x,20);
		//wait_ms(1000);
		//return;
	}
	void console_Parameter(byte* com_buffer_frame)
	{

		//int v=0;
		//v+= com_buffer_frame[2]<<24;
		//v+= com_buffer_frame[3]<<16;
		//v+= com_buffer_frame[4]<<8;	
		//v+= com_buffer_frame[5];
		//float vf=(float)v/16384.0;// 131071.0;//2^17
		float fv = convert_byte4_to_float(&com_buffer_frame[2]);
		float vf = (float)fv;
		byte para = com_buffer_frame[1];
//		MY_Debug(para);
//		MY_Debug_LN(vf);	
		// PID parameter
		if (para == 'R') {DTS_Sensitivity_B18_per_nm = vf;}
		
		if (para == 'P') mPID_ZLOOP->SetPID_P(vf);
		if (para == 'I') mPID_ZLOOP->SetPID_I(vf);
		if (para == 'D') mPID_ZLOOP->SetPID_D(vf);
//		if (para == 'P') mCScanner[PIEZO_Z].mPID_Scanner->SetPID_P(vf);
//		if (para == 'I') mCScanner[PIEZO_Z].mPID_Scanner->SetPID_I(vf);
//		if (para == 'D') mCScanner[PIEZO_Z].mPID_Scanner->SetPID_D(vf);
		
//		mUSerial.print(para);
//		mUSerial.println(vf);
//		if (para == 'P') mPID_ZLOOP->SetPID_P(vf*DTS_Sensitivity_B18_per_nm);
//		if (para == 'I') mPID_ZLOOP->SetPID_I(vf*DTS_Sensitivity_B18_per_nm);
//		if (para == 'D') mPID_ZLOOP->SetPID_D(vf*DTS_Sensitivity_B18_per_nm);

			// when set:DTS_Sensitivity_B18_per_nm, also recalculate pid_input_Gain_adjustm, DSet_01
		if (para == 'X') {console_XYScanReset();N_x = vf;}
		if (para == 'Y') {console_XYScanReset();N_y = vf;}
		if (para == 'x') {console_XYScanReset();DX_NM = vf;}
		if (para == 'y') {console_XYScanReset();DY_NM = vf;}
		if (para == 'm') {console_XYScanReset();XL_NM = vf;console_XYScanReset();}
		if (para == 'n') {console_XYScanReset();YL_NM = vf;console_XYScanReset();}

		if (para == 'S') {console_XYScanReset();scan_rate = vf * 2;}
		if (para == 'W') {mZLoopPID_WorkingDistance_nm = vf;calculate_scan_parameter();}
		if (para == 'N') N_FramesToScan = vf;
		//indentation
		if (para == 'T') mI_TriggerForce_nN = vf;
		if (para == 'k') mI_StiffnessPRC_nN_per_nm = vf;
		if (para == 's') mI_PRC_ADCValue_per_nm = vf;// sensitivity
		if (para == 'd') mI_MaxDepth = vf / SCANNER_RANGE_Z_NM;
		if (para == 'e') mI_step_size_nm = vf;
		if (para == 'f') mI_LoopDelay_uS = vf;
		if (para == 'g') mI_HalfNumberOfSamplingPoints = vf / 2;

			//if(para=='t') mTF_DC_Gain=vf;

		if (para == 'a') {sys_idle_package_index = vf;}
				//mUSerial.write(&para,1);
				//mUSerial.println(v,DEC);
	}
	byte console_DRpot(byte *com_buffer_frame)
	{
		//AA 55 52 00 05 01 00 00 55 AA 
		//AA 55 52 00 03 C8 00 00 55 AA 
		//AA 55 52 00 03 79 00 00 55 AA 
		byte address_IC = com_buffer_frame[1];
		byte address_channel = com_buffer_frame[2];
		byte R_value = com_buffer_frame[3];

		//byte  mDDS_XY_Scanner_State = write_digital_potentiometer(address_IC, address_channel, R_value);

		mUSerial.print(address_IC, DEC);
		mUSerial.print(address_channel, DEC);
		mUSerial.println(R_value, DEC);
		//DIGITAL_PIN_TOGGLE(23);
		return mDDS_XY_Scanner_State;
	}

	void console_PiezoFeedforward_output(byte* com_buffer_frame)
		//AA 55 44 FF 00 03 FF FF 55 AA
		//AA 55 44 00 00 01 00 00 55 AA
	{
		byte channel = com_buffer_frame[1];	
		uint32_t value = 0;
		value += com_buffer_frame[2] << 24;
		value += com_buffer_frame[3] << 16;
		value += com_buffer_frame[4] << 8;
		value += com_buffer_frame[5];
		//value=Min(value,BIT18MAX);
		//value=Max(value,0);

		float position_output_01 = (float)value / BIT32MAX;	
		
		mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 1;
		if (channel < NUM_OF_SCANNER)
		{
			piezo_predict_Position01_To_Voltage_DAC18(channel, position_output_01);
			position_feedforward_output_01[channel] = position_output_01;
		}
		if (channel == NUM_OF_SCANNER)// set value for all channel
			for (int i = 0; i < NUM_OF_SCANNER; i++)
			{
				//V18_Dac[i]=
				piezo_predict_Position01_To_Voltage_DAC18(i, position_output_01);
				position_feedforward_output_01[i] = position_output_01;
			}
		else
		/////////////////////////////////////////////////////
		if (channel >= 100)// set value for all channel
		{
			mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 2;
			channel -= 100;
			channel %= NUM_OF_SCANNER + 1;// for safety
			
			
			if (channel < NUM_OF_SCANNER)
				mCScanner[channel].MoveToPosition01(position_output_01);
			if (channel == NUM_OF_SCANNER)// set value for all channel
				for (int i = 0; i < NUM_OF_SCANNER; i++)
				{
					mCScanner[i].MoveToPosition01(position_output_01);
				}
		}

		
		
	}

	void console_DAC_output(byte* com_buffer_frame)
		//AA 55 44 FF 00 03 FF FF 55 AA
		//AA 55 44 00 00 01 00 00 55 AA
	{
		byte channel = com_buffer_frame[1];	
		uint32_t value = 0;
		value += com_buffer_frame[2] << 24;
		value += com_buffer_frame[3] << 16;
		value += com_buffer_frame[4] << 8;
		value += com_buffer_frame[5];
		value = Min(value, BIT18MAX);
		value = Max(value, 0);

//		if (channel == PIEZO_Z)
//			V18_Dac[PIEZO_Z] = value;// hold the value in Z axis
		if (channel < NUM_OF_SCANNER)
			mAFM_DAC.DAC_write(channel, value);
		if (channel == NUM_OF_SCANNER)// set value for all channel
		{
			mAFM_DAC.DAC_write(0, value);
			mAFM_DAC.DAC_write(1, value);
			mAFM_DAC.DAC_write(2, value);
//			mAFM_DAC.DAC_write(3, value);
		}

		mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 0;
	}
	//void console_StartApproach_only_coarse_move()
	//{
	//	sys_state=SS_Approach;
	//	Vdf_infinite=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);	
	//	mAFM_DAC.DAC_write(PIEZO_Z,BIT18MAX);
	//	wait_ms(20);
	//
	//	//if(Vdf_infinite>127140 | Vdf_infinite<119276)//([-0.3 -0.9]/4+2.5)/5*2^18=127139.84,119275.52
	//	if(Math_Abs(Vdf_infinite-BIT18MAX_HALF)<7864)
	//	{
	//		sys_state=SS_Idle;
	//		send_back_approach_heart_beat(Vdf_infinite,0,255);//send back an package: initial vdf error
	//	}
	//}
	uint32_t ADC_read_MedianFilter(byte port, int num, int delay_time_us)
	{
		RunningMedian mSortFilter = RunningMedian(num);
		mSortFilter.clear();
		for (int k = 0;k < mSortFilter.getSize();k++)
		{
			mSortFilter.add(mAFM_SEM.ADC_Read_N(port));
			wait_us(delay_time_us);
		}
		return mSortFilter.getMedian();
	}
	uint32_t ADC_read_average(byte port, int num, int delay_time_us)
	{
		if (num == 1)
			return mAFM_SEM.ADC_Read_N(port);

		float t = 0;
		//int num=30;
		for (int k = 0;k < num;k++)
		{
			wait_us(delay_time_us);
			t += (float)mAFM_SEM.ADC_Read_N(port);
		}
		t /= (float)num;
		return (uint32_t)t;
	}


	void console_StartApproach()
	{
		//mUSerial.println(sys_state,DEC);
		if (sys_state == SS_Approach) 
		{
			//mTimer_Approach.start();
			return;// avoid multi trigger from GUI
		}
		
		//mAFM_DAC.DAC_write(PIEZO_Z,0);	
		console_ResetScannerModel(PIEZO_Z);

		wait_ms(50);
		//ADC_sensor_buffer = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
		Vdf_infinite = ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR, 5, 10);// ADC_read_average(ADC_PORT_ZlOOP_SENSOR,10,50);
		ADC_sensor_buffer = Vdf_infinite;
		//ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR,20,100);// coarse positioner could vibrate
		//ADC_read_average(ADC_PORT_ZlOOP_SENSOR,100,100);;

			//while(1)
			//{Vdf_infinite=(int)mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);	
			//	mUSerial.println(Vdf_infinite);
			//	wait_ms(10);
			//}
			//	counter_large_step_approaching=0;// reset counter
			//if(Vdf_infinite>127140 | Vdf_infinite<119276)//([-0.3 -0.9]/4+2.5)/5*2^18=127139.84,119275.52
#if (ADC_PORT_ZlOOP_SENSOR==ADC_CHANNEL_PRC)
				//	if(ADC_PORT_ZlOOP_SENSOR==ADC_CHANNEL_PRC)		
		if (Math_Abs(Vdf_infinite - (int)BIT18MAX_HALF) > ((int)BIT18MAX_HALF - 5000))// leave at least 5000 ADC range for motion
		{
			console_ResetScannerModel(PIEZO_Z);
			send_back_approach_heart_beat(Vdf_infinite, 0, 255);//send back an package: initial vdf error
		}
#else
			//if(ADC_PORT_ZlOOP_SENSOR==ADC_PORT_TUNING_FORK)
		if (Math_Abs(Vdf_infinite - (int)BIT18MAX_HALF) > 7864)
		{
			console_WithDrawZScanner_SetSystemIdle();
			send_back_approach_heart_beat(Vdf_infinite, 0, 255);//send back an package: initial vdf error
		}	
#endif
			// each time reset
		step_counter_Approach = 0;
		step_size_increament_Appraoch = 1;
		Z_position_DAC_Approach = 0;
		ADC_sensor_buffer = Vdf_infinite;
		send_back_approach_heart_beat(Vdf_infinite, 0, 2);// received approach command from PC, send back Vdf_infinite
		
		sys_state = SS_Approach;
	}
	void console_CancelApproach() 
	{
		// each time reset
		step_counter_Approach = 0;
		Z_position_DAC_Approach = 0;
		console_WithDrawZScanner_SetSystemIdle();
	}

	void console_StartZScannerEngage()
	{
		console_ResetScannerModel(PIEZO_Z);
		wait_ms(100);
		Vdf_infinite = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 50, 100);
		
//		Vdf_infinite = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 50, 100)-1000;
//#warning "Vdf_infinite = ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 50, 100)-1000; for remote debug only"
		sys_state = SS_Engage;
		Z_position_DAC_ZScannerEngage = 0;
	}

	//#define console_YScan_Enable() y_enable = 1 

	inline void console_YScan_Enable(){y_enable = 1;}
	inline void console_YScan_Disable(){y_enable = 0;}

	inline void  console_XYScanReset()
	{
		console_WithDrawZScanner_SetSystemIdle();
		calculate_scan_parameter();
		XYscanning_Initialize();
	} 
	void console_XYScanStart()
	{	//if (pImage!=NULL)
		//	delete pImage;
		if (sys_state == SS_Scan)	
			//if the system was scanning, then pause by user, 
				//then recover from pause can directly go to scan
			mode_pause0_scan1_pending2 = 1;
		else
		{
			console_StartZScannerEngage();
			mode_pause0_scan1_pending2 = 2;// wait for Z scanner engage to be finished and then start to scan
		}
	}
	;
	inline void console_XYScanPause(){mode_pause0_scan1_pending2 = 0;}

	void console_GetData(byte* com)
	{
		if (com[1] == 'A')// read ADC  & com[2]=='P') 
			;
		if (com[1] == 'S' & com[2] == 'P') 
			;
//			send_back_debug_infomation();
		if (com[1] == 's' & com[2] == 'g') 
			console_ReadStrainGaugeData(com[3]);
	}
	inline void console_ReadStrainGaugeData(int value){switch_read_SG = value;}
	//void console_TF_Scan_Enable()
	//{
	//	//Z_scanner_port=PIEZO_T;
	//	V18_Dac[PIEZO_T]=PIEZO_T_CenterBit18;
	//	mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_T,V18_Dac[PIEZO_T]);
	//
	//	z_output_01=0.5;
	//	mPID_ZLOOP->Reset();
	//}
	//void console_TF_Scan_Disable()
	//{
	//	Z_scanner_port=SCANNER_Z_ONLY;
	//
	//}

	void console_Control(byte* com)
	{


		if (com[1] == 'A' & com[2] == 'P') 
			console_StartApproach();
		if (com[1] == 'A' & com[2] == 'C') 
			console_CancelApproach();

		if (com[1] == 'Z' & com[2] == 'E') 
		{
			console_WithDrawZScanner_SetSystemIdle();
			console_StartZScannerEngage();
		}
		if (com[1] == 'Z' & com[2] == 'W') 
			console_WithDrawZScanner_SetSystemIdle();

		if (com[1] == 'Y' & com[2] == 'E') 
			console_YScan_Enable();
		if (com[1] == 'Y' & com[2] == 'D') 
			console_YScan_Disable();
		//// tf scan switch with Z piezo
		//if (com[1]=='T' & com[2]=='E') 
		//	console_TF_Scan_Enable();
		//if (com[1]=='T' & com[2]=='D') 
		//	console_TF_Scan_Disable();

		if (com[1] == 'S')
		{
			if (com[2] == 'R') 
				console_XYScanReset();
			if (com[2] == 'S') 
				console_XYScanStart();
			if (com[2] == 'P') 
				console_XYScanPause();				
		}
		if (com[1] == 'I' & com[2] == 'D') 
			console_StartIndentation();
	}
	int  Communication_Command_Console()
	{
//		static bool mlock = false;
//		if (mlock ==true) return -1;
//		mlock = true;// lock

		//REGION_LOCK_RETURN0();

		bool echo_b = false;
		//int byte_ready = 0;// mUSerial.available();// time consumption= 84 us
		//if (byte_ready == 0) return 0;
		//Rval = mUSerial.read();
		memset(com_buffer, 0, LENGTH_COM_BUFFER_PC2MCU_CONSOLE);//sizeof(com_buffer)
		
		int byte_ready = mUSerial.readBytes(com_buffer, LENGTH_COM_BUFFER_PC2MCU_CONSOLE);
		
		if (byte_ready == 0) 
		{
			//mlock = false;// unlock
			return byte_ready;
		}
		//MY_Debug_StringValue_LN("byte_ready", byte_ready);	
		
		

		// if there are multi frames , the progrma can still detect it
		for (int k = 0; k < LENGTH_COM_BUFFER_PC2MCU_CONSOLE - LENGTH_COM_FRAME_PC2MCU; k++)
			//for (int k = 0; k < LENGTH_COM_FRAME_PC2MCU + 1; k++)
			if (com_buffer[k] == COM_HEADER1)
				if (com_buffer[k + 1] == COM_HEADER2)
					if (com_buffer[k + LENGTH_COM_FRAME_PC2MCU - 2] == COM_TAIL1)
						if (com_buffer[k + LENGTH_COM_FRAME_PC2MCU - 1] == COM_TAIL2)
						{
							if (echo_b == true) mUSerial.write(com_buffer, LENGTH_COM_FRAME_PC2MCU); //echo
							for (byte ind = 0; ind < LENGTH_COM_DATA_PC2MCU; ind++)
								com_buffer_frame[ind] = com_buffer[k + ind + 2];
							memset(com_buffer, 0, sizeof(com_buffer));
							if (echo_b == true) mUSerial.write(com_buffer_frame, LENGTH_COM_DATA_PC2MCU); //echo

													//AA 55,52 03 f0 00 00 00, 55 AA
							if (com_buffer_frame[0] == 'R')//'R'=0x52
								console_DRpot(com_buffer_frame);
							if (com_buffer_frame[0] == 'P')//set parameters
								console_Parameter(com_buffer_frame);
							if (com_buffer_frame[0] == 'D')//output DAC value, D=0x44
								console_DAC_output(com_buffer_frame);
							if (com_buffer_frame[0] == 'F')//feedforward position output
								console_PiezoFeedforward_output(com_buffer_frame);						

							if (com_buffer_frame[0] == 'C')
								console_Control(com_buffer_frame);

							if (com_buffer_frame[0] == 'G')
								console_GetData(com_buffer_frame);
							if (com_buffer_frame[0] == 'r' 
								& com_buffer_frame[1] == 's' 
								& com_buffer_frame[2] == 't') 
							{
								wait(3);// wait for PC software to disconnect serial port
								Software_Reset();
							}
							;
							if (com_buffer_frame[0] > 127)
								console_PC2MCU_Command(com_buffer_frame);


						}// for if,
		
		//mlock = false;// unlock
		//REGION_UNLOCK();	
		return byte_ready;//com_buffer_frame[0];
	

	}

	void console_PC2MCU_Command(byte* com)
	{
#define CMD_PC2MCU_BASE	(32768)//2^15
#define CMD_PC2MCU_DATA_CAPTURE		(CMD_PC2MCU_BASE+1)
#define CMD_PC2MCU_WAVE_TEST		(CMD_PC2MCU_BASE+2)

		int cmd_value = com[0];
		cmd_value <<= 8;
		cmd_value += com[1];
		float cmd_dataf = convert_byte4_to_float(&com_buffer_frame[2]);// use com[2345]as data 32bit

		switch (cmd_value)
		{
		case  CMD_PC2MCU_DATA_CAPTURE:	console_PC2MCU_StartCaptureData(cmd_dataf);	break;
		case  CMD_PC2MCU_WAVE_TEST:	console_PC2MCU_StartWaveTest(cmd_dataf);	break;

		default: break;
		}

			
	}
	void console_PC2MCU_StartWaveTest(float data)
	{
		sys_state = SS_WaveTest;
	}	
	void console_PC2MCU_StartCaptureData(float data)
	{
		//sys_state = SS_DataCapture;
		process_data_capture_blocking(data);
	}
	//////////////////////// console end
	//void test()
	//{
	//	mUSerial.print("T");
	//	mUSerial.println(TOC(),DEC);
	//
	//	TIC();
	//}

	//void send_SG_rs232(byte* data,int length)
	//{
	//	int L=length+2+3+2;// fix to 16 bytes
	//	byte * com=new byte[L]{0};
	//	com[0]=COM_HEADER1;
	//	com[1]=COM_HEADER2;
	//	com[2]='S';
	//	com[3]='G';
	//	com[4]=(byte)length;
	//	com[L-2]=COM_TAIL1;
	//	com[L-1]=COM_TAIL2;
	//	//byte com[L]={
	//	//	COM_HEADER1,COM_HEADER2, 
	//	//	'S','G',(LENGTH_I2C_DATA_SG-2),
	//	//	0,0,0,
	//	//	0,0,0,
	//	//	0,0,0,
	//	//	COM_TAIL1,COM_TAIL2};
	//	memcpy(&com[5],data,length);
	//	mUSerial.write(com,L);
	//	delete com;
	//
	//
	//
	//}
	void read_SG_data(int* pSwitch_read_SG) {}
	;
//	{
//#define sampling_time_us_read_SG_data (1000000.0/10.0)//Math_Max<=24Hz
//		if (PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(sampling_time_us_read_SG_data) == false) return;
//
//			/// for I2C communication 
//#define ADDRESS_I2C_SLAVE   (0x50)   /// unsigned int 	// The following is not required: chipAddress = (chipAddress<<1) | (0<<0);	
//				// 	Wire1.beginTransmission(chipAddress); 
//				// 	Wire1.write(0x20); 
//				// 	Wire1.endTransmission();
//				// 	wait_ms(10); 
//		//*p_LED=1;
//
//		byte Buffer_SG_data[LENGTH_I2C_DATA_SG] = {0};
//		Wire1.requestFrom(ADDRESS_I2C_SLAVE, LENGTH_I2C_DATA_SG);// time use=586 us
//		static byte frame_counter = 0;
//		while (Wire1.available() >= LENGTH_I2C_DATA_SG)    // slave may send less than requested
//		{	
//			for (int k = 0;k < LENGTH_I2C_DATA_SG;k++)
//			{
//				Buffer_SG_data[k] = Wire1.read();    // receive a byte as character
//				//mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
//				//mUSerial.print(" ");
//				//mUSerial.println("raw");
//			}
//
//			if (frame_counter != Buffer_SG_data[0])
//			{
//				frame_counter = Buffer_SG_data[0];
//				//send_SG_rs232();
//				byte com[LENGTH_COM_FRAME_MCU2PC - 4] = {"SG"};
//				com[2] = 9;
//				memcpy(&com[3], &Buffer_SG_data[1], 9);
//				if (*pSwitch_read_SG > 0)
//					send_system_package_string_to_PC((char*)com);
//				if (*pSwitch_read_SG == 1) *pSwitch_read_SG = 0;// read only once
//				//for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
//				//{mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
//				//mUSerial.print(" ");}
//				//mUSerial.println("d");
//			}
//		}
//		*p_LED=0;
//	}


	//void setup()
	//{
	//  Wire.begin(4);                // join i2c bus with address #4
	//  Wire.onReceive(receiveEvent); // register event
	//  mUSerial.begin(9600);           // start serial for output
	//}
	//
	//void loop()
	//{
	//  wait_ms(100);
	//}

	// function that executes whenever data is received from master
	// this function is registered as an event, see setup()
	void receiveEvent(int howMany)
	{
//		while (1 < Wire.available()) // loop through all but the last
//		{
//			char c = Wire.read(); // receive byte as a character
//			mUSerial.print(c);         // print the character
//		}
//		int x = Wire.read();    // receive byte as an integer
//		mUSerial.println(x);         // print the integer
	}
	void convert_uint32_to_byte4(uint32_t x, byte*b)
	{
		//value+=com_buffer_frame[2]<<24;
		//value+=com_buffer_frame[3]<<16;
		//value+=com_buffer_frame[4]<<8;
		//value+=com_buffer_frame[5];
		int L [] = {24, 16, 8, 0};
		for (int k = 0;k < 4;k++)
			b[k] = (byte)((x & (0xff << L[k])) >> L[k]);
	}
	void convert_uint32_to_byte3(uint32_t x, byte*b)
	{
		//value+=com_buffer_frame[2]<<24;
		//value+=com_buffer_frame[3]<<16;
		//value+=com_buffer_frame[4]<<8;
		//value+=com_buffer_frame[5];
		int L [] = {16, 8, 0};
		for (int k = 0;k < 3;k++)
			b[k] = (byte)((x & (0xff << L[k])) >> L[k]);
	}
	void convert_uint32_to_byte2(uint32_t x, byte*b)
	{
		//value+=com_buffer_frame[2]<<24;
		//value+=com_buffer_frame[3]<<16;
		//value+=com_buffer_frame[4]<<8;
		//value+=com_buffer_frame[5];
		int L [] = {8, 0};
		for (int k = 0;k < 2;k++)
			b[k] = (byte)((x & (0xff << L[k])) >> L[k]);
	}

	//send back heart beat, 16 bytes
	void send_back_approach_heart_beat(uint32_t v, uint32_t step, byte done)
	{
		//static unsigned long lastTime =0;
		//if (millis()-lastTime<500000) return;	
		//// send @ each 500,000us
		//lastTime=millis();// update timer		
		//	if (step%1000==0)
		//for (int k=0;k<5;k++)
		{
//			byte com [] = {
//				COM_HEADER1,
//				COM_HEADER2,
//				'C',
//				'A',
//				'P',
//				0,
//				0,
//				0,
//				0,//5
//				0,
//				0, 
//				0, 
//				0,//9
//				(byte)done,
//				COM_TAIL1,
//				COM_TAIL2
//			};
//			convert_uint32_to_byte4(v, &com[5]);
//			convert_uint32_to_byte4(step, &com[9]);
			byte com[LENGTH_COM_DATA_MCU2PC] = {
				'C',//0
				'A',//1
				'P',//2
				0,//---data from 3
				0,
				0,
				0,
				0,//7
				0, 
				0, 
				0,
				(byte)done,
			};
			convert_uint32_to_byte4(v, &com[3]);
			convert_uint32_to_byte4(step, &com[3 + 4]);
			send_Data_In_Frame_To_PC(com);
//			wait_ms(2);
		}
	}
	
	void send_system_package_string_to_PC(const char*inf)
	{
		send_Data_In_Frame_To_PC((byte*)inf);
	}
	//void send_system_package16_to_PC(float Z_position01,uint32_t adc18,uint32_t dac18)
	//{
	//	byte com[LENGTH_COM_FRAME_MCU2PC]={0};
	//	com[0]=COM_HEADER1;
	//	com[1]=COM_HEADER2,
	//	com[LENGTH_COM_FRAME_MCU2PC-2]=COM_TAIL1;
	//	com[LENGTH_COM_FRAME_MCU2PC-1]=COM_TAIL2;
	//	com[2]='s';
	//	com[3]='p';
	//
	//	byte vB3[3]={0};
	//	uint32_t v24=(uint32_t)(Z_position01*(float)BIT24MAX);
	//	convert_uint32_to_byte3(v24,vB3);
	//	memcpy(&com[5],vB3,3);
	//
	//	convert_uint32_to_byte3(adc18,vB3);	
	//	memcpy(&com[8],vB3,3);
	//
	//	convert_uint32_to_byte3(dac18,vB3);	
	//	memcpy(&com[11],vB3,3);		
	//	
	//	mUSerial.write(com,LENGTH_COM_FRAME_MCU2PC); 
	//}
	float some_function(float x, float y)
	{
		struct InnerFuncs
		{
			float inner_function(float x)
			{ 
				// some code
				return x*x;
			}
			// put more functions here if you wish...
		} inner;

		float z;
		z = inner.inner_function(x);
		return z + y; 
	}

	//////////////////////-- process-------------------
	void process_ZScannerEngage()
	{
		//when time due, continue to do approaching work
		//if (PERIOD_CHECK_TIME_US_DUE_ZSCANNERENGAGE(sampling_period_us_of_ZScannerEngage_Process) == false) return;

			//DIGITAL_PIN_TOGGLE(23);
		*p_Tdio1 = 1;

			//static uint32_t Z_position_DAC_ZScannerEngage=0;
		V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
		//////////////////////////
		mPID_ZLOOP->Reset();
		//mUSerial.println(VWset_deltaV_ADC_b18,DEC);
		if (Math_Abs((int32_t)Vdf_infinite - (int32_t)V18_Adc[ADC_PORT_ZlOOP_SENSOR]) > VWset_deltaV_ADC_b18 << 1)// reach working woltage
		{	

			sys_state = SS_Scan;
			send_system_package_string_to_PC("CZEdone");
			if (mode_pause0_scan1_pending2 == 2)// pending for scan
			{
				mode_pause0_scan1_pending2 = 1;
			}
			// when z piezo engaged, run z loop to servo the tip
			process_ScanRealTimeLoop_Initialize((float)Z_position_DAC_ZScannerEngage / BIT18MAX);// start to run Zloop periodically
			Z_position_DAC_ZScannerEngage = 0;
			
			
//			mPID_ZLOOP->SetPID_P(0.0005);
//			mPID_ZLOOP->SetPID_I(0.0002);
//			mPID_ZLOOP->SetPID_D(0);
			return;
		}
		////////////////// STEP MOVE
#define TIME_Z_SCANNER_ENGAGE (10.0)//Second
#define STEP_SIZE_Z_SCANNER_ENGAGE (BIT18MAX_0D75/(TIME_Z_SCANNER_ENGAGE*sampling_frequency_of_ZScannerEngage_Process))

		Z_position_DAC_ZScannerEngage += (STEP_SIZE_Z_SCANNER_ENGAGE);	

			// fail to engage, because Z scanner elongate to 0.75
		if (Z_position_DAC_ZScannerEngage > BIT18MAX_0D75)
		{
			console_WithDrawZScanner_SetSystemIdle();
			Z_position_DAC_ZScannerEngage = 0;
			send_system_package_string_to_PC("CZEfail");
			return;
		}

		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, Z_position_DAC_ZScannerEngage);
		V18_Dac[PIEZO_Z] = Z_position_DAC_ZScannerEngage;
		
		*p_Tdio1 = 0;
	}
	//void process_Approach_only_coarse_move()// only coarse move
	//{
	//	static uint32_t step_counter_Approach=0;
	//	//step_counter_Approach++;
	//	//ADC_read_DAC_write(0,&V18_Adc[ADC_PORT_ZlOOP_SENSOR],PIEZO_Z,BIT18MAX);
	//	int vdf=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//	if (vdf>Vdf_infinite+threshold_approach_delta_B18)// touched
	//	{
	//		mAFM_DAC.DAC_write(PIEZO_Z,0);
	//		sys_state=SS_Idle;
	//		step_counter_Approach=0;
	//		send_back_approach_heart_beat(vdf,step_counter_Approach,1);
	//	}
	//	else
	//		if ((step_counter_Approach++)%100000==0)// control the step time
	//			send_back_approach_heart_beat(vdf,step_counter_Approach,0);
	//	if (step_counter_Approach> 1250000000)//  10,000,000/800
	//	{
	//		mAFM_DAC.DAC_write(PIEZO_Z,0);
	//		sys_state=SS_Idle;
	//		step_counter_Approach=0;
	//		send_back_approach_heart_beat(vdf,step_counter_Approach,128);// Math_Max steps exceeded
	//	}
	//}
	void process_Approach()// fine probing + coarse move
	{
		//while(1)
		//	{
		//		
		//		mUSerial.println(threshold_approach_delta_B18);
		//		wait(0.5);
		//}
		//when time due, continue to do approaching work
		//if (PERIOD_CHECK_TIME_US_DUE_APPROACH(sampling_period_us_of_Approach_Process) == false) return;
		//DIGITAL_PIN_TOGGLE(23);
		*p_LED = 1;
		*p_Tdio3 = 1;
//		
//		*p_LED = 0;
//		*p_Tdio3 = 0;
//	}
//		void test()
//		{
		//static uint32_t step_counter_Approach=0;
		//static uint32_t Z_position_DAC_Approach=0;
		step_counter_Approach++;
		//V18_Adc[ADC_PORT_ZlOOP_SENSOR] = mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR); ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR, 5, 10);
		V18_Adc[ADC_PORT_ZlOOP_SENSOR] = ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR, 5, 10);
		//ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 5, 10);//20_10-->1.97 kHz, 40_10-->1.13 kHz
		uint32_t vdf = V18_Adc[ADC_PORT_ZlOOP_SENSOR];
//---------------------------------------------------------------------------------------------------
//-------------------- touched
		if (
			Math_Abs((float)vdf - (float)Vdf_infinite) > threshold_approach_delta_B18// touched
			|
			Math_Abs((float)vdf - (float)ADC_sensor_buffer) > threshold_approach_delta_B18)
		{		
			console_WithDrawZScanner_SetSystemIdle();
			//		counter_large_step_approaching++;
			send_back_approach_heart_beat(vdf, step_counter_Approach, 1);
			step_counter_Approach = 0;
			Z_position_DAC_Approach = 0;
			// done, GUI use step_counter_Approach to adjust coarse position
			return;
		}
		ADC_sensor_buffer = vdf;// update

		if (step_size_increament_Appraoch < mStepSize_Appoach)// step size increase slowly to avoid vibration in the beginning
			step_size_increament_Appraoch += 2;
		Z_position_DAC_Approach += step_size_increament_Appraoch;//(mStepSize_Appoach);
		
//------------------- one big step finished without touch
		//	if (Z_position_DAC_Approach>BIT18MAX_HALF)// one big step finished without touch
		if (Z_position_DAC_Approach > BIT18MAX_0D9)// one big step finished without touch
		{
			// slow return to avoid vibration
			float z_return = Z_position_DAC_Approach;
			while (z_return > 0)
			{
				z_return -= mStepSize_Appoach;
				if (z_return < 0) break;
				mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, z_return);
			}

			console_ResetScannerModel(PIEZO_Z);
//			console_WithDrawZScanner_SetSystemIdle();// let PC move coarse and re-trigger
			send_back_approach_heart_beat(vdf, step_counter_Approach, 0);// one large step finished
			step_counter_Approach = 0;
			Z_position_DAC_Approach = 0;
			
			sys_state = SS_ApproachWait;
			return;
		}
		
//------------------- continue to walk small step
		// generate a vibration to tapping

		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, Z_position_DAC_Approach);

		float dv = 5.0*BIT18MAX / SCANNER_RANGE_Z_NM;// vibration
		for (int k = 0;k < 3;k++)
		{
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, Z_position_DAC_Approach + dv);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, Z_position_DAC_Approach);
		}

		sys_state = SS_Approach;
		*p_LED = 0;
		*p_Tdio3 = 0;
	}
	void send_engaged_package_to_PC(int indx, int indy, float vHeight, float vError)
	{
		static float indx_store = 0;
		if (vHeight == indx_store) return;
		indx_store = vHeight;
//		if (pointer_out_frame_buffer == LENGTH_IMAGE_FRAME_BUFFER)
//		{
//			Serial_write_reset_input_output();
//			send_image_package_to_PC_sub(indx, indy, vHeight, vError);
//			send_image_package_to_PC_sub(indx, indy, vHeight, vError);
		send_image_package_to_PC_sub(indx, indy, vHeight, vError);
		//		}
	}
	void send_image_package_to_PC(int indx, int indy, float vHeight_01, float vError_pn1)
	{
		//// avoid send the same point
		static int indx_store = -1000;
		if (indx == indx_store) return;
		indx_store = indx;
		send_image_package_to_PC_sub(indx, indy, vHeight_01, vError_pn1);
	}
	void send_image_package_to_PC_dac(int indx, int indy, float vHeight_01, float vError_pn1, uint32_t value_dac)
	{
		//// avoid send the same point
		static int indx_store = -1000;
		if (indx == indx_store) return;
		indx_store = indx;
		send_image_package_to_PC_sub_dac(indx, indy, vHeight_01, vError_pn1, value_dac);
	}
	void send_image_package_to_PC_sub_dac(int indx, int indy, float vHeight_01, float vError_pn1, uint32_t value_dac)
		//	vHeight=0:1, vError=-1:1
	{
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 'I';
		com[1] = 'M';
		byte ind[2] = {0};
		convert_uint32_to_byte2(indx, ind);
		com[2] = ind[0];
		com[3] = ind[1];
//		Serial_write(ind, 2);
		convert_uint32_to_byte2(indy, ind);
//		Serial_write(ind, 2);
		com[4] = ind[0];
		com[5] = ind[1];

		byte valueH3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
		uint32_t vH24 = (uint32_t)(vHeight_01*(float)BIT24MAX);
		convert_uint32_to_byte3(vH24, valueH3b);

		byte valueE3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
		//uint32_t vE24=(uint32_t)(vError*(float)BIT24MAX);
		// converet -1:1-->0:2, later divide by 2
		uint32_t vE24 = (uint32_t)((vError_pn1 + 1)*(float)BIT24MAX);
		vE24 >>= 1;// divide by 2
		convert_uint32_to_byte3(vE24, valueE3b);

//		Serial_write(valueH3b, SIZE_IMAGE_BUFFER_BIT24);
		com[6] = valueH3b[0];
		com[7] = valueH3b[1];
		com[8] = valueH3b[2];
//		Serial_write(valueE3b, SIZE_IMAGE_BUFFER_BIT24);
		com[9] = valueE3b[0];
		com[10] = valueE3b[1];
		com[11] = valueE3b[2];
		// DAC value
		byte value_DAC3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
		convert_uint32_to_byte3(value_dac, value_DAC3b);
		com[12] = value_DAC3b[0];
		com[13] = value_DAC3b[1];
		com[14] = value_DAC3b[2];
		
		send_Data_In_Frame_To_PC(com);
	}
	void send_image_package_to_PC_sub(int indx, int indy, float vHeight_01, float vError_pn1)
		//	vHeight=0:1, vError=-1:1
	{
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 'I';
		com[1] = 'M';
		byte ind[2] = {0};
		convert_uint32_to_byte2(indx, ind);
		com[2] = ind[0];
		com[3] = ind[1];
//		Serial_write(ind, 2);
		convert_uint32_to_byte2(indy, ind);
//		Serial_write(ind, 2);
		com[4] = ind[0];
		com[5] = ind[1];

		byte valueH3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
		uint32_t vH24 = (uint32_t)(vHeight_01*(float)BIT24MAX);
		convert_uint32_to_byte3(vH24, valueH3b);

		byte valueE3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
		//uint32_t vE24=(uint32_t)(vError*(float)BIT24MAX);
		// converet -1:1-->0:2, later divide by 2
		uint32_t vE24 = (uint32_t)((vError_pn1 + 1)*(float)BIT24MAX);
		vE24 >>= 1;// divide by 2
		convert_uint32_to_byte3(vE24, valueE3b);

//		Serial_write(valueH3b, SIZE_IMAGE_BUFFER_BIT24);
		com[6] = valueH3b[0];
		com[7] = valueH3b[1];
		com[8] = valueH3b[2];
//		Serial_write(valueE3b, SIZE_IMAGE_BUFFER_BIT24);
		com[9] = valueE3b[0];
		com[10] = valueE3b[1];
		com[11] = valueE3b[2];
		
		send_Data_In_Frame_To_PC(com);
	}
	
//	void send_image_package_to_PC_direct(int indx, int indy, float vHeight, float vError)
//	//	vHeight=0:1, vError=-1:1
//	{
//		static int indx_store = -1000;
//		if (indx == indx_store) return;
//		indx_store = indx;
//		
//		
//		byte COM[LENGTH_COM_FRAME_MCU2PC] = {0};
//		COM[0]=(COM_HEADER1);
//		COM[1] = (COM_HEADER2);
//		COM[2] = ('I');
//		COM[3] = ('M');
//		byte ind[2] = {0};
//		convert_uint32_to_byte2(indx, ind);
////		Serial_write(ind, 2);
//		COM[4] = ind[0];
//		COM[5] = ind[1];
//		convert_uint32_to_byte2(indy, ind);
////		Serial_write(ind, 2);
//		COM[6] = ind[0];
//		COM[7] = ind[1];
//
//		byte valueH3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
//		uint32_t vH24 = (uint32_t)(vHeight*(float)BIT24MAX);
//		convert_uint32_to_byte3(vH24, valueH3b);
//
//		byte valueE3b[SIZE_IMAGE_BUFFER_BIT24] = {0};
//		//uint32_t vE24=(uint32_t)(vError*(float)BIT24MAX);
//		vError += 1;// converet -1:1-->0:2, later divide by 2
//		uint32_t vE24 = (uint32_t)(vError*(float)BIT24MAX);
//		vE24 >>= 1;// divide by 2
//		convert_uint32_to_byte3(vE24, valueE3b);
//
////		Serial_write(valueH3b, SIZE_IMAGE_BUFFER_BIT24);
////		Serial_write(valueE3b, SIZE_IMAGE_BUFFER_BIT24);
//		COM[8] = valueH3b[0];
//		COM[9] = valueH3b[1];
//		COM[10] = valueH3b[2];
//		
//		COM[11] = valueE3b[0];
//		COM[12] = valueE3b[1];
//		COM[13] = valueE3b[2];
//
//
//		COM[LENGTH_COM_FRAME_MCU2PC - 2] = (COM_TAIL1);
//		COM[LENGTH_COM_FRAME_MCU2PC - 1] = (COM_TAIL2);
//		
//		mUSerial.write(COM, LENGTH_COM_FRAME_MCU2PC);
//		mUSerial.write(COM, LENGTH_COM_FRAME_MCU2PC);
//
//	}
	
	
	//void send_image_package_to_PC_store(int indx,int indy)
	//{
	//	// avoid send the same point
	//	//static int indx_store=-1000;
	//	//if (indx==indx_store) return;
	//	//indx_store=indx;
	//	
	//	Serial_write_reset_input_output();
	//	Serial_write(COM_HEADER1);
	//	Serial_write(COM_HEADER2);
	//	Serial_write('I');
	//	Serial_write('M');
	//	byte ind[2]={0};
	//	convert_uint32_to_byte2(indx,ind);
	//	Serial_write(ind,2);
	//	convert_uint32_to_byte2(indy,ind);
	//	Serial_write(ind,2);
	//
	//	int current_image_line=indy;
	//	current_image_line%=SIZE_IMAGE_BUFFER_LINES;
	//
	//	if (indx>=0)
	//	{
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			Serial_write(pImageHF[k][current_image_line][Math_Abs(indx)]);
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			Serial_write(pImageEF[k][current_image_line][Math_Abs(indx)]);
	//	}
	//	else
	//	{
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			Serial_write(pImageHB[k][current_image_line][Math_Abs(indx)]);
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			Serial_write(pImageEB[k][current_image_line][Math_Abs(indx)]);
	//	}
	//
	//	Serial_write(COM_TAIL1);
	//	Serial_write(COM_TAIL2);
	//}
	//void old_send_back_image_package_to_PC(int indx,int indy)
	//{
	//	// avoid send the same point
	//	static int indx_store=-1000;
	//	if (indx==indx_store) return;
	//	indx_store=indx;
	//
	//	mUSerial.write(COM_HEADER1);
	//	mUSerial.write(COM_HEADER2);
	//	mUSerial.write('I');
	//	mUSerial.write('M');
	//	byte ind[2]={0};
	//	convert_uint32_to_byte2(indx,ind);
	//	mUSerial.write(ind,2);
	//	convert_uint32_to_byte2(indy,ind);
	//	mUSerial.write(ind,2);
	//
	//	int current_image_line=indy;
	//	current_image_line%=SIZE_IMAGE_BUFFER_LINES;
	//	////debug
	//	/////////////////////////////////////////
	//	//	//store image
	//	//	//int current_image_line=indy;
	//	//	current_image_line%=SIZE_IMAGE_BUFFER_LINES;
	//	//	//byte f0b1=0;
	//	//	//if (indx>=0) f0b1=0;
	//	//	//if (indx<0)  f0b1=1;
	//	//	float z_output_01=65536.0/(float)BIT24MAX;
	//	//	float er=256.0/(float)BIT24MAX;
	//	//
	//	//	byte valueH3b[SIZE_IMAGE_BUFFER_BIT24]={0};
	//	//	uint32_t vH24=(uint32_t)(z_output_01*(float)BIT24MAX);
	//	//	convert_uint32_to_byte3(vH24,valueH3b);
	//	//
	//	//	byte valueE3b[SIZE_IMAGE_BUFFER_BIT24]={0};
	//	//	uint32_t vE24=(uint32_t)(er*(float)BIT24MAX);
	//	//	convert_uint32_to_byte3(vE24,valueE3b);
	//	//	//6 us
	//	//	for (int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//	//	{
	//	//		if (indx>=0) 
	//	//		{
	//	//			pImageHF[k][current_image_line][Math_Abs(indx)]=valueH3b[k];
	//	//			pImageEF[k][current_image_line][Math_Abs(indx)]=valueE3b[k];
	//	//		}
	//	//		else
	//	//		{
	//	//			pImageHB[k][current_image_line][Math_Abs(indx)]=valueH3b[k];
	//	//			pImageEB[k][current_image_line][Math_Abs(indx)]=valueE3b[k];
	//	//		}
	//	//
	//	//	}// 7us
	//
	//	/////////////////////////
	//	if (indx>=0)
	//	{
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			mUSerial.write(pImageHF[k][current_image_line][Math_Abs(indx)]);
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			mUSerial.write(pImageEF[k][current_image_line][Math_Abs(indx)]);
	//	}
	//	else
	//	{
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			mUSerial.write(pImageHB[k][current_image_line][Math_Abs(indx)]);
	//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//			mUSerial.write(pImageEB[k][current_image_line][Math_Abs(indx)]);
	//	}
	//
	//	mUSerial.write(COM_TAIL1);
	//	mUSerial.write(COM_TAIL2);
	//}

//	void send_back_debug_infomation()
//		// should send each byte and then give cpu to others
//	{
////		static int c = 0;
////		c++;
////		if (c < 100000)
////			return;
////		c = 0;
//
//
//			//PERIOD_TIME_CHECK_EXIT();
//			//	int dt=500000;
//			///*	{
//			//		do
//			//		{*/	
//			//			static unsigned long time_store =millis();	
//			//			unsigned long time_now=millis();	
//			//			if((time_now - time_store)<(dt))
//			//				return;	
//			//			else 
//			//				time_store=time_now;
//			//	//	}
//			//	//	while(0);
//			//	//}
//
//				//		mUSerial.print("F ");
//				//		mUSerial.println(measured_sampling_frequency_of_system,DEC);
//				//mUSerial.print("DSet_01 ");
//				//mUSerial.println(DSet_01,DEC);
//				//mUSerial.print("DInput_01 ");
//				//mUSerial.println(DInput_01,DEC);
//		mUSerial.print("sys_state ");
//		mUSerial.print(sys_state, DEC);
//		mUSerial.print("*XY_state: ");	
//		mUSerial.print(mDDS_XY_Scanner_State, DEC);	
//
//		mUSerial.print("*ADC_inf: ");	
//		mUSerial.print(Vdf_infinite, DEC);	
//		mUSerial.print("*ADC: ");	
//		mUSerial.print(V18_Adc[ADC_PORT_ZlOOP_SENSOR], DEC);
//		mUSerial.print("*vdf mV: ");
//		mUSerial.print((int)((float)V18_Adc[ADC_PORT_ZlOOP_SENSOR] * 5.0 / (float)BIT18MAX * 1000), DEC);
////		mUSerial.print("*Set: ");
////		mUSerial.print((int)(DSet_01 * 100000), DEC);
////		mUSerial.print("*IN: ");
////
////		mUSerial.print((int)(DInput_01 * 100000), DEC);
////		//mUSerial.println(DInput_01,DEC);
////		mUSerial.print("*Out: ");
////
////		mUSerial.print((int)(DOutput_01 * 100000), DEC);
//
//
//		//mUSerial.print("*Vout: ");
//		//mUSerial.print((int)(z_output_01 * 100000), DEC);
//
//		mUSerial.print("*X: ");
//		mUSerial.print(VDACx, DEC);
//		mUSerial.print("*Y: ");
//		mUSerial.print(VDACy, DEC);
//
//		mUSerial.print("*switch_read_SG: ");
//		mUSerial.print(switch_read_SG, DEC);
//
//		mUSerial.println('\n', DEC);
//
//			//mUSerial.println(DOutput_01,DEC);
//
//	}


	void process_ScanRealTimeLoop_Initialize(float position_01)
	{
		calculate_scan_parameter();
		mPID_ZLOOP->Reset(position_01);
		//mTimer_ZLoop.start();  
		//z_output_01 = position_01;//0.5;
	}
	
	void test_scanner_wave_output_DAC()
	{
		int value = wave_triangle_0ToMax(8192, BIT18MAX, false);//2048
		static int v_last = 0;		
//		read back
//		send_system_package_to_PC(20,
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true),
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
		send_system_package_to_PC(20,
			v_last,
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, true),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
		// next step
//		value = (int)((float)value / 3 );//+  8.738133333333333e04
		mAFM_DAC.DAC_write_all(value);

		v_last = value;
	}
	void test_scanner_wave_output_CloseLoopPosition()
	{
		mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 2;
		int value = wave_triangle_0ToMax(8192, BIT18MAX, false);
		static int v_last = 0;		
//		read back
//		send_system_package_to_PC(20,
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true),
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
		send_system_package_to_PC(20,
			v_last,
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, true),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
		// next step
		mCScanner[PIEZO_X].MoveToPosition01(value / BIT18MAX);

		v_last = value;
	}	
	void test_sensor_drift()
	{
		send_system_package_to_PC(10,
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, true),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false));
		
		send_system_package_to_PC(11,
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
	}
	
	void process_WaveTest()
	{
		//ADC_read_DAC_write(ADC_PORT_ZlOOP_SENSOR,&V18_Adc[PIEZO_Z],PIEZO_Z,V18_Dac[PIEZO_Z]);
		//if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(300000) == false) return;//old 1e6
		//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],V18_Dac[PIEZO_Z]);

		int wait_count = 1000000;//50000;
		CHECK_COUNT_DUE(wait_count);
//		test_sensor_drift();		
//		test_scanner_wave_output_CloseLoopPosition();
		
		test_scanner_wave_output_DAC();

//		
	}
	
	void process_Idle()
	{
		//ADC_read_DAC_write(ADC_PORT_ZlOOP_SENSOR,&V18_Adc[PIEZO_Z],PIEZO_Z,V18_Dac[PIEZO_Z]);
		//if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(300000) == false) return;//old 1e6
		//send_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],V18_Dac[PIEZO_Z]);

//		mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 = 2;
//		mCScanner[PIEZO_Y].MoveToPosition01(0.7);
		
		int wait_count = 1000000;// 50000;
		if (mSW_Idle_Scanner_DAC0_OpenLoop1_CloseLoop2 == 2)
		{
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true)));
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, mCScanner[PIEZO_Y].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false)));
			
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, mCScanner[PIEZO_Y].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true)));
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false)));
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, mCScanner[PIEZO_Z].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false)));
			wait_count = 100;//100;
		}

		CHECK_COUNT_DUE(wait_count);
//		test_sensor_drift();		
//		test_scanner_wave_output_CloseLoopPosition();
		
//		test_scanner_wave_output_DAC();
//		return;
//		
		//----------------------------------------------
		
		extern CTemperature mCTemperature;
		uint32_t Temperature_MCU = mCTemperature.Read(false);
		
//		Temperature_MCU = mCScanner[PIEZO_Z].mScannerPosition01*BIT18MAX;
		Temperature_MCU = mAFM_DAC.ReadDACValue(PIEZO_Y);

		send_system_package_to_PC(20,
			Temperature_MCU,
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, true),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false),
			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false));
		
		return;
		
//-------------------------------------------------------
		if (sys_idle_package_index == 0)
		{
			int Temperature_SEM = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_TEMPERATURE, true);
			int value_cantilever = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false);
			extern CTemperature mCTemperature;
			uint32_t Temperature_MCU = mCTemperature.Read(false);
			send_system_package_to_PC(sys_idle_package_index, value_cantilever, Temperature_SEM, Temperature_MCU);//position_feedforward_output_01[PIEZO_Z]*BIT32MAX)
		}
		if (sys_idle_package_index == 1)
		{
			int scsg_y = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);
			int scsg_x = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false);
			int scsg_z = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);

			send_system_package_to_PC(sys_idle_package_index, scsg_x, scsg_y, scsg_z);
		}
		if (sys_idle_package_index == 2)
		{
			int value_cali = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_CALIBRATION, true);
			int value_cantilever = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false);
			int scsg_z = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);

			send_system_package_to_PC(sys_idle_package_index, value_cantilever, value_cali, scsg_z);
		}
		if (sys_idle_package_index == 3)
		{
			int x = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, true);
			int p = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC, false);
			int scsg_z = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false);

			send_system_package_to_PC(sys_idle_package_index, x, p, scsg_z);
		}
		//send_back_debug_infomation();
		toggle_pin_led();
	}
	void send_system_package_to_PC(byte index, uint32_t v1, uint32_t v2, uint32_t v3, uint32_t v4, uint32_t v5, uint32_t v6 = 0, uint32_t v7 = 0)
	{
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 's';
		com[1] = 'p';
		com[2] = index;
		int k = 3;
		convert_uint32_to_byte3(v1, &com[k]);k += 3;
		convert_uint32_to_byte3(v2, &com[k]);k += 3;
		convert_uint32_to_byte3(v3, &com[k]);k += 3;
		convert_uint32_to_byte3(v4, &com[k]);k += 3;
		convert_uint32_to_byte3(v5, &com[k]);k += 3;
		convert_uint32_to_byte3(v6, &com[k]);k += 3;
		convert_uint32_to_byte3(v7, &com[k]);k += 3;
//		convert_uint32_to_byte3(v2, &com[3 + 3]);// byte3
//		convert_uint32_to_byte3(v3, &com[3 + 3 + 3]);//byte3
//		mCPackageToPC_SystemPackage->prepare_package_Byte12_to_PC(com);
		send_Data_In_Frame_To_PC(com);
	}
	
	void send_system_package_to_PC(byte index, uint32_t v1, uint32_t v2, uint32_t v3)
	{
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 's';
		com[1] = 'p';
		com[2] = index;
		convert_uint32_to_byte3(v1, &com[3]);// byte3
		convert_uint32_to_byte3(v2, &com[3 + 3]);// byte3
		convert_uint32_to_byte3(v3, &com[3 + 3 + 3]);//byte3
//		mCPackageToPC_SystemPackage->prepare_package_Byte12_to_PC(com);
		send_Data_In_Frame_To_PC(com);
	}
	void send_system_package_to_PC(uint32_t v1, uint32_t v2, uint32_t v3)
	{
		byte com[LENGTH_COM_DATA_MCU2PC] = {0};
		com[0] = 's';
		com[1] = 'p';
		//com[2]=0;
		convert_uint32_to_byte4(v1, &com[2]);// byte4
		convert_uint32_to_byte3(v2, &com[2 + 4]);// byte3
		convert_uint32_to_byte3(v3, &com[2 + 4 + 3]);//byte3
//		mCPackageToPC_SystemPackage->prepare_package_Byte12_to_PC(com);
		send_Data_In_Frame_To_PC(com);
	}

	void send_Data_In_Frame_To_PC(byte* data, int length = LENGTH_COM_DATA_MCU2PC)
	{
		length = Min(length, LENGTH_COM_DATA_MCU2PC);
		byte COM[LENGTH_COM_FRAME_MCU2PC] = {0};
		COM[0] = (COM_HEADER1);
		COM[1] = (COM_HEADER2);
		memcpy(COM + 2, data, length);
		COM[LENGTH_COM_FRAME_MCU2PC - 2] = (COM_TAIL1);
		COM[LENGTH_COM_FRAME_MCU2PC - 1] = (COM_TAIL2);
		mUSerial.write(COM, LENGTH_COM_FRAME_MCU2PC);
	}

	////////////////////////////////////
	void test_usb_show(bool lock1, const char* str)
	{
		static byte x = 0;
		do
		{
						
			MY_Debug(str);
			MY_Debug_LN(x++);		
			wait(0.01);
			toggle_pin_led();
		} while (lock1);
		
	}
	

	void test_system_time_consumption()
	{
		//		time test:
		//	while (1)
		//{
		//	toggle_pin_p(p_Tdio4);
		//		wait_multi(5000000);			
		//}	
//		mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false));// 3.87 us
//		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false))); // 86us-->29.18us
//		mAFM_DAC.DAC_write(0, BIT18MAX);	// 71 us-->25.76us
		
		
		
		
//		sys_state = SS_Scan;
//////		DigitalIn* u = new DigitalIn(PE_10);
//		while (1)
//		{
//			p_Tdio2->write_hal(1);	
//				
//			for (int k = 0;k < 500;k++)
////				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);
////				mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false)));
////				mAFM_DAC.DAC_write(0, BIT18MAX);
//			
//			
//			p_Tdio2->write_hal(0);	
//			for (int k = 0;k < 500;k++)
////				mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);
////				mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, mCScanner[PIEZO_X].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_X, false)));
////				mAFM_DAC.DAC_write(0, BIT18MAX);
//			
//		}
//		{
//			p_Tdio2->write_hal(1);
//				
//				AFM_ProcessScheduler_Realtime();
//				
//				
//				p_Tdio2->write_hal(0);
//			}


		//while (1)
		//{
		//	*p_Tdio2 = 1;
		//	for (int k = 0;k < 500;k++)
		//		//mCScanner[PIEZO_X].update(100);
		//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);
		//
		//	*p_Tdio2 = 0;
		//	for (int k = 0;k < 500;k++)
		//		//mCScanner[PIEZO_X].update(100);
		//			mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Y, true);
		//}
	}
	void test_usb_hardware()
	{
		DigitalOut p1(PA_11);
		DigitalOut p2(PA_12);
		
		DigitalOut p3(PA_8);
		while (1)
		{
			p1 = 1;
			p2 = 1;
			p3 = 0;
			wait(0.1);
			p1 = 0;
			p2 = 0;
			p3 = 1;
			wait(0.1);
		}
	}
	void AFM_main_setup() 
	{	
		CAFM_ClockDefine mCAFM_ClockDefine;
		mCAFM_ClockDefine.AFM_SystemInit(); 
//		test_usb_hardware();
		mAFM_DAC.Initialize();
		mAFM_DAC.DAC_write(PIEZO_X, 0);
		mAFM_DAC.DAC_write(PIEZO_Y, 0);
		mAFM_DAC.DAC_write(PIEZO_Z, 0);
		
		mUSerial.begin(); 
//		while (1)
//		{
//			send_system_package_to_PC(100, 1, 2, 3, 4, 5, 6, 7);
//			wait(0.5);
//		}
//		while (1)
//		{
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
////			HAL_Delay(500);
////			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
////			mUSerial.println(12);
//			float x = 1, y = 3;
//			p_Tdio2->write_hal(1);	
////			FOR_REPEAT(500000, x = x*y ;);
//			p_Tdio2->write_hal(0);	
//			FOR_REPEAT(1, x = x/y ;);
//			MY_Debug_LN(x);
//		}


		mCTickTimer_RealTime.start();
		Initial_parameters();

//		while (1)// wait for connection
//		{
//					
//			int byte_ready = mUSerial.readBytes(com_buffer, 1);//LENGTH_COM_FRAME_PC2MCU * 2);
//			if (byte_ready > 0) 
//			{
//				mUSerial.println(com_buffer[0]++);
//				break;
//			}
//			wait(0.1);
//			int x = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC);//ADC_PORT_ZlOOP_SENSOR
//			MY_Debug_LN(x);
//			//Software_Reset();
//			toggle_pin_led();
//		}
		
		//int x = mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC);//ADC_PORT_ZlOOP_SENSOR
		//mTicker_AFM_Communication.attach_us(&AFM_Communication_Process, mPeriod_Communication_us);			
//		MY_Debug_LN("MCU starts.");	
//				double x=1,y=3;
//		x=x/y+100;
//		MY_Debug_LN(x);

		mAFM_DAC.Initialize();
		mAFM_SEM.SetFrequency(1000000);
		calculate_scan_parameter();
		
		
	

//		TIC();
//		
//		process_ScanRealTimeLoop();
//		int t_us = TOC();
		
		
		//mUSerial.println(t_us, DEC);
		
//		measured_sampling_frequency_of_system = 5.0 * 10000000.0 / t_us;
 
		console_WithDrawZScanner_SetSystemIdle();
		console_XYScanReset();
 
//		MY_Debug_StringValue_LN("sensor value median: ", ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR, 30, 1000));
//
//		MY_Debug_StringValue_LN("sensor value average: ", ADC_read_average(ADC_PORT_ZlOOP_SENSOR, 30, 1000));


		MY_Debug_LN("Setup() done.");
	
		//mTicker_AFM_Realtime.attach_us(&AFM_ProcessScheduler_Realtime, mPeriod_Realtime_us);
		
		//test_usb_show(0,"setup");
//		float x = mPID_ZLOOP->GetKp();
//		
//		float y = mPID_ZLOOP->GetKi();
//		
//		float z = mPID_ZLOOP->GetKd();
		
		
		Initialze_XYZScanner_CloseLoop();
		

//		while (1)
//		{
//			wait_us(30);
//			p_Tdio2->write(1);
//			wait_us(30);
//			p_Tdio2->write(0);
////			toggle_pin_p(p_Tdio2);
//		}
	}
	void Initialze_XYZScanner_CloseLoop()
	{
//		mCScanner[PIEZO_X] = mCScannerX;		
//		mCScanner[PIEZO_Y] = mCScannerY;
//		mCScanner[PIEZO_Z] = mCScannerZ;
		
		for (int k = 0;k < NUM_OF_SCANNER;k++)
			mCScanner[k].Initial(k, mPeriod_RealtimePID_us);

		
		_Float_ ADC18_Min[NUM_OF_SCANNER] = {6719, 35349, 211968};//{4561, 35500, 231916};// {8164, 34928, 232626};
		_Float_ ADC18_Max[NUM_OF_SCANNER] = {233590, 243630, 51845};//{241554, 244873, 61734};//{241802, 244504, 62131};


		GetPositionSensorRange(ADC18_Min, ADC18_Max);
		
		MY_Debug_StringValue_LN("ADC18_Min[0]", ADC18_Min[0]);
		MY_Debug_StringValue_LN("ADC18_Min[1]", ADC18_Min[1]);
		MY_Debug_StringValue_LN("ADC18_Min[2]", ADC18_Min[2]);
		
		MY_Debug_StringValue_LN("ADC18_Max[0]", ADC18_Max[0]);
		MY_Debug_StringValue_LN("ADC18_Max[1]", ADC18_Max[1]);
		MY_Debug_StringValue_LN("ADC18_Max[2]", ADC18_Max[2]);
		// send twice to make sure PC will receive
		send_system_package_to_PC(13, ADC18_Min[0], ADC18_Min[1], ADC18_Min[2]);
		send_system_package_to_PC(13, ADC18_Min[0], ADC18_Min[1], ADC18_Min[2]);		
		send_system_package_to_PC(14, ADC18_Max[0], ADC18_Max[1], ADC18_Max[2]);
		send_system_package_to_PC(14, ADC18_Max[0], ADC18_Max[1], ADC18_Max[2]);
		
		for (int k = 0;k < NUM_OF_SCANNER;k++)
		{
			mCScanner[k].SetSensorRange(ADC18_Min[k], ADC18_Max[k]);
//		mCScanner[PIEZO_Z].SetSensorRange(ADC18_Min[PIEZO_Z], ADC18_Max[PIEZO_Z]);
//		mCScanner[PIEZO_Z].SetSensorRange(ADC18_Min[PIEZO_Z], ADC18_Max[PIEZO_Z]);
			mCScanner[k].MoveToPosition01(0);//0.5
		}
	}
	bool GetPositionSensorRange(_Float_ *ADC18_Min, _Float_ *ADC18_Max)
	{	
//		for (int k = 0;k < 10;k++)
//		{			
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_ALL, 0);
//			wait(0.3);
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_ALL, BIT18MAX);			
//			wait(0.3);
//		}// here must use square wave to move the piezos;
		
		wave_triangle_0ToMax(1, BIT18MAX, true);// reset

		for (int k = 0;k < BIT18MAX / 1024 * 40;k++)
		{
			int value = wave_triangle_0ToMax(1024, BIT18MAX, false);//2048
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_ALL, value);
		}

		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_ALL, 0);
		wait(1);//5
		ADC18_Min[PIEZO_X] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_X, 10, 1000);
		ADC18_Min[PIEZO_Y] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Y, 10, 1000);
		ADC18_Min[PIEZO_Z] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Z, 10, 1000);
		
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_ALL, BIT18MAX);

		wait(1);//5
		ADC18_Max[PIEZO_X] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_X, 10, 1000);
		ADC18_Max[PIEZO_Y] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Y, 10, 1000);
		ADC18_Max[PIEZO_Z] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Z, 10, 1000);		
	}
	;
	
	bool GetPositionSensorRange_old_ok(_Float_ *ADC18_Min, _Float_ *ADC18_Max)
	{	
		for (int k = 0;k < 10;k++)
		{
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, 0);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, 0);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, 0);
			wait(0.1);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, BIT18MAX);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, BIT18MAX);
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, BIT18MAX);
			wait(0.1);
		}
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, 0);
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, 0);
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, 0);
		wait(5);
		ADC18_Min[PIEZO_X] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_X, 10, 1000);
		ADC18_Min[PIEZO_Y] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Y, 10, 1000);
		ADC18_Min[PIEZO_Z] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Z, 10, 1000);
		
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_X, BIT18MAX);
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Y, BIT18MAX);
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, BIT18MAX);
		wait(5);
		ADC18_Max[PIEZO_X] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_X, 10, 1000);
		ADC18_Max[PIEZO_Y] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Y, 10, 1000);
		ADC18_Max[PIEZO_Z] = mAFM_SEM.ADC_Read_N_Average(ADC_CHANNEL_Z, 10, 1000);		
	}
	;
	
	void read_SG_data_temp();
//	{
//		int  switch_read_SG = 2;
//		if (PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(sampling_time_us_read_SG_data) == false) return;
//
//			/// for I2C communication 
//#define ADDRESS_I2C_SLAVE   (0x50)   /// unsigned int 	// The following is not required: chipAddress = (chipAddress<<1) | (0<<0);	
//				// 	Wire1.beginTransmission(chipAddress); 
//				// 	Wire1.write(0x20); 
//				// 	Wire1.endTransmission();
//				// 	wait_ms(10); 
//		*p_LED=1;
//
//		byte Buffer_SG_data[LENGTH_I2C_DATA_SG] = {0};
//		Wire1.requestFrom(ADDRESS_I2C_SLAVE, LENGTH_I2C_DATA_SG);// time use=586 us
//		static byte frame_counter = 0;
//		while (Wire1.available() >= LENGTH_I2C_DATA_SG)    // slave may send less than requested
//		{	
//			for (int k = 0;k < LENGTH_I2C_DATA_SG;k++)
//			{
//				Buffer_SG_data[k] = Wire1.read();    // receive a byte as character
//				//mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
//				//mUSerial.print(" ");
//				//mUSerial.println("raw");
//			}
//
//			if (frame_counter != Buffer_SG_data[0])
//			{
//				frame_counter = Buffer_SG_data[0];
//				//send_SG_rs232();
//				byte com[LENGTH_COM_FRAME_MCU2PC - 4] = {"SG"};
//				com[2] = 9;
//				memcpy(&com[3], &Buffer_SG_data[1], 9);
//
//							//uint32_t ad1=mAFM_SEM.ADC_Read_N(ADC_CHANNEL_PRC);
//							//uint32_t amp_ad0=analogRead(A2);
//							//convert_uint32_to_byte3(amp_ad0,&com[3]);
//
//
//				if (switch_read_SG > 0)
//					send_system_package_string_to_PC((char*)com);
//				if (switch_read_SG == 1) switch_read_SG = 0;// read only once
//				//for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
//				//{mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
//				//mUSerial.print(" ");}
//				//mUSerial.println("d");
//			}
//		}
//		*p_LED=0;
//	}
	void AFM_main_loop() 
	{		

		//sys_state=SS_Scan;
		//test_system_time_consumption();
		//process_data_capture_blocking();
//		static byte x = 0;
//		sys_state = SS_Scan;// for test only
//		send_image_package_to_PC(x++, 0, 0.5, 0.5);
//		int value = 0;
//		while (1)
//		{
//			value = 0;
//			mAFM_DAC.DAC_write(0, value);
//			mAFM_DAC.DAC_write(1, value);
//			mAFM_DAC.DAC_write(2, value);
//			mAFM_DAC.DAC_write(3, value);
//			value = BIT18MAX;
//			mAFM_DAC.DAC_write(0, value);
//			mAFM_DAC.DAC_write(1, value);
//			mAFM_DAC.DAC_write(2, value);
//			mAFM_DAC.DAC_write(3, value);
//		}
		
		
		
		
		Communication_Command_Console();
		AFM_ProcessScheduler_Realtime();
//		AFM_ProcessScheduler_NonRealtime();

		//test_usb_show(0,"Loop");		
	}
	
	void piezo_predict_Position01_To_Voltage_DAC18(int axis, float PositionInput)//,int steps
	{	// input output range 0~1
		// this function is non-re_enter_able

			//alpha: 0.4000
			// beta: 9.1500
			//gamma: 1.5000
			//   dp: 1.0030
			//    n: 1
			//	pmdl.alpha = 0.35;
			//pmdl.beta = 7;
			//pmdl.gamma = 4;
			//pmdl.dp = 1.09;
			//pmdl.n = 1.02;
		const	float alpha = 0.35, beta = 7, gamma = 4, dp = 1.09;

		static float H_now[NUM_OF_PIEZO_MODEL] = {0};
		static float PositionNow[NUM_OF_PIEZO_MODEL] = {0};
		static uint32_t value_dac[NUM_OF_PIEZO_MODEL] = {0};// save the value for output when input is the same as current state;// display bug fixed, 20151030, add static
	
		float deltaPosition = PositionInput - PositionNow[axis];
		float temp_PositionInput = PositionInput;
		float model_outputV = 0;
		float temp_deltaPosition = 0;
		float dH = 0;	

			//////////////////////////////////////////////////////////// reset model
			// model reset
		if (PositionInput <= EPS)
		{
			H_now[axis] = 0;
			PositionNow[axis] = 0;
			value_dac[axis] = 0;
			mAFM_DAC.FinePositioner_MoveToPositionB18(axis, value_dac[axis]);
			//ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac[axis]);// speed up for both read and write
		}
		else
		{	////////////////////////////////////////////////////////////// normail feedforward
			while (Math_Abs(deltaPosition) > EPS)// 85us each round
			{
				//fastDigitalWrite(23,true);
				if (Math_Abs(deltaPosition) < MAX_STEP_SIZE_PIEZO_MODEL_01)
					temp_deltaPosition = deltaPosition;// last step
				else// equal steps
				{
					if (deltaPosition > 0)// move direction
						temp_deltaPosition = MAX_STEP_SIZE_PIEZO_MODEL_01;
					else
						temp_deltaPosition = -MAX_STEP_SIZE_PIEZO_MODEL_01;
				}
				temp_PositionInput = PositionNow[axis] + temp_deltaPosition;
				deltaPosition -= temp_deltaPosition;
				//float H_now = H_now;
				//H_now = H_now.^n;
				dH = (alpha / dp*temp_deltaPosition - beta*H_now[axis]*temp_deltaPosition*SIGN(temp_deltaPosition) - gamma*Math_Abs(H_now[axis])*temp_deltaPosition) /
					(1 - alpha / dp + beta*H_now[axis]*SIGN(temp_deltaPosition) + gamma*Math_Abs(H_now[axis]));
		//	% dH = alpha*dV-beta*Math_Abs(dV)*H(k)-gamma*dV*Math_Abs(H(k));   //   % dV * (alpha - (gamma + beta * SIGN(dV * H)) * Math_Abs(H)^n);
				H_now[axis] += dH;
				PositionNow[axis] = temp_PositionInput;
				//	% y(k) = dp.*ExcitationVoltage(k) -H_now;
				//ExcitationVoltage=(temp_PositionInput+H_now)/dp;
				model_outputV = (temp_PositionInput + H_now[axis]) / dp;

							//mUSerial.print("dH ");
							//mUSerial.println(dH,DEC);
							//mUSerial.print("H_now[axis] ");
							//mUSerial.println(H_now[axis],DEC);

										//model_outputV+= data_polyFit(axis, temp_PositionInput)-temp_PositionInput;

													//mUSerial.print("model_outputV after");
													//mUSerial.println(model_outputV,DEC);

				model_outputV = Max(model_outputV, 0.0);
				model_outputV = Min(model_outputV, 1.0);

							//deltaPosition = PositionInput - PositionNow[axis];
							//if (axis>0)
				value_dac[axis] = (uint32_t)(model_outputV*(float)BIT18MAX);
				mAFM_DAC.FinePositioner_MoveToPositionB18(axis, value_dac[axis]);		
				//ADC_read_DAC_write(axis,pV18_Adc_value,axis,value_dac[axis]);// speed up for both read and write
				//fastDigitalWrite(23,false);
				//wait_ms(1);
			}
		}
		//	return value_dac[axis];// bug fix, 20151030
	}	
	

};
#endif // !__AFM_FUNCTIONS__