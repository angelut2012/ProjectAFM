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
#include "RunningMedian.h"
//#include "SPI_ADC_DAC.h"
#include "tic_toc.h"
#include "PID_v1.h"
//#include "I2C_DRpot.h"USB_Debug_Value_LN
#include "console_serial.h"
#include "piezo_feed_forward.h"


/////////////////////////////////////////AFM global members///////////////////////////////////////////////////////////////////////



//#define mUSerial SerialUSB
//#define USB_Debug_Initialize(x)	SerialUSB.begin(x)
//#define USB_Debug(x)  SerialUSB.print(x)
//#define USB_Debug_LN(x)  SerialUSB.println(x)
//#define USB_Debug2(x,y)  SerialUSB.print(x,y)
//#define USB_Debug_LN2(x,y)  SerialUSB.println(x,y)
//#define USB_Debug_StringValue_LN(str,value)  do{SerialUSB.print(str);SerialUSB.println(value);}while(0)
//
#define MY_Debug_LN(x) mUSerial.println(x)
#define MY_Debug_LN2(x,y) mUSerial.println(x,y)
#define MY_Debug(x) mUSerial.print(x)
//#define MY_Debug2(x,y) mUSerial.print(x,y)
#define MY_Debug_StringValue_LN(str,value)  do{mUSerial.print(str);mUSerial.println(value);}while(0)

//the port use for PIEZO_Z and PIEZO_T(tuning fork)
byte Z_scanner_port = SCANNER_Z_ONLY;
#define PIEZO_T_CenterBit18 (140581.0) //80.441/150*2^18
#define PIEZO_T_Center01 (PIEZO_T_CenterBit18/BIT18MAX) //80.441/150*2^18

/////////////////////----------------------------------
// send package16 to PC via RTOS
CPackageToPC* mCPackageToPC_SystemPackage = new CPackageToPC(2);
////////////////////////// median filter


// strain gauge global switch ON_OFF
int switch_read_SG = -1;// >0: read; ==1, read only once; ==2 continuously read
///////////////// approaching
//#if (ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)
//	#define  sampling_period_us_of_Approach_Process (6000.0)
//#else
#define  sampling_period_us_of_Approach_Process (250.0)//old=2000.0
//#endif
#define  sampling_frequency_of_Approach_Process ( 1000000.0/ sampling_period_us_of_Approach_Process)
static uint32_t step_counter_Approach = 0;
static uint32_t step_size_increament_Appraoch = 1;
static uint32_t Z_position_DAC_Approach = 0;
static uint32_t ADC_sensor_buffer = 0;

// z scanner engage
#define  sampling_period_us_of_ZScannerEngage_Process (1000.0)
#define  sampling_frequency_of_ZScannerEngage_Process ( 1000000.0/ sampling_period_us_of_ZScannerEngage_Process)
static uint32_t Z_position_DAC_ZScannerEngage = 0;

//int counter_large_step_approaching=0;
//DueTimer //mTimer_ZLoop = DueTimer(1);
//DueTimer mTimer_Approach = DueTimer(2);
///////////////////////
//typedef 
enum Sys_State {SS_Idle, SS_Approach, SS_Engage, SS_Scan, SS_XYScanReset, SS_Indent};

Sys_State sys_state = SS_Idle;
uint32_t V18_Adc[4] = {0};//
uint32_t V18_Dac[4] = {0};//
double   position_feedforward_output_01[4] = {0};//

int Vdf_infinite = BIT18MAX_HALF;


#define PRC_sensitivity_ADC18_per_nm (41.2)
#if (ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)
double threshold_approach_delta = PRC_sensitivity_ADC18_per_nm * 4;//7nm  threashold to avoid peak-to-peak noise //112*5;// vpp*5 102*4;//87*11;//;3nm112*3;
#else
double threshold_approach_delta = (150.0 / MAX_RANGE_Z_NM*BIT18MAX);//1638;//0.025V/4*bit18=5nm;
#endif
//extern DueTimer //mTimer_ZLoop;

//---------------XY scanning----------------------//
// unit: DAC value: 0~BIT18MAX

////////////////////////// input for  xy scan
int N_FramesToScan = 1;
int XL_NM = 0, DX_NM = 1000, YL_NM = 0, DY_NM = 1000;

int XL = 0, YL = 0, DX = 0, DY = 0; //to be calculated in calculate_scan_parameter()
// XL_NM * DAC_PER_NM_X, DX = DX_NM * DAC_PER_NM_X, 
// YL_NM * DAC_PER_NM_Y, DY = DY_NM * DAC_PER_NM_Y;

//int scan_rate=100;// 100 is 1 line/s
double scan_rate = 0.5;//line per second 
int N_x = 128, N_y = 128;// image size
int diry_input = 0;

//// Zloop
#define SamplingTime_us_Zloop  (400)// (330)//1000 
#define SamplingFrequency_Zloop  (1000000.0/SamplingTime_us_Zloop)// Hz


int mode_pause0_scan1_pending2 = 0;
int y_enable = 1;
int dds_reset = 0;
int enable = 1;
//output for xy scan
int indx = 0, indy = 0;// x y index for image
int VDACx = 0, VDACy = 0, VDACz = 0;//value output to DAC 
int SX = 0, SY = 0;
//typedef int 
enum DDS_XY_Scanner_State {DDS_XY_Reset, DDS_XY_Scan, DDS_XY_Idle};
// sys_ini=0;// 0:initial, power on
//const int DDS_XY_Reset = 0; // slow move;
//const int DDS_XY_Scan = 1; // scan move;
//const int DDS_XY_Idle = 2; //  idle

static DDS_XY_Scanner_State mDDS_XY_Scanner_State = DDS_XY_Reset; // the mDDS_XY_Scanner_State of the system
/////////////////////////////////////////////

/////////////////////// image
#define SIZE_IMAGE_BUFFER_LINES (5)
#define SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE (512)
#define SIZE_IMAGE_BUFFER_FORWARD0_BACKWORD1	(2)
#define SIZE_IMAGE_BUFFER_BIT24	(3)

// store images
//image, [SIZE_IMAGE_BUFFER_FORWARD0_BACKWORD1][SIZE_IMAGE_BUFFER_BIT24]
//[SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]
//forward_backward,uint_24bit_HML3bytes(DAC is 18bit),line_Y,x_points,

////byte pImageHF[SIZE_IMAGE_BUFFER_BIT24][SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};
////// Z height image data
////byte pImageEF[SIZE_IMAGE_BUFFER_BIT24][SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};
////byte pImageHB[SIZE_IMAGE_BUFFER_BIT24][SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};
////// Z height image data
////byte pImageEB[SIZE_IMAGE_BUFFER_BIT24][SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};


// 4 dimentional array is time-consuming
//byte pImageH[SIZE_IMAGE_BUFFER_FORWARD0_BACKWORD1][SIZE_IMAGE_BUFFER_BIT24]
//[SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};
//// Z height image data
//byte pImageE[SIZE_IMAGE_BUFFER_FORWARD0_BACKWORD1][SIZE_IMAGE_BUFFER_BIT24]
//[SIZE_IMAGE_BUFFER_LINES][SIZE_IMAGE_BUFFER_MAX_POINTS_PER_LINE]={0};

//int current_image_line=-1;// curent image line
///////////////////////////////////

//#define VDF_SENSITIVITY_PM_PER_VPN10 (192.0*1000.0)//192 nm/V
//
//#define  CONV_F2V_05_TO_PN10(x) ((-(double)x)*4+10)
//#define  CONV_F2V_VPN10_TO_PM(x)	(x*VDF_SENSITIVITY_PM_PER_VPN10)
//#define  CONV_ADC_VALUE_TO_PM(x)  (CONV_F2V_VPN10_TO_PM(CONV_F2V_05_TO_PN10(x*5/BIT18MAX)))//(192*1000/20*5)/262144;
//
//double sensitivity_pm_per_Vpn10=VDF_SENSITIVITY_PM_PER_VPN10;
//double sensitivity_pm_per_V05=VDF_SENSITIVITY_PM_PER_VPN10*20/5   /1000;// Alao input from rs232
//// input from rs232, when updated each time, recalculate sense_k,sense_b, DSet_01
////double sense_k=-5*sensitivity_pm_per_V05/BIT18MAX/MAX_RANGE_Z_PM;
////double sense_b=10*sensitivity_pm_per_V05/MAX_RANGE_Z_PM;


//double conv_WorkingVoltage(uint32_t VWset_deltaV_input_mV)
//{
//	return 
//		(
//		-((double)VWset_deltaV_input_mV/1000)
//		/(-20.0/5.0)+2.5//500 input, get 2.625V
//		)
//		*sensitivity_pm_per_V05/MAX_RANGE_Z_PM;// 0~1
//}
//double DSet_01=conv_WorkingVoltage(VWset_deltaV_input_mV);//: set working distance (0~1) 


//-------------------------- indentation------------------------------------------------------------------//
const int mI_MaxStep = 4096;//6000;
// indentation parameters, input from GUI
double mI_MaxDepth = 1;//MAX_RANGE_Z_NM for savety reason only
double mI_StiffnessPRC_nN_per_nm = 40.0;//40N/m, 40 nN/nm
double mI_TriggerForce_nN = mI_StiffnessPRC_nN_per_nm * 37.5;// 100nm
double mI_PRC_ADCValue_per_nm = 86.9;//
double mI_LoopDelay_uS = 500;//
int	 mI_HalfNumberOfSamplingPoints = mI_MaxStep / 2;// number of points to sample and store and send back
// paramerter to be calculated
double mI_PRC_ADC_ValueInitial = 0;// initial value of ADC PRC value;
double mI_PRC_Force_nN_per_ADCValue = mI_StiffnessPRC_nN_per_nm / mI_PRC_ADCValue_per_nm;
double mI_PRC_Force_nN_Now = 0;
double mI_step_size_nm = 1;//min=0.0816
double mI_step_size_01 = mI_step_size_nm / MAX_RANGE_Z_NM;// 4.675524715375971e-05
bool mI_direction_indent_True_withdraw_False = true;
#define mI_Withdraw_Done_Threshold_nm (1)// should not be smaller than noise level
double mI_Withdraw_Done_Threshold_ADC = mI_PRC_ADCValue_per_nm*mI_Withdraw_Done_Threshold_nm;


uint32_t mIndentData[3][mI_MaxStep] = {0};

uint32_t VWset_deltaV_input_mV = 500 / 4;// set working voltage;// receive from rs232
double TF_Sinsitivity = 1;//192/2;// nm/V   // receive from rs232
int32_t VWset_deltaV_ADC_b18 = 0;// use in engage//CONV_DELTA_WORKING_VOLTAGE_MV_TO_ADC(VWset_deltaV_input_mV);// update VWset_deltaV_ADC_b18 each time when set VWset_deltaV_input_mV
double mTF_DC_Gain = 1;// gain for tf DC actuation

const double TF_SensorRange = 1000;// nm, assumed range, or Zmax
//double pid_input_Gain_adjust=0;//5.0*TF_Sinsitivity/BIT18MAX/TF_SensorRange;

//
//double DSet_01=(-(double)VWset_deltaV_input_mV/1000.0/4.0+2.5)// voltage 0~5 //2.375V
//	*BIT18MAX/5// adc value 0~2^18
//	*pid_input_Gain_adjust;// pid range 1000 nm
double DSet_01 = 0;
double DInput_01 = 0;//% input from ADC, voltage for delta frequency
double DOutput_01 = 0;

//////////// send image package back tp PC///////////////////
///////send one byte each time
byte com_image_frame_buffer[LENGTH_IMAGE_FRAME_BUFFER] = {0};// 
int pointer_out_frame_buffer = LENGTH_IMAGE_FRAME_BUFFER;

PID mZ_Loop_PID(&DInput_01, &DOutput_01, &DSet_01, 0.05, 0.02, 0, true);// use true for PRC imagining scanning
static double z_output_01 = 0;
byte com_buffer[LENGTH_COM_BUFFER_PC2MCU * 2] = {0};
byte com_buffer_frame[LENGTH_COM_BUFFER_PC2MCU - 4] = {0};

double measured_sampling_frequency_of_system = 0;

//--------------------------------------------------------------------------------------------
extern CSEM mAFM_SEM;

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
}


inline void console_StartIndentation()
{
	//parameters initialize
	mI_PRC_Force_nN_per_ADCValue=mI_StiffnessPRC_nN_per_nm/mI_PRC_ADCValue_per_nm;
	mI_PRC_Force_nN_Now=0;

	mI_step_size_01=mI_step_size_nm/MAX_RANGE_Z_NM;// 4.675524715375971e-05
	mI_direction_indent_True_withdraw_False=true;
	mI_Withdraw_Done_Threshold_ADC=mI_PRC_ADCValue_per_nm*mI_Withdraw_Done_Threshold_nm;
	memset(&(mIndentData[0][0]),0,3*mI_MaxStep*sizeof(mIndentData[0][0]));

	mI_HalfNumberOfSamplingPoints=LIMIT_MAX_MIN(mI_HalfNumberOfSamplingPoints,mI_MaxStep/2,0);// to avoid usefull data to be covered

	//-------------------------
	sys_state=SS_Indent;
	console_ResetScannerModel(PIEZO_Z);
	V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	
	mI_PRC_ADC_ValueInitial=mAFM_SEM.ADC_Read_N_average(ADC_PORT_ZlOOP_SENSOR,20,10);//1.97 kHz
	//ADC_read_average(ADC_PORT_ZlOOP_SENSOR,50,100);
	//USB_Debug_LN("start indent");
}

void send_IndentData_Package_To_PC(uint32_t v1,uint32_t v2,uint32_t v3)
{	
	byte com[LENGTH_COM_BUFFER_MCU2PC]={0};
	com[0]=COM_HEADER1;
	com[1]=COM_HEADER2,
		com[LENGTH_COM_BUFFER_MCU2PC-2]=COM_TAIL1;
	com[LENGTH_COM_BUFFER_MCU2PC-1]=COM_TAIL2;

	com[0+2]='s';
	com[1+2]='p';
	convert_uint32_to_byte4(v1,&com[2+2]);// byte4
	convert_uint32_to_byte3(v2,&com[2+4+2]);// byte3
	convert_uint32_to_byte3(v3,&com[2+4+3+2]);//byte3
	mUSerial.write(com,LENGTH_COM_BUFFER_MCU2PC); 		
}
//
//void process_Indent_First_SendDataThen_temp()
//{
//
//	int dt=200;
//	while(1)
//	{
//		DAC_write(PIEZO_X,0);
//		DAC_write(PIEZO_Y,0);
//	
//		delay(dt);
//		DAC_write(PIEZO_X,BIT18MAX);
//		DAC_write(PIEZO_Y,0);
//		delay(dt);
//		DAC_write(PIEZO_X,BIT18MAX);
//		DAC_write(PIEZO_Y,BIT18MAX);
//		delay(dt);
//		DAC_write(PIEZO_X,0);
//		DAC_write(PIEZO_Y,BIT18MAX);
//		delay(dt);
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
//	DAC_write(PIEZO_Z,0);
//	delay(10);
//	pData[0]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
//	//
//	//DAC_write(PIEZO_Z,BIT18MAX_HALF);
//	////piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,0.5);
//	DAC_write(PIEZO_Z,v1);
//	int st=1;
//	int re=100;
//	int len=data_length/re/2;
//	// sin
//	double N=100;
//	double T=10;
//	double v=0,V=0;
//
//
//	for (int k=0;k<1e6;k++)
//	{
//		V+=T;
//		v=wave_triangle(V,BIT18MAX);
//		DAC_write(PIEZO_Y,v*2);
//		//wait_us(dtime);
//			//pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
//			//wait_us(dtime);
//	}
//
//		for (int m=0;m<4;m++)// send finish signal multi times, to make sure PC receive it
//	{
//		delay(100);
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
//		//v=abs(v);
//		v=wave_triangle(V,BIT18MAX);
//
//		DAC_write(PIEZO_Z,v*13/15);
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
//	//	DAC_write(PIEZO_Z,v2);
//	//	for (int k=st;k<(st+len);k++)
//	//	{
//	//		pData[k]=ADC_start_convert_delay_read(ADC_PORT_ZlOOP_SENSOR,dtime);
//	//		wait_us(dtime);
//	//	}
//	//	st+=len;
//	//	DAC_write(PIEZO_Z,v1);
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
//		delay(7);//old delay=4 , but lose point sometimes
//	}
//	for (int m=0;m<4;m++)// send finish signal multi times, to make sure PC receive it
//	{
//		delay(100);
//		send_IndentData_Package_To_PC(BIT24MAX,BIT24MAX,BIT24MAX);// finished
//	}
//	// tuning fork indent
//	//uint32_t amp_ad0=analogRead(A0);
//	//prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
//}



void process_Indent_First_SendDataThen()
{
	//if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(1300)==false) return;
	int index=0;
	int withdraw_delay_steps=mI_HalfNumberOfSamplingPoints;// add more step while withdrawing
	int index_store_at_max_point=0;
	double V18_Adc_Diff=0;
	//while(1)
#define MAX_INDENT_STEPS (MAX_RANGE_Z_NM*2000)// 0.05 nm/ step

#define PRE_ACC_DISTANCE (0.2)// take 4 um/20um
	double mI_step_size_pre_acceleration=mI_step_size_01/50000;
	double mI_step_size_pre_speed=0;
	//double mI_step_size_pre_step=0;
	position_feedforward_output_01[PIEZO_Z]=10.0/MAX_RANGE_Z_NM;

	for (int k=0;k<MAX_INDENT_STEPS;k++)
	{
		fastDigitalWrite(23,true);
		//while(PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(mI_LoopDelay_uS)==false)
		//	;
		//-----------motion direction 
		if (mI_direction_indent_True_withdraw_False==true)
		{
			if (position_feedforward_output_01[PIEZO_Z]>PRE_ACC_DISTANCE)
				position_feedforward_output_01[PIEZO_Z]+=mI_step_size_01;
			else// acceleration, use 1000 steps to go to position PRE_ACC_DISTANCE
			{
				if (mI_step_size_pre_speed<mI_step_size_01)
					mI_step_size_pre_speed+=mI_step_size_pre_acceleration;
				//mI_step_size_pre_speed=Max(mI_step_size_pre_speed,0.00001);// make sure at lease a little step
				position_feedforward_output_01[PIEZO_Z]+=mI_step_size_pre_speed;
			}
		}
		else
			position_feedforward_output_01[PIEZO_Z]-=mI_step_size_01;
		//------------------------------------------- move and sensing
		//V18_Dac[PIEZO_Z]=
			piezo_predict_Position01_To_Voltage_DAC18
			(PIEZO_Z,position_feedforward_output_01[PIEZO_Z]);//,&V18_Adc[ADC_PORT_ZlOOP_SENSOR]);// time=80 uS
		wait_us(mI_LoopDelay_uS);

		//for noise rms test
		V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
		//use for indent
		//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,25,10);//20_10-->1.97 kHz, noise rms=   11.9427ADC18, 0.311 nm;//40_10-->1.13 kHz


		//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,5,10);// good for capture vibration noise

		//MY_Debug_StringValue_LN("v now: ",V18_Adc[ADC_PORT_ZlOOP_SENSOR]);
		//MY_Debug_StringValue_LN("v inf: ",mI_PRC_ADC_ValueInitial);
		//MY_Debug_StringValue_LN("v ff:",position_feedforward_output_01[PIEZO_Z]);
		//delay(200);


		//--------------------judge status----------
		fastDigitalWrite(23,false);
		// 85 us for DAC and ADC part delay_1us
		// 15 us for the following part
		V18_Adc_Diff=abs((double)V18_Adc[ADC_PORT_ZlOOP_SENSOR]-(double)mI_PRC_ADC_ValueInitial);// use abs for both direction
		mI_PRC_Force_nN_Now= V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;

		// store data
		//index=k%mI_MaxStep;
		index=MOD_range(k,mI_MaxStep); //here % operator has problem with int
		mIndentData[0][index]=(uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX);
		mIndentData[1][index]=V18_Adc[ADC_PORT_ZlOOP_SENSOR];
		mIndentData[2][index]=V18_Dac[PIEZO_Z];

		if	(mI_direction_indent_True_withdraw_False==true)// judge while indenting
		{
			if (mI_PRC_Force_nN_Now>=mI_TriggerForce_nN)// reach trigger force
			{
				// old: trigger 
				//mI_direction_indent_True_withdraw_False=false;// start to withdraw
				//index_store_at_max_point=index;
				//new:  read adc again to avoid sudden noise
				wait_us(10);
				V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,20,10);
				V18_Adc_Diff=abs((double)V18_Adc[ADC_PORT_ZlOOP_SENSOR]-(double)mI_PRC_ADC_ValueInitial);// use abs for both direction
				mI_PRC_Force_nN_Now= V18_Adc_Diff*mI_PRC_Force_nN_per_ADCValue;

				if (mI_PRC_Force_nN_Now>=mI_TriggerForce_nN)
				{
					mI_direction_indent_True_withdraw_False=false;// start to withdraw
					index_store_at_max_point=index;
				}
			}
			if 	(position_feedforward_output_01[PIEZO_Z]>=mI_MaxDepth)// max indent depth without touching, withdraw to 0 directly
				break;	

		}		
		else//if	(mI_direction_indent_True_withdraw_False==false)// judge while withdraw
		{	
			withdraw_delay_steps--;// start delay counter
			if (withdraw_delay_steps<=0) break;
			if (position_feedforward_output_01[PIEZO_Z]<=0)// Z piezo to end		
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
	console_WithDrawZScanner_SetSystemIdle();
	int index_send=0;
	//for (int k=0;k<mI_MaxStep;k++)
	for (int k=(index_store_at_max_point-mI_HalfNumberOfSamplingPoints);k<(index_store_at_max_point+mI_HalfNumberOfSamplingPoints);k++)	
	{
		// keep the order
		//index_send=k%mI_MaxStep; has error
		index_send=MOD_range(k,mI_MaxStep); //here % operator has problem with int
		send_IndentData_Package_To_PC(mIndentData[0][index_send],mIndentData[1][index_send],mIndentData[2][index_send]);
		delay(7);//old delay=4 , but lose point sometimes
	}
	for (int m=0;m<4;m++)// send finish signal multi times, to make sure PC receive it
	{
		delay(100);
		send_IndentData_Package_To_PC(BIT24MAX,BIT24MAX,BIT24MAX);// finished
	}
	// tuning fork indent
	//uint32_t amp_ad0=analogRead(A0);
	//prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
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
//		prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX)
//			,V18_Adc[ADC_PORT_ZlOOP_SENSOR],BIT24MAX);// finished
//		console_WithDrawZScanner_SetSystemIdle();
//	}
//
//	prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX)
//		,V18_Adc[ADC_PORT_ZlOOP_SENSOR],V18_Dac[PIEZO_Z]);
//
//
//	// tuning fork indent
//	//uint32_t amp_ad0=analogRead(A0);
//	//prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],amp_ad0);
//
//}
uint32_t VWset_deltaV_input_mV=500/4;// set working voltage;// receive from rs232
double TF_Sinsitivity=1;//192/2;// nm/V   // receive from rs232
int32_t VWset_deltaV_ADC_b18=0;// use in engage//CONV_DELTA_WORKING_VOLTAGE_MV_TO_ADC(VWset_deltaV_input_mV);// update VWset_deltaV_ADC_b18 each time when set VWset_deltaV_input_mV
double mTF_DC_Gain=1;// gain for tf DC actuation

const double TF_SensorRange=1000;// nm, assumed range, or Zmax
//double pid_input_Gain_adjust=0;//5.0*TF_Sinsitivity/BIT18MAX/TF_SensorRange;

//
//double DSet_01=(-(double)VWset_deltaV_input_mV/1000.0/4.0+2.5)// voltage 0~5 //2.375V
//	*BIT18MAX/5// adc value 0~2^18
//	*pid_input_Gain_adjust;// pid range 1000 nm
double DSet_01=0;
double DInput_01=0;//% input from ADC, voltage for delta frequency
double DOutput_01=0;

//////////// send image package back tp PC///////////////////
///////send one byte each time
byte com_image_frame_buffer[LENGTH_IMAGE_FRAME_BUFFER]={0};// 
int pointer_out_frame_buffer=LENGTH_IMAGE_FRAME_BUFFER;
int rtos_send_image_frame_to_PC()
{
	if (pointer_out_frame_buffer<LENGTH_IMAGE_FRAME_BUFFER)
	{
		mUSerial.write(com_image_frame_buffer[pointer_out_frame_buffer]);
		pointer_out_frame_buffer++;
	}
	return pointer_out_frame_buffer;
}
int rtos_buffer_in_index=0;
void Serial_write_reset_input_output()
{rtos_buffer_in_index=0;pointer_out_frame_buffer=0;}

void Serial_write(byte *d,int L)
{
	for (int k=0;k<L;k++)
		Serial_write(d[k]);
}
inline void Serial_write(byte d) _ALWAYS_INLINE_;
inline void Serial_write(byte d)
{
	com_image_frame_buffer[rtos_buffer_in_index]=d;
	rtos_buffer_in_index++;
}
/////////////////////////////////////////////////////////////////////
void calculate_scan_parameter()
{
	const double VADC_Ref_V=5.0;	

	VWset_deltaV_ADC_b18=(double)(VWset_deltaV_input_mV)/1000.0/VADC_Ref_V*BIT18MAX;

	double VdeltaF_FarAway_01=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,30,100)/BIT18MAX;
#if(ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)
	// for PRC, use VWset_deltaV_input_mV as nm,
	DSet_01=VdeltaF_FarAway_01-(double)(VWset_deltaV_input_mV)/PRC_sensitivity_ADC18_per_nm/BIT18MAX;
#else	// tuning fork
	DSet_01=VdeltaF_FarAway_01-(double)(VWset_deltaV_input_mV)/1000.0/VADC_Ref_V;
#endif	
	/////////////////////
	//XL_NM,YL_NM,DX_NM,DY_NM;
	XL = XL_NM * DAC_PER_NM_X, DX = DX_NM * DAC_PER_NM_X, 
		YL = YL_NM * DAC_PER_NM_Y, DY = DY_NM * DAC_PER_NM_Y;
	//MY_Debug("DSet_01");
	//MY_Debug_LN(DSet_01);
	//USB_Debug("DSet_01");
	//USB_Debug_LN(DSet_01);
}
//for akiyama probe  
PID mZ_Loop_PID(&DInput_01, &DOutput_01, &DSet_01,0.05,0.02,0,true);// use true for PRC imagining scanning
// for madcity lab probe
//PID mZ_Loop_PID(&DInput_01, &DOutput_01, &DSet_01,0.05,0.02,0, DIRECT);// DIRECT 

int y_reverse(int u)
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
	diry_input =  y_reverse(diry_input);
	//  static int px = XL, py = YL;
	//	static int sawx = XL, sawy = YL;
	static int frame_counter = 0;

	// output
	VDACx = 0;
	VDACy = 0;
	indx = 0;
	indy = 0;
	int Nx = (int)((double)SamplingFrequency_Zloop / scan_rate);

	//mUSerial.print("Nx"); 
	//mUSerial.println(Nx,DEC);
	//int Nx = SamplingFrequency_Zloop * scan_second_per_line;
	int Ny = N_y;
	diry_input = -diry_input;
	static int sx = -XL * Nx / DX , sy = -YL * Ny / DY , dirx = 1, diry_core = diry_input; // for index kernel
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

		if  (dds_reset == 1)
			mDDS_XY_Scanner_State = DDS_XY_Reset;
	}
	//////////////////////////////// DDS_XY_Scan

	if (mDDS_XY_Scanner_State == DDS_XY_Scan)
	{
		if ( mode_pause0_scan1_pending2 == 0)
			mDDS_XY_Scanner_State = DDS_XY_Idle;

		if ( dds_reset == 1)
			mDDS_XY_Scanner_State = DDS_XY_Reset;

		// do scan
		if ( frame_counter < N_FramesToScan && enable == 1)
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
			if ( diry_core == diry_input) // to avoid continue adding
			{
				frame_counter = frame_counter + 1;
				if (frame_counter>=N_FramesToScan)// finished predefined frames, stop scanning
					console_XYScanReset();
			}
			diry_core = -diry_input;
		}

	} // scan
	////////////////////////////////////////////// DDS_XY_Reset
	if (mDDS_XY_Scanner_State == DDS_XY_Reset)
	{ 
		if (sx == 0 && sy == 0)
		{
			mDDS_XY_Scanner_State = DDS_XY_Idle;
			dds_reset=0;
		}

		// reset  slow move
		frame_counter = 0;
		dirx = 1;
		diry_core = diry_input;

		//mUSerial.println("sxsy:");
		//mUSerial.println(sx,DEC);
		//mUSerial.println(sy,DEC);
		int  stepx = max(1,abs(sx) >> 12)*SIGN(sx); //sx/3000
		int  stepy = max(1,abs(sy) >> 12)*SIGN(sy); // xy/3000
		if (enable == 1)
		{
			if (sx != 0)
				sx = sx - stepx;

			if (sy != 0)
				sy = sy - stepy;

		}
		if (abs(sx) < 1) sx = 0;
		if (abs(sy) < 1) sy = 0;
	}// reset
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

	////    indsx=(sx*double((dirx==-1))+(2*Nx-sx)*double((dirx==1)))/2;

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

	//V18_Dac[PIEZO_X]= 
		piezo_predict_Position01_To_Voltage_DAC18 (PIEZO_X,(double)VDACx/(double)BIT18MAX);	
	//V18_Dac[PIEZO_Y]= 
		piezo_predict_Position01_To_Voltage_DAC18 (PIEZO_Y,(double)VDACy/(double)BIT18MAX);	

	//double piezo_model=(double)VDACx/(double)BIT18MAX;

	//piezo_model=piezo_predict_Position01_To_Voltage_DAC18 (PIEZO_X,piezo_model);
	//piezo_model*=(double)BIT18MAX;
	//VDACx=(int)piezo_model;	
	//DAC_write(1, (uint32_t)VDACx);


	//piezo_model*=(double)BIT18MAX;
	//VDACy=(int)piezo_model;
	//DAC_write(2, (uint32_t)VDACy);
	//dac_output(VDACx, VDACy, VDACz);
	//mUSerial.println(VDACx,DEC);
	//mUSerial.println(piezo_model*1000,DEC);
	return mDDS_XY_Scanner_State;
}
void XYscanning_Initialize()
	// also call initialize when change dds parameters XL YL DX DY
{
	dds_reset=1;
	mode_pause0_scan1_pending2=0;

	sys_state=SS_XYScanReset;
	// put this into main loop, execute once each loop
	//for (int k=0;k<50;k++)
	//{XYscanning();wait_us(500);}
}
void process_XYscanningReset()
{
	int state=XYscanning();
	delay(1);// otherwise, it is too fast
	if (state==DDS_XY_Idle)
		sys_state=SS_Idle;

}
//  int indx = 0, indy = 0, VDACx = 0, VDACy = 0, sys_state_out = 0, SX = 0, SY = 0;
////double DInput_01=-1000;//% input from ADC, voltage for delta frequency
////double DSet_01=0;// set working voltage
////double DOutput_01=0;

static double z_output_01=0;
void process_ScanRealTimeLoop()
{
	if (PERIOD_CHECK_TIME_US_DUE_Z_REAL_TIME_SERVO_LOOP(SamplingTime_us_Zloop)==false) return;

	//DIGITAL_PIN_TOGGLE(22);
	fastDigitalWrite(22,true);
	//////////////////////////////////////
	//ADC_read_DAC_write(ADC_PORT_ZlOOP_SENSOR,&V18_Adc[ADC_PORT_ZlOOP_SENSOR],PIEZO_Z,V18_Dac[PIEZO_Z]);
	V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	DInput_01=(double)V18_Adc[ADC_PORT_ZlOOP_SENSOR]*BIT18MAX_RECIPROCAL;
	mZ_Loop_PID.Compute();

	//USB_Debug("DInput_01 ");
	//USB_Debug_LN(DInput_01);
	//USB_Debug("DOutput_01 ");
	//USB_Debug_LN(DOutput_01);

	z_output_01+=DOutput_01;
	z_output_01=LIMIT_MAX_MIN(z_output_01,1,0);

	//mUSerial.print("_zoutput_01 accumu ");
	//mUSerial.println(z_output_01,DEC);
	//V18_Dac[PIEZO_Z]=(DOutput_01/(double)MAX_RANGE_Z_PM)*(double)BIT18MAX;
	//piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,(double)V18_Dac[PIEZO_Z]/(double)BIT18MAX);
	//double z_output_01=Z_position_pm/(double)MAX_RANGE_Z_PM;	

	//// for piezo_z and tuning fork switch
	//if (Z_scanner_port==PIEZO_Z)
	//	V18_Dac[PIEZO_Z]=piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,z_output_01);
	//else
	//{
	//	V18_Dac[Z_scanner_port]=(z_output_01-PIEZO_T_Center01)*BIT18MAX+PIEZO_T_CenterBit18;
	//	DAC_write(PIEZO_T,V18_Dac[Z_scanner_port]);
	//}

	if (Z_scanner_port==SCANNER_Z_ONLY)// Z piezo only with 
		//V18_Dac[PIEZO_Z]=
		piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,z_output_01);
	else
		if (Z_scanner_port==SCANNER_Z_LPF)// Z piezo only with LPF
		{
			double z_output_01_lpf=IIR_filter_LPF(z_output_01);
			//V18_Dac[PIEZO_Z]=
				piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,z_output_01_lpf);
		}
		else
			if (Z_scanner_port==SCANNER_T_ONLY)// switched
			{	
				V18_Dac[PIEZO_T]=(z_output_01*mTF_DC_Gain+PIEZO_T_Center01)*BIT18MAX;
			}
			else
				if (Z_scanner_port==SCANNER_ZT)// switched
				{
					double z_output_01_hpf=IIR_filter_HPF(z_output_01);
					V18_Dac[PIEZO_T]=(z_output_01_hpf*mTF_DC_Gain+PIEZO_T_Center01)*BIT18MAX;
					V18_Dac[PIEZO_T]=LIMIT_MAX_MIN(V18_Dac[PIEZO_T],1,BIT18MAX);
					mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_T,V18_Dac[PIEZO_T]);

					double z_output_01_lpf=IIR_filter_LPF(z_output_01);
					//V18_Dac[PIEZO_Z]=
						piezo_predict_Position01_To_Voltage_DAC18(PIEZO_Z,z_output_01_lpf);
				}
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

				int	xy_state=XYscanning();

				////TICX(2);
				////store image
				//int current_image_line=indy;
				//current_image_line%=SIZE_IMAGE_BUFFER_LINES;
				////byte f0b1=0;
				////if (indx>=0) f0b1=0;
				////if (indx<0)  f0b1=1;
				//byte valueH3b[SIZE_IMAGE_BUFFER_BIT24]={0};
				//uint32_t vH24=(uint32_t)(z_output_01*(double)BIT24MAX);
				//convert_uint32_to_byte3(vH24,valueH3b);

				//byte valueE3b[SIZE_IMAGE_BUFFER_BIT24]={0};
				//uint32_t vE24=(uint32_t)(mZ_Loop_PID.GetError()*(double)BIT24MAX);
				//convert_uint32_to_byte3(vE24,valueE3b);
				////6 us
				//for (int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
				//{
				//	if (indx>=0) 
				//	{
				//		pImageHF[k][current_image_line][abs(indx)]=valueH3b[k];
				//		pImageEF[k][current_image_line][abs(indx)]=valueE3b[k];
				//	}
				//	else
				//	{
				//		pImageHB[k][current_image_line][abs(indx)]=valueH3b[k];
				//		pImageEB[k][current_image_line][abs(indx)]=valueE3b[k];
				//	}

				//}// 7us
				////TOCX_P(2);

				if(xy_state==(int)DDS_XY_Scan)
					prepare_image_package_to_PC(indx,indy,z_output_01,mZ_Loop_PID.GetError());
				else// after engage send out (0,0) point continuously
					prepare_engaged_package_to_PC(indx,indy,z_output_01,mZ_Loop_PID.GetError());

				//test prepare_image_package_to_PC(indx,indy,0.5,0.1);
				fastDigitalWrite(22,false);
}
///////////////////////////////// console
byte com_buffer[LENGTH_COM_BUFFER_PC2MCU * 2] = {0};
byte com_buffer_frame[LENGTH_COM_BUFFER_PC2MCU-4] = {0};
float convert_byte4_to_float(byte* pb) _ALWAYS_INLINE_;
float convert_byte4_to_float(byte* pb)
	// accuracy 10^-4
{
	float* pf=(float*)(pb);
	float fx=*pf;
	return fx;
	//float x=4.5f;
	//byte b[4]={102,64,156,197};
	//float xx=10000.565556546551f;//convert result--->10000.5654296875, accuracy 10^-4
	//byte *pb=(byte*)&xx;
	//float x=convert_byte4_to_float(pb);
	//mUSerial.println(x,20);
	//delay(1000);
	//return;
}
byte console_Parameter(byte* com_buffer_frame)
{

	//int v=0;
	//v+= com_buffer_frame[2]<<24;
	//v+= com_buffer_frame[3]<<16;
	//v+= com_buffer_frame[4]<<8;	
	//v+= com_buffer_frame[5];
	//double vf=(double)v/16384.0;// 131071.0;//2^17
	float fv=convert_byte4_to_float(&com_buffer_frame[2]);
	double vf=(double)fv;
	byte para = com_buffer_frame[1];
	MY_Debug(para);
	MY_Debug_LN(vf);	
	// PID parameter
	if(para=='R') {TF_Sinsitivity=vf;}
	if(para=='P') mZ_Loop_PID.SetPID_P(vf*TF_Sinsitivity);
	if(para=='I') mZ_Loop_PID.SetPID_I(vf*TF_Sinsitivity);
	if(para=='D') mZ_Loop_PID.SetPID_D(vf*TF_Sinsitivity);

	// when set:TF_Sinsitivity, also recalculate pid_input_Gain_adjustm, DSet_01
	if(para=='X') {console_XYScanReset();N_x=vf;}
	if(para=='Y') {console_XYScanReset();N_y=vf;}
	if(para=='x') {console_XYScanReset();DX_NM=vf;}
	if(para=='y') {console_XYScanReset();DY_NM=vf;}
	if(para=='m') {console_XYScanReset();XL_NM=vf;console_XYScanReset();}
	if(para=='n') {console_XYScanReset();YL_NM=vf;console_XYScanReset();}

	if(para=='S') {console_XYScanReset();scan_rate=vf;}
	if(para=='W') {VWset_deltaV_input_mV=vf;calculate_scan_parameter();}
	if(para=='N') N_FramesToScan=vf;
	//indentation
	if(para=='T') mI_TriggerForce_nN=vf;
	if(para=='k') mI_StiffnessPRC_nN_per_nm=vf;
	if(para=='s') mI_PRC_ADCValue_per_nm=vf;// sensitivity
	if(para=='d') mI_MaxDepth=vf/MAX_RANGE_Z_NM;
	if(para=='e') mI_step_size_nm=vf;
	if(para=='f') mI_LoopDelay_uS=vf;
	if(para=='g') mI_HalfNumberOfSamplingPoints=vf/2;

	//if(para=='t') mTF_DC_Gain=vf;

	//if(para=='a') {Z_scanner_port=vf;console_TF_Scan_Enable();}
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

	byte  mDDS_XY_Scanner_State = write_digital_potentiometer(address_IC, address_channel, R_value);

	mUSerial.print(address_IC,DEC);
	mUSerial.print(address_channel,DEC);
	mUSerial.println(R_value,DEC);
	DIGITAL_PIN_TOGGLE(23);
	return mDDS_XY_Scanner_State;
}

void console_PiezoFeedforward_output(byte* com_buffer_frame)
	//AA 55 44 FF 00 03 FF FF 55 AA
	//AA 55 44 00 00 01 00 00 55 AA
{
	byte channel = com_buffer_frame[1];	
	uint32_t value = 0;
	value+=com_buffer_frame[2]<<24;
	value+=com_buffer_frame[3]<<16;
	value+=com_buffer_frame[4]<<8;
	value+=com_buffer_frame[5];
	//value=Min(value,BIT18MAX);
	//value=Max(value,0);

	double position_output_01=(double)value/BIT32MAX;	

	if(channel<4)
	{
		//V18_Dac[channel]=
			piezo_predict_Position01_To_Voltage_DAC18(channel,position_output_01);
		position_feedforward_output_01[channel]=position_output_01;
	}
	if(channel==0xff)// set value for all channel
		for (int i = 0; i < 4; i++)
		{
			//V18_Dac[i]=
				piezo_predict_Position01_To_Voltage_DAC18(i,position_output_01);
			position_feedforward_output_01[i]=position_output_01;
		}

}

void console_DAC_output(byte* com_buffer_frame)
	//AA 55 44 FF 00 03 FF FF 55 AA
	//AA 55 44 00 00 01 00 00 55 AA
{
	byte channel = com_buffer_frame[1];	
	uint32_t value = 0;
	value+=com_buffer_frame[2]<<24;
	value+=com_buffer_frame[3]<<16;
	value+=com_buffer_frame[4]<<8;
	value+=com_buffer_frame[5];
	value=Min(value,BIT18MAX);
	value=Max(value,0);

	if (channel==PIEZO_Z)
		V18_Dac[PIEZO_Z]=value;// hold the value in Z axis
	if(channel<4)
		DAC_write(channel,value);
	if(channel==0xff)// set value for all channel
	{
		V18_Dac[PIEZO_Z]=value;// hold the value in Z axis
		DAC_write(0,value);
		DAC_write(1,value);
		DAC_write(2,value);
		DAC_write(3,value);
	}
}
//void console_StartApproach_only_coarse_move()
//{
//	sys_state=SS_Approach;
//	Vdf_infinite=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);	
//	DAC_write(PIEZO_Z,BIT18MAX);
//	delay(20);
//
//	//if(Vdf_infinite>127140 | Vdf_infinite<119276)//([-0.3 -0.9]/4+2.5)/5*2^18=127139.84,119275.52
//	if(abs(Vdf_infinite-BIT18MAX_HALF)<7864)
//	{
//		sys_state=SS_Idle;
//		send_back_approach_heart_beat(Vdf_infinite,0,255);//send back an package: initial vdf error
//	}
//}
uint32_t ADC_read_MedianFilter(byte port, int num,int delay_time_us)
{
	RunningMedian mSortFilter= RunningMedian(num);
	mSortFilter.clear();
	for(int k=0;k<mSortFilter.getSize();k++)
	{
		mSortFilter.add(mAFM_SEM.ADC_Read_N(port));
		wait_us(delay_time_us);
	}
	return  mSortFilter.getMedian();
}
uint32_t ADC_read_average(byte port, int num,int delay_time_us)
{
	if (num==1)
		return mAFM_SEM.ADC_Read_N(port);

	double t=0;
	//int num=30;
	for (int k=0;k<num;k++)
	{
		wait_us(delay_time_us);
		t+=(double)mAFM_SEM.ADC_Read_N(port);
	}
	t/=(double)num;
	return (uint32_t)t;
}


void console_StartApproach()
{
	//mUSerial.println(sys_state,DEC);
	if(sys_state==SS_Approach) 
	{
		//mTimer_Approach.start();
		return;// avoid multi trigger from GUI
	}
	sys_state=SS_Approach;
	//DAC_write(PIEZO_Z,0);	
	console_ResetScannerModel(PIEZO_Z);

	delay(50);
	Vdf_infinite=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR,20,100);// coarse positioner could vibrate
	//ADC_read_average(ADC_PORT_ZlOOP_SENSOR,100,100);;

	//while(1)
	//{Vdf_infinite=(int)mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);	
	//	mUSerial.println(Vdf_infinite);
	//	delay(10);
	//}
	//	counter_large_step_approaching=0;// reset counter
	//if(Vdf_infinite>127140 | Vdf_infinite<119276)//([-0.3 -0.9]/4+2.5)/5*2^18=127139.84,119275.52
#if (ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)
	//	if(ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)		
	if(abs(Vdf_infinite-(int)BIT18MAX_HALF)>((int)BIT18MAX_HALF-5000))// leave at least 5000 ADC range for motion
	{
		console_ResetScannerModel(PIEZO_Z);
		send_back_approach_heart_beat(Vdf_infinite,0,255);//send back an package: initial vdf error
	}
#else
	//if(ADC_PORT_ZlOOP_SENSOR==ADC_PORT_TUNING_FORK)
	if(abs(Vdf_infinite-(int)BIT18MAX_HALF)>7864)
	{
		console_WithDrawZScanner_SetSystemIdle();
		send_back_approach_heart_beat(Vdf_infinite,0,255);//send back an package: initial vdf error
	}	
#endif
	// each time reset
	step_counter_Approach=0;
	step_size_increament_Appraoch=1;
	Z_position_DAC_Approach=0;
	ADC_sensor_buffer=Vdf_infinite;
	send_back_approach_heart_beat(Vdf_infinite,0,2);// received approach command from PC, send back Vdf_infinite
}
void console_CancelApproach() 
{
	// each time reset
	step_counter_Approach=0;
	Z_position_DAC_Approach=0;
	console_WithDrawZScanner_SetSystemIdle();
}

void console_StartZScannerEngage()
{
	console_ResetScannerModel(PIEZO_Z);
	Sleep(100);
	Vdf_infinite =ADC_read_average(ADC_PORT_ZlOOP_SENSOR,50,100);
	sys_state=SS_Engage;
	Z_position_DAC_ZScannerEngage=0;
}

//#define console_YScan_Enable() y_enable = 1 

inline void console_YScan_Enable(){y_enable = 1;}
inline void console_YScan_Disable(){y_enable = 0;}

inline void  console_XYScanReset() _ALWAYS_INLINE_;
inline void  console_XYScanReset()
{
	console_WithDrawZScanner_SetSystemIdle();
	calculate_scan_parameter();
	XYscanning_Initialize();
} 
void console_XYScanStart()
{	//if (pImage!=NULL)
	//	delete pImage;
	if (sys_state==SS_Scan)	
		//if the system was scanning, then pause by user, 
			//then recover from pause can directly go to scan
				mode_pause0_scan1_pending2=1;
	else
	{
		console_StartZScannerEngage();
		mode_pause0_scan1_pending2=2;// wait for Z scanner engage to be finished and then start to scan
	}
};
inline void console_XYScanPause(){mode_pause0_scan1_pending2=0;}

void console_GetData(byte* com)
{
	if (com[1]=='A')// read ADC  & com[2]=='P') 
		;
	if (com[1]=='S' & com[2]=='P') 
		send_back_debug_infomation();
	if (com[1]=='s' & com[2]=='g') 
		console_ReadStrainGaugeData(com[3]);
}
inline void console_ReadStrainGaugeData(int value){switch_read_SG=value;}
//void console_TF_Scan_Enable()
//{
//	//Z_scanner_port=PIEZO_T;
//	V18_Dac[PIEZO_T]=PIEZO_T_CenterBit18;
//	mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_T,V18_Dac[PIEZO_T]);
//
//	z_output_01=0.5;
//	mZ_Loop_PID.Reset();
//}
//void console_TF_Scan_Disable()
//{
//	Z_scanner_port=SCANNER_Z_ONLY;
//
//}

void console_Control(byte* com)
{


	if (com[1]=='A' & com[2]=='P') 
		console_StartApproach();
	if (com[1]=='A' & com[2]=='C') 
		console_CancelApproach();

	if (com[1]=='Z' & com[2]=='E') 
	{
		console_WithDrawZScanner_SetSystemIdle();
		console_StartZScannerEngage();
	}
	if (com[1]=='Z' & com[2]=='W') 
		console_WithDrawZScanner_SetSystemIdle();

	if (com[1]=='Y' & com[2]=='E') 
		console_YScan_Enable();
	if (com[1]=='Y' & com[2]=='D') 
		console_YScan_Disable();
	//// tf scan switch with Z piezo
	//if (com[1]=='T' & com[2]=='E') 
	//	console_TF_Scan_Enable();
	//if (com[1]=='T' & com[2]=='D') 
	//	console_TF_Scan_Disable();

	if (com[1]=='S')
		if(com[2]=='R') console_XYScanReset();
		else if(com[2]=='S') console_XYScanStart();
		else if(com[2]=='P') console_XYScanPause();
		if (com[1]=='I' & com[2]=='D') 
			console_StartIndentation();
}
int command_console(bool echo_b)
{
	int byte_ready=mUSerial.available();// time consumption= 84 us
	if ( byte_ready==0) return 0;
	//Rval = mUSerial.read();
	mUSerial.readBytes(com_buffer, LENGTH_COM_BUFFER_PC2MCU * 2);
	for (int k = 0; k < LENGTH_COM_BUFFER_PC2MCU + 1; k++)
		if (com_buffer[k] == COM_HEADER1)
			if (com_buffer[k + 1] == COM_HEADER2)
				if (com_buffer[k + LENGTH_COM_BUFFER_PC2MCU - 2] == COM_TAIL1)
					if (com_buffer[k + LENGTH_COM_BUFFER_PC2MCU - 1] == COM_TAIL2)
					{
						if (echo_b==true) mUSerial.write(com_buffer, LENGTH_COM_BUFFER_PC2MCU); //echo
						for (byte ind = 0; ind < LENGTH_COM_BUFFER_FRAME; ind++)
							com_buffer_frame[ind] = com_buffer[k + ind+2];
						memset(com_buffer, 0, sizeof(com_buffer));
						if (echo_b==true) mUSerial.write(com_buffer_frame, LENGTH_COM_BUFFER_FRAME); //echo

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
						if (com_buffer_frame[0]=='r' 
							& com_buffer_frame[1]=='s' 
							& com_buffer_frame[2]=='t') 
							Software_Reset();


					}
					return com_buffer_frame[0];
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
void read_SG_data(int* pSwitch_read_SG)
{
#define sampling_time_us_read_SG_data (1000000.0/10.0)//max<=24Hz
	if (PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(sampling_time_us_read_SG_data)==false) return;

	/// for I2C communication 
#define ADDRESS_I2C_SLAVE   (0x50)   /// unsigned int 	// The following is not required: chipAddress = (chipAddress<<1) | (0<<0);	
	// 	Wire1.beginTransmission(chipAddress); 
	// 	Wire1.write(0x20); 
	// 	Wire1.endTransmission();
	// 	delay(10); 
	fastDigitalWrite(23,true);

	byte Buffer_SG_data [LENGTH_I2C_DATA_SG]={0};
	Wire1.requestFrom(ADDRESS_I2C_SLAVE, LENGTH_I2C_DATA_SG);// time use=586 us
	static byte frame_counter=0;
	while(Wire1.available()>=LENGTH_I2C_DATA_SG)    // slave may send less than requested
	{	
		for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
		{
			Buffer_SG_data[k] = Wire1.read();    // receive a byte as character
			//mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
			//mUSerial.print(" ");
			//mUSerial.println("raw");
		}

		if (frame_counter!=Buffer_SG_data[0])
		{
			frame_counter=Buffer_SG_data[0];
			//send_SG_rs232();
			byte com[LENGTH_COM_BUFFER_MCU2PC-4]={"SG"};
			com[2]=9;
			memcpy(&com[3],&Buffer_SG_data[1],9);
			if (*pSwitch_read_SG>0)
				send_system_package16_char_to_PC((char*)com);
			if (*pSwitch_read_SG==1) *pSwitch_read_SG=0;// read only once
			//for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
			//{mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
			//mUSerial.print(" ");}
			//mUSerial.println("d");
		}
	}
	fastDigitalWrite(23,false);
}


//void setup()
//{
//  Wire.begin(4);                // join i2c bus with address #4
//  Wire.onReceive(receiveEvent); // register event
//  mUSerial.begin(9600);           // start serial for output
//}
//
//void loop()
//{
//  delay(100);
//}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	while(1 < Wire.available()) // loop through all but the last
	{
		char c = Wire.read(); // receive byte as a character
		mUSerial.print(c);         // print the character
	}
	int x = Wire.read();    // receive byte as an integer
	mUSerial.println(x);         // print the integer
}
void convert_uint32_to_byte4(uint32_t x, byte*b)
{
	//value+=com_buffer_frame[2]<<24;
	//value+=com_buffer_frame[3]<<16;
	//value+=com_buffer_frame[4]<<8;
	//value+=com_buffer_frame[5];
	int L[]={24,16,8,0};
	for (int k=0;k<4;k++)
		b[k] = (byte)((x & (0xff<<L[k]))>>L[k]);
}
void convert_uint32_to_byte3(uint32_t x, byte*b)
{
	//value+=com_buffer_frame[2]<<24;
	//value+=com_buffer_frame[3]<<16;
	//value+=com_buffer_frame[4]<<8;
	//value+=com_buffer_frame[5];
	int L[]={16,8,0};
	for (int k=0;k<3;k++)
		b[k] = (byte)((x & (0xff<<L[k]))>>L[k]);
}
void convert_uint32_to_byte2(uint32_t x, byte*b)
{
	//value+=com_buffer_frame[2]<<24;
	//value+=com_buffer_frame[3]<<16;
	//value+=com_buffer_frame[4]<<8;
	//value+=com_buffer_frame[5];
	int L[]={8,0};
	for (int k=0;k<2;k++)
		b[k] = (byte)((x & (0xff<<L[k]))>>L[k]);
}

//send back heart beat, 16 bytes
void send_back_approach_heart_beat(uint32_t v,uint32_t step,byte done)
{
	//static unsigned long lastTime =0;
	//if (millis()-lastTime<500000) return;	
	//// send @ each 500,000us
	//lastTime=millis();// update timer		
	//	if (step%1000==0)
	//for (int k=0;k<5;k++)
	{
		byte com[]={COM_HEADER1,COM_HEADER2,
			'C','A','P',
			0,0,0,0,//5
			0,0,0,0,//9
			(byte)done,
			COM_TAIL1,COM_TAIL2};
		convert_uint32_to_byte4(v,&com[5]);
		convert_uint32_to_byte4(step,&com[9]);
		mUSerial.write(com,sizeof(com) / sizeof(com[0]));
		delay(2);
	}
}
//void send_system_package16_char_to_PC()
void send_system_package16_char_to_PC(const char*inf)
{
	//mUSerial.println(inf);

	byte com[LENGTH_COM_BUFFER_MCU2PC]={0};
	com[0]=COM_HEADER1;
	com[1]=COM_HEADER2,
		com[LENGTH_COM_BUFFER_MCU2PC-2]=COM_TAIL1;
	com[LENGTH_COM_BUFFER_MCU2PC-1]=COM_TAIL2;
	memcpy(&com[2],inf,Min(strlen(inf),LENGTH_COM_BUFFER_MCU2PC-4));
	mUSerial.write(com,LENGTH_COM_BUFFER_MCU2PC); 
}
//void send_system_package16_to_PC(double Z_position01,uint32_t adc18,uint32_t dac18)
//{
//	byte com[LENGTH_COM_BUFFER_MCU2PC]={0};
//	com[0]=COM_HEADER1;
//	com[1]=COM_HEADER2,
//	com[LENGTH_COM_BUFFER_MCU2PC-2]=COM_TAIL1;
//	com[LENGTH_COM_BUFFER_MCU2PC-1]=COM_TAIL2;
//	com[2]='s';
//	com[3]='p';
//
//	byte vB3[3]={0};
//	uint32_t v24=(uint32_t)(Z_position01*(double)BIT24MAX);
//	convert_uint32_to_byte3(v24,vB3);
//	memcpy(&com[5],vB3,3);
//
//	convert_uint32_to_byte3(adc18,vB3);	
//	memcpy(&com[8],vB3,3);
//
//	convert_uint32_to_byte3(dac18,vB3);	
//	memcpy(&com[11],vB3,3);		
//	
//	mUSerial.write(com,LENGTH_COM_BUFFER_MCU2PC); 
//}
double some_function( double x, double y)
{
	struct InnerFuncs
	{
		double inner_function(double x)
		{ 
			// some code
			return x*x;
		}
		// put more functions here if you wish...
	} inner;

	double z;
	z = inner.inner_function(x);
	return z+y; 
}

//////////////////////-- process-------------------
void process_ZScannerEngage()
{
	//when time due, continue to do approaching work
	if (PERIOD_CHECK_TIME_US_DUE_ZSCANNERENGAGE(sampling_period_us_of_ZScannerEngage_Process)==false) return;

	//DIGITAL_PIN_TOGGLE(23);
	fastDigitalWrite(23,true);

	//static uint32_t Z_position_DAC_ZScannerEngage=0;
	V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//////////////////////////
	mZ_Loop_PID.Reset();
	//mUSerial.println(VWset_deltaV_ADC_b18,DEC);
	if (abs((int32_t)Vdf_infinite-(int32_t)V18_Adc[ADC_PORT_ZlOOP_SENSOR])>VWset_deltaV_ADC_b18)// reach working woltage
	{	

		sys_state=SS_Scan;
		send_system_package16_char_to_PC("CZEdone");
		if (mode_pause0_scan1_pending2==2)// pending for scan
		{
			mode_pause0_scan1_pending2=1;
		}
		// when z piezo engaged, run z loop to servo the tip
		process_ScanRealTimeLoop_Initialize((double)Z_position_DAC_ZScannerEngage/BIT18MAX);// start to run Zloop periodically
		Z_position_DAC_ZScannerEngage=0;
		return;
	}
	////////////////// STEP MOVE
#define TIME_Z_SCANNER_ENGAGE (10.0)//Second
#define STEP_SIZE_Z_SCANNER_ENGAGE (BIT18MAX_0D75/(TIME_Z_SCANNER_ENGAGE*sampling_frequency_of_ZScannerEngage_Process))

	Z_position_DAC_ZScannerEngage+=(STEP_SIZE_Z_SCANNER_ENGAGE);	

	// fail to engage, because Z scanner elongate to 0.75
	if (Z_position_DAC_ZScannerEngage>BIT18MAX_0D75)
	{
		console_WithDrawZScanner_SetSystemIdle();
		Z_position_DAC_ZScannerEngage=0;
		send_system_package16_char_to_PC("CZEfail");
		return;
	}

	mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z,Z_position_DAC_ZScannerEngage);
	V18_Dac[PIEZO_Z]=Z_position_DAC_ZScannerEngage;
	fastDigitalWrite(23,false);
}
//void process_Approach_only_coarse_move()// only coarse move
//{
//	static uint32_t step_counter_Approach=0;
//	//step_counter_Approach++;
//	//ADC_read_DAC_write(0,&V18_Adc[ADC_PORT_ZlOOP_SENSOR],PIEZO_Z,BIT18MAX);
//	int vdf=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
//	if (vdf>Vdf_infinite+threshold_approach_delta)// touched
//	{
//		DAC_write(PIEZO_Z,0);
//		sys_state=SS_Idle;
//		step_counter_Approach=0;
//		send_back_approach_heart_beat(vdf,step_counter_Approach,1);
//	}
//	else
//		if ((step_counter_Approach++)%100000==0)// control the step time
//			send_back_approach_heart_beat(vdf,step_counter_Approach,0);
//	if (step_counter_Approach> 1250000000)//  10,000,000/800
//	{
//		DAC_write(PIEZO_Z,0);
//		sys_state=SS_Idle;
//		step_counter_Approach=0;
//		send_back_approach_heart_beat(vdf,step_counter_Approach,128);// max steps exceeded
//	}
//}
void process_Approach()// fine probing + coarse move
{
	//when time due, continue to do approaching work
	if (PERIOD_CHECK_TIME_US_DUE_APPROACH(sampling_period_us_of_Approach_Process)==false) return;
	//DIGITAL_PIN_TOGGLE(23);
	fastDigitalWrite(23,true);
	//static uint32_t step_counter_Approach=0;
	//static uint32_t Z_position_DAC_Approach=0;
	step_counter_Approach++;
	V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=ADC_read_average(ADC_PORT_ZlOOP_SENSOR,5,50);//20_10-->1.97 kHz, 40_10-->1.13 kHz
	uint32_t vdf=V18_Adc[ADC_PORT_ZlOOP_SENSOR];

	if (
		abs((double)vdf-(double)Vdf_infinite)>threshold_approach_delta// touched
		|
		abs((double)vdf-(double)ADC_sensor_buffer)>threshold_approach_delta
		)
	{		
		console_WithDrawZScanner_SetSystemIdle();
		//		counter_large_step_approaching++;
		send_back_approach_heart_beat(vdf,step_counter_Approach,1);
		step_counter_Approach=0;
		Z_position_DAC_Approach=0;
		// done, GUI use step_counter_Approach to adjust coarse position
		return;
	}
	ADC_sensor_buffer=vdf;// update
#if (ADC_PORT_ZlOOP_SENSOR==ADC_PORT_PRC)
#define TIME_APPROACHING_COARSE_STEP (1)//(2.5)//Second
#else
#define TIME_APPROACHING_COARSE_STEP (5)//5 Second, tuning fork
#endif
#define STEP_SIZE_APPROACHING (BIT18MAX_0D9/(TIME_APPROACHING_COARSE_STEP*sampling_frequency_of_Approach_Process))

	if (step_size_increament_Appraoch<STEP_SIZE_APPROACHING)// step size increase slowly to avoid vibration in the beginning
		step_size_increament_Appraoch++;
	Z_position_DAC_Approach+=step_size_increament_Appraoch;//(STEP_SIZE_APPROACHING);
	//	if (Z_position_DAC_Approach>BIT18MAX_HALF)// one big step finished without touch
	if (Z_position_DAC_Approach>BIT18MAX_0D9)// one big step finished without touch
	{
		// slow return to avoid vibration
		double z_return=Z_position_DAC_Approach;
		while(z_return>0)
		{
			z_return-=STEP_SIZE_APPROACHING;
			if (z_return<0) break;
			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z,z_return);
		}

		//console_ResetScannerModel(PIEZO_Z);
		console_WithDrawZScanner_SetSystemIdle();// let PC move coarse and re-trigger
		send_back_approach_heart_beat(vdf,step_counter_Approach,0);// one large step finished
		step_counter_Approach=0;
		Z_position_DAC_Approach=0;
		return;
	}
	// generate a vibration to tapping

	mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z,Z_position_DAC_Approach);

	double dv=5.0*BIT18MAX/MAX_RANGE_Z_NM;// vibration
	for (int k=0;k<5;k++)
	{
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z,Z_position_DAC_Approach+dv);
		mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z,Z_position_DAC_Approach);
	}

	fastDigitalWrite(23,false);
}
void prepare_engaged_package_to_PC(int indx,int indy,double vHeight,double vError)
{
	if (pointer_out_frame_buffer==LENGTH_IMAGE_FRAME_BUFFER)
	{
		Serial_write_reset_input_output();
		prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
		prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
		prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
	}
}
void prepare_image_package_to_PC(int indx,int indy,double vHeight,double vError)
{
	//// avoid send the same point
	static int indx_store=-1000;
	if (indx==indx_store) return;
	indx_store=indx;

	// test
	//vHeight=0.1;
	//vError=0.8;

	Serial_write_reset_input_output();
	prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
	prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
	prepare_image_package_to_PC_sub(indx,indy,vHeight,vError);
}
void prepare_image_package_to_PC_sub(int indx,int indy,double vHeight,double vError)
	//	vHeight=0:1, vError=-1:1
{
	Serial_write(COM_HEADER1);
	Serial_write(COM_HEADER2);
	Serial_write('I');
	Serial_write('M');
	byte ind[2]={0};
	convert_uint32_to_byte2(indx,ind);
	Serial_write(ind,2);
	convert_uint32_to_byte2(indy,ind);
	Serial_write(ind,2);


	byte valueH3b[SIZE_IMAGE_BUFFER_BIT24]={0};
	uint32_t vH24=(uint32_t)(vHeight*(double)BIT24MAX);
	convert_uint32_to_byte3(vH24,valueH3b);

	byte valueE3b[SIZE_IMAGE_BUFFER_BIT24]={0};
	//uint32_t vE24=(uint32_t)(vError*(double)BIT24MAX);
	vError+=1;// converet -1:1-->0:2, later divide by 2
	uint32_t vE24=(uint32_t)(vError*(double)BIT24MAX);
	vE24>>=1;// divide by 2
	convert_uint32_to_byte3(vE24,valueE3b);

	Serial_write(valueH3b,SIZE_IMAGE_BUFFER_BIT24);
	Serial_write(valueE3b,SIZE_IMAGE_BUFFER_BIT24);


	Serial_write(COM_TAIL1);
	Serial_write(COM_TAIL2);
	//////////////////////-- repeat

	//int current_image_line=indy;
	//current_image_line%=SIZE_IMAGE_BUFFER_LINES;
	//if (indx>=0)
	//{
	//	for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//		Serial_write(pImageHF[k][current_image_line][abs(indx)]);
	//	for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//		Serial_write(pImageEF[k][current_image_line][abs(indx)]);
	//}
	//else
	//{
	//	for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//		Serial_write(pImageHB[k][current_image_line][abs(indx)]);
	//	for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
	//		Serial_write(pImageEB[k][current_image_line][abs(indx)]);
	//}
}
//void prepare_image_package_to_PC_store(int indx,int indy)
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
//			Serial_write(pImageHF[k][current_image_line][abs(indx)]);
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			Serial_write(pImageEF[k][current_image_line][abs(indx)]);
//	}
//	else
//	{
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			Serial_write(pImageHB[k][current_image_line][abs(indx)]);
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			Serial_write(pImageEB[k][current_image_line][abs(indx)]);
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
//	//	double z_output_01=65536.0/(double)BIT24MAX;
//	//	double er=256.0/(double)BIT24MAX;
//	//
//	//	byte valueH3b[SIZE_IMAGE_BUFFER_BIT24]={0};
//	//	uint32_t vH24=(uint32_t)(z_output_01*(double)BIT24MAX);
//	//	convert_uint32_to_byte3(vH24,valueH3b);
//	//
//	//	byte valueE3b[SIZE_IMAGE_BUFFER_BIT24]={0};
//	//	uint32_t vE24=(uint32_t)(er*(double)BIT24MAX);
//	//	convert_uint32_to_byte3(vE24,valueE3b);
//	//	//6 us
//	//	for (int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//	//	{
//	//		if (indx>=0) 
//	//		{
//	//			pImageHF[k][current_image_line][abs(indx)]=valueH3b[k];
//	//			pImageEF[k][current_image_line][abs(indx)]=valueE3b[k];
//	//		}
//	//		else
//	//		{
//	//			pImageHB[k][current_image_line][abs(indx)]=valueH3b[k];
//	//			pImageEB[k][current_image_line][abs(indx)]=valueE3b[k];
//	//		}
//	//
//	//	}// 7us
//
//	/////////////////////////
//	if (indx>=0)
//	{
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			mUSerial.write(pImageHF[k][current_image_line][abs(indx)]);
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			mUSerial.write(pImageEF[k][current_image_line][abs(indx)]);
//	}
//	else
//	{
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			mUSerial.write(pImageHB[k][current_image_line][abs(indx)]);
//		for(int k=0;k<SIZE_IMAGE_BUFFER_BIT24;k++)
//			mUSerial.write(pImageEB[k][current_image_line][abs(indx)]);
//	}
//
//	mUSerial.write(COM_TAIL1);
//	mUSerial.write(COM_TAIL2);
//}

void send_back_debug_infomation()
	// should send each byte and then give cpu to others
{
	static int c=0;
	c++;
	if (c<100000)
		return;
	c=0;


	//PERIOD_TIME_CHECK_EXIT();
	//	int dt=500000;
	///*	{
	//		do
	//		{*/	
	//			static unsigned long time_store =millis();	
	//			unsigned long time_now=millis();	
	//			if((time_now - time_store)<(dt))
	//				return;	
	//			else 
	//				time_store=time_now;
	//	//	}
	//	//	while(0);
	//	//}

	//		mUSerial.print("F ");
	//		mUSerial.println(measured_sampling_frequency_of_system,DEC);
	//mUSerial.print("DSet_01 ");
	//mUSerial.println(DSet_01,DEC);
	//mUSerial.print("DInput_01 ");
	//mUSerial.println(DInput_01,DEC);
	mUSerial.print("sys_state ");
	mUSerial.print(sys_state,DEC);
	mUSerial.print("*XY_state: ");	
	mUSerial.print(mDDS_XY_Scanner_State,DEC);	

	mUSerial.print("*ADC_inf: ");	
	mUSerial.print(Vdf_infinite,DEC);	
	mUSerial.print("*ADC: ");	
	mUSerial.print(V18_Adc[ADC_PORT_ZlOOP_SENSOR],DEC);
	mUSerial.print("*vdf mV: ");
	mUSerial.print((int)((double)V18_Adc[ADC_PORT_ZlOOP_SENSOR]*5.0/(double)BIT18MAX*1000),DEC);
	mUSerial.print("*Set: ");
	mUSerial.print((int)(DSet_01*100000),DEC);
	mUSerial.print("*IN: ");

	mUSerial.print((int)(DInput_01*100000),DEC);
	//mUSerial.println(DInput_01,DEC);
	mUSerial.print("*Out: ");

	mUSerial.print((int)(DOutput_01*100000),DEC);


	mUSerial.print("*Vout: ");
	mUSerial.print((int)(z_output_01*100000),DEC);

	mUSerial.print("*X: ");
	mUSerial.print(VDACx,DEC);
	mUSerial.print("*Y: ");
	mUSerial.print(VDACy,DEC);

	mUSerial.print("*switch_read_SG: ");
	mUSerial.print(switch_read_SG,DEC);

	mUSerial.println('\n',DEC);

	//mUSerial.println(DOutput_01,DEC);

}

double measured_sampling_frequency_of_system=0;

void process_ScanRealTimeLoop_Initialize(double position_01)
{
	calculate_scan_parameter();
	mZ_Loop_PID.Reset();
	//mTimer_ZLoop.start();  
	z_output_01=position_01;//0.5;
}
void process_Idle()
{
	//ADC_read_DAC_write(ADC_PORT_ZlOOP_SENSOR,&V18_Adc[PIEZO_Z],PIEZO_Z,V18_Dac[PIEZO_Z]);
	if (PERIOD_CHECK_TIME_US_DUE_SEND_SYSTEM_PACKAGE(400000)==false) return;//old 1e6
	//prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[ADC_PORT_ZlOOP_SENSOR],V18_Dac[PIEZO_Z]);
	V18_Adc[PIEZO_Z]=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);
	uint32_t amp_ad0=analogRead(A0);
	prepare_system_package_to_PC((uint32_t)(position_feedforward_output_01[PIEZO_Z]*BIT32MAX),V18_Adc[PIEZO_Z],amp_ad0);
}

void prepare_system_package_to_PC(uint32_t v1,uint32_t v2,uint32_t v3)
{
	byte com[12]={0};
	com[0]='s';
	com[1]='p';
	//com[2]=0;
	convert_uint32_to_byte4(v1,&com[2]);// byte4
	convert_uint32_to_byte3(v2,&com[2+4]);// byte3
	convert_uint32_to_byte3(v3,&com[2+4+3]);//byte3
	mCPackageToPC_SystemPackage->prepare_package_Byte12_to_PC(com);
}

int test(int x)//	test(200000); pass test, no problem
{
	x--;
	mUSerial.println(x,DEC);
	if (x>0) test(x);
}

int serialEvent()
{
	DIGITAL_PIN_TOGGLE(23);
}

////////////////////////////////////

void AFM_main_setup() {

	mTimerGlobal.start();
	//delay(1000);
	int bdr=  115200;
	double timeout=1000.0*9.0/bdr*LENGTH_COM_BUFFER_PC2MCU*2*1.3;

	mUSerial.begin();//bdr
	mUSerial.setTimeout(timeout); // for 
	MY_Debug_LN("MCU starts.");		

	//USB_Debug_Initialize(115200);
	//while(!SerialUSB) 
	//{
	//	MY_Debug_LN("wait for SerialUSB to plug in.");	
	//	delay(500);
	//}
	//USB_Debug_LN("USB_Debug_LN run.");	


	ADC_DAC_initialize(50);//7 12 MHz
	//DRpot_initialize(FREQUENCY_MAX/2);
	DRpot_initialize(FREQUENCY_MAX/8);//use low speed  for SG measurement
	calculate_scan_parameter();
	//DAC_initialize(1);
	mZ_Loop_PID.SetSampleTime(SamplingTime_us_Zloop);
	mZ_Loop_PID.SetOutputLimits(-MAX_STEP_SIZE_PIEZO_MODEL,MAX_STEP_SIZE_PIEZO_MODEL);//10,000,000// 10um


	TIC();
	process_ScanRealTimeLoop();
	int t_us=TOC();
	mUSerial.println(t_us,DEC);
	measured_sampling_frequency_of_system=5.0*10000000.0/t_us;

	//mTimer_ZLoop.setFrequency(SamplingFrequency_Zloop);//SamplingFrequency_Zloop);
	////mTimer_ZLoop.setFrequency(SamplingFrequency_Zloop);//SamplingFrequency_Zloop);
	//mTimer_ZLoop.attachInterrupt(process_ScanRealTimeLoop);

	//mTimer_Approach.setFrequency(sampling_frequency_of_Approach_Process);
	//mTimer_Approach.attachInterrupt(process_Approach);

	fastPinMode(22,OUTPUT);// real time z loop in scan
	fastPinMode(23,OUTPUT);// z scanner engage
	//mUSerial.println(measured_sampling_frequency_of_system,DEC);
	console_WithDrawZScanner_SetSystemIdle();
	console_XYScanReset();
	analogReadResolution(12);//analogRead(5-6 us)
	//Keyboard.begin();


	MY_Debug_StringValue_LN("sensor value median: ",ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR,30,1000));

	MY_Debug_StringValue_LN("sensor value average: ",ADC_read_average(ADC_PORT_ZlOOP_SENSOR,30,1000));


	MY_Debug_LN("Setup() done.");
	
	mTicker_AFM_Realtime.attach_us(AFM_Realtime_loop, PEORID_AFM_REALTIME_US);
}

void read_SG_data_temp()
{
	int  switch_read_SG=2;
	if (PERIOD_CHECK_TIME_US_DUE_READ_SG_DATA(sampling_time_us_read_SG_data)==false) return;

	/// for I2C communication 
#define ADDRESS_I2C_SLAVE   (0x50)   /// unsigned int 	// The following is not required: chipAddress = (chipAddress<<1) | (0<<0);	
	// 	Wire1.beginTransmission(chipAddress); 
	// 	Wire1.write(0x20); 
	// 	Wire1.endTransmission();
	// 	delay(10); 
	fastDigitalWrite(23,true);

	byte Buffer_SG_data [LENGTH_I2C_DATA_SG]={0};
	Wire1.requestFrom(ADDRESS_I2C_SLAVE, LENGTH_I2C_DATA_SG);// time use=586 us
	static byte frame_counter=0;
	while(Wire1.available()>=LENGTH_I2C_DATA_SG)    // slave may send less than requested
	{	
		for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
		{
			Buffer_SG_data[k] = Wire1.read();    // receive a byte as character
			//mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
			//mUSerial.print(" ");
			//mUSerial.println("raw");
		}

		if (frame_counter!=Buffer_SG_data[0])
		{
			frame_counter=Buffer_SG_data[0];
			//send_SG_rs232();
			byte com[LENGTH_COM_BUFFER_MCU2PC-4]={"SG"};
			com[2]=9;
			memcpy(&com[3],&Buffer_SG_data[1],9);

			//uint32_t ad1=mAFM_SEM.ADC_Read_N(ADC_PORT_PRC);
			//uint32_t amp_ad0=analogRead(A2);
			//convert_uint32_to_byte3(amp_ad0,&com[3]);


			if (switch_read_SG>0)
				send_system_package16_char_to_PC((char*)com);
			if (switch_read_SG==1) switch_read_SG=0;// read only once
			//for (int k=0;k<LENGTH_I2C_DATA_SG;k++)
			//{mUSerial.print(Buffer_SG_data[k],DEC);         // print the character
			//mUSerial.print(" ");}
			//mUSerial.println("d");
		}
	}
	fastDigitalWrite(23,false);
}
void AFM_main_loop() 
{	
	//int x=mAFM_SEM.ADC_Read_N(ADC_PORT_TUNING_FORK);
	//delay(5);
	//return;
//#define px (23)
//	SET_PIN_L(px);
//	//SET_PIN_L(24);
//	//SET_PIN_L(25);
//	//SET_PIN_L(26);
//	//SET_PIN_L(27);
//delay(500);
//	SET_PIN_H(px);
//	//SET_PIN_H(24);
//	//SET_PIN_H(25);
//	//SET_PIN_H(26);
//	//SET_PIN_H(27);
//	delay(500);
//	return;

	////////return;
	//uint32_t x=mAFM_SEM.ADC_Read_N(ADC_PORT_PRC);
	//MY_Debug_LN(x);
	//delay(1);
	////wait_us(500);
	//return;



	//double x=piezo_predict_Position01_To_Voltage_DAC18(0,0.8);
	//MY_Debug_LN(x);
	//delay(100);
	//return;
	//DueFlashStorage mFlash;
	//byte x=0,y;
	//
	//{TIC();
	////for (int addr=0;addr<1000;addr++)
	//	//mFlash.write(0,(byte*)x,1024);// time =20389us
	//	mFlash.write(0,1);// time =8150us
	//int tw=TOC();
	//MY_Debug_StringValue_LN("write:",tw);

	//TIC();
	//	for (int addr=0;addr<1000;addr++)
	//		y=mFlash.read(addr);
	//int tr=TOC();
	//MY_Debug_StringValue_LN("read:",tr);
	////delay(100);
	//}
	//
	//while(1);
	//return;

	//while(1)
	//{
	//	MY_Debug_StringValue_LN("ave: ",ADC_read_average(ADC_PORT_ZlOOP_SENSOR,40,100));
	//	delay(500);
	//	MY_Debug_StringValue_LN("adc: ",mAFM_SEM.ADC_Read_N(ADC_PORT_PRC));
	//		delay(500);
	//	MY_Debug_StringValue_LN("mf: ",ADC_read_MedianFilter(ADC_PORT_ZlOOP_SENSOR,40,100));
	//	delay(500);
	//}



	//process_ScanRealTimeLoop();
	//delay(100);
	//int x=mAFM_SEM.ADC_Read_N(ADC_PORT_PRC);
	//int x=mAFM_SEM.ADC_Read_N(ADC_PORT_TUNING_FORK);
	//DAC_write(PIEZO_T,0);
	//delay(5);
	//DAC_write(PIEZO_T,BIT18MAX_HALF);
	//delay(5);
	//DAC_write(PIEZO_T,BIT18MAX);
	//delay(5);
	//DAC_write(PIEZO_Z,x);

	////double xx=x/BIT18MAX*3.3;
	////MY_Debug_LN(xx);
	//delay(100);
	//return;


	//static int x=0;
	//if (x++>1000)
	//{x=0;
	//prepare_engaged_package_to_PC(10,10,0.021,-0.2612);
	//}
	////#define SerialUSB mUSerial 
	//	SerialUSB.begin(115200);
	//	byte in_buf[16]={0};
	//	while(1)
	//	{	//int y=SerialUSB.read();
	//		TIC();
	//	
	//		
	//		int x=SerialUSB.available();
	//		//int x=mUSerial.available();
	//		int t=TOC();
	//		SerialUSB.print("time");
	//		SerialUSB.println(t);
	//		SerialUSB.print("length");
	//		SerialUSB.println(x);
	//
	//		int q=SerialUSB.readBytes( in_buf,16);
	//		delay(100);
	//		//if (in_buf[0]!=0)
	//		{in_buf[15]++;
	//			SerialUSB.write(in_buf,16);
	//			in_buf[0]=x;
	//		}
	//		//delay(1);
	//	}
	//byte x=0;
	//while(1)
	//{
	//	mUSerial.readBytes(&x,1);
	//	delay(100);
	//	mUSerial.write(x+1);
	//delay(100);
	//}
	//static	byte d[]={0xAA,0x55,0x49,0x4D,0x00,0x7F,0x00,0x41,0x01,0x00,0x00,0x00,0x01,0x00,0x55,0xAA};
	//d[13]++;	
	//mUSerial.write(d,16);
	//   delay(100);
	//Keyboard.write('m');
	//Keyboard.sendReport();
	//HID_SendReport();

	//delay(5);
	//return;

	//while(1)
	//{
	//	DAC_write(PIEZO_Z,0);
	//	wait_us(500);
	//	DAC_write(PIEZO_Z,BIT18MAX);
	//	
	//	wait_us(500);
	//}
	//V18_Adc[ADC_PORT_ZlOOP_SENSOR]=mAFM_SEM.ADC_Read_N(ADC_PORT_PRC);
	////ADC_read_DAC_write(ADC_PORT_TUNING_FORK,&V18_Adc[ADC_PORT_ZlOOP_SENSOR],PIEZO_Z,V18_Dac[PIEZO_Z]);
	////V18_Dac[PIEZO_Z]=V18_Adc[ADC_PORT_ZlOOP_SENSOR];

	//	TIC();
	//analogRead(A0);
	//mUSerial.println(TOC());
	//return;


	//read_SG_data_temp();
	//delay(80);
	//return;
	//int x=mAFM_SEM.ADC_Read_N(ADC_PORT_ZlOOP_SENSOR);	
	//	mUSerial.println(x);
	//	delay(10);
	//	return;


	//mUSerial.println(x,DEC);


	wait_us(10);
	command_console(false);
	if (switch_read_SG > 0)
		read_SG_data(&switch_read_SG);	// should not call this too frequently. <24Hz;


	mCPackageToPC_SystemPackage->rtos_send_image_frame_to_PC();	
	rtos_send_image_frame_to_PC();

	
	
}
void AFM_Realtime_loop()
{	
	//switch For dense case values compiler generates jump table,
	switch (sys_state)
	{
	case SS_Scan:			process_ScanRealTimeLoop();break;
	case SS_XYScanReset:	process_XYscanningReset();break;		
		//case  SS_Scan:			break;// FOR TIMER FUNS
	case  SS_Approach:		process_Approach();break;//
	case  SS_Engage:		process_ZScannerEngage(); break;	
	case  SS_Indent:		process_Indent_First_SendDataThen(); break;	
		//case  SS_Indent:		process_Indent(); break;	
	case  SS_Idle:			process_Idle(); 	
		//send_back_debug_infomation();
		break;
	default:;
	}

}	
#endif // !__AFM_FUNCTIONS__
//if (sys_state==SS_Scan)
//	process_ScanRealTimeLoop();	
//else if (sys_state==SS_Approach)
//	process_Approach();
//else if (sys_state==SS_Engage)
//	process_ZScannerEngage();
//if (sys_state==SS_Idle)	//do nothing

// delete following
//	while(1)
//{	DAC_write(PIEZO_Z,0);
//	delay(100);
//	DAC_write(PIEZO_Z,BIT18MAX);
//	delay(100);
//}
//double x[50],y[50],z=0.2;
//mUSerial.print("*");

//TICX(1);
//for (int k=0;k<1000000;k++)
//	z=x/y;
////TOCX_P(1);
//mUSerial.println(TOCX(1)*100,DEC);
////TIC();
////for (int k=0;k<1000000;k++)
////	z=x*y;
////TOC_P();


//mUSerial.println(TOC(),DEC);

//mUSerial.print("/");
//TIC();
//for (int k=0;k<1000000;k++)
//	z=x/y;
//TOC_P();
//		fast pin ok
//static bool x=true;
//x=!x;
//fastDigitalWrite(22,x);
//


// put your main code here, to run repeatedly:
//XYscanning();



////static int v=262143;
//////static
////	bool x=0;
////if (x==0)
////{DAC_write(0, v);//1V 262143
////DAC_write(1, v);//1V 262143
////DAC_write(2, v);//1V 262143
////x=true;
////}
////else
////{DAC_write(0, 0);//1V 262143
////DAC_write(1, 0);//1V 262143
////DAC_write(2, 0);//1V 262143
////x=false;
////}
//delay(1);
//mUSerial.print (re,DEC);
//TIC;
//delay(500);
//
//mUSerial.println(V18_Adc[ADC_PORT_ZlOOP_SENSOR],DEC);

//TIC();

//read_SG_data();

//TOC_P();
//mUSerial.print("T");
//mUSerial.println(TOC,DEC);
////
////mUSerial.print(" DAc");
////mUSerial.println(V18_Dac[PIEZO_Z],DEC);
////delay(1);
////mUSerial.print("Vadc ");	
////mUSerial.println(V18_Adc[ADC_PORT_ZlOOP_SENSOR],DEC);
////mUSerial.print("Vdac ");	
////mUSerial.println(V18_Dac[PIEZO_Z],DEC);
//////mUSerial.print("z");	
//////mUSerial.println(Z,DEC);
////
////mUSerial.print("DInput_01 ");	
////mUSerial.println(DInput_01,DEC);	
////
////mUSerial.print("DOutput_01 ");	
////mUSerial.println(DOutput_01,DEC);	
////
////mUSerial.print("DSet_01 ");	
////mUSerial.println(DSet_01,DEC);
////mUSerial.println();
//delay(500);
//fastDigitalToggle(22);

//int Asin=analogRead(MCU_AD0);
//mUSerial.print("Asin ");	
//mUSerial.print(Asin,DEC);
//pow()



//////////////////////// mUSerial .Write speed test
//TICX(1);
//for (int k=0;k<1000;k++)
//	mUSerial.write(0x31);
//TOCX_P(1);

//delay(1000);
//byte com[1000]={0};
//for (int k=0;k<1000;k++)
//	com[k]=0x32;

//TICX(2);
//mUSerial.write(com,1000);
//TOCX_P(2);
//
//delay(1000);
//return;



//static int i=0;
//byte k=40;
//fastDigitalWrite(23,true);
//for (i=0;i<16;i++)
//	mUSerial.write(k);
//
//fastDigitalWrite(23,false);

//uint32_t x=mAFM_SEM.ADC_Read_N (ADC_PORT_ZlOOP_SENSOR);

//ADC_read_DAC_write(ADC_PORT_ZlOOP_SENSOR,&V18_Adc[ADC_PORT_ZlOOP_SENSOR],PIEZO_Z,V18_Dac[PIEZO_Z]);
//mUSerial.println(V18_Adc[ADC_PORT_ZlOOP_SENSOR],DEC);
//delay(100);
//return;


//byte b[4];
//convert_uint32_to_byte4(x,b);
//int k=0;
//mUSerial.write(b[k++]);
//mUSerial.write(b[k++]);
//mUSerial.write(b[k++]);
//mUSerial.write(b[k++]);
//int x=100;

//ADC->
//delay(100);
//return;
//DAC_write(0,value);
//DAC_write(1,value);

//DAC_write(2,0);
//delay(1000);
//DAC_write(2,BIT18MAX_HALF);
//delay(1000);
//return;
//send_back_debug_infomation();
//return;
