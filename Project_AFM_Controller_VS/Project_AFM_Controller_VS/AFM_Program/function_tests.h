#ifndef __FUNCTION_TEST__
#define __FUNCTION_TEST__


#include "defines.h"
#include "constant_define.h"

#include "define_AFM_platform.h"

#include "mbed.h"
#include "USBSerial.h"


#include "tm_stm32_adc.h"
#include "ADC_Temperature.h"
#include "define_AFM_hardware.h"
#include "SWSPI.h"


 
 

#include "DAC.h"
#include "ADC_Chain.h"



//#include "AFM_functions.h"


extern USBSerial mUSerial;


extern DigitalOut *p_LED;//(PORT_LED);

double  wave_triangle_0ToMax(double delta, double value_max, bool reset)
{
	static double v = 0;
	static bool direction = true;
	if (reset = true) {v = 0;direction = true;}
	if (direction == true)
		v += delta;
	else
		v -= delta;
	if (v >= value_max | v <= 0)
		direction = !value_max;
	return v;
}


//////////////////////////////////////hardware test functions
void toggle_pin(PinName port)
{
	static bool v = 0;
	v = !v;
	DigitalOut p(port);
	p = v;
}
void toggle_pin_led(){toggle_pin(PORT_LED);};
void toggle_pin_p(DigitalOut* p)
{
	static bool v = 0;
	v = !v;
	//DigitalOut p(port);
	*p = v;
}
void blink_IOs(PinName p0 = PA_0, int p_start = 0, int N = 16)
{
//	int N = 16;
//	PinName p0 = PA_0;
	static bool v = 0;
	v = !v;
	DigitalOut* p[N];
	for (int k = p_start;k < p_start + N;k++)
		p[k] = new DigitalOut((PinName)(p0 + k));
	
	//wait(0.1);
	for (int k = 0;k < N;k++)
		p[k]->write(v);
//	wait(0.1);
//	for (int k = 0;k < N;k++)
//		p[k] ->write(0);		
	
	for (int k = p_start;k < p_start + N;k++)
		delete p[k];
	
}

void test_blink_all_IOs()	// do full erase use ST-Link, after flash this function
{
	int k = 0;
	while (1)
		//while (k++ < 20)
	{		
//	blink_IOs(PA_0,0,13);
//	blink_IOs(PA_0, 15, 1);
//	blink_IOs(PB_0,0,3);
//	blink_IOs(PB_0, 4, 12);
	// do not wirte PA13,PA14,PB3	otherwise have to use ST-link in reset connection mode
	
		// do not blink PA13,PA14,PB3 for ever
		blink_IOs(PA_0);
		blink_IOs(PB_0);
		blink_IOs(PC_0);
		blink_IOs(PD_0);
		blink_IOs(PE_0);
		blink_IOs(PF_0);	
		blink_IOs(PG_0);
		blink_IOs(PH_0);
		blink_IOs(PI_0);
		blink_IOs(PJ_0);
		blink_IOs(PK_0);
	}
	
}




void test_blink_LED()
{
	while (1)
	{
		
		toggle_pin_led();
		wait(0.6);
		
//		p_LED->write(1);
//		wait(0.4);
//		
//		p_LED->write(0);
//		wait(0.6);
	}
}

void test_read_temperature()
{
	mUSerial.begin();
	//Temperature_Initial();
	extern CTemperature mCTemperature;
	while (1)
	{
		wait(0.3);
	
		char cha[30];
		mUSerial.process();

		if (mUSerial.readBytes(cha, 1) > 0)
		{
			cha[0]++;		
			int t = 0;
			double y = mCTemperature.Read();
			mUSerial.print("T\t");
			mUSerial.println(y);
		}
	}
}

////--------------------------
void test_HV_DAC_write()
{
extern	CDAC mAFM_DAC;	
	
	mAFM_DAC.Initialize();
//	while (1)	// 10 kHz, data rate=20k Hz
//	{
//		mAFM_DAC.DAC_write(PIEZO_Z, 0);
//		mAFM_DAC.DAC_write(PIEZO_Z, BIT18MAX);
//	}
	
//	while (1)//HVA gain adjustment
//	{
////		mAFM_DAC.DAC_write(PIEZO_Z, 0);
////		mAFM_DAC.DAC_write(PIEZO_X, 0);
////		mAFM_DAC.DAC_write(PIEZO_Y, 0);
////		wait(2);
//		mAFM_DAC.DAC_write(PIEZO_Z, BIT18MAX);//4000
//		mAFM_DAC.DAC_write(PIEZO_X, BIT18MAX);//4000
//		mAFM_DAC.DAC_write(PIEZO_Y, BIT18MAX);//4000
//		wait(2);
//		toggle_pin_led();
//	}	
	
//	while (1)	// 10 kHz, data rate=20k Hz
//	{
//		mAFM_DAC.DAC_write(PIEZO_Z, 0);
//		mAFM_DAC.DAC_write(PIEZO_X, 0);
//		mAFM_DAC.DAC_write(PIEZO_Y, 0);
//		wait(0.5);
//		mAFM_DAC.DAC_write(PIEZO_Z, BIT18MAX);//4000
//		mAFM_DAC.DAC_write(PIEZO_X, BIT18MAX);//4000
//		mAFM_DAC.DAC_write(PIEZO_Y, BIT18MAX);//4000
//		wait(0.5);
//		toggle_pin_led();
//	}
	
	
	
	mUSerial.begin();

	
	
	while (1)	
	//for (int m = 0;m < 4;m++)	
	{
		int m = 0;
		toggle_pin_led();
		
//		mAFM_DAC.Initialize(m, 18, 500);

		for (int ch = 0;ch < 4;ch++)	
		{
			mAFM_DAC.DAC_write(ch, 0);
			wait(0.2);
			mAFM_DAC.DAC_write(ch, BIT_N1(18)-1);
			wait(0.2);
		
			mUSerial.print("mode: ");
			mUSerial.print(m);
			mUSerial.print("\t");
			mUSerial.process();
		}
	
	}
}
void test_SEM_ADC_read()
{
	mUSerial.begin();
	int value[6] = {0};
	extern 	CSEM mAFM_SEM;
	mAFM_SEM.SetFrequency(2000000);
	
//	while (1)
//	for (int k = 0;k < 6;k++)
//	{
//		int v = mAFM_SEM.ADC_Read_N(k);
//		mUSerial.print(k);
//		mUSerial.print("\t");
//		mUSerial.println(v);
//		mUSerial.process();
//		wait(0.3);
//	
//	}
	
//	byte x = 0;
//	while (1)
//		
//		;
//	{
//		x++;
//		if (x == 0) mUSerial.println(value[0]);
//		mAFM_SEM.ADC_Read_Chain(value, 6);		
//		toggle_pin_led();
//		mUSerial.process();
//	}
	
	while (1)
	//for (int m=0;m<4;m++)	
	{ toggle_pin_led();
		
		wait(0.2);
		int m = 2;
 
		mAFM_SEM.ADC_Read_Chain(value, 6, m);		


		mUSerial.print("mode: ");
		mUSerial.print(m);
		mUSerial.print("\t");
		for (int k = 0;k < 6;k++)
		{

			//
			//mUSerial.print("channel: ");
			//mUSerial.print(k);
			mUSerial.print("\t");
			mUSerial.print(value[k] * 0.00001907348f, 4);//
			
			//mUSerial.print("\t");
//			int v = mAFM_SEM.ADC_Read_N(k);
//			mUSerial.println(v);
			//mUSerial.printB(value[k],18);
			mUSerial.process();
		}
		mUSerial.println();
	}

}


void test_usb_print()
{
	mUSerial.begin();  

	while (1)
	{
		toggle_pin_led();
//		toggle_pin_p(p_Tdio2);
		float x = 0.3333333;
		float y = 0.5;
		for (int k = 0;k < 1000000;k++)
			y = x*x;//sin(x);
		   //			MY_Debug(x);
		   //			MY_Debug("\t");
		
	}


	//		while (1)
	//		{
	//			toggle_pin_led();
	//			toggle_pin_p(p_Tdio2);
	//		AFM_Communication_Process();
	////		static double x = 0;
	////		x += 0.01;
	//		
	//			double x = 0.3333333;
	//			double y = 0.5;
	//			//for (int k = 0;k < 1000000;k++)
	//				 y =x*x;//sin(x);
	//				//			MY_Debug(x);
	//				//			MY_Debug("\t");
	//				//		
	//						wait(0.2);
	//			MY_Debug_LN(y);
	////			MY_Debug_LN(1000000.000123456);
	////			MY_Debug_LN(-1000000.000123456);
	////			MY_Debug_LN(0.0123456789123456789);
	////MY_Debug_LN(-0.000123456789123456789);
	////MY_Debug_LN("");
	//		}





}
//
//char cha[30];
//
//
//DigitalOut ptest(Sspi_cs2);
//
//
//DigitalOut p_SPI_SEM_cs(Sspi_cs);
//DigitalOut p_SPI_SEM_cs2(Sspi_cs2);
////SWSPI mSPI_SEM(Sspi_mosi, Sspi_miso, Sspi_clk);
//Ticker mTicker_AFM_Realtime;
//
//
//
//
//void function_periodic() {
//
//	//wait_us(150);
//	{
//
//
//		static bool v = true;
//		v = !v;
//		//p_Power_SEM.write(v);
//		int t = us_ticker_read();
//		mUSerial.println(t);
//	}
//
//}
//
//
//
//
////DigitalOut *mSPI_CS_AFM = new DigitalOut(Sspi_cs);
//
void setup() 
{

	


	mUSerial.begin();
//	Temperature_Initial();
	//mTicker_AFM_Realtime.attach_us(&function_periodic, 200);//1000000/5000



	//mAFM_DAC.DAC_initialize();
	

}
void loop()
{
	byte com_buffer[128];
	int byte_ready = mUSerial.readBytes(com_buffer, 128);
	if (byte_ready>0)
		mUSerial.write(com_buffer,128);
	
}
#endif // !__FUNCTION_TEST__
///////#include "main_include.h"

