


#ifndef __ADC_CHAIN_INCLUDE__
#define __ADC_CHAIN_INCLUDE__

#include "mbed.h"

#include "SWSPI.h"
#include "constant_define.h"
#include "define_AFM_platform.h"
#include "define_AFM_hardware.h"

//#define _ALWAYS_INLINE_ __attribute__((always_inline))
//#pragma once


//ADCs test code
//	
//CSEM mAFM_SEM;
//	//for (int m=0;m<4;m++)	
//	{
//		int m=2;
// 
//		mAFM_SEM.ADC_read( value,6,m);
// 
//		
//
//
//			mUSerial.print("mode: ");
//			mUSerial.print(m);
//			mUSerial.print("\t");
//		for (int k=0;k<7;k++)
//		{
//
//			//
//			//mUSerial.print("channel: ");
//			//mUSerial.print(k);
//			mUSerial.print("\t");
//			mUSerial.print(value[k]*0.00001907348f,4);//
//			
//			//mUSerial.print("\t");
//			//mUSerial.printB(value[k],18);
//			
//		}
//mUSerial.println();
//	}








class CSEM
{
#define NUM_OF_ADC		(6)
#define ADC_CHANNEL_Z	(0)//ADC6 = Z,
#define ADC_CHANNEL_PRC	(1)//ADC5 = PRC, 
	
#define ADC_CHANNEL_X	 (3)	// new connection 20160711		//(2)//ADC4 = X, 
#define ADC_CHANNEL_Y	(2)//(3)//ADC3 = Y, 
#define ADC_CHANNEL_TEMPERATURE	(4)//ADC2 = Temperature, 
#define ADC_CHANNEL_CALIBRATION (5) //PRC_calibration.		

#define ADC_CHANNEL_Tz (4) 
#define ADC_CHANNEL_Txy	(5)
#define ADC_All (ADC_CHANNEL_Txy)	
	
#define  ADC_DATA_LENGTH (18)	

private:
	SWSPI *mSPI_SEM;

	DigitalOut *mSPI_CS_AFM;
	DigitalOut *mSPI_CS_TF;
	int  buffer_ADC_value_array[NUM_OF_ADC];// = {0};
	//int  buffer_ADC_value_array[NUM_OF_ADC];// = {0};

public:
	CSEM(int frequencyHz=2000000)
	{
		mSPI_SEM=new SWSPI(Sspi_mosi, Sspi_miso, Sspi_clk);	
		mSPI_SEM->SetFrequency(frequencyHz);
		//mSPI_SEM->format(18,2);// to be adjusted
		mSPI_SEM->SetMode(1);//2
		mSPI_SEM->SetDataLength(ADC_DATA_LENGTH);


		mSPI_CS_AFM = new DigitalOut(Sspi_cs);
		mSPI_CS_TF= new DigitalOut(Sspi_cs2); 


		// get ready for first convertion
		mSPI_CS_AFM->write(0);
		mSPI_CS_TF->write(0);

	};
	~CSEM(void)		
	{
		delete mSPI_SEM;
		delete mSPI_CS_AFM;
		delete mSPI_CS_TF;
	}
	void SetFrequency(int frequencyHz) {mSPI_SEM->SetFrequency(frequencyHz);};

//ADCs test code, OK 20160326
//	
//CSEM mAFM_SEM;
//	//for (int m=0;m<4;m++)	
//	{
//		int m=2;
// 
//		mAFM_SEM.ADC_read( value,6,m);
// 
//		
//
//
//			mUSerial.print("mode: ");
//			mUSerial.print(m);
//			mUSerial.print("\t");
//		for (int k=0;k<7;k++)
//		{
//
//			//
//			//mUSerial.print("channel: ");
//			//mUSerial.print(k);
//			mUSerial.print("\t");
//			mUSerial.print(value[k]*0.00001907348f,4);//
//			
//			//mUSerial.print("\t");
//			//mUSerial.printB(value[k],18);
//			
//		}
//mUSerial.println();
//	}
	int ADC_Read_N_Average(int axis, int number, int time_us)
	{
		int s = 0;
		for (int k = 0;k < number;k++)
		{
			s += ADC_Read_N(axis);
			wait_us(time_us);
		}
		s /= number;
		return s;
	}
	void ADC_Read_MultiChannel_Average(int axis)
	{
		int average[NUM_OF_ADC] = {0};
		#define NofA (8)		
		for (int n = 0;n < NofA;n++)	
		{
			ADC_Read_N(axis, true);
			for (int k = 0;k < axis+1;k++)
			{
				average[k] += ADC_Read_N(k,false);
			}
		}	
		//average and store to buffer_ADC_value_array[]
		for (int k = 0;k < axis+1;k++)	
		{
			average[k] >>= 3;	
			buffer_ADC_value_array[k] = average[k];
		}
	}
	int ADC_Read_N(int axis, bool update=true)
			// true read PRC=154 us, true read Y, 264 us
			// new read only, true read Y  207us
			// new clock 78.5 us
	{
		if (axis <= NUM_OF_ADC)
		{
			if(update==true)
				ADC_Read_Chain(buffer_ADC_value_array, axis + 1);
			return buffer_ADC_value_array[axis];
		}
		else 			
			return 0;
	}		

		
//ADC6 = Z,
//ADC5 = PRC, 
//ADC4 = X, 
//ADC3 = Y, 
//ADC2 = Temperature, 
//ADC1 = PRC_calibration.
	void ADC_Read_Chain(int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)// tested ok
	{
		//mSPI_SEM->SetMode(mode_0123);

		//mSPI_CS_AFM->write(1);
		//mSPI_SEM->sclk->write(1);
		//wait_us(10);
		//mSPI_SEM->transfer_read(1);// send one clock pulse

		mSPI_CS_AFM->write(0);// start the convertion
#ifdef SPI_WAIT 
		wait_us(2);// time =(500--1300 ns) for ADS8885
#endif	
		//int value_array[NUM_OF_ADC]=0;

		//mSPI_SEM->transfer_read(1);// send one clock pulse
		
		for (int k=0;k<number_of_ADC;k++)
//			value_array[k]=mSPI_SEM->transfer(0xaa55,ADC_DATA_LENGTH);// data readout at the next clock
		value_array[k] = mSPI_SEM->transfer_read(ADC_DATA_LENGTH);// data readout at the next clock

		mSPI_CS_AFM->write(1);

	};

	void ADC_read_bool_ok(int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)// also OK
	{
		//mSPI_SEM->SetMode(mode_0123);

		//mSPI_CS_AFM->write(1);
		//mSPI_SEM->sclk->write(1);
		//wait_us(10);

		mSPI_CS_AFM->write(0);// start the convertion

		wait_us(2);// time =(500--1300 ns) for ADS8885


		//int value_array[NUM_OF_ADC]=0;
		int data_length=ADC_DATA_LENGTH*number_of_ADC;//[NUM_OF_ADC]={18,18,18,18,18,18};
		bool *vb=new bool[data_length+1];

		mSPI_SEM->write_bool_array(vb,vb,data_length+1);

		for (int k=0;k<number_of_ADC;k++)
			value_array[k]=ConvertBoolArray2Int(&vb[1+k*ADC_DATA_LENGTH],ADC_DATA_LENGTH);// data readout at the next clock


		mSPI_CS_AFM->write(1);
		delete vb;

	};
	int ConvertBoolArray2Int(bool* bit_array, int length)
	{
		int out=0;
		for (int k=0;k<length;k++)
			out|=bit_array[k]<<(length-k-1);
		return out;
	}
//-----------------------------------------------------------------------
	//void ADC_read_int_old(int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)
	//{
	//	mSPI_CS_AFM->write(1);
	//	mSPI_SEM->sclk->write(1);
	//	wait_us(10);

	//	mSPI_CS_AFM->write(0);// start the convertion

	//	wait_us(10);// time =(500--1300 ns) for ADS8885

	//	//int value_array[NUM_OF_ADC]=0;
	//	//int data_length[NUM_OF_ADC]={18,18,18,18,18,18};
	//	mSPI_SEM->format(ADC_DATA_LENGTH,mode_0123);// to be adjusted
	//	wait_us(100);
	//	for (int k=0;k<number_of_ADC;k++)
	//	{
	//		value_array[k]=mSPI_SEM->write(0);	//1<<18-1
	//		//wait_us(100);
	//	}

	//	mSPI_CS_AFM->write(1);

	//};


	//
	//void ADC_read_test(bool * vb , int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)
	//{
	//	mSPI_SEM->SetMode(mode_0123);

	//	mSPI_CS_AFM->write(1);
	//	mSPI_SEM->sclk->write(1);
	//	wait_us(10);

	//	mSPI_CS_AFM->write(0);// start the convertion

	//	wait_us(10);// time =(500--1300 ns) for ADS8885



	//	//int value_array[NUM_OF_ADC]=0;
	//	int data_length=ADC_DATA_LENGTH*number_of_ADC;//[NUM_OF_ADC]={18,18,18,18,18,18};
	//	//bool *vb=new bool[data_length+1];

	//	mSPI_SEM->write(vb,vb,data_length );

	//	for (int k=0;k<number_of_ADC;k++)
	//		value_array[k]=ConvertBoolArray2Int(&vb[k*ADC_DATA_LENGTH],ADC_DATA_LENGTH);


	//	mSPI_CS_AFM->write(1);
	//	//delete vb;

	//};
	
	
	//-------------------------------- DAC
	
	void DAC_write(int value, int mode_0123)//, int number_of_ADC = NUM_OF_ADC, int mode_0123 = 2)
	{
		//mSPI_SEM->SetMode(mode_0123);

		//mSPI_CS_AFM->write(1);
		//mSPI_SEM->sclk->write(1);
		//wait_us(10);
		mSPI_SEM->SetDataLength(16);
		mSPI_SEM->SetMode(mode_0123);
		
		mSPI_CS_AFM->write(0);// start the convertion

		wait(5);//  

		value	&= 0b0000111111111111;
		//value |= 0b0011000000000000;// set bit12,13
		//value = SET_BIT1(value, 12);
		//value = SET_BIT1(value, 13);
		//int value_array[NUM_OF_ADC]=0;

		mSPI_SEM->transfer(value, 16);// send one clock pulse
		//mSPI_SEM->write(value);

		mSPI_CS_AFM->write(1);
		
		wait_us(1);// Math_Min wait time =50 ns

	};
};


#endif

