


#ifndef __ADC_CHAIN_INCLUDE__
#define __ADC_CHAIN_INCLUDE__

#include "mbed.h"
#include "SWSPI.h"
#include "tm_stm32_spi.h"
#include "constant_define.h"
#include "define_AFM_platform.h"
#include "define_AFM_hardware.h"

//#define _ALWAYS_INLINE_ __attribute__((always_inline))
//#pragma once


//ADCs test code
//	
//CSEM mAFM_SEM;
//	for (int m=0;m<4;m++)	
//	{
//		mAFM_SEM.mAFM_SEM.ADC_Read_N(value,6,m);
//
//		for (int k=0;k<6;k++)
//		{
//			mUSerial.print("mode: ");
//			mUSerial.print(m);
//			mUSerial.print("\t");
//			
//			mUSerial.print("channel: ");
//			mUSerial.print(k);
//			mUSerial.print("\t");
//			mUSerial.println(value[k]);
//
//			//
//			wait(0.1);
//			mUSerial.process();
//		}
//		mUSerial.println();
//	}






class CSEM
{
#define NUM_OF_ADC (6)
#define  ADC_DATA_LENGTH (18)	

private:
	SWSPI *mSPI_SEM;

	DigitalOut *mSPI_CS_AFM;
	DigitalOut *mSPI_CS_TF;



public:
	CSEM(void)
	{

		mSPI_CS_AFM = new DigitalOut(Sspi_cs);
		mSPI_CS_TF= new DigitalOut(Sspi_cs2); 


		// get ready for first convertion
		mSPI_CS_AFM->write(0);
		mSPI_CS_TF->write(0);

	};
	~CSEM(void)		
	{
		//delete mSPI_CS_AFM;
		delete mSPI_CS_AFM;
		delete mSPI_CS_TF;
	}

	void ADC_read_old(int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)
	{
		//mSPI_CS_AFM->write(1);
		//mSPI_SEM->sclk->write(1);
		//wait_us(10);

		mSPI_CS_AFM->write(0);// start the convertion

		wait_us(10);// time =(500--1300 ns) for ADS8885

		//int value_array[NUM_OF_ADC]=0;
		//int data_length[NUM_OF_ADC]={18,18,18,18,18,18};
		//mSPI_SEM->format(ADC_DATA_LENGTH,mode_0123);// to be adjusted
		wait_us(100);
		for (int k=0;k<number_of_ADC;k++)
		{

			
			//value_array[k]=mSPI_SEM->write(0);	//1<<18-1
			//wait_us(100);
		}

		mSPI_CS_AFM->write(1);

	};
	void mAFM_SEM.ADC_Read_N(int* value_array,int number_of_ADC=NUM_OF_ADC,int mode_0123=2)
	{
		mSPI_SEM->SetMode(mode_0123);

		mSPI_CS_AFM->write(1);
		mSPI_SEM->sclk->write(1);
		wait_us(10);

		mSPI_CS_AFM->write(0);// start the convertion

		wait_us(10);// time =(500--1300 ns) for ADS8885



		//int value_array[NUM_OF_ADC]=0;
		int data_length=ADC_DATA_LENGTH*number_of_ADC;//[NUM_OF_ADC]={18,18,18,18,18,18};
		bool *vb=new bool[data_length+1];

		//mSPI_SEM->write(vb,vb,data_length);
		 TM_SPI_ReadMulti(SPI2, (uint8_t*)value_array, 0,128);


		for (int k=0;k<number_of_ADC;k++)
			value_array[k]=ConvertBoolArray2Int(&vb[1+k*ADC_DATA_LENGTH],ADC_DATA_LENGTH);


		mSPI_CS_AFM->write(1);

	};
	int ConvertBoolArray2Int(bool* bit_array, int length)
	{
		int out=0;
		for (int k=0;k<length;k++)
			out|=bit_array[k]<<(length-k-1);
		return out;

	}
};

#endif

