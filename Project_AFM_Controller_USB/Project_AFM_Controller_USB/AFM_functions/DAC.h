// pass test 20150227 by JC
//speed Math_Max=57.4 kHz, Math_Min =6 kHz
#ifndef _SPI_DAC_INCLUDED
#define _SPI_DAC_INCLUDED

#include "mbed.h"
#include <SWSPI.h>
#include "constant_define.h"
#include "define_AFM_platform.h"
#include "define_AFM_hardware.h"
	
#define _ALWAYS_INLINE_ __attribute__((always_inline))
//#pragma once
class CDAC
{
#define  DAC_DATA_LENGTH (24)//(4+18+2)	

#define PDAC0   (Dspi_cs1) //DAC SS on pin 28
#define PDAC1   (Dspi_cs2)
#define PDAC2   (Dspi_cs3)
#define PDAC3   (Dspi_cs4)
#define NUM_OF_DAC (4)
private:
	//const int pdac[4];// = {PDAC0, PDAC1, PDAC2, PDAC3};
	SWSPI *mSPI_DAC;
	DigitalOut *mSPI_CS[NUM_OF_DAC];	
	uint32_t mDAC_Value[NUM_OF_DAC];
	
public:
	uint32_t ReadDACValue(byte channel){return mDAC_Value[channel];};
	CDAC(void)		
	{
		mSPI_DAC= new SWSPI(Dspi_mosi, Dspi_miso, Dspi_clk);	
		//for (int k = 0;k < NUM_OF_DAC;k++)
		int k = 0;
		mSPI_CS[k] = new DigitalOut(PDAC0); k++;
		mSPI_CS[k] = new DigitalOut(PDAC1); k++;
		mSPI_CS[k] = new DigitalOut(PDAC2); k++;
		mSPI_CS[k] = new DigitalOut(PDAC3); k++;
	};
	~CDAC(void)		
	{
		delete mSPI_DAC;
		for (int k = 0;k < NUM_OF_DAC;k++)
			delete mSPI_CS[k];
	};

void Initialize(int clock_Hz = 2000000)
{
	int mode_0123 = 0;
	int data_bit_lenth = DAC_DATA_LENGTH;
	mSPI_DAC->SetFrequency(clock_Hz);
	mSPI_DAC->SetDataLength(data_bit_lenth);
	mSPI_DAC->SetMode(mode_0123);
//	mSPI_DAC->format(data_bit_lenth, mode_0123);// to be adjusted
	
	
	//SPI.begin(4);  // initialize SPI:
	//SPI.setClockDivider(4, clock_devider);
	//SPI.setDataMode(4, SPI_MODE1);
	//wait_ms(50);
	//default is 4MHz
	//SPI.setBitOrder(4, MSBFIRST); //To set MSB or LSB first.
	
	//Math_Max: CPOL0, CPHA1 = SPI_MODE1
	//To change clock phase [Math_Max: Change on rising edge] and polarity [Math_Max: Idles low] (see SPI Lib reference)
}
//////////////////////////////////
void FinePositioner_MoveToPositionB18(byte channel, uint32_t value)// real move , write dac
{
	int32_t position=value;
	if (channel == PIEZO_Z)
	{
		position = (int32_t)BIT18MAX - (int32_t)value;
		DAC_write(channel, position);
	}
	else if (channel == PIEZO_ALL)
	{
		DAC_write(PIEZO_Y, position);
		DAC_write(PIEZO_X, position);
		position = (int32_t)BIT18MAX - (int32_t)value;
		DAC_write(PIEZO_Z, position);
	}
	else// simple XY
		DAC_write(channel,position);
}

int  DAC_write_all(uint32_t value) 
{
	DAC_write(PIEZO_Y, value);
	DAC_write(PIEZO_X, value);
	DAC_write(PIEZO_Z, value);
}
int  DAC_write(byte channel, uint32_t value) // long is 4 bytes for ARM32bit
//		data rate = 20k Hz
{
	value = LIMIT_MAX_MIN(value,BIT18MAX,0);

	mDAC_Value[channel]=value;
	mSPI_CS[channel]->write(0);
//	int value_back=mSPI_DAC->write(value);

//--- send out 3 bytes, this also works	
//	value <<= 2;
//	byte v3 = (byte)(value & 0x000000ff);
//	value >>= 8;
//	byte v2 = (byte)(value & 0x000000ff);
//	value >>= 8;
//	byte v1 = (byte)(value & 0x0000000f);
//	v1 |= 0x10;
//	mSPI_DAC->transfer(v1, 8);
//	mSPI_DAC->transfer(v2, 8);
//	mSPI_DAC->transfer(v3, 8);
	
	
//--- send out 24bit in Int	
//	dac MAX5318 data format, total data=24 bit:
//		r3,r2,r1,r0,d17--d0,x,x
//		 0, 0, 0, 1,data_18,0,0
	value <<=2;
	value |= 0x100000;//BIT_N1(20)	
//	int value_back = mSPI_DAC->transfer(value, DAC_DATA_LENGTH);
	int value_back;
	 mSPI_DAC->transfer_write(value, DAC_DATA_LENGTH);
	mSPI_CS[channel]->write(1);
	return value_back;
}
};

#endif



//  //Write data [R3 R2 R1 R0 D17 D16 D15 ... D1 D0 X X]
//  //DIN register write 0001...
////  SPI.transfer(4,B00010100);
////  SPI.transfer(4,B00000000);
////  SPI.transfer(4,B00000000);
////      wait_ms(1);
////  //OFFSET register write 0010...
//  //SPI.transfer(4,B00100000);
//  //SPI.transfer(4,B00000000);
//  //SPI.transfer(4,B00000000);
//
//  //Gain register write 0011...
//  //SPI.transfer(4,00111111);
//  //SPI.transfer(4,11111111);
//  //SPI.transfer(4,11111100);
//
//
//  //Config register write 0100...
//  SPI.transfer(4,01000111);//Normal mode,bus hold disabled,normal operation,Busy input disabled
//  SPI.transfer(4,10000000);//[output disabled
//  SPI.transfer(4,00000000);
//      wait_ms(1);
// digitalWrite(PDAC1, HIGH);