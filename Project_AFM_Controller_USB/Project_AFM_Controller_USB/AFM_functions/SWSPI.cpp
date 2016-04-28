/* SWSPI, Software SPI library
* Copyright (c) 2012-2014, David R. Van Wagner, http://techwithdave.blogspot.com
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include "mbed.h"
#include "SWSPI.h"

SWSPI::SWSPI(PinName mosi_pin, PinName miso_pin, PinName sclk_pin,int frequencyHz)
{
	mosi = new DigitalOut(mosi_pin);
	miso = new DigitalIn(miso_pin);
	sclk = new DigitalOut(sclk_pin);
	//format(8);
	SetFrequency(frequencyHz);
}

SWSPI::~SWSPI()
{
	delete mosi;
	delete miso;
	delete sclk;
}
void SWSPI::SetDataLength(int length)
{
	this->bits = length;
}
void SWSPI::SetMode(int mode_0123)
{
	//this->bits = bits;
	this->mode = mode_0123;
	polarity = (mode_0123 >> 1) & 1;
	phase = mode_0123 & 1;
	sclk->write(polarity);
	//sclk->write(1);
}
void SWSPI::format(int bits, int mode_0123)
{
	this->bits = bits;
	this->mode = mode_0123;
	polarity = (mode_0123 >> 1) & 1;
	phase = mode_0123 & 1;
	sclk->write(polarity);
	//sclk->write(1);
}

void SWSPI::SetFrequency(int frequencyHz)
{
	this->freq = frequencyHz;
	period_us = 1000000.0 / 2 / (float)frequencyHz;
}

void SWSPI::write_bool_array(bool* value_array_in,bool* value_array_out,int length)//zero bit FIRST, veryfied
{
	//sclk->write(0);
	//wait_us(period_us);
	//sclk->write(1);
	//wait_us(period_us);

	for (int bit = 0;bit<length; ++bit)// send high bit first
	{   
		sclk->write(0);
#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	

		mosi->write(value_array_in[bit]);        
		value_array_out[bit]=miso->read();

		sclk->write(1);
#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	
	}
}
//template <T> SWSPI::write(T value,int data_length)//MSB_FIRST
//{
//	this->bits=data_length;
//	return write(value);
//}
//int SWSPI::write(int value,int data_length)//MSB_FIRST
//{
//	this->bits=data_length;
//	return write(value);
//}
int SWSPI::write(int value)//MSB_FIRST
{
	int read = 0;
	for (int bit = bits-1; bit >= 0; --bit)// send high bit first
	{
		mosi->write(((value >> bit) & 0x01) != 0);

		if (phase == 0)
		{
			if (miso->read())
				read |= (1 << bit);
		}

		sclk->write(!polarity);


#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	

		if (phase == 1)
		{
			if (miso->read())
				read |= (1 << bit);
		}

		sclk->write(polarity);

#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	
	}

	return read;
}
//template <typename T>  T SWSPI::transfer (T value,int data_bit_length)//MSB_FIRST
//{
//	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
//		//#error("(data_length> sizeof(value))")
//
//	T read = 0;
//	for (int bit = data_bit_length-1; bit >= 0; --bit)// send high bit first
//	{
//		mosi->write(((value >> bit) & 0x01) != 0);
//
//		if (phase == 0)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(!polarity);
//
//		//wait(1.0/freq/2);
//
//		wait_us(period_us);
//		if (phase == 1)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(polarity);
//
//		// wait(1.0/freq/2);
//		wait_us(period_us);
//	}
//
//	return read;
//}
void SWSPI::transfer_write(uint32_t value, int data_bit_length)
{
	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
		//#error("(data_length> sizeof(value))")

	uint32_t read = 0;
	for (int bit = data_bit_length - 1; bit >= 0; --bit)// send high bit first
	{		
		sclk->write(!polarity);	
#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	
		mosi->write(((value >> bit) & 0x01) != 0);

//		if (phase == 0)
//		{
//			if (miso->read())
//				read |= (1 << (bit));
//		}		

		sclk->write(polarity);
#ifdef SPI_WAIT 
		wait_us(period_us);	
#endif		
		
//		if (phase == 1)
//		{
//			if (miso->read())
//				read |= (1 << (bit));
//		}
		
		
		
	}

//	return read;
}
uint32_t SWSPI::transfer_read(int data_bit_length)
{
	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
		//#error("(data_length> sizeof(value))")

	uint32_t read = 0;
	for (int bit = data_bit_length - 1; bit >= 0; --bit)// send high bit first
	{		
		sclk->write(!polarity);	
#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	
//		mosi->write(((value >> bit) & 0x01) != 0);

		if (phase == 0)
		{
			if (miso->read())
				read |= (1 << (bit));
		}		

		sclk->write(polarity);
#ifdef SPI_WAIT 
		wait_us(period_us);	
#endif		
		
		if (phase == 1)
		{
			if (miso->read())
				read |= (1 << (bit));
		}
		
		
		
	}

	return read;
}
uint32_t SWSPI::transfer(uint32_t value, int data_bit_length)
{
	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
		//#error("(data_length> sizeof(value))")

	uint32_t read = 0;
	for (int bit = data_bit_length - 1; bit >= 0; --bit)// send high bit first
	{		
		sclk->write(!polarity);	
#ifdef SPI_WAIT 
		wait_us(period_us);//wait(1.0/freq/2);
#endif	
		mosi->write(((value >> bit) & 0x01) != 0);

		if (phase == 0)
		{
			if (miso->read())
				read |= (1 << (bit));
		}		

		sclk->write(polarity);
#ifdef SPI_WAIT 
		wait_us(period_us);	
#endif		
		
		if (phase == 1)
		{
			if (miso->read())
				read |= (1 << (bit));
		}
		
		
		
	}

	return read;
}
//unsigned long SWSPI::transfer(unsigned long value, int data_bit_length)
//{
//	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
//		//#error("(data_length> sizeof(value))")
//
//	unsigned long read = 0;
//	for (int bit = data_bit_length - 1; bit >= 0; --bit)// send high bit first
//	{
//		mosi->write(((value >> bit) & 0x01) != 0);
//
//		if (phase == 0)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(!polarity);
//
//		//wait(1.0/freq/2);
//
//		wait_us(period_us);
//		if (phase == 1)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(polarity);
//
//		// wait(1.0/freq/2);
//		wait_us(period_us);
//	}
//
//	return read;
//}				
//unsigned long long SWSPI::transfer(unsigned long long value, int data_bit_length)
//{
//	//if (data_bit_length> sizeof(value)*8) data_bit_length= sizeof(value)*8;
//		//#error("(data_length> sizeof(value))")
//
//	unsigned long long read = 0;
//	for (int bit = data_bit_length - 1; bit >= 0; --bit)// send high bit first
//	{
//		mosi->write(((value >> bit) & 0x01) != 0);
//
//		if (phase == 0)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(!polarity);
//
//		//wait(1.0/freq/2);
//
//		wait_us(period_us);
//		if (phase == 1)
//		{
//			if (miso->read())
//				read |= (1 << bit);
//		}
//
//		sclk->write(polarity);
//
//		// wait(1.0/freq/2);
//		wait_us(period_us);
//	}
//
//	return read;
//}	