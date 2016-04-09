#ifndef __USB_SERIAL__
#define __USB_SERIAL__
//#include "definesUSB.h"
#include "defines.h"
#include <mbed.h>

#include "stm32fxxx_hal.h"

//#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_usb_device.h"
#include "tm_stm32_usb_device_cdc.h"

#include "constant_define.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define LENGTH_USB_STRING (256)

class USBSerial
{
public:
	USBSerial(void);
	~USBSerial(void);
	void begin();
	void setTimeout() {};
	void process();

	size_t write(uint8_t);
	size_t write( char);	
	size_t write(const char *);
	size_t write(const uint8_t *buffer, size_t length);


	size_t readBytes( char *buffer, size_t length) { return readBytes((uint8_t *)buffer, length); };
	size_t readBytes( uint8_t *buffer, size_t length);



	//	size_t print(char);
	//	size_t print(unsigned char, int = DEC);
	//	size_t print(int, int = DEC);
	//	size_t print(unsigned int, int = DEC);
	//	size_t print(long, int = DEC);
	//	size_t print(unsigned long, int = DEC);
	//	size_t print(double, int = 2);
	//
	//
	//	size_t println(const char []);
	//	size_t println(char);
	//	size_t println(unsigned char, int = DEC);
	//	size_t println(int, int = DEC);
	//	size_t println(unsigned int, int = DEC);
	//	size_t println(long, int = DEC);
	//	size_t println(unsigned long, int = DEC);


	size_t print(const char * str) {return write(str);};
	
	void println(const char * str) { print(str);println();};

	size_t print(uint32_t num, int length = DEC){return print((int)num, length);};	
	size_t println(uint32_t num, int length = DEC){return println((int)num, length);};
	size_t print(int num, int length=DEC){	uint8_t L = convertNum2String(num);	return write(mStr);	};	
	size_t println(int num, int length=DEC)	{ 		uint8_t L=print(num, length);		print("\n");	return L;}	;	

	size_t print(double num, int length = 5)
	{	
		int x=(int)num;
		uint8_t L = convertNum2String(x);
		write(mStr);
		write('.');		

		num-=x;


		//x=(int)(num*1000000000.0);

		length=LIMIT_MAX_MIN(length,9,1);// Math_Max is 10^9;
		x=(int)(num*Math_powX_Y(10,length));
		convertNum2String(x);
		write(mStr);
		return L;
	};	
	void  println()	{ print("\n");}	;	
	size_t println(double num, int length = DEC)	{ 		uint8_t L = print(num, length);		print("\n");	return L;}	;	
	size_t print(float num, int length = DEC){return print((double)num,length);};	
	size_t println(float num, int length = DEC)	{ 		uint8_t L = print(num, length);		print("\n");	return L;}	;	
	void printB(long value,int bit_length)
	{

		int v=0;
		for (int bit = bit_length-1; bit >= 0; --bit)// send high bit first
		{
			v=(((value >> bit) & 0x01) != 0);
			print(v);
		}
	}
	void printBs(bool *value,int bit_length)
	{

		int v=0;
		for (int k =0; k<bit_length; k++)// send high bit first
		{			
			print(value[k]);
		}
	}
	// loating number has problem
	size_t convertNum2String(double number)
	{
		memset(mStr, 0, LENGTH_USB_STRING);
		uint8_t L = sprintf(mStr, "%f", number);
		return L;
	}
	size_t convertNum2String(float number)
	{
		memset(mStr,0, LENGTH_USB_STRING);
		uint8_t L = sprintf(mStr, "%f", number);
		return L;
	}
	size_t convertNum2String(int number)
	{
		memset(mStr, 0, LENGTH_USB_STRING);
		uint8_t L = sprintf(mStr, "%d", number);
		return L;
	}   
	//	#include <iostream>
	double Math_powX_Y(double X, int Y)
	{
		double out=1;
		if (Y>=0)
			for (int c=0;c<Y;c++)
				out*=X;
		else
			out=1;

		return out;		
	}
	//template <typename T> T Math_powX_Y(T X, int Y)
	//{
	//	if (Y<1) 
	//		return 1;
	//	else 
	//	{
	//	while (Y--)
	//		X*=X;

	//	return X;
	//	}
	//}

private: 

	/* USB CDC settings */
	TM_USBD_CDC_Settings_t USB_FS_Settings;
	DigitalOut *usb_LED;
	uint16_t mCountProcess;
	char mStr[LENGTH_USB_STRING];

};



#endif