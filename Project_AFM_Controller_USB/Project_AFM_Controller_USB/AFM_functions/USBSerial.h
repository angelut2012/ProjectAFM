#ifndef __USB_SERIAL__
#define __USB_SERIAL__
//#include "definesUSB.h"
//#include "defines.h"

#ifdef __cplusplus
extern "C"
{
#endif
	#include <usbd_core.h>
	#include <usbd_cdc.h>
	#include "usbd_cdc_if.h"
	#include <usbd_desc.h>

	extern USBD_HandleTypeDef USBD_Device;
	extern void SysTick_Handler(void);
//	void OTG_FS_IRQHandler(void);
//	void OTG_HS_IRQHandler(void);
	extern PCD_HandleTypeDef hpcd;
	
	int VCP_read(void *pBuffer, int size);
	int VCP_write(const void *pBuffer, int size);// use 0.4 ms to write 500 byte
	extern char g_VCPInitialized;
	
#ifdef __cplusplus
}
#endif
#include "TickTimer.h"
extern CTickTimer mCTickTimer_RealTime;

//#include "DigitalOut.h"
//mbed::DigitalOut p3(Tdio3);
//void SysTick_Handler(void)
//{
//	HAL_IncTick();
//	HAL_SYSTICK_IRQHandler();
//	mCTickTimer_RealTime.mSystemTick++;
////	p3.toggle();
//}
//
//#ifdef USE_USB_FS
//void OTG_FS_IRQHandler(void)
//{
//	HAL_PCD_IRQHandler(&hpcd);
//}
//#elif defined(USE_USB_HS)
//void OTG_HS_IRQHandler(void)
//{
//	HAL_PCD_IRQHandler(&hpcd);
//}
//#else
//#error USB peripheral type not defined
//#endif

#include "constant_define.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define LENGTH_USB_STRING (256)

class USBSerial
{
private:

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    asm("bkpt 255");
  }
  
  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    asm("bkpt 255");
  }
  
  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7; 
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK)
  {
    asm("bkpt 255");
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    asm("bkpt 255");
  }
}
public:
	USBSerial(void){};
	~USBSerial(void){};

	void setTimeout() {};
	void process();//Sends all remaining data from CDC TX buffer to USB out

	size_t write(uint8_t num){return write((char)num);};
	size_t write( char ch)	
{
	//return TM_USBD_CDC_Putc(TM_USB_FS,ch);
	return write((const uint8_t *)(&ch), 1);
};	
	size_t write(const char *buffer) 
{
	//return TM_USBD_CDC_Puts(TM_USB_FS, buffer);	
	int length=strlen(buffer);
	return write((const uint8_t *)buffer, length);
};
	size_t write(const uint8_t *buffer, size_t length){return VCP_write(buffer, length);};


	size_t readBytes( char *buffer, size_t length) { return readBytes((uint8_t *)buffer, length); };
	size_t readBytes( uint8_t *buffer, size_t length){return VCP_read(buffer,length);};



	//	size_t print(char);
	//	size_t print(unsigned char, int = DEC);
	//	size_t print(int, int = DEC);
	//	size_t print(unsigned int, int = DEC);
	//	size_t print(long, int = DEC);
	//	size_t print(unsigned long, int = DEC);
	//	size_t print(float, int = 2);
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

	size_t print(float num, int length = 5)
	{	
		
		if (num < 0)
		{
			write('-');
			num = -num;
		}				
		int x=(int)num;
		uint8_t L = convertNum2String(x);
		write(mStr);
		write('.');		

		num-=x;


		//x=(int)(num*1000000000.0);

		
		for(int k=0;k<9;k++)
		{
			float temp=num*10;
			if (temp<1)
			{
				num=temp;
				length--;
				write('0');	
			}
			else
				break;
		}
		length=LIMIT_MAX_MIN(length,9,1);// Math_Max is 10^9;
		x=(int)(num*Math_powX_Y(10,length));
		convertNum2String(x);
		write(mStr);
		return L;
	};	
	void  println()	{ print("\n");}	;	
	size_t println(float num, int length = DEC)	{ 		uint8_t L = print(num, length);		print("\n");	return L;}	;	
	size_t print(double num, int length = DEC){return print((float)num,length);};	
	size_t println(double num, int length = DEC)	{ 		uint8_t L = print(num, length);		print("\n");	return L;}	;	
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
	size_t convertNum2String(float number)
	{
		memset(mStr, 0, LENGTH_USB_STRING);
		uint8_t L = sprintf(mStr, "%f", number);
		return L;
	}
	size_t convertNum2String(double number)
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
	float Math_powX_Y(float X, int Y)
	{
		float out=1;
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


void	begin()
{   
//	HAL_Init();
//	SystemClock_Config();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Project_AFM_Controller_USB_fops);
	USBD_Start(&USBD_Device);
	char x=0;
	while(1)// wait for the PC to send the first byte, otherwise, the usb will be stucked.
		if (readBytes(&x, 1)>0)
			break;
}






private: 

	/* USB CDC settings */
	//TM_USBD_CDC_Settings_t USB_FS_Settings;
	//DigitalOut *usb_LED;
	//DigitalOut *usb_LED;
	uint16_t mCountProcess;
	char mStr[LENGTH_USB_STRING];

};



#endif
