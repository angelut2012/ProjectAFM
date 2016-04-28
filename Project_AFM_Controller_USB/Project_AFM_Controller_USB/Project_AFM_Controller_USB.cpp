#include "main_include.h"
//using namespace mbed;
extern USBSerial mUSerial;

#include "PinNames.h"

DigitalOut p(Tdio2);

DigitalIn pi(Tdio1);

	extern AFM_Core mAFM_Core;

int main(void )
	//AA 55 43 41 43 00 00 00 55 AA   withdraw
{	
	//
	//mUSerial.begin();
	//char x=0,y=0;
	////while(1)
	////if (mUSerial.readBytes(&x, 1)>0)//100)
	//		while(1)
	//		{
	//	//		//mUSerial.readBytes(&x, 1);//100
	//	//		//if (L>0)	
	//			mUSerial.println(y++);
	//			wait(0.01);
	//	}
	////
	mAFM_Core.AFM_main_setup();
	
//	byte d[500];
//	for (int k = 0;k < 500;k++)
//		d[k] = k;
//	
//	while (1)
//	{
//		p.toggle();
//		wait(0.1);
//		mUSerial.write(d, 100);		
//	}

		
	
	
	
	
	
	
	while (1)
		mAFM_Core.AFM_main_loop();

//------------------------------------------------------------------------------

	//main_AFM();
	
	bool v = true;
	static bool t = 0;
		uint8_t byte[100];

	for (;;)
	{
		int L=mUSerial.readBytes(byte, 1);//100
			//continue;
		//VCP_write("\r\nYou typed ", 12);
		byte[0]++;
		if (L>0)
		{
			mUSerial.write(byte,L);
			int x=mCTickTimer_RealTime.read_us();
			mUSerial.println(x);
//			mUSerial.println(HAL_RCC_GetHCLKFreq());
			
			{
				v = !v;
				p = v;
			}
		}
		//VCP_write("\r\n", 2);

		//HAL_Delay(100);
	}
	return 0;
}