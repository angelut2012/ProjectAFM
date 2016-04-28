
//#include "stm32f7xx_hal_tim.h"

#ifndef __TICK_TIMER_HEADER__
#define  __TICK_TIMER_HEADER__

#include "stm32f7xx_hal.h"


//stm32f7xx_hal.c
// HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)// use strong define to override the default system define
//{
//  /*Configure the SysTick to have interrupt in 1ms time basis*/
////  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 100000);// change system default tick=10 us
//
//  /*Configure the SysTick IRQ priority */
//	HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);
//
//	  /* Return function status */
//	return HAL_OK;
//}
//#ifdef __cplusplus
//extern "C"
//{
//#endif
//
//	
//#ifdef __cplusplus
//}
//#endif

class CTickTimer 
{
//#define mTick_us 100// 32// (0.5)
public:
	unsigned int mSystemTick;
	CTickTimer()
	{
		mSystemTick=0;
//		mSystemTick_b=0;

		//double f = 1000000.0 / mTick_us;
//		Initial(160);
	}
	;
	
	unsigned int read_us()
	{
	//# define mTick_us(0.5)
		return read_ticker() << __TICK_TIMES__;// 32 us each tick
//		return mSystemTick;// *mTick_us;
	}
	;
	unsigned int read_ticker()
	{
	//# define mTick_us(0.5)
//		return mSystemTick;// << __TICK_TIMES__;// 32 us each tick
//		return mSystemTick;// *mTick_us;
		mSystemTick = HAL_GetTick();//-mSystemTick_b;
		return mSystemTick;
	}
	;
	void reset()
	{		
//		mSystemTick_b=mSystemTick;
		HAL_ResetTick();
	}
	;
	void start()
	{
		
		reset();
	}
	;
//private:
	
//	void Initial(int frequency)
//	{
//		reset();
//		// interrupt at 100Hz		
//		int x=SysTick_Config(SystemCoreClock / frequency);
//		x++;
//	}
};


#endif // !__TICK_TIMER_HEADER__
