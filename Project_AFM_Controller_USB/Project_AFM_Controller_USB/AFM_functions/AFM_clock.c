#include "AFM_clock.h"
#include "stm32f7xx_hal.h"
void wait_us(float time_us)
{
	//time_us -= time_us / __TICK_PERIOD_US__ * 2.6;
	//uint32_t	time_tick = (uint32_t)(time_us*HAL_RCC_GetHCLKFreq() / 30.52e6);// HAL_RCC_GetHCLKFreq()= 216MHz
	////	uint32_t	time_tick = time_us * 7.077326343381389;
	//while (time_tick--)
	//	;

		float x=1,y=1;
		FOR_REPEAT(time_us,{x*=y;x*=y;x*=y;});
}
	
void wait_ms(float time_ms)
{
	wait_us(time_ms * 1000);
}
	
void wait(float time_s)
{
	wait_us(time_s * 1000000);
}
	
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	//		mCTickTimer_RealTime.mSystemTick++;
			//	p3.toggle();
}	
 void HAL_ResetTick(void)
{
//	uwTick = 0;
}

#ifdef USE_USB_FS

extern PCD_HandleTypeDef hpcd;
void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
#elif defined(USE_USB_HS)
void OTG_HS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}
#else
#error USB peripheral type not defined
#endif