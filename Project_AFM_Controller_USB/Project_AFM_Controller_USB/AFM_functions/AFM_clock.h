
#ifndef __AFM_clock_H__
#define __AFM_clock_H__

#ifdef __cplusplus
extern "C" {
#endif
 
#ifndef BIT_N1
#define BIT_N1(x) (1<<x)
#endif // !BIT_N1


#define __TICK_TIMES__ (5)// <32 us will cause problem in usb commnication

#define __TICK_PERIOD_US__ (BIT_N1(__TICK_TIMES__)) //(32)
#define __TICK_FREQUENCY_HZ__  (1000000/__TICK_PERIOD_US__)


	void wait_us(float time_us);
	void wait_ms(float time_ms);
	void wait(float time_s);
	void SystemClock_Config(void);
	void SysTick_Handler(void);
	void HAL_ResetTick(void);
	void OTG_FS_IRQHandler(void);          
#ifdef __cplusplus
}
#endif

#endif 