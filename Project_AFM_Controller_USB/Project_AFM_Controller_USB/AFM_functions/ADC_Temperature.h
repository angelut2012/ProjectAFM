#ifndef __ADC_Temperature__
#define __ADC_Temperature__  

///* C++ detection */
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include "tm_stm32_adc.h"
#include "constant_define.h"
#include "define_AFM_platform.h"	
//	\verbatim
//CHANNEL   ADC1   ADC2   ADC3
//0         PA0    PA0    PA0
//1         PA1    PA1    PA1
//2         PA2    PA2    PA2
//3         PA3    PA3    PA3
//4         PA4    PA4    PF6
//5         PA5    PA5    PF7
//6         PA6    PA6    PF8
//7         PA7    PA7    PF9
//8         PB0    PB0    PF10
//9         PB1    PB1    PF3
//10        PC0    PC0    PC0
//11        PC1    PC1    PC1
//12        PC2    PC2    PC2
//13        PC3    PC3    PC3
//14        PC4    PC4    PF4
//15        PC5    PC5    PF5
//\endverbatim
class CTemperature 
{
public: 
	CTemperature()
	{
//		TM_ADC_Channel_t mTemperatureController;// = TM_ADC_Channel_11;
//		mTemperatureController = TM_ADC_Channel_11;
//		TM_ADC_Init(ADC1, mTemperatureController);
	}
	;	
	~CTemperature()
	{
		
	}
	;
// alalog input conflict with USB
	
	float Read(bool Convert2Degree = true)
	{
//		TM_ADC_Channel_t mTemperatureController;// = TM_ADC_Channel_11;
//		mTemperatureController = TM_ADC_Channel_11;
//		const float VCC = 2.8;
//		const float Sensitivity_VperC = 0.02;
//		float T = 0;
//		T = (float)TM_ADC_Read(ADC1, mTemperatureController);
//		if (Convert2Degree == true)
//		{	
//			T = T / 4095.0;//BIT(12);
//			T = T* VCC;//--> Volt	
//			T = (T - 0.5) / Sensitivity_VperC + 25	- 2.4;// degree
//		}
//		return T;
		return 0;
	}
	;

	
};

//	
///* C++ detection */
//#ifdef __cplusplus
//}
//#endif
#endif
