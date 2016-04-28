#ifndef _UNIVERSIAL_FUNCTION_
#define _UNIVERSIAL_FUNCTION_

//#define _ALWAYS_INLINE_ __attribute__((always_inline))


template <class T> T  MOD_range(T value, T range_max);
template <class T> T MOD_range(T value, T range_max)
{
	// speed up of "index=k%mI_MaxStep;", and % operator has problem with int
	while(value>=range_max)
		value-=range_max;
	while(value<0)
		value+=range_max;
	return value;
}
// wave_triangle, 45 degree

//long wave_triangle(long t, long period)
//{
//	long period_half=period>>1;// fast
//	t=MOD_range(t+period_half,period);
//	t-=period_half;
//	t=abs(t);
//	return t;
//}

//wave_triangle, 45 degree, maximum value period/2
//template <class T> T  wave_triangle( T t, T period)
//{
//	static T t_old=0;
//	t-=t_old;
//	T period_half=period/2;
//	//if (typeid(period)==typeid(float) | typeid(period)==typeid(float) )//typeid is disabled on Arduino
//	//	period_half=period/2;
//	//else
//	//	period_half=period>>1;
//	T nt=MOD_range(t+period_half,period);
//	t_old=t-nt;// save for speed up MOD_range
//	nt-=period_half;
//	nt=abs(nt);
//	return nt;
//}



//long  wave_triangle( long t, long period)
//{
//	long period_half=period/2;
//	t=(t+period_half)%period;
//	t-=period_half;
//	t=abs(t);
//	return t;
//}
//
//
//float  wave_triangle_0ToMax(float delta, float value_max, bool reset);
//

#endif