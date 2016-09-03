#pragma once

#include "defines.h"
#include "constant_define.h"

#include "PID_v1.h"

#include "USBSerial.h"

extern USBSerial mUSerial;// global


class CScanner
{public:
	CPID *mPID_Scanner;

	float mPosition01Raw_MIN;
	float mPosition01Raw_MAX;
		
	float mTemperature;
	float mScannerPosition01;
	float mPositionFromSensorNow01;
	int mAxis;
	
	bool mDirection_PID;
	~CScanner(void) {}	;
	CScanner(void)
	{
		mDirection_PID = true;// if Vpiezo increase, sensor readout increase,==> set derection true
	}
	;
	void Initial(int axis,float mPeriod_Realtime_us)
	{
		mAxis = axis;
		
		const bool pid_direction [] = {false,false,false};
		mDirection_PID = pid_direction[mAxis];
		mPID_Scanner = new CPID(mDirection_PID);
		
		
		mPID_Scanner->SetSampleTime(mPeriod_Realtime_us);
		mPID_Scanner->SetReferenceValue(mScannerPosition01);
		mPID_Scanner->SetOutputLimits(0, 1);// limit the change in each period
//		mPID_ZLOOP->SetPID_P(0.004);//(0.001
//		mPID_ZLOOP->SetPID_I(0.001);//(0.0001
		
//		mPID_Scanner->SetStepSize(0.01);
//		mPID_Scanner->SetPID_P(0.2);//use P=0.01, I=0.002 OK, 20160416
//		mPID_Scanner->SetPID_I(0.1);//(0.0001
//		mPID_Scanner->SetPID_D(0);
//		const float step_size [] = {0.001, 0.0001, 0.0001};
//		const float pid_p [] = {0.02,0.02,0.02};
//		const float pid_i [] = {0.01, 0.01, 0.01};
//		const float pid_d [] = {0.00001,0,0};
		

		
//		const float step_size [] = {0.05, 0.05, 0.05};
//		const float pid_p [] = {0.2, 0.1, 0.1};
//		const float pid_i [] = {0.01, 0.0151, 0.0151};
//		const float pid_d [] = {0.0001, 0.00001, 0.00001};		
		
		// old
//		const float step_size [] = {0.05, 0.05, 0.05};
//		const float pid_p [] = {0.2, 0.07, 0.07};
//		const float pid_i [] = {0.01, 0.0051, 0.0051};
//		const float pid_d [] = {0.0001, 0.00001, 0.00001};				
		//20160901, overcome xy overshot
//		Z=0,Y=1,X=2
		
		// y slow
//		const float step_size [] = {0.05, 0.0001, 0.001};
//		const float pid_p [] = {0.2, 0.002, 0.02};
//		const float pid_i [] = {0.01, 0.000051, 0.00051};
//		const float pid_d [] = {0.0001, 0.000001, 0.00001};	
		
		const float step_size [] = {0.05, 0.0005, 0.001};
		const float pid_p [] = {0.2, 0.02, 0.02};
		const float pid_i [] = {0.01, 0.00051, 0.00051};
		const float pid_d [] = {0.0001, 0.00001, 0.00001};	
		
		mPID_Scanner->SetStepSize(step_size[mAxis]);
		mPID_Scanner->SetPID_P(pid_p[mAxis]);//use P=0.01, I=0.002 OK, 20160416
		mPID_Scanner->SetPID_I(pid_i[mAxis]);//(0.0001
		mPID_Scanner->SetPID_D(pid_d[mAxis]);
		
		mPosition01Raw_MIN = 0;
		mPosition01Raw_MAX = BIT18MAX;
		mScannerPosition01 = 0;
		mPositionFromSensorNow01 = 0;
		
	}
	;
	void SetTemperature(float valuez)
	{
		mTemperature = valuez;
		
//		if (mAxis==PIEZO_Z)
//		mTemperature = valuez;
//		else
//		{
//			const	float	z2xyp1		=			-0.00001159124509559680000000	;
//			const float	 z2xyTp2		=			2.37301442007774000000000000	;
//			const float	 z2xyTp3		=			-61218.29975568410000000000000000	+761.5195000000022;
//			mTemperature=(z2xyp1*valuez + z2xyTp2)*valuez + z2xyTp3;
//		}
	}	;

	float GetPositionError01()
	{
		return mPID_Scanner->GetError();
	}
		uint32_t ComputePID(uint32_t positionADC18)//time use=  28.2 us 
	{
		mPositionFromSensorNow01 = GetSensorPosition01(positionADC18);		
		float outputDAC18 = mPID_Scanner->ComputePositioner(mPositionFromSensorNow01)*BIT18MAX;
		return outputDAC18;		
	}
	;
	
	float GetDestinationPosition01()
	{
		return mScannerPosition01;
	}
	;
	
	// SetDestinationPosition01 only set reference position, let main cloop do PID 
	void SetDestinationPosition01(float position01)
	{
		mScannerPosition01 = position01;
		mPID_Scanner->SetReferenceValue(mScannerPosition01);
	}
	;
	void SetSensorRange(float ADC18_Min, float ADC18_Max)
	{
		mPosition01Raw_MIN = ADC18_Min;
		mPosition01Raw_MAX = ADC18_Max;
	}
	;
	float GetSensorPosition01()
	{
		return mPositionFromSensorNow01;
	}
	;
//	float GetSensorPosition01(uint32_t positionADC18)
//	{
//		float position_feedback01 = ((float)positionADC18 - mPositionADC18_MIN) / (mPositionADC18_MAX - mPositionADC18_MIN);
//		mPositionFromSensorNow01 = LIMIT_MAX_MIN(position_feedback01, 1, 0);
//		return mPositionFromSensorNow01;
//	}
//	;
	
	// temperature compensation
	float GetSensorPosition01(uint32_t positionADC18)
	{
		float p_raw = GetSensorPosition01_raw( positionADC18);

		mPositionFromSensorNow01 = ((float)p_raw - mPosition01Raw_MIN) / (mPosition01Raw_MAX - mPosition01Raw_MIN);
		return mPositionFromSensorNow01;
	}
	float GetSensorPosition01_raw(uint32_t positionADC18)
	{

//float Px[]={
//	-2.18189580236137
//2.45599605316321e-05
//2.78209530128587e-06
//1.02468143713129e-10
//1.73591388851994e-10
//};
//	float  Pz[]=	{
//4.15764792644348,
//-4.18560151355447e-05,
//1.31311384092856e-05,
//-2.62174364080383e-11,
//4.76839638324102e-11};
//	#define p00	P[0]
//	#define p01	P[1]
//	#define p10	P[2]
//	#define p11	P[3]
//	#define p02	P[4]
		//double p00 = 8.315295849407488e4;
		//double p01 = -0.837120302163872;
		//double p10 = 0.262622768185710;
		//double p11 = -5.243487281607523e7;
		//double p02 = 9.536792745046144e7;	 
//static	const	float	xp00		=	-1.25976130991591000000000000	;
//static	const	float	xp01		=	0.04064898312247370000000000	;
//static	const	float	xp10		=	0.00000774214599727934000000	;
//static	const	float	xp11		=	0.00000002591495044575960000	;
//static	const	float	xp02		=	-0.00027860784460091000000000	;
//static	const	float	yp00		=	-1.39306871111335000000000000	;
//static	const	float	yp01		=	0.04023992323571540000000000	;
//static	const	float	yp10		=	0.00000758641170006858000000	;
//static	const	float	yp11		=	0.00000009225240571389740000	;
//static	const	float	yp02		=	-0.00013983813379961800000000	;
//static	const	float	zp00		=	5.56940871292436000000000000	;
//static	const	float	zp01		=	-0.00006552734101777870000000	;
//static	const	float	zp10		=	0.00001553239623067130000000	;
//static	const	float	zp11		=	-0.00000000004439813004644080	;
//static	const	float	zp02		=	0.00000000014588477288958600	;
		static	const	float	xp00		=	-1.25976130991591000000000000	*BIT18MAX;
		static	const	float	xp01		=	0.04064898312247370000000000	*BIT18MAX;
		static	const	float	xp10		=	0.00000774214599727934000000	*BIT18MAX;
		static	const	float	xp11		=	0.00000002591495044575960000	*BIT18MAX;
		static	const	float	xp02		=	-0.00027860784460091000000000	*BIT18MAX;
		static	const	float	yp00		=	-1.39306871111335000000000000	*BIT18MAX;
		static	const	float	yp01		=	0.04023992323571540000000000	*BIT18MAX;
		static	const	float	yp10		=	0.00000758641170006858000000	*BIT18MAX;
		static	const	float	yp11		=	0.00000009225240571389740000	*BIT18MAX;
		static	const	float	yp02		=	-0.00013983813379961800000000	*BIT18MAX;
		static	const	float	zp00		=	5.56940871292436000000000000	*BIT18MAX;
		static	const	float	zp01		=	-0.00006552734101777870000000	*BIT18MAX;
		static	const	float	zp10		=	0.00001553239623067130000000	*BIT18MAX;
		static	const	float	zp11		=	-0.00000000004439813004644080	*BIT18MAX;
		static	const	float	zp02		=	0.00000000014588477288958600	*BIT18MAX;



		float position_compensated =positionADC18;
//		if (mAxis==PIEZO_Z)
//			position_compensated= zp00 + positionADC18*zp10 + mTemperature*(zp01 + mTemperature*zp02 + positionADC18*zp11);
		
		
		//else if (mAxis==PIEZO_X)
		//position_compensated= xp00 + positionADC18*xp10 + mTemperature*(xp01 + mTemperature*xp02 + positionADC18*xp11);
		//else if (mAxis==PIEZO_Y)
		//position_compensated= yp00 + positionADC18*yp10 + mTemperature*(yp01 + mTemperature*yp02 + positionADC18*yp11);

		//float position_compensated = p00 + positionADC18*p10 + mTemperature*(p01 + mTemperature*p02 + positionADC18*p11);
			return position_compensated;
//		float position_feedback01 = ((float)positionADC18 - mPosition01Raw_MIN) / (mPosition01Raw_MAX - mPosition01Raw_MIN);
//		mPositionFromSensorNow01 = LIMIT_MAX_MIN(position_feedback01, 1, 0);
//		return mPositionFromSensorNow01;
	}
	;
	

};

