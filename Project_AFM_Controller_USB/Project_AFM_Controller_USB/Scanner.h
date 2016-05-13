#pragma once

#include "defines.h"
#include "constant_define.h"

#include "PID_v1.h"
class CScanner
{public:
	CPID *mPID_Scanner;

	float mPositionADC18_MIN;
	float mPositionADC18_MAX;
		
	float mScannerPosition01;
	float mPositionFromSensorNow01;
	int mAxis;

	CScanner(void);
	~CScanner(void);
	void Initial(int axis,float mPeriod_Realtime_us)
	{
		mAxis = axis;
		bool direction = false;//,bool direction =true
		mPID_Scanner = new CPID(direction);
		
		
		mPID_Scanner->SetSampleTime(mPeriod_Realtime_us);
		mPID_Scanner->SetReferenceValue(mScannerPosition01);
		mPID_Scanner->SetOutputLimits(0, 1);// limit the change in each period
//		mPID_ZLOOP->SetPID_P(0.004);//(0.001
//		mPID_ZLOOP->SetPID_I(0.001);//(0.0001
		
//		mPID_Scanner->SetStepSize(0.01);
//		mPID_Scanner->SetPID_P(0.2);//use P=0.01, I=0.002 OK, 20160416
//		mPID_Scanner->SetPID_I(0.1);//(0.0001
//		mPID_Scanner->SetPID_D(0);
		const float step_size [] = {0.001, 0.0001, 0.0001};
		const float pid_p [] = {0.02,0.02,0.02};
		const float pid_i [] = {0.01, 0.01, 0.01};
		const float pid_d [] = {0.00001,0,0};
		mPID_Scanner->SetStepSize(step_size[mAxis]);
		mPID_Scanner->SetPID_P(pid_p[mAxis]);//use P=0.01, I=0.002 OK, 20160416
		mPID_Scanner->SetPID_I(pid_i[mAxis]);//(0.0001
		mPID_Scanner->SetPID_D(pid_d[mAxis]);
		
		mPositionADC18_MIN = 0;
		mPositionADC18_MAX = BIT18MAX;
		mScannerPosition01 = 0;
		mPositionFromSensorNow01 = 0;
		
	}
	;
	uint32_t update(uint32_t positionADC18)//time use=  28.2 us 
	{
		mPositionFromSensorNow01 = GetSensorPosition01(positionADC18);
		float outputDAC18 = mPID_Scanner->ComputePositioner(mPositionFromSensorNow01)*BIT18MAX;
		return outputDAC18;		
	}
	;
	
	// MoveToPosition01 only set reference position, let main cloop do PID 
	void MoveToPosition01(float position01)
	{
		mScannerPosition01 = position01;
		mPID_Scanner->SetReferenceValue(mScannerPosition01);
	}
	;
	void SetSensorRange(float ADC18_Min, float ADC18_Max)
	{
		mPositionADC18_MIN = ADC18_Min;
		mPositionADC18_MAX = ADC18_Max;
	}
	;
	float GetSensorPosition01()
	{
		return mPositionFromSensorNow01;
	}
	;
	float GetSensorPosition01(uint32_t positionADC18)
	{
		float position_feedback01 = ((float)positionADC18 - mPositionADC18_MIN) / (mPositionADC18_MAX - mPositionADC18_MIN);
		mPositionFromSensorNow01 = LIMIT_MAX_MIN(position_feedback01, 1, 0);
		return mPositionFromSensorNow01;
	}
	;
};

