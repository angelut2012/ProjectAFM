#ifndef PID_v1_h
#define PID_v1_h
//#define LIBRARY_VERSION	1.0.0


#include "mbed.h"
#include "constant_define.h"

class CPID
{
private:
	float mPosition;
	float mStepSize;
  public:

	float GetPositionOutput()
	{
		return mPosition;
	}
	;
	void SetStepSize(float value)
	{
		mStepSize = value;
	}
	;
  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
//    CPID(float, float, float,        // * constructor.  links the CPID to the Input, Output, and 
//        float, float, float,bool Direction_DirectTrue_ReverseFalse);     //   Setpoint.  Initial tuning parameters are also set here
	CPID(bool Direction_DirectTrue_ReverseFalse);     //   Setpoint.  Initial tuning parameters are also set here
    //void SetMode(int Mode);               // * sets CPID to either Manual (0) or Auto (non-0)
	//void FastCompute(); 14us
	float ComputePI_PRC_Loop(float mInput);
	float Compute(float mInput);        
	float ComputePositioner(float mInput);
	// * performs the CPID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

	void SetOutputLimits(float Min, float Max); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	// set only one parameter
	void SetReferenceValue(float value)
	{
		mySetpoint = value;
//		mPosition = value;// this line cause PID reset, do not use
	};
//	void SetPID_P(float value){SetTunings(value, dispKi, dispKd);};
//	void SetPID_I(float value){SetTunings(dispKp, value, dispKd);};
//	void SetPID_D(float value){SetTunings(dispKp, dispKi, value);};
	void SetPID_P(float value){kp = value;};
	void SetPID_I(float value){ki = value *(mSampleTimeIn_us / 1000000.0);};
	void SetPID_D(float value){kd = value / (mSampleTimeIn_us / 1000000.0);};
	
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(float);              // * sets the frequency, in Milliseconds, with which 
                                          //   the CPID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	float GetKp();						  // These functions query the pid for interal values.
	float GetKi();						  //  they were created mainly for the pid front-end,
	float GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the CPID.
	bool GetDirection(){return mDirection_DirectTrue_ReverseFalse;};					  //
	//float millis(){return 1;};
	void Reset(float position_01=0)
	{
	   lastInput = mySetpoint;
	   ITerm=0;
		mPosition = position_01;
	   //if(ITerm > outMax) ITerm = outMax;
	   //else if(ITerm < outMin) ITerm = outMin;
	};
	float GetError(){return error;};
  private:
	  float error;
	void Initialize();
	  //available but not commonly used functions ********************************************************
/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
	void SetTunings(float Kp, float Ki, float Kd)
	{
		if (Kp < 0 || Ki < 0 || Kd < 0) return;

		dispKp = Kp; dispKi = Ki; dispKd = Kd;

		kp = Kp;
		ki = Ki * (mSampleTimeIn_us / 1000000);// convert second
		kd = Kd / (mSampleTimeIn_us / 1000000);

			/*	if(controllerDirection ==REVERSE)
			{
			kp = (0 - kp);
			ki = (0 - ki);
			kd = (0 - kd);
			}
			*/
	};
		                      //   of changing tunings during runtime for Adaptive control
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	bool mDirection_DirectTrue_ReverseFalse;
public:
    float myInput;           
    float myOutput;            
    float mySetpoint;          
                            
			  
//	unsigned long lastTime;
	float ITerm, lastInput;

	float mSampleTimeIn_us;//us
	float outMin, outMax;
	bool inAuto;
};
#endif

