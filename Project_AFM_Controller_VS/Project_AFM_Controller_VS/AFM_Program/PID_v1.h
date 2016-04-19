#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0


#include "mbed.h"


class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
//    PID(double, double, double,        // * constructor.  links the PID to the Input, Output, and 
//        double, double, double,bool Direction_DirectTrue_ReverseFalse);     //   Setpoint.  Initial tuning parameters are also set here
	PID(bool Direction_DirectTrue_ReverseFalse);     //   Setpoint.  Initial tuning parameters are also set here
    //void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)
	//void FastCompute(); 14us
	double ComputePI(double mInput);
	double Compute(double mInput);                     // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

	void SetOutputLimits(double Min, double Max); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	// set only one parameter
	void SetReferenceValue(double value){mySetpoint = value;};
//	void SetPID_P(double value){SetTunings(value, dispKi, dispKd);};
//	void SetPID_I(double value){SetTunings(dispKp, value, dispKd);};
//	void SetPID_D(double value){SetTunings(dispKp, dispKi, value);};
	void SetPID_P(double value){kp = value;};
	void SetPID_I(double value){ki = value *(mSampleTimeIn_us / 1000000.0);};
	void SetPID_D(double value){kd = value / (mSampleTimeIn_us / 1000000.0);};
	
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(double);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	bool GetDirection(){return mDirection_DirectTrue_ReverseFalse;};					  //
	//double millis(){return 1;};
	void Reset()
	{
	   //ITerm = *myOutput;
	   lastInput = myInput;
	   ITerm=0;
	   //if(ITerm > outMax) ITerm = outMax;
	   //else if(ITerm < outMin) ITerm = outMin;
	};
	double GetError(){return error;};
  private:
	  double error;
	void Initialize();
	  //available but not commonly used functions ********************************************************
/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
	void SetTunings(double Kp, double Ki, double Kd)
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
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				//   format for display purposes
	double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	bool mDirection_DirectTrue_ReverseFalse;

    double myInput;           
    double myOutput;            
    double mySetpoint;          
                            
			  
//	unsigned long lastTime;
	double ITerm, lastInput;

	double mSampleTimeIn_us;//us
	double outMin, outMax;
	bool inAuto;
};
#endif

