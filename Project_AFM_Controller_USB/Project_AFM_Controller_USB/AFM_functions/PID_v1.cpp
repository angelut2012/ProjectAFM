/**********************************************************************************************
* Arduino CPID Library - Version 1.0.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#include "PID_v1.h"

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up 
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
CPID::CPID(bool Direction_DirectTrue_ReverseFalse)
{

	ITerm = 0;// ITerm must be initialized, otherwise will be nan, cause problem
	myOutput = 0;
	myInput = 0;
	mySetpoint = 0;
	inAuto = false;

	CPID::SetOutputLimits(-131072, 131071);				//default output limit corresponds to 
	//the arduino pwm limits

	mSampleTimeIn_us = 500;		//1					//default Controller Sample Time is 1 us

	//CPID::SetControllerDirection(ControllerDirection);
	//CPID::SetTunings(Kp, Ki, Kd);
	mDirection_DirectTrue_ReverseFalse = Direction_DirectTrue_ReverseFalse;
	//   lastTime = millis()-mSampleTimeIn_us;				
	mPosition = 0;
}
//CPID::CPID(float Input, float Output, float Setpoint,
//		 float Kp, float Ki, float Kd,bool Direction_DirectTrue_ReverseFalse)
//{
//
//	myOutput = Output;
//	myInput = Input;
//	mySetpoint = Setpoint;
//	inAuto = false;
//
//	CPID::SetOutputLimits(-131072,131071);				//default output limit corresponds to 
//	//the arduino pwm limits
//
//	mSampleTimeIn_us = 1000;							//default Controller Sample Time is 1 us
//
//	//CPID::SetControllerDirection(ControllerDirection);
//	CPID::SetTunings(Kp, Ki, Kd);
//	mDirection_DirectTrue_ReverseFalse=Direction_DirectTrue_ReverseFalse;
//	//   lastTime = millis()-mSampleTimeIn_us;				
//}


/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
//float CPID::Compute(float mInput)// old
//{
//	// if(!inAuto) return false;
//	/*Compute all the working error variables*/
//	float input = mInput;
//	float dInput = (input - lastInput);
//
//	error = mySetpoint - input;
//	ITerm+= (ki * error);
//	if(ITerm > outMax) ITerm= outMax;
//	else if(ITerm < outMin) ITerm= outMin;
//
//	/*Compute CPID Output*/
//	float output=0;
//	if (mDirection_DirectTrue_ReverseFalse==false)
//	{//output = kp * error + ITerm- kd * dInput;
//		output+= kp * error;
//		output+= ITerm;
//		output-= kd * dInput;
//	}
//	else
//	{//output =-( kp * error + ITerm- kd * dInput;)
//		output-= kp * error;
//		output-= ITerm;
//		output+= kd * dInput;
//	}		
//
//	output=LIMIT_MAX_MIN(output, 0.05, -0.05);
////	mPosition += output;
////	mPosition = LIMIT_MAX_MIN(mPosition, outMax, outMin);
////	if(output > outMax) output = outMax;
////	else if(output < outMin) output = outMin;
//	myOutput = output;
//	myInput = input;
//	/*Remember some variables for next time*/
//	lastInput = input;
//	// return true;
//	//   lastTime = now;
//	//return true;
//	//}
//	//else return false;
//	return output;
//}
float CPID::ComputePositioner(float mInput)
{
	// if(!inAuto) return false;
	/*Compute all the working error variables*/
	float input = mInput;
	float dInput = (input - lastInput);

	error = mySetpoint - input;
	ITerm += (ki * error);
	
	
//	if (ITerm > outMax) ITerm = outMax;      // this will cause problem, when outMax and outMin are both >0
//	else if (ITerm < outMin) ITerm = outMin;

//	if (ITerm > mStepSize) ITerm = mStepSize;
//	else if (ITerm < -mStepSize) ITerm = -mStepSize;

	ITerm = LIMIT_MAX_MIN(ITerm,1,-1);
//	if (ITerm > 1) ITerm = 1;
//	else if (ITerm < -1) ITerm = -1;
	
	/*Compute CPID Output*/
	float output = 0;
	if (mDirection_DirectTrue_ReverseFalse == false)
	{//output = kp * error + ITerm- kd * dInput;
		output += kp * error;
		output += ITerm;
		output -= kd * dInput;
	}
	else
	{//output =-( kp * error + ITerm- kd * dInput;)
		output -= kp * error;
		output -= ITerm;
		output += kd * dInput;
	}		

	output = LIMIT_MAX_MIN(output, mStepSize, -mStepSize);
	mPosition += output;
	
	mPosition = LIMIT_MAX_MIN(mPosition, outMax, outMin);
//	if(output > outMax) output = outMax;
//	else if(output < outMin) output = outMin;
	myOutput = output;
	myInput = input;
	/*Remember some variables for next time*/
	lastInput = input;
	// return true;
	//   lastTime = now;
	//return true;
	//}
	//else return false;
	return mPosition;
}
float CPID::ComputePI_PRC_Loop(float mInput)// for PID PRC sensor loop
{
	// if(!inAuto) return false;
	/*Compute all the working error variables*/
	float input = mInput;
	float dInput = (input - lastInput);

	error = mySetpoint - input;
	ITerm += (ki * error);
	

//	if (ITerm > outMax) ITerm = outMax;
//	else if (ITerm < outMin) ITerm = outMin;
//	ITerm = LIMIT_MAX_MIN(ITerm, 0.001, -0.001);
	ITerm = LIMIT_MAX_MIN(ITerm, 0.01, -0.01);
//	if (ITerm > 1) ITerm = 0.01;
//	else if (ITerm < -1) ITerm = -0.01;

	/*Compute CPID Output*/
	float output = 0;
	if (mDirection_DirectTrue_ReverseFalse == false)
	{//output = kp * error + ITerm- kd * dInput;
		output += kp * error;
		output += ITerm;
		output -= kd * dInput;
		
		output = LIMIT_MAX_MIN(output, mStepSize, -mStepSize);
		mPosition -= output;
	}
	else
	{//output =-( kp * error + ITerm- kd * dInput;)
		output -= kp * error;
		output -= ITerm;
		output += kd * dInput;
		
		output = LIMIT_MAX_MIN(output, mStepSize, -mStepSize);
		mPosition += output;
	}		

//	output = LIMIT_MAX_MIN(output, mStepSize, -mStepSize);
//	mPosition += output;
	
	mPosition = LIMIT_MAX_MIN(mPosition, outMax, outMin);
//	if(output > outMax) output = outMax;
//	else if(output < outMin) output = outMin;
	myOutput = output;
	myInput = input;
	/*Remember some variables for next time*/
	lastInput = input;
	// return true;
	//   lastTime = now;
	//return true;
	//}
	//else return false;
	return mPosition;
}
//float CPID::ComputePI_old(float mInput)
//{
//	// if(!inAuto) return false;
//	/*Compute all the working error variables*/
//	float input = mInput;
//	float dInput = (input - lastInput);
//
//	error = mySetpoint - input;
//	ITerm += (ki * error);
//	if (ITerm > outMax) ITerm = outMax;
//	else if (ITerm < outMin) ITerm = outMin;
//
//	/*Compute CPID Output*/
//	float output = 0;
//	if (mDirection_DirectTrue_ReverseFalse == false)
//	{//output = kp * error + ITerm- kd * dInput;
//		output += kp * error;
//		output += ITerm;
//		//output -= kd * dInput;
//	}
//	else
//	{//output =-( kp * error + ITerm- kd * dInput;)
//		output -= kp * error;
//		output -= ITerm;
//		//output += kd * dInput;
//	}		
//
//	if (output > outMax) output = outMax;
//	else if (output < outMin) output = outMin;
//	myOutput = output;
//
//	/*Remember some variables for next time*/
//	lastInput = input;
//	// return true;
//	//   lastTime = now;
//	//return true;
//	//}
//	//else return false;
//	return output;
//}



/* SetSampleTime(...) *********************************************************
* sets the period, in micro seconds, at which the calculation is performed	
******************************************************************************/
void CPID::SetSampleTime(float NewSampleTimeIn_us)
{
	if (NewSampleTimeIn_us > 0)
	{
		float ratio  = NewSampleTimeIn_us/mSampleTimeIn_us;
		ki *= ratio;
		kd /= ratio;
		mSampleTimeIn_us = NewSampleTimeIn_us;
	}
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void CPID::SetOutputLimits(float Min, float Max)
{
	if(Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if(inAuto)
	{
		if(myOutput > outMax) myOutput = outMax;
		else if(myOutput < outMin) myOutput = outMin;

		if(ITerm > outMax) ITerm= outMax;
		else if(ITerm < outMin) ITerm= outMin;
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/ 
//void CPID::SetMode(int Mode)
//{
//    bool newAuto = (Mode == AUTOMATIC);
//    if(newAuto == !inAuto)
//    {  /*we just went from manual to auto*/
//        CPID::Initialize();
//    }
//    inAuto = newAuto;
//}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void CPID::Initialize()
{
	ITerm = myOutput;
	lastInput = myInput;
	if(ITerm > outMax) ITerm = outMax;
	else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
* The CPID will either be connected to a DIRECT acting process (+Output leads 
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
/*void CPID::SetControllerDirection(int Direction)
{
if(inAuto && Direction !=controllerDirection)
{
kp = (0 - kp);
ki = (0 - ki);
kd = (0 - kd);
}   
controllerDirection = Direction;
}
*/
/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the CPID.  they're here for display 
* purposes.  this are the functions the CPID Front-end uses for example
******************************************************************************/
//float CPID::GetKp(){ return  dispKp; }
//float CPID::GetKi(){ return  dispKi;}
//float CPID::GetKd(){ return  dispKd;}
float CPID::GetKp(){ return  kp; }
float CPID::GetKi(){ return  ki;}
float CPID::GetKd(){ return  kd;}
int CPID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}


//void CPID::FastCompute()
//{
//
//      /*Compute all the working error variables*/
//	  //float (*myInput) = *myInput;
//      float error = *mySetpoint - (*myInput);
//      ITerm+= (ki * error);
//      if(ITerm > outMax) ITerm= outMax;
//      else if(ITerm < outMin) ITerm= outMin;
//      float dInput = ((*myInput) - lastInput);
// 
//      /*Compute CPID Output*/
//      float output = kp * error + ITerm- kd * dInput;
//      
//	  if(output > outMax) output = outMax;
//      else if(output < outMin) output = outMin;
//	  *myOutput = output;	  
//      /*Remember some variables for next time*/
//      lastInput = (*myInput);
//}
