#include "AFM_functions.h"

AFM_Core mAFM_Core;

//enum Sys_State {SS_Idle, SS_Approach, SS_Engage, SS_Scan, SS_XYScanReset, SS_Indent};

//#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else



//			float ad=mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, true);
//			float da = mCScanner[PIEZO_Z].update(ad);
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, da);
//			float er = mCScanner[PIEZO_Z].mPID_Scanner->GetError();
//			MY_Debug_StringValue_T("ADC", ad);
//			MY_Debug_StringValue_T("DAC", da);
//			MY_Debug_StringValue_T("In", mCScanner[PIEZO_Z].mPID_Scanner->myInput);
//			
//			MY_Debug_StringValue_T("outd", mCScanner[PIEZO_Z].mPID_Scanner->myOutput);
//			
//			MY_Debug_StringValue_T("po", mCScanner[PIEZO_Z].mPID_Scanner->mPosition);
//			MY_Debug_StringValue_LN("er", er);
//			mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, mCScanner[PIEZO_Z].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, true)));
//			MY_Debug_LN(mCScanner[PIEZO_Z].mPID_Scanner->GetError());

//			if (
//			    (mAFM_Core.sys_state != SS_Approach)
//			    & 
//			    (mAFM_Core.sys_state != SS_ApproachWait)
//			    &
//			    (mAFM_Core.sys_state !=SS_Indent)
//			    )

//int ch[3];
//	ch[PIEZO_Z] = ADC_CHANNEL_Z;
//	ch[PIEZO_Y] = ADC_CHANNEL_Y;
//	ch[PIEZO_X] = ADC_CHANNEL_X;
void AFM_Core::AFM_ProcessScheduler_Realtime()
{	
	//AFM_Communication_Process();

	//when time is due, execute tasks, and reset timer
	//		if (mCTickTimer_RealTime.read_us() >= mAFM_Core.mPeriod_RealtimePID_us)
	//			if (mCTickTimer_RealTime.read_us() >0)
	{			


		//			mCTickTimer_RealTime.reset();

		//CHECK_COUNT_DUE(TIMES_PID_LOOP);			

		//switch For dense case values compiler generates jump table,
		switch (mAFM_Core.sys_state)
		{
		case AFM_Core::SS_Scan:			mAFM_Core.process_ScanRealTimeLoop();break;
		case AFM_Core::SS_XYScanReset:	mAFM_Core.process_XYscanningReset();break;		
			//		case  AFM_Core::SS_Approach:		mAFM_Core.process_Approach();break;//
		case  AFM_Core::SS_Engage:		mAFM_Core.process_ZScannerEngage(); break;	

			// non realtime process			
		case  AFM_Core::SS_Approach:		mAFM_Core.process_Approach();break;//		
			//			case  AFM_Core::SS_Indent :		mAFM_Core.process_Indent_First_SendDataThen_vibration_test(); break;	
		case  AFM_Core::SS_Indent:		mAFM_Core.process_Indent_First_SendDataThen(); break;		
		//case  AFM_Core::SS_DataCapture:		mAFM_Core.process_data_capture(); break;		

		case  AFM_Core::SS_Idle:			mAFM_Core.process_Idle(); 			break;			
		case  AFM_Core::SS_WaveTest:			mAFM_Core.process_WaveTest(); 			break;
			
			
			break;
		default:
			;
		}



		if (mAFM_Core.sys_state < SS_Engage)
			// in approach and indent, Mdirectly use DAC value, do not use pid to control the piezo positions
		{

			////				//			for (int k = 0;k < NUM_OF_SCANNER;k++)
			////				//				mAFM_DAC.FinePositioner_MoveToPositionB18(k, mCScanner[k].update(mAFM_SEM.ADC_Read_N(ch[k], true)));
			////				//FOR_REPEAT(1,mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, mCScanner[PIEZO_Z].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, true))));



			//				changed to use open loop
			//mAFM_DAC.FinePositioner_MoveToPositionB18(PIEZO_Z, mCScanner[PIEZO_Z].update(mAFM_SEM.ADC_Read_N(ADC_CHANNEL_Z, false)));
		}			
		//			CHECK_COUNT_DUE2(5);



	}

}
void AFM_Core::AFM_ProcessScheduler_NonRealtime()
{	
	//		//switch For dense case values compiler generates jump table,
	//	switch (mAFM_Core.sys_state)
	//	{
	//	case  AFM_Core::SS_Approach:		mAFM_Core.process_Approach();break;//		
	//	//case  AFM_Core::SS_Indent :		mAFM_Core.process_Indent_First_SendDataThen_vibration_test(); break;	
	//	case  AFM_Core::SS_Indent:		mAFM_Core.process_Indent_First_SendDataThen_data_capture(); break;		
	////	case  AFM_Core::SS_Indent:		mAFM_Core.process_Indent_First_SendDataThen(); break;	
	//	
	//	case  AFM_Core::SS_Idle:			mAFM_Core.process_Idle(); 			break;
	//	default:
	//		;
	//	}

}	