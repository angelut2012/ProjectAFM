#include "AFM_functions.h"

AFM_Core mAFM_Core;
 
//enum Sys_State {SS_Idle, SS_Approach, SS_Engage, SS_Scan, SS_XYScanReset, SS_Indent};
#define REGION_LOCK()  static bool mlock = false;if (mlock == true) return;mlock = true; 
#define REGION_UNLOCK()    mlock = false;
//#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
void AFM_Core::AFM_Communication_Process()
{
//	REGION_LOCK();
//	static bool mlock = false;
//	if (mlock == true) return;
//	mlock = true;// lock
	
	mUSerial.process();
	mAFM_Core.Communication_Command_Console();
//	REGION_UNLOCK();
//		mlock = false;// lock
}
 void AFM_Core::AFM_ProcessScheduler_Realtime()
	{	

		//switch For dense case values compiler generates jump table,
		switch (mAFM_Core.sys_state)
		{
		case AFM_Core::SS_Scan:			mAFM_Core.process_ScanRealTimeLoop();break;
		case AFM_Core::SS_XYScanReset:	mAFM_Core.process_XYscanningReset();break;		
		case  AFM_Core::SS_Approach:		mAFM_Core.process_Approach();break;//
		case  AFM_Core::SS_Engage:		mAFM_Core.process_ZScannerEngage(); break;	

			break;
		default:
			;
		}

	}
void AFM_Core::AFM_ProcessScheduler_NonRealtime()
{	
		//switch For dense case values compiler generates jump table,
	switch (mAFM_Core.sys_state)
	{
	case  AFM_Core::SS_Indent:		mAFM_Core.process_Indent_First_SendDataThen(); break;	
		//case  SS_Indent:		process_Indent(); break;	
	case  AFM_Core::SS_Idle:			mAFM_Core.process_Idle(); 	
		break;
	default:
		;
	}

}	