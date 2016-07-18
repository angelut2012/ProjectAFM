// integrated, 20160610

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace NameSpace_AFM_Project
{
    public class CCoarsePositioner
    {
        uint mSensorMode;
        private bool moving;
        const int NUM_OF_AXIS = (3);
        const int X_AXIS = (0);
        const int Y_AXIS = (1);
        const int Z_AXIS = (2);
        //const int T_AXIS = (3);
        const int AXIS_DELTA = (3);     //0
        const int X_CP_AXIS = (AXIS_DELTA + X_AXIS);
        const int Y_CP_AXIS = (AXIS_DELTA + Y_AXIS);
        const int Z_CP_AXIS = (AXIS_DELTA + Z_AXIS);

        // Math.Sign(x)  ((x > 0) ? 1 : ((x < 0) ? -1 : 0))      
        uint mSystemIndex;
        uint mResult;
        uint mStatus;
        uint mOpenLoopFrequency;
        double[] mDirection;// +-1, control the direction of each axis

        public void MY_DEBUG(string inf)
        {
            if (string.IsNullOrEmpty(inf) == false)
                System.Diagnostics.Debug.WriteLine(inf);
        }

        public CCoarsePositioner()
        {
            mDirection = new double[6]{-1,-1,-1,  -1,-1,-1};
            // -1 negative, 
            //+1 positive, move away from cable
            //mDirection[X_CP_AXIS] = -1;
            //mDirection[Y_CP_AXIS] = -1;
            //mDirection[Z_CP_AXIS] = -1;
            moving = false;
        }

        // ~CCoarsePositioner(void)
        //{
        //	delete mDirection;
        //}
        public bool IsMoving()
        {
            return moving;
        }

        public int Initialize()
        {
            //(uint)mSystemIndex = 0;
            //try
            //{
            //    Stop(X_CP_AXIS);
            //    Stop(Y_CP_AXIS);
            //    Stop(Z_CP_AXIS);
            //}
            //catch (Exception ex)
            //{
            //    MY_DEBUG("coarse nanopositioner  initial error!");
            //}
            for (int k = 0; k < 10; k++)
            {
                try
                {
                    CCoarseController.SA_ReleaseSystems();
                    Thread.Sleep(5 * k + 5);
                    mResult = CCoarseController.SA_InitSystems(CCoarseController.SA_SYNCHRONOUS_COMMUNICATION);	 // init systems	
                    if (mResult == CCoarseController.SA_OK)
                        break;
                    Thread.Sleep(5);
                }
                catch (Exception ex)
                {
                    MY_DEBUG("coarse nanopositioner  initial error!");
                }
            }
            if (mResult != CCoarseController.SA_OK)
            {
                MY_DEBUG("Nanopositioner initial error!");
                return 1;
            }

            CCoarseController.SA_SetAccumulateRelativePositions_S((uint)mSystemIndex, Z_CP_AXIS, CCoarseController.SA_NO_ACCUMULATE_RELATIVE_POSITIONS);
            Stop(X_CP_AXIS);
            Stop(Y_CP_AXIS);
            Stop(Z_CP_AXIS);

            mOpenLoopFrequency = 300;

            SetSpeedCloseLoop(X_CP_AXIS, 0);
            SetSpeedCloseLoop(Y_CP_AXIS, 0);
            SetSpeedCloseLoop(Z_CP_AXIS, 0);
            //SetSensorModeDisable();
            //SetSensorModeEnable();
            SetSensorModePowerSave();

            int position = 0;
            uint sensor_type = 1;    // linear positioners with nanosensor
            for (uint channelIndex = X_CP_AXIS; channelIndex < NUM_OF_AXIS; channelIndex++)
            {
                CCoarseController.SA_SetSensorType_S(mSystemIndex, channelIndex, sensor_type);   // Set Sensor Type
                CCoarseController.SA_SetClosedLoopMaxFrequency_S(mSystemIndex, channelIndex, 100);   // Set Frequency
                CCoarseController.SA_SetPosition_S(mSystemIndex, channelIndex, position);   // Set position to 0
            }
            SetChannelVoltage(X_CP_AXIS, 0);
            SetChannelVoltage(Y_CP_AXIS, 0);
            SetChannelVoltage(Z_CP_AXIS, 0);
            return 0;
        }

        void SetSpeedCloseLoop(uint channel, uint speed)
        {        
            //if (speed > 18500)
            //    speed = 18500;
            if (speed < 50)
                speed = 50;
            CCoarseController.SA_SetClosedLoopMoveSpeed_S(mSystemIndex, channel, speed);


            mResult = CCoarseController.SA_SetClosedLoopMaxFrequency_S(mSystemIndex, channel, speed*10);
            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("Setfrequency error!\n");
            }
        }

        void SetSpeedOpenLoop(uint frequency)
        {
            if (frequency < 1)
                frequency = 1;
            if (frequency > 18500)
                frequency = 18500;

            mOpenLoopFrequency = frequency;
        }

        public void MoveDistance_OpenLoop(uint channel, double distance)
        {
            MoveDistance_OpenLoop((uint)channel, distance, mOpenLoopFrequency);
        }

        double NM2STEPS(double x) { return ((x) / 1500.0 * 4095.0); }
        double STEPS2NM(double x) { return ((x) * 1500.0 / 4095.0); }

        public void MoveDistance_OpenLoop(uint channel, double distance, uint frequency)
        {
            distance *= mDirection[(uint)channel];

            if (moving == true)
            {
                MY_DEBUG("Coarse positioner busy.");
                return;
            }

            moving = true;

            string str = null;
            if (channel == 0) str = "x";
            if (channel == 1) str = "y";
            if (channel == 2) str = "z";
            MY_DEBUG("CoarsePositioiner:\t" + str + "\t" + distance.ToString() + "\t" + frequency.ToString());

            distance = NM2STEPS(distance);
            int number_of_steps = 0, step_left = (int)distance;
            int dir = Math.Sign(distance);
            distance = Math.Abs(distance);

            double STEP_MAX = (NM2STEPS(1000));
            double STEP_MIN = (NM2STEPS(50));
            if (distance <= STEP_MIN)
            {
                MoveFineSteps((uint)channel, 1 * dir, (uint)STEP_MIN, frequency);
            }

            if (distance < STEP_MAX && distance > STEP_MIN)
            {
                MoveFineSteps((uint)channel, 1 * dir, (uint)distance, frequency);
            }

            if (distance < STEP_MAX * 2 && distance > STEP_MAX)
            {
                MoveFineSteps((uint)channel, 2 * dir, (uint)(distance / 2), frequency);
            }

            if (distance > STEP_MAX * 2)
            {
                number_of_steps = (int)(distance / STEP_MAX - 1);
                step_left = (int)(distance - STEP_MAX * (double)(number_of_steps));
                step_left /= 2;

                MoveFineSteps((uint)channel, number_of_steps * dir, (uint)STEP_MAX, frequency);
                WaitForIdle((uint)channel);
                Thread.Sleep((int)(number_of_steps * 1000 / frequency + 500));
                //20160209
                //WaitForIdle can not block the controller.
                // here sleep time must be long enough, otherwise the previous command will be ignored.
                MoveFineSteps((uint)channel, 2 * dir, (uint)step_left, frequency);
            }
            SetChannelVoltage((uint)channel, 0);
            moving = false;
        }

        void MoveToPosition(double x, double y, double z, double t)
        {

        }

        void MoveToPosition(int channel, double position)
        {

        }

        double GetPosition(int axis)
        {
            return 0;
        }

        void SetSensorMode(uint mode)
        {
            mSensorMode = mode;
            mResult = CCoarseController.SA_SetSensorEnabled_S((uint)mSystemIndex, mode);
            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("SetSensorMode error!\n");
            }
        }

        public void SetSensorModeDisable()
        {
            SetSensorMode(CCoarseController.SA_SENSOR_DISABLED);
        }

        public void SetSensorModeEnable()
        {
            SetSensorMode(CCoarseController.SA_SENSOR_ENABLED);
        }

        public void SetSensorModePowerSave()
        {
            SetSensorMode(CCoarseController.SA_SENSOR_POWERSAVE);
        }

        void Stop(uint axis)
        {
            CCoarseController.SA_Stop_S((uint)mSystemIndex, axis);
        }

        //////////////////////////////////////

        void MoveToFinePosition(uint channel, uint position, uint speed)
        {
            if (position > 4095 || position < 0)
            {
                MY_DEBUG("MoveToFinePosition input exceeding error!\n");
            }
            if (position > 4095)
                position = 4095;
            if (position < 0)
                position = 0;

            CCoarseController.SA_ScanMoveAbsolute_S((uint)mSystemIndex, (uint)channel, position, speed);
        }

        void MoveFineDistance(int channel, int distance, uint speed)
        {
            //CCoarseController.SA_Stop_S((uint)mSystemIndex,(uint)channel);
            mResult = CCoarseController.SA_ScanMoveRelative_S((uint)mSystemIndex, (uint)channel, distance, speed);

            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("MoveFineDistance error!\n");
            }
        }

        void MoveFineSteps(uint channelIndex, int steps, uint amplitude, uint frequency)
        {
            // minimum step size=100, otherwise will mResult in error,
            // step=0-4095
            mResult = CCoarseController.SA_StepMove_S((uint)(uint)mSystemIndex, (uint)(uint)channelIndex, steps, amplitude, frequency);

            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("MoveFineSteps error!\n");
            }
        }

        void MoveWait(uint channelIndex, int stepsize)
        {
            // close loop inside the controller, close loop for this function
            CCoarseController.SA_GotoPositionRelative_S((uint)mSystemIndex, (uint)channelIndex, stepsize, 1000);
            WaitForIdle(mSystemIndex);
        }

        void WaitForIdle(uint channelIndex)
        {
            mResult = 0;
            do
            {
                //pin_ptr<uint *> pinnedPtr =  (uint * )&mStatus;
                uint pR = 0;
                mResult = CCoarseController.SA_GetStatus_S((uint)mSystemIndex, (uint)channelIndex, ref pR);		  // get mStatus
                if (mResult != CCoarseController.SA_OK & pR != CCoarseController.SA_OK)
                {
                    //Initialize();
                    MY_DEBUG("MoveWait GetStatus error!\n");
                }
                Thread.Sleep(3);
            }
            while (mStatus == CCoarseController.SA_TARGET_STATUS);	   // until target reach.	 
        }

        // void  MoveDistance(uint channelIndex, int  stepsize)
        // {	
        // 	// close loop inside the controller, open loop for this function
        // 	SetSpeedCloseLoop((uint)channelIndex,0);
        // 	 mResult = CCoarseController.SA_GotoPositionRelative_S((uint)mSystemIndex, (uint)channelIndex, stepsize, 1000);	
        // 	if (mResult != CCoarseController.SA_OK) 
        // 	{
        // 		//Initialize();
        // 		MY_DEBUG("MoveWait GetStatus error!\n");
        // 	}
        // }
        // 
        public void MoveDistance(uint channel, double distance, uint frequency)
        {
            distance *= mDirection[(uint)channel];
            if (mSensorMode == CCoarseController.SA_SENSOR_DISABLED)
                MoveDistance_OpenLoop(channel, distance, frequency);
            else
                MoveDistance_CloseLoop(channel, distance, frequency);
        }
        public void MoveDistance_CloseLoop(uint channel, double distance, uint frequency)//uint channelIndex, int stepsize, int speed)// close loop
        {
            CCoarseController.SA_GotoPositionRelative_S(mSystemIndex, channel, (int)distance, 1000);
            //SetSpeedCloseLoop((uint)channelIndex,abs(stepsize)*100);
            //SetSpeedCloseLoop((uint)channel, (uint)Math.Abs(frequency));
            //mResult = CCoarseController.SA_GotoPositionRelative_S((uint)mSystemIndex, (uint)channel, (int)distance, 1000);
            //if (mResult != CCoarseController.SA_OK)
            //{
            //    //Initialize();
            //    MY_DEBUG("MoveWait GetStatus error!\n");
            //}
            //SetSpeedCloseLoop((uint)channelIndex,0);
        }

        void SetPosition(uint channelIndex, int position)
        {
            mResult = CCoarseController.SA_SetPosition_S((uint)mSystemIndex, (uint)channelIndex, position);
            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("SetPosition error!\n");
            }
        }

        uint GetFinePosition(uint channelIndex)
        {
            uint level = 0;
            mResult = CCoarseController.SA_GetVoltageLevel_S((uint)mSystemIndex, (uint)channelIndex, ref level);
            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("GetFinePosition error!\n");
            }
            return level;
        }

        int GetPosition(uint channelIndex)
        {
            int position = 0;
            mResult = CCoarseController.SA_GetPosition_S((uint)mSystemIndex, (uint)channelIndex, ref position);
            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("GetPosition error!\n");
            }
            return position;
        }

        // 
        // int  ResetFinePosition(void)
        // {
        // 	//CCoarseController.SA_StepMove_S(pM->(uint)mSystemIndex, Z_CP_AXIS,1,1,50);       // use this command to reset the positioner voltage to 2047
        // 	double temp_stepV = 100;
        // 	double temp_stepNM = temp_stepV*2*1535.0/4096;
        // 	int p=0,pnm1=0,pnm2=0;
        // 	int k=0;
        // 	for (k=0;k<10;k++)
        // 	{
        // 		pnm1= GetPosition(Z_CP_AXIS);
        // 		MoveWait(Z_CP_AXIS, temp_stepNM);
        // 		//MoveFineDistance(Z_CP_AXIS, -temp_stepV, Math.Abs(temp_stepV)*1000);
        // 		Thread.Sleep(20, 1);
        // 		MoveFineSteps(Z_CP_AXIS, -2, temp_stepV, temp_stepV*10); // here we should at least move 2 steps
        // 		Thread.Sleep(20, 1);
        // 		p= GetFinePosition(Z_CP_AXIS);
        // 		pnm2= GetPosition(Z_CP_AXIS);
        // 		if (abs(pnm1-pnm2)>20)
        // 		{
        // 			MoveWait(Z_CP_AXIS, pnm1-pnm2);
        // 			continue;
        // 		}
        // 
        // 		if (p==2047)
        // 			break;
        // 	} 
        // 	if (p!=2047)
        // 	{
        // 		sprintf(inf, "times: %d\t servo initial value: %d\n",k,p);
        // 		MY_DEBUG(inf);
        // 	}
        // 	return p;
        // }
        //
        int redo_count = 0;
        public void SetChannelVoltage(uint channelIndex, uint V_0_150)
        {
            // minimum step size=100, otherwise will mResult in error,
            // step=0-4095  SA_StepMove_S
            //mResult = CCoarseController.SA_GotoGripperOpeningRelative_S((uint)(uint)mSystemIndex, (uint)(uint)channelIndex, V_0_4095, 1);
            if (V_0_150 > 150) V_0_150 = 150;
            V_0_150 *= (uint)(4095.0 / 150.0);

            mResult = CCoarseController.SA_ScanMoveAbsolute_S((uint)mSystemIndex, (uint)(uint)channelIndex, V_0_150, 100000);

            if (mResult != CCoarseController.SA_OK)
            {
                //Initialize();
                MY_DEBUG("SetChannelVoltage error!\n");
                Initialize();

                MY_DEBUG("redo initialize=" + redo_count.ToString());
                if (redo_count++ < 10)
                {
                    SetChannelVoltage(channelIndex, V_0_150);
                }
            }
            redo_count = 0;
        }
    }
}