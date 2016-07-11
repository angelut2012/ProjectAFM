using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Diagnostics;
using System.Runtime.InteropServices;
using TDx.TDxInput;
// start other program

namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form
    {
        //double Threshold_Joystick_MoveSensitivity = 20;     //150;
        //double Threshold_Joystick_RotateSensitivity = 0.5;     //150;

        ////////////////////////////////////////////////////

        private delegate void SetDeviceTextCallback();
        private delegate void SetMotionTextCallback();
        private delegate void SetKeyTextCallback(double keyCode);

        private TDx.TDxInput.Sensor JS_sensor;
        private TDx.TDxInput.Keyboard JS_keyboard;
        private TDx.TDxInput.Device JS_device;
        private System.DateTime JS_lastUpdate;

        private double JS_manipulator_number;    // 0, left  manupulator; 1 right manipulator; -1 prior stage
        private double JS_axis;
        private double JS_direction;
        private double JS_stepSize;

        double[,] JS_axis_position = new double[3, 4];     // store the position for all the axes

        uint JSmResult;
        uint mSystemIndex = 0;
        uint channel_x = 0;
        uint channel_y = 1;
        uint channel_z = 2;
        Thread mThread_GUI_UpdateCoarsePositionerEncoder;
        bool mLock_GUI_UpdateCoarsePositionerEncoder = false;
        //private void button_test_geji_Click(object sender, EventArgs e)
        //{
        //    JoyStick_Initialize();
        //}

        public bool JoyStick_Initialize()
        {
            Process[] processlist = Process.GetProcessesByName("3dxsrv");
            if (processlist.Length < 1)
            {
                //string p_string = @"C:\Program Files\3Dconnexion\3DxWare\3DxWinCore32\3DxService.exe";
                string p_string = @" C:\Program Files\3Dconnexion\3Dconnexion 3DxSoftware\3DxWare64\3dxsrv.exe";
                ProcessStartInfo sinfo = new ProcessStartInfo(p_string, null);
                sinfo.RedirectStandardOutput = true;
                sinfo.UseShellExecute = false;
                sinfo.CreateNoWindow = true;
                Process p = new Process();
                p.StartInfo = sinfo;
                p.Start();
                //String  output = p.StandardOutput.ReadToEnd();
                //Console.Write(output);
            }

            //JS_stepSize = 1000; // Convert.ToDouble(textBox_StepLength.Text);

            //JS_manipulator_number = 2;

            try
            {
                JS_device = new TDx.TDxInput.Device();
                JS_sensor = JS_device.Sensor;
                JS_keyboard = JS_device.Keyboard;

                JS_lastUpdate = System.DateTime.Now;

                // Add the event handlers
                JS_device.DeviceChange += new TDx.TDxInput._ISimpleDeviceEvents_DeviceChangeEventHandler(JoyStick_device_DeviceChange);
                JS_sensor.SensorInput += new TDx.TDxInput._ISensorEvents_SensorInputEventHandler(JoyStick_sensor_SensorInput);
                JS_keyboard.KeyDown += new TDx.TDxInput._IKeyboardEvents_KeyDownEventHandler(JoyStick_keyboard_KeyDown);
                JS_keyboard.KeyUp += new TDx.TDxInput._IKeyboardEvents_KeyUpEventHandler(JoyStick_keyboard_KeyUp);

                // Associate a configuration with this JS_device'
                //JS_device.LoadPreferences("csMonitor");

                //Connect everything up
                JS_device.Connect();

                //device_DeviceChange(0);
                JSmResult = CCoarseController.SA_SetClosedLoopMoveSpeed_S(mSystemIndex, channel_z, 5000000);    // set speed 
                JSmResult = CCoarseController.SA_SetClosedLoopMaxFrequency_S(mSystemIndex, channel_z, 12000);    // set frequency

                return true;
            }
            catch (COMException e)
            {
                Console.WriteLine("{0} Caught exception #1.", e);
                return false;
            }
        }

        void JoyStick_device_DeviceChange(int reserved)
        {
            int key;
            string strKeyLabel;
            for (key = 1; key <= JS_keyboard.Keys; key++)
            {
                strKeyLabel = JS_keyboard.GetKeyLabel(key);
                Console.WriteLine("KeyLabel is {0}.", strKeyLabel);
            }
        }

        void JoyStick_sensor_SensorInput()
        {
            /*  if (false)
              {
                 // Adjust values to account for gui update rate
                 // as this is a velocity JS_device
                 System.DateTime now = System.DateTime.Now;
                 System.TimeSpan deltaUpdate = now - JS_lastUpdate;
                 JS_lastUpdate = now;

                 timeFactor = (double)deltaUpdate.Milliseconds / JS_sensor.Period;
                 if (!isRunning)
                    timeFactor = 1;
              }
              else
              {
                 // In a value monitor the update rate is irrelevant
                 timeFactor = 1;
              }
            */

            TDx.TDxInput.Vector3D translation;
            translation = JS_sensor.Translation;
            // translation.Length = translation.Length * timeFactor;

            TDx.TDxInput.AngleAxis rotation;
            rotation = JS_sensor.Rotation;
            // rotation.Angle = rotation.Angle * timeFactor;

            double[] value_movement = new double[3];
            double[] value_rotation = new double[3];

            value_movement[0] = translation.X;
            value_movement[1] = translation.Y;
            value_movement[2] = translation.Z;
            value_rotation[0] = rotation.X;
            value_rotation[1] = rotation.Y;
            value_rotation[2] = rotation.Z;

            // for (double k = 0; k < 3; k++)         // some processings ...........
            // {
            //    if (Math.Abs(value_movement[k]) < Threshold_Joystick_MoveSensitivity)
            //        value_movement[k] = 0;
            //    else
            //        value_movement[k] /= Threshold_Joystick_MoveSensitivity;

            //    if (Math.Abs(value_rotation[k]) < Threshold_Joystick_RotateSensitivity)
            //        value_rotation[k] = 0;
            //    //else
            //    //  value_rotation[k] *= Threshold_Joystick_MoveSensitivity;
            //}

            //Console.WriteLine("{0},{1},{2}", value_rotation[0], value_rotation[1], value_rotation[2]);


            // here to use functions to realize Zheng's function.........................
            double outStepX;
            double outStepY;
            double outStepZ;

            //value_movement[0] = 0.0;
            //value_movement[1] = -2167.0;
            //value_movement[2] = 1315.0;

            //value_rotation[0] = 1;
            //value_rotation[1] = 0;
            //value_rotation[2] = 0;


            JoyStick_Smooth_Move_GongZheng(
                value_movement[0], value_movement[1], value_movement[2],
                value_rotation[0], value_rotation[1], value_rotation[2],
                5000, 4,
                out outStepX, out outStepY, out outStepZ
                );

            Console.WriteLine("joystick:{0}\t{1}\t{2}\t  out:{3}\t{4}\t{5}", value_movement[0], value_movement[1], value_movement[2], outStepX, outStepY, outStepZ);
            MY_DEBUG(outStepZ.ToString());

            //  JSmResult = CCoarseController.SA_GotoPositionRelative_S(mSystemIndex, channel, (int)outStepZ * 1000, 1000);

            //test move at speed
            //mManipulator[1].MoveAtSpeed(0, value_movement[0]*1000);
            //return;

            // xyz,movement
            //double move = 0;
            //double max_ind = 0;

            //for (double k = 0; k < 3; k++)
            //    if (move < Math.Abs(value_movement[k]))
            //    {
            //        move = Math.Abs(value_movement[k]);
            //        max_ind = k;
            //    }
            //move = value_movement[max_ind];

            //JS_stepSize = 20;    //Convert.ToDouble(textBox_StepLength.Text);

            //if (JS_manipulator_number <= 2)
            //{
            //    double[,] axis_ind = new double[3, 3] { { 0, 2, 1 }, { 0, 2, 1 }, { 0, 2, 1 } };
            //    double[,] dir_ind = new double[3, 3] // the direction of each axis
            //    { { 1, 1, -1 },// left manipulator
            //    { -1, 1, -1 }, // right manipulator
            //    { 1, 1, 1 } }; // stage
            //    // move xyz
            //    JS_axis = axis_ind[JS_manipulator_number, max_ind];

            //if (JS_manipulator_number < 2)
            //{
            //    if (Math.Abs(move) > 0)
            //    {
            //        JS_direction = Math.Sign(move);
            //        JS_direction *= dir_ind[JS_manipulator_number, JS_axis];
            //        //ManipulatorMoveDistance(JS_manipulator_number, JS_axis, JS_direction, JS_stepSize * Math.Abs(move));
            //    }
            //    // T
            //    //ManipulatorMoveDistance(JS_manipulator_number, 3, 1, JS_stepSize * value_rotation[2]);
            //}
            //if (JS_manipulator_number == 2)
            //{
            //    //mPrior.xyStage.MoveAtVelocity(value_movement[0] * JS_stepSize, value_movement[2] * JS_stepSize);
            //    //mPrior.zFocus.MoveAtVelocity(value_movement[1] * JS_stepSize);
            //    //JS_axis_position[2, 0] = mPrior.xyStage.XPosition;
            //    //JS_axis_position[2, 1] = mPrior.xyStage.YPosition;
            //    //JS_axis_position[2, 2] = mPrior.zFocus.Position;
            //    //UIUpdatePosition();
            //}
        }
        //if (JS_manipulator_number == 3)
        //{
        //    double syringe_speed = (translation.X / 2 + translation.Y / 2) * 100 * 100;
        //        //Convert.ToDouble(textBox_StepLength.Text) *
        //        //Convert.ToDouble(textBox_Speed1.Text);
        //    if (syringe_speed > 5)
        //        syringe_speed = 5;
        //    if (syringe_speed < -5)
        //        syringe_speed = -5;
        //    mSyringe.MoveAtSpeed(syringe_speed);
        //    Console.WriteLine(syringe_speed);
        //}
        //}


        void JoyStick_keyboard_KeyDown(int key)
        {
            Debug.WriteLine("key down:" + Convert.ToString(key));
            JS_stepSize = 100;        // Convert.ToDouble(textBox_StepLength.Text);
            // syringe
            //if (key == 3)// key_1, inject
            //    SyringeInjectD();
            ////     SyringeInjectCompensate();
            //if (key == 4)// key_2, suck           
            //    SyringeSuckD();
            //     SyringeSuckCompensate();
            //if (key == 5)// key_panel, inject
            //    //              SyringeInjectD();
            //    //SyringeInjectD();
            //if (key == 6)// key_fit, suck           
            //        //SyringeSuckD();
            //        //SyringeSuckD();

            //if (key == 20)// key_t, ahead
            //    ManipulatorMoveDistance(JS_manipulator_number, 3, 1, JS_stepSize);
            //if (key == 21)// key_f, back            
            //    ManipulatorMoveDistance(JS_manipulator_number, 3, -1, JS_stepSize);
            //// stop all
            //if (key == 29)// key_ esc, stop all
            //{
            //    mManipulator[0].StopAll();
            //    mManipulator[1].StopAll();
            //    mPrior.m_scan.KillAllMotion();
            //}
        }

        void JoyStick_keyboard_KeyUp(int key)
        {
            JS_stepSize = 100; // Convert.ToDouble(textBox_StepLength.Text);
            if (key == 11)//+, *2
                JS_stepSize *= 2;
            if (key == 12)// -, /2
                JS_stepSize /= 2;

            if (JS_stepSize >= 4096)
                JS_stepSize = 4096;
            if (JS_stepSize <= 1.0 / 128.0)
                JS_stepSize = 1.0 / 128.0;

            //textBox_StepLength.Text = JS_stepSize.ToString();

            // JS_manipulator_number
            if (key == 1)
                JS_manipulator_number = 0;    //left
            if (key == 2)
                JS_manipulator_number = 1;   // right
            if (key == 10)
                JS_manipulator_number = 2;   // stage
            if (key == 5)     // key_panel,
                JS_manipulator_number = 3;    //syringe
        }


        void JoyStick_Smooth_Move_GongZheng(double trans_x, double trans_y, double trans_z,
            double rota_x, double rota_y, double rota_z,
            double inSEMMag_base, double inSEMMag_expo,
            out double outStepX, out double outStepY, out double outStepZ
            )
        {
            outStepX = 0;
            outStepY = 0;
            outStepZ = 0;
            // Parameters ...........
            m_bCFVT = 1;
            m_bConnexion = true;
            m_bZDOnly = true;

            if (1 == m_bCFVT)
            {
                MainWindow pWnd = this;   //CMainFrame *pWnd=(CMainFrame*)AfxGetMainWnd();
                //if (pWnd.theSmarAct.bSystemInitialized)
                //{
                //if(pWnd.m_bSEMConnected)
                //{
                if (m_bConnexion)
                {
                    double rz = rota_z;       // valueRZ;
                    double rx = rota_x;       // valueRX;
                    double ty = trans_y;      // valueTY;
                    if (m_bZDOnly)
                    {
                        if (Math.Abs(ty) > MOUSEVALMINCOARSE)
                        {
                            //CMainFrame *pWnd= (CMainFrame*)AfxGetMainWnd();
                            pWnd.dSEMMag_base = inSEMMag_base;
                            pWnd.dSEMMag_expo = inSEMMag_expo;

                            double curMag = pWnd.dSEMMag_base * Math.Pow(10, pWnd.dSEMMag_expo - 2);
                            //curMag = 20;
                            //curMag = pWnd.m_wndView.theMiddleWindow.m_iMagInput;

                            double stepMin, stepMax, step, amp, freMin, freMax, fre;
                            double stepSlp, freSlp;
                            if (curMag >= CRMAGVT2)
                            {
                                amp = 800;
                                freMin = CRMAGCOVT / 8 + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                            }
                            else if (curMag >= CRMAGVT1)
                            {
                                amp = 800;
                                freMin = CRMAGCOVT / (-0.0008 * curMag + 48) + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                                //double sp = (7.0-2.0) / (CRMAGVT2 - CRMAGVT1);
                                //freMax = curMag * sp + 2.0 - CRMAGVT1 * sp; 
                            }
                            else if (curMag >= CRMAGVT0)
                            {
                                amp = 11231 * Math.Pow(curMag, -0.284) + 0.5;
                                //freMin = CRMAGCOVT / (209.43*Math.Pow(curMag,-0.188) + 0.5);
                                freMin = CRMAGCOVT / 40 + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                            }
                            else
                            {
                                amp = 1960;
                                //fre = 4.8305*curMag+205.25;
                                freMin = CRMAGCOVT / (1350.6 * Math.Exp(-0.006 * curMag)) + 0.5;
                                freMax = CRMAGCOVT / (7675.6 * Math.Pow(curMag, -1.102)) + 0.5;
                                stepMin = 1;
                                stepMax = 1176.1 * Math.Pow(curMag, -1.168) + 0.5;
                            }

                            freSlp = -1.0f * (freMax - freMin) / (MOUSEVALMAX - MOUSEVALMINCOARSE);
                            //lstep = unsigned double(freSlp*disRXZ + freMin - freSlp*MOUSEVALMINCOARSE + 0.5);
                            //lstep = Math.Min(freMax, lstep);
                            //lstep = Math.Max(freMin, lstep);
                            fre = freSlp * Math.Abs(ty) + freMax - freSlp * MOUSEVALMINCOARSE + 0.5;
                            fre = Math.Max(freMin, fre);
                            fre = Math.Min(freMax, fre);

                            stepSlp = 1.0f * (stepMax - stepMin) / (MOUSEVALMAX - MOUSEVALMINCOARSE);
                            step = stepSlp * Math.Abs(ty) + stepMin - stepSlp * MOUSEVALMINCOARSE + 0.5;
                            step = Math.Max(stepMin, step);
                            outStepZ = Math.Min(stepMax, step);

                            //if(SETVT & 1 << ManiSelcVT)
                            //{
                            //    leds[0] = 9;
                            //    leds[1] = fre / 256;
                            //    leds[2] = fre % 256;
                            //    leds[3] = amp / 256;
                            //    leds[4] = amp % 256;
                            //    leds[5] = step;
                            //    leds[6] = ty>0?orientCR[ManiSelcVT][2]:3-orientCR[ManiSelcVT][2]; 
                            //    leds[7] = noCR[ManiSelcVT][2];
                            //    leds[14] = ManiSelcVT + 1;
                            //    DYNCALL(writeData)(leds);
                            //}
                        }
                    }//  if (m_bZDOnly)
                    //else
                    {
                        if (!m_bXDOnly)
                            rz = 0;
                        if (!m_bYDOnly)
                            rx = 0;
                        if (rx * rx + rz * rz > MOUSEVALMINCOARSE * MOUSEVALMINCOARSE)
                        {
                            double disRXZ = Math.Sqrt(rz * rz + rx * rx);
                            //CMainFrame *pWnd=(CMainFrame*)AfxGetMainWnd();
                            double curMag = pWnd.dSEMMag_base * Math.Pow(10, pWnd.dSEMMag_expo - 2);
                            //curMag = 20;
                            //curMag = pWnd.m_wndView.theMiddleWindow.m_iMagInput;
                            //#ifdef COARSECALI
                            //                            CString cst_step;
                            //                            CString cst_fre_min;
                            //                            CString cst_fre_max;
                            //                            CString cst_amp;
                            //                            pWnd.tDiagAC.m_cEditStep.GetWindowTextA(cst_step);
                            //                            pWnd.tDiagAC.m_cEditFreMax.GetWindowTextA(cst_fre_max);
                            //                            pWnd.tDiagAC.m_cEditFreMin.GetWindowTextA(cst_fre_min);
                            //                            pWnd.tDiagAC.m_cEditAmp.GetWindowTextA(cst_amp);
                            //                            double step = atoi(cst_step);
                            //                            double amp = atoi(cst_amp);
                            //                            double freMax = atoi(cst_fre_max);
                            //                            double freMin = atoi(cst_fre_min);
                            //                            freMax = 500000.0 / freMax;
                            //                            freMin = 500000.0 / freMin;
                            //                            double fre, freX, freY, stepX, stepY;
                            //                            fre = freMax;
                            //                            double freSlp = 0.0f;
                            //                            double stepSlp = 0.0f;
                            //#else
                            double stepMin, stepMax, step, stepX, stepY, amp, freMin, freMax, fre, freX, freY;
                            double stepSlp, freSlp;
                            if (curMag >= CRMAGVT2)
                            {
                                amp = 800;
                                freMin = CRMAGCOVT / 8 + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                            }
                            else if (curMag >= CRMAGVT1)
                            {
                                amp = 800;
                                freMin = CRMAGCOVT / (-0.0008 * curMag + 48) + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                                //double sp = (7.0-2.0) / (CRMAGVT2 - CRMAGVT1);
                                //freMax = curMag * sp + 2.0 - CRMAGVT1 * sp; 
                            }
                            else if (curMag >= CRMAGVT0)
                            {
                                amp = 11231 * Math.Pow(curMag, -0.284) + 0.5;
                                //freMin = CRMAGCOVT / (209.43*Math.Pow(curMag,-0.188) + 0.5);
                                freMin = CRMAGCOVT / 40 + 0.5;
                                freMax = CRMAGCOVT / 8 + 0.5;
                                stepMin = 1;
                                stepMax = 1;
                            }
                            else
                            {
                                amp = 1960;
                                //fre = 4.8305*curMag+205.25;
                                freMin = CRMAGCOVT / (1350.6 * Math.Exp(-0.006 * curMag)) + 0.5;
                                freMax = CRMAGCOVT / (7675.6 * Math.Pow(curMag, -1.102)) + 0.5;
                                stepMin = 1;
                                stepMax = 1176.1 * Math.Pow(curMag, -1.168) + 0.5;
                            }

                            //amp = Math.Min(AMPMAX, amp);
                            //amp = Math.Max(AMPMIN, amp);
                            //freMax = Math.Min(FREMAX, freMax);
                            //freMin = Math.Max(FREMIN, freMin);
                            //if(freMax < freMin)
                            //	swap(freMax, freMin);

                            freSlp = -1.0f * (freMax - freMin) / (MOUSEVALMAX - MOUSEVALMINCOARSE);
                            //lstep = unsigned double(freSlp*disRXZ + freMin - freSlp*MOUSEVALMINCOARSE + 0.5);
                            //lstep = Math.Min(freMax, lstep);
                            //lstep = Math.Max(freMin, lstep);
                            fre = freSlp * disRXZ + freMax - freSlp * MOUSEVALMINCOARSE + 0.5;
                            fre = Math.Max(freMin, fre);
                            fre = Math.Min(freMax, fre);

                            stepSlp = 1.0f * (stepMax - stepMin) / (MOUSEVALMAX - MOUSEVALMINCOARSE);
                            step = stepSlp * disRXZ + stepMin - stepSlp * MOUSEVALMINCOARSE + 0.5;
                            step = Math.Max(stepMin, step);
                            step = Math.Min(stepMax, step);
                            //#endif

                            if (disRXZ > MOUSEVALMINCOARSE)
                            {
                                bool CRXMov = true;
                                bool CRYMov = true;
                                //#ifdef COARSECALI
                                //                                freY = 0;
                                //                                freX = fre;
                                //                                stepY = 0;
                                //                                stepX = step;
                                //#else
                                //double rx_cr = Math.Min(double(Math.Abs(rx)), MOUSEVALMAX);
                                //double rz_cr = Math.Min(double(Math.Abs(rz)), MOUSEVALMAX);
                                double rx_cr = Math.Abs(rx);
                                double rz_cr = Math.Abs(rz);
                                if (rx_cr > rz_cr)
                                {
                                    if (tickCRX >= tickCRY)
                                    {
                                        tickCRY = rx_cr - (rz_cr - tickCRY);
                                        tickCRX = 0;
                                    }
                                    else
                                    {
                                        CRXMov = false;
                                        tickCRY = rx_cr;
                                        tickCRX += rz_cr;
                                    }
                                }
                                else
                                {
                                    if (tickCRY >= tickCRX)
                                    {
                                        tickCRX = rz_cr - (rx_cr - tickCRX);
                                        tickCRY = 0;
                                    }
                                    else
                                    {
                                        CRYMov = false;
                                        tickCRX = rz_cr;
                                        tickCRY += rx_cr;
                                    }
                                }
                                //freY = fre;
                                //freX = fre;
                                //stepY = step;
                                //stepX = step;
                                //freY = Math.Min(freMax, double(1.0 * fre * Math.Abs(rx) / disRXZ + 0.5));
                                //freX = Math.Min(freMax, double(1.0 * fre * Math.Abs(rz) / disRXZ + 0.5));
                                //stepY = Math.Max(stepMin, double(1.0 * step * Math.Abs(rx) / disRXZ + 0.5));
                                //stepX = Math.Max(stepMin, double(1.0 * step * Math.Abs(rz) / disRXZ + 0.5));
                                if (curMag < CRMAGVT0)
                                {
                                    freY = freMax - 1.0 * Math.Pow(Math.Abs(rx) / disRXZ, 0.25 * (freMax - fre));
                                    freX = freMax - 1.0 * Math.Pow(Math.Abs(rz) / disRXZ, 0.25 * (freMax - fre));
                                }
                                else
                                {
                                    freY = freMax - 1.0 * Math.Pow(Math.Abs(rx) / disRXZ, 0.1 * (freMax - fre));
                                    freX = freMax - 1.0 * Math.Pow(Math.Abs(rz) / disRXZ, 0.1 * (freMax - fre));
                                    //freY = freMax - 1.0 * double(Math.Abs(rx) / disRXZ) * (freMax - fre);
                                    //freX = freMax - 1.0 * double(Math.Abs(rz) / disRXZ) * (freMax - fre);
                                }
                                //stepY = stepMin + 1.0 * Math.Abs(rx) / disRXZ * (step - stepMin);
                                //stepX = stepMin + 1.0 * Math.Abs(rz) / disRXZ * (step - stepMin);

                                stepY = stepMin + 1.0 * Math.Abs(rx) / disRXZ * (step - stepMin);
                                stepX = stepMin + 1.0 * Math.Abs(rz) / disRXZ * (step - stepMin);
                                stepY = Math.Max(1, stepY);
                                stepX = Math.Max(1, stepX);


                                outStepX = stepX;
                                outStepY = stepY;
                                //outStepZ = stepZ;
                                return;
                                //stepY = step;
                                //stepX = step;
                                //freY *= 1.2;
                                //freX *= 1.2;
                                //#endif

                                //if (SETVT && 1 << ManiSelcVT)
                                //{
                                //if(pWnd.tDiagAC)
                                //{
                                //    //pWnd.tDiagAC.m_cLB_C_str.Format("x: %d_ %d %f %d %d", pWnd.tDiagAC.m_cLB_C_str_index++, fre, freSlp, freX, stepX);
                                //    //pWnd.tDiagAC.m_cLB_C.AddString(pWnd.tDiagAC.m_cLB_C_str);
                                //    //pWnd.tDiagAC.m_cLB_C.SetCurSel(pWnd.tDiagAC.m_cLB_C.GetCount()-1);
                                //    //pWnd.tDiagAC.m_cLB_C_str.Format("y: %d_ %d %d %d %d", pWnd.tDiagAC.m_cLB_C_str_index++, freMin, freMax, freY, stepY);
                                //    //pWnd.tDiagAC.m_cLB_C.AddString(pWnd.tDiagAC.m_cLB_C_str);
                                //    //pWnd.tDiagAC.m_cLB_C.SetCurSel(pWnd.tDiagAC.m_cLB_C.GetCount()-1);
                                //    //pWnd.tDiagAC.m_cLB_C_str.Format("d: %d_ %d %d %f", pWnd.tDiagAC.m_cLB_C_str_index++, rx, rz, disRXZ);
                                //    //pWnd.tDiagAC.m_cLB_C.AddString(pWnd.tDiagAC.m_cLB_C_str);
                                //    //pWnd.tDiagAC.m_cLB_C.SetCurSel(pWnd.tDiagAC.m_cLB_C.GetCount()-1);
                                //    //pWnd.tDiagAC.m_cLB_C_str.Format("t: %d_ %d %d %d %d %d %d", pWnd.tDiagAC.m_cLB_C_str_index++, rz_cr, rx_cr, tickCRX, tickCRY, CRXMov, CRYMov);
                                //    //pWnd.tDiagAC.m_cLB_C.AddString(pWnd.tDiagAC.m_cLB_C_str);
                                //    //pWnd.tDiagAC.m_cLB_C.SetCurSel(pWnd.tDiagAC.m_cLB_C.GetCount()-1);
                                //}

                                //if (0 != rz && CRXMov)
                                //{
                                // need to move the SmarAct -----------  SA_GotoPositionRelative_S

                                //    leds[0] = 9;
                                //    //leds[1] = fre / 256;
                                //    //leds[2] = fre % 256;
                                //    //leds[3] = amp / 256;
                                //    //leds[4] = amp % 256;
                                //    //leds[5] = freX;
                                //    leds[1] = freX / 256;
                                //    leds[2] = freX % 256;
                                //    leds[3] = amp / 256;
                                //    leds[4] = amp % 256;
                                //    leds[5] = stepX;
                                //    leds[6] = rz>0?orientCR[ManiSelcVT][0]:3-orientCR[ManiSelcVT][0]; 
                                //    leds[7] = noCR[ManiSelcVT][0];
                                //    leds[14] = ManiSelcVT + 1;
                                //    DYNCALL(writeData)(leds);
                                //}

                                //if (0 != rx && CRYMov)




                                //{
                                // need to move the SmarAct -----------  SA_GotoPositionRelative_S

                                //    leds[0] = 9;
                                //    //leds[1] = fre / 256;
                                //    //leds[2] = fre % 256;
                                //    //leds[3] = amp / 256;
                                //    //leds[4] = amp % 256;
                                //    //leds[5] = freY;
                                //    leds[1] = freY / 256;
                                //    leds[2] = freY % 256;
                                //    leds[3] = amp / 256;
                                //    leds[4] = amp % 256;
                                //    leds[5] = stepY;
                                //    leds[6] = rx>0?orientCR[ManiSelcVT][1]:3-orientCR[ManiSelcVT][1];
                                //    leds[7] = noCR[ManiSelcVT][1];
                                //    leds[14] = ManiSelcVT + 1;
                                //    DYNCALL(writeData)(leds);

                                //}
                                //}
                            }
                        }
                    }
                }
                //}
                //else
                //	::MessageBox(NULL, "bug, error, sem not connected", "", MB_OK | MB_TOPMOST);
                //}
                //else
                //	;
                //::MessageBox(NULL, "SmarAct not initialized!", "", MB_OK | MB_TOPMOST);
            }
            else if (0 == m_bCFVT)
            {
                //if(0==valueRZ && 0==valueRX && 0==valueTY)
                //{
                //ReadoutCount++;
                //if (ReadoutCount == CXINTERVALFINEREADCOUNT)
                //{
                //    //leds[0] = 2;
                //    //DYNCALL(writeData)(leds);
                //    //ConvertDisplayPosVT();
                //    //setOpenglPos();
                //    if (posReferenceZBool)
                //        setRefZPosDisplay();
                //    ReadoutCount = 0;
                //}
                //}

                //CMainFrame *pWnd=(CMainFrame*)AfxGetMainWnd();
                //if (pWnd.theSmarAct.bSystemInitialized)
                //{
                //if(pWnd.m_bSEMConnected)
                //{
                if (m_bConnexion)
                {
                    double rz = rota_z;       //   valueRZ;
                    double rx = rota_x;      // valueRX;
                    double ty = trans_y;      // valueTY;

                    if (m_bZDOnly)
                    {
                        if (Math.Abs(ty) > MOUSEVALMINFINE)
                        {
                            double cTemp = 1.0 * (Math.Abs(ty) - MOUSEVALMINFINE) / (MOUSEVALMAX - MOUSEVALMINFINE);
                            if (cTemp < 0.0)
                                cTemp = 0.0;
                            if (cTemp > 1.0)
                                cTemp = 1.0;

                            cTemp = Math.Pow(cTemp, 3);

                            MainWindow pWnd = this;  //	CMainFrame *pWnd=(CMainFrame*)AfxGetMainWnd();
                            double curMag = pWnd.dSEMMag_base * Math.Pow(10, pWnd.dSEMMag_expo - 2);
                            //curMag = 5000.0;
                            //curMag = pWnd.m_wndView.theMiddleWindow.m_iMagInput;

                            curMag = DISPERPIX * 1000000.0 / (curMag * SEMIMGTYPEWIDTH);
                            double iTemp = 5 * cTemp * curMag;
                            if (iTemp > 0)
                            {
                                double[] dis_lkmag = new double[3];
                                dis_lkmag[0] = 0;
                                dis_lkmag[1] = 0;
                                dis_lkmag[2] = ty > 0 ? iTemp : -iTemp;

                                //if(pWnd.tDebugDiag)
                                //{
                                //	pWnd.tDebugDiag.m_cLB_C_str.Format("%d zJoy %d %d %d", pWnd.tDebugDiag.m_cLB_C_str_index++, ledsVT[ManiSelcVT][0]*256+ledsVT[ManiSelcVT][1], ledsVT[ManiSelcVT][2]*256+ledsVT[ManiSelcVT][3], ledsVT[ManiSelcVT][4]*256+ledsVT[ManiSelcVT][5]);
                                //	pWnd.tDebugDiag.m_cLB_C.AddString(pWnd.tDebugDiag.m_cLB_C_str);
                                //	pWnd.tDebugDiag.m_cLB_C.SetCurSel(pWnd.tDebugDiag.m_cLB_C.GetCount()-1);
                                //}

                                //if(ledsVTPre_ind < 0)
                                //	memcpy(ledsVTPre+LTMPOSCONTAINERSIZE+ledsVTPre_ind, ledsVT[ManiSelcVT], 6);
                                //else
                                //	memcpy(ledsVTPre+ledsVTPre_ind, ledsVT[ManiSelcVT], 6);
                                //ledsVTPre_ind += 6;
                                //if(LTMPOSCONTAINERSIZE == ledsVTPre_ind)
                                //	ledsVTPre_ind = 0;

                                //if (pWnd.m_bLinkMani)
                                //{
                                //    setPosAllVT(dis_lkmag, pWnd);
                                //    //for(double lm_ind=0;lm_ind<NUMMANIVT;lm_ind++)
                                //    //	setPosVT(lm_ind, dis_lkmag, pWnd);
                                //}
                                //else
                                //{
                                //    //setPosVT(ManiSelcVT, dis_lkmag, pWnd);
                                //}
                            }
                        }
                    }
                    //else
                    {
                        if (!m_bXDOnly)
                            rz = 0;
                        if (!m_bYDOnly)
                            rx = 0;
                        if (rx * rx + rz * rz > MOUSEVALMINFINE * MOUSEVALMINFINE)
                        {
                            double disRXZ = Math.Sqrt(rz * rz + rx * rx);
                            double cTemp = 1.0 * (disRXZ - MOUSEVALMINFINE) / (MOUSEVALMAX - MOUSEVALMINFINE);
                            if (cTemp < 0.0)
                                cTemp = 0.0;
                            if (cTemp > 1.0)
                                cTemp = 1.0;

                            cTemp = Math.Pow(cTemp, 3);

                            MainWindow pWnd = this; // CMainFrame *pWnd=(CMainFrame*)AfxGetMainWnd();
                            double curMag = pWnd.dSEMMag_base * Math.Pow(10, pWnd.dSEMMag_expo - 2);
                            //curMag = 5000.0;
                            //curMag = pWnd.m_wndView.theMiddleWindow.m_iMagInput;

                            curMag = DISPERPIX * 1000000.0 / (curMag * SEMIMGTYPEWIDTH);
                            double iTemp = 5 * cTemp * curMag;
                            if (iTemp > 0)
                            {
                                double itx = 1.0f * rx / disRXZ;
                                double itz = 1.0f * rz / disRXZ;

                                double[] dis_lkmag = new double[3];
                                dis_lkmag[0] = itz * iTemp;
                                dis_lkmag[1] = -itx * iTemp;
                                dis_lkmag[2] = 0;

                                //if(pWnd.tDebugDiag)
                                //{
                                //	pWnd.tDebugDiag.m_cLB_C_str.Format("%d xyJoy %d %d %d", pWnd.tDebugDiag.m_cLB_C_str_index++, ledsVT[ManiSelcVT][0]*256+ledsVT[ManiSelcVT][1], ledsVT[ManiSelcVT][2]*256+ledsVT[ManiSelcVT][3], ledsVT[ManiSelcVT][4]*256+ledsVT[ManiSelcVT][5]);
                                //	pWnd.tDebugDiag.m_cLB_C.AddString(pWnd.tDebugDiag.m_cLB_C_str);
                                //	pWnd.tDebugDiag.m_cLB_C.SetCurSel(pWnd.tDebugDiag.m_cLB_C.GetCount()-1);
                                //}

                                //if(ledsVTPre_ind < 0)
                                //	memcpy(ledsVTPre+LTMPOSCONTAINERSIZE+ledsVTPre_ind, ledsVT[ManiSelcVT], 6);
                                //else
                                //	memcpy(ledsVTPre+ledsVTPre_ind, ledsVT[ManiSelcVT], 6);
                                //ledsVTPre_ind += 6;
                                //if(LTMPOSCONTAINERSIZE == ledsVTPre_ind)
                                //	ledsVTPre_ind = 0;

                                //if (pWnd.m_bLinkMani)
                                //{
                                //    setPosAllVT(dis_lkmag, pWnd);
                                //    //for(double lm_ind=0;lm_ind<NUMMANIVT;lm_ind++)
                                //    //	setPosVT(lm_ind, dis_lkmag, pWnd);
                                //}
                                //else
                                //    setPosVT(ManiSelcVT, dis_lkmag, pWnd);
                            }
                        }
                    }
                }
            }

        }   // JoyStick_Smooth_Move_GongZheng
        //----------------- update UI coarse position--------------------------------------------------------------------------
        int mCoarsePositionX = 0;
        int mCoarsePositionY = 0;
        int mCoarsePositionZ = 0;

        void timer_Update_CoarseReadout_Tick(object sender, EventArgs e)
        {
            mThread_GUI_UpdateCoarsePositionerEncoder = new Thread(ThreadFunction_GUI_UpdateCoarsePositionerEncoder);
            mThread_GUI_UpdateCoarsePositionerEncoder.Start();
            label_Encoder_X.Text = Convert.ToString(mCoarsePositionX);
            label_Encoder_Y.Text = Convert.ToString(mCoarsePositionY); 
            label_Encoder_Z.Text = Convert.ToString(mCoarsePositionZ);
        }
        void ThreadFunction_GUI_UpdateCoarsePositionerEncoder() // update the SmarAct Coarse encoder values
        {      
            if (mLock_GUI_UpdateCoarsePositionerEncoder == true) return;

            mLock_GUI_UpdateCoarsePositionerEncoder = true;

            JSmResult = CCoarseController.SA_GetPosition_S(mSystemIndex, channel_x, ref mCoarsePositionX);        // X axis
            //label_Encoder_X.Text = Convert.ToString(mCoarsePositionX);

            JSmResult = CCoarseController.SA_GetPosition_S(mSystemIndex, channel_y, ref mCoarsePositionY);        // Y axis
            //label_Encoder_Y.Text = Convert.ToString(mCoarsePositionY);

            JSmResult = CCoarseController.SA_GetPosition_S(mSystemIndex, channel_z, ref mCoarsePositionZ);        // Z axis
            //label_Encoder_Z.Text = Convert.ToString(mCoarsePositionZ);

            mLock_GUI_UpdateCoarsePositionerEncoder = false;
        }


    }
}