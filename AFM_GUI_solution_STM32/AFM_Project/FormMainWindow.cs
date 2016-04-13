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
using System.Collections.Generic;
//using CLRWrapper;

namespace NameSpace_AFM_Project
{

    public partial class MainWindow : Form
    {
        public string mDataPath = "d:\\AFMdata\\";

        public MKernel.KernelClass mKernelClass;
        // coarse positioner
        public CCoarsePositioner mCCoarsePositioner;
        public uint mCaxis_x = 0;
        public uint mCaxis_y = 1;
        public uint mCaxis_z = 2;
        public uint[] mCaxis = { 0, 1, 2 };

        //-------
        public void DOS_Command(string cmdstr)
        { System.Diagnostics.Process.Start("cmd.exe", cmdstr); }
        public void SpeakVoice(string words)
        {
            //C:\Program Files (x86)\Reference Assemblies\Microsoft\Framework\.NETFramework\v4.0
            System.Speech.Synthesis.SpeechSynthesizer mVoiceSpeaker = new System.Speech.Synthesis.SpeechSynthesizer();
            mVoiceSpeaker.SpeakAsync(words);
            //mVoiceSpeaker.Speak(words);
        }

        public void PlaySimpleSound(string file_name)
        {//@"c:\Windows\Media\chimes.wav"
            System.Media.SoundPlayer simpleSound = new System.Media.SoundPlayer(file_name);
            simpleSound.Play();
        }
        public void SoundNotice(int k, int sleep_ms)
        { SoundNotice(k); Thread.Sleep(sleep_ms); }
        public void SoundNotice(int k)
        {
            if (k == 0) System.Media.SystemSounds.Exclamation.Play();// ok
            if (k == 1) System.Media.SystemSounds.Hand.Play();// fail
            if (k == 2) System.Media.SystemSounds.Beep.Play();
            if (k == 3) System.Media.SystemSounds.Asterisk.Play();
            if (k == 4) System.Media.SystemSounds.Question.Play();
            if (k == 5) PlaySimpleSound(@"camera.wav");
            if (k == 6) PlaySimpleSound(@"chimes.wav");

        }


        static T Swap<T>(ref T lhs, ref T rhs) where T : System.IComparable<T>
        {
            T temp;
            temp = lhs;
            lhs = rhs;
            rhs = temp;
            return temp;
        }

        public uint MIN_MAX(uint v, uint down, uint up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public int MIN_MAX(int v, int down, int up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public double MIN_MAX(double v, double down, double up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public void MY_DEBUG(string inf)
        {
            if (string.IsNullOrEmpty(inf) == false)
                System.Diagnostics.Debug.WriteLine(inf);
            //string str = "The quick brown fox jumped over the gentleman.";
            //{
            //    byte[] bytes = Encoding.ASCII.GetBytes(inf);
            //    unsafe
            //    {
            //        fixed (byte* p = bytes)
            //        {
            //            sbyte* sp = (sbyte*)p;
            //            //SP is now what you want
            //            mCCoarsePositioner.MY_DEBUG_CLR(sp);
            //        }
            //    }

            //}
        }
        public void MY_DEBUG(string inf, int x) { MY_DEBUG(inf + Convert.ToString(x)); }
        public void MY_DEBUG(int x) { MY_DEBUG(Convert.ToString(x)); }

        public string GetCurrentTimeString() { return DateTime.Now.ToString("yyyyMMddHHmmss"); }
        public long GetCurrentTimeLong() { return DateTime.Now.ToFileTimeUtc(); }
        public long TIC() { return TIC_TOC(0); }
        public long TOC() { return TIC_TOC(1); }// 1e-7 S
        private long TIC_TOC_t;
        private long TIC_TOC(int s)
        {
            long t = 0;
            if (s == 0)
                t = TIC_TOC_t = GetCurrentTimeLong();// tic
            else
                t = GetCurrentTimeLong() - TIC_TOC_t;
            return t;
        }

        // variable
        //  public Manipulator //mManipulator=new Manipulator();
        Thread mThread_ResteHome;

        public delegate void DelegateFunction();
        public DelegateFunction mDelegateFunction;


        const int LENGTH_COM_BUFFER_PC2MCU = 10;
        const int LENGTH_COM_BUFFER_MCU2PC = 16;

        const int COM_HEADER1 = (0xAA);//170
        const int COM_HEADER2 = (0x55);//85
        const int COM_TAIL1 = (0x55);
        const int COM_TAIL2 = (0xAA);

        const double BIT18MAX = (262143.0);
        const double BIT24MAX = (16777215.0);
        const double BIT18MAX_HALF = (BIT18MAX / 2.0);
        const double BIT18MAX_0D75 = (BIT18MAX * 3.0 / 4.0);
        const double BIT32MAX = (4294967295.0);

        public const int PIEZO_Z = (0);
        public const int PIEZO_X = (1);
        public const int PIEZO_Y = (2);
        public const int PIEZO_T = (3);

        //#define MAX_RANGE_Z_NM ( 7.299788679537674*1000.0)

        const double MAX_RANGE_Z_NM = (14 * 1000.0);//(21.387973775678940 * 1000.0);
        const double MAX_RANGE_X_NM = (50 * 1000.0);
        const double MAX_RANGE_Y_NM = (50 * 1000.0);
        public double[] MAX_RANGE_AXIS_NM = { MAX_RANGE_Z_NM, MAX_RANGE_X_NM, MAX_RANGE_Y_NM };
        /// <image defines>
        const int max_image_width = 512;

        double para_Nx = max_image_width;
        double para_Ny = max_image_width;
        double para_Dx = 0;
        double para_Dy = 0;
        public double para_XL = 0;
        public double para_YL = 0;
        double para_ScanRate = 0;
        double para_Sensitivity = 0;
        double para_SetDeltaVoltage_mV = 0;
        double para_Z_PID_P = 0;
        double para_Z_PID_I = 0;
        double para_Z_PID_D = 0;
        double[] para_IC0_DR = { 128, 128, 128, 128, 0, 0 };//R0~R3,DO1,DO2, counted as only four
        double para_NumberOfFrameToScan = 1;
        double para_TF_DC_Gain = 1;

        double para_NumberOfFrameFinished = 0;

        const int NumberOfParameters = 20;
        public CParameter mCParameter = new CParameter();
        // Form_Indentation mForm_Indentation;// = new Form_Indentation(this);

        public CIniFile mCIniFile = new CIniFile(".\\config.ini");

        /// <summary>
        /// ////////////////////////////////
        /// </summary>
        Bitmap mImageBmpH;//= new Bitmap(pictureBox_Height.Image); 
        //int[,] mImageArrayHL = new int[max_image_width, max_image_width];
        //int[,] mImageArrayEL = new int[max_image_width, max_image_width];
        //int[,] mImageArrayHR = new int[max_image_width, max_image_width];
        //int[,] mImageArrayER = new int[max_image_width, max_image_width];
        public double[,] mImageArrayHL;// = new double[para_Nx, para_Ny];
        public double[,] mImageArrayEL;// = new double[para_Nx, para_Ny];
        public double[,] mImageArrayHR;//= new double[para_Nx, para_Ny];
        public double[,] mImageArrayER;//= new double[para_Nx, para_Ny];
        int point_now_x = 0;//+-(1~Nx) for MKernel
        int point_now_y = 0;
        Thread mThread_SaveImage;

        int mCounter_ComReadByte = 0;
        int mApproach_heat_beat_received = -1;
        double mApproach_CoarseStepCounter = 0;
        double mApproach_TimesCounter = 0;
        bool mApproach_state = false;


        public double[,] mIndentData = new double[3, 10000];
        public int mIndentData_index = 0;
        public bool mSwitch_IndentTrue_FinishFalse = false;

        public SerialPort serialVirtual_echo;
        //public SerialPort serialVirtual_Coarse;
        bool mB_serialVirtual_Arduino_busy = false;

        string Sys_Inf = null;
        Thread mThread_UI_Update;
        bool mThread_UI_Update_running = true;
        bool mSWitchShowImage = false;// not show by default
        bool mSwitch_ShowComDdata = true;


        public double Z_position_now = 0;

        public MainWindow()
        {
            //TIC();
            //Thread.Sleep(10);
            //long x = TOC();
            InitializeComponent();

            mCCoarsePositioner = new CCoarsePositioner();
            mCCoarsePositioner.Initialize();


            // serialVirtual_Coarse = new SerialPort("COM12", 115200, Parity.None, 8, StopBits.One);
            //serialVirtual_Coarse.Open();
            string sPortName = "COM35";
            //var portExists = SerialPort.GetPortNames();//.Any(new var x => x == sPortName);
            serialVirtual_echo = new SerialPort(sPortName, 115200, Parity.None, 8, StopBits.One);
            serialVirtual_echo.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceivedHandler_com2com);
            serialVirtual_echo.Encoding = Encoding.GetEncoding("Windows-1252");
            serialVirtual_echo.ReceivedBytesThreshold = 1;// LENGTH_COM_BUFFER_MCU2PC;
            serialVirtual_echo.ReadTimeout = 1;
            //serialVirtual_echo.Open();


            //mImageBmpH = new Bitmap(256, 200);
            //pictureBox_Height.Image = mImageBmpH;

            ImageArray_SizeReset();
            ImageArray_ValueReset();
            mThread_SaveImage = new Thread(ThreadFunction_SaveImage);

            SetStyle(ControlStyles.SupportsTransparentBackColor, true);
            listBox_SelectIdlePackage.SetSelected(0, true);

            mDelegateFunction = new DelegateFunction(Function_UpdateUI);

            mThread_UI_Update = new Thread(ThreadFunction_UpdateUI);
            mThread_UI_Update.Start();

            propertyGrid_AFM_Parameter.SelectedObject = mCParameter;


            //mForm_Indentation = new Form_Indentation(this);
            Form_Indentation(this);// initialize indent 

            //mDataPath += GetCurrentTimeString() + "\\";
            System.IO.Directory.CreateDirectory(mDataPath);
        }
        public void ThreadFunction_UpdateUI()
        {
            mKernelClass = new MKernel.KernelClass();
            MY_DEBUG("MKernel started.");
            //MessageBox.Show("MKernel started.");
            LoadAFMParamter();
            while (mThread_UI_Update_running == true)
            {
                Function_UpdateUI();
                for (int k = 0; k < 100; k++)
                {
                    MY_DEBUG(Sys_Inf);
                    Sys_Inf = null;
                    Thread.Sleep(5);
                }
            }
        }
        void LoadAFMParamter()
        {
            string in_str = in_str = "out_data=[-1;-1];out_data=load(\'AFM_parameter.txt\');";//'out_data=load(''AFM_parameter.txt\'');'
            object Oin_str = (object)in_str;
            string out_str = null;
            object Oout_str = (object)out_str;
            double in_data = 0;
            object Oin_data = (object)in_data;
            double[,] out_data = new double[NumberOfParameters, 1];
            object Oout_data = (object)out_data;
            mKernelClass.StringEval(2, ref Oout_str, ref Oout_data, Oin_str, Oin_data);

            out_data = (double[,])Oout_data;
            if (out_data.Length == NumberOfParameters)
            {
                int k = 1;// when convert back from matlab, array starts at 1 instead of 0
                k++;
                k++;
                para_Nx = out_data[k++, 1];//max_image_width;
                para_Ny = out_data[k++, 1];//max_image_width;
                para_Dx = out_data[k++, 1];//0;
                para_Dy = out_data[k++, 1];//0;
                para_XL = out_data[k++, 1];//0;
                para_YL = out_data[k++, 1];//0;
                para_ScanRate = out_data[k++, 1];//0;
                para_Sensitivity = out_data[k++, 1];//0;
                para_SetDeltaVoltage_mV = out_data[k++, 1];//0;
                para_Z_PID_P = out_data[k++, 1];//0;
                para_Z_PID_I = out_data[k++, 1];//0;
                para_Z_PID_D = out_data[k++, 1];//0;
                para_NumberOfFrameToScan = out_data[k++, 1];
                para_TF_DC_Gain = out_data[k++, 1];

                // ADD MORE HERE
                para_IC0_DR[0] = out_data[k++, 1];
                para_IC0_DR[1] = out_data[k++, 1];
                para_IC0_DR[2] = out_data[k++, 1];
                para_IC0_DR[3] = out_data[k++, 1];


                UpdateGUITextBox_Invoke(ref para_Z_PID_P, textBox_Z_PID_P);//, 0);//, 100);
                UpdateGUITextBox_Invoke(ref para_Z_PID_I, textBox_Z_PID_I);//, 0);//, 100);
                UpdateGUITextBox_Invoke(ref para_Z_PID_D, textBox_Z_PID_D);//, 0);//, 100);
                UpdateGUITextBox_Invoke(ref para_Nx, textBox_Nx);//, 32, 512);
                UpdateGUITextBox_Invoke(ref para_Ny, textBox_Ny);//, 32, 512);
                UpdateGUITextBox_Invoke(ref para_Dx, textBox_Dx);//, 1, MAX_RANGE_X_NM);
                UpdateGUITextBox_Invoke(ref para_Dy, textBox_Dy);//, 1, MAX_RANGE_Y_NM);
                UpdateGUITextBox_Invoke(ref para_XL, textBox_XL);//, 1, MAX_RANGE_X_NM);
                UpdateGUITextBox_Invoke(ref para_YL, textBox_YL);//, 1, MAX_RANGE_Y_NM);
                UpdateGUITextBox_Invoke(ref para_ScanRate, textBox_ScanRate);//, 0.01, 5);
                UpdateGUITextBox_Invoke(ref para_Sensitivity, textBox_Sensitivity);//, 0.01, 500);
                UpdateGUITextBox_Invoke(ref para_SetDeltaVoltage_mV, textBox_SetDeltaValueNm);//, 1);//, 1000);
                UpdateGUITextBox_Invoke(ref para_NumberOfFrameToScan, textBox_NumberOfFrameToScan);//, 1, 2000);
                UpdateGUITextBox_Invoke(ref para_TF_DC_Gain, textBox_TF_DC_Gain);//, 1, 2000);


                UpdateGUITextBox_Invoke(ref para_IC0_DR[0], textBox_IC0_R0);
                UpdateGUITextBox_Invoke(ref para_IC0_DR[1], textBox_IC0_R1);
                UpdateGUITextBox_Invoke(ref para_IC0_DR[2], textBox_IC0_R2);
                UpdateGUITextBox_Invoke(ref para_IC0_DR[3], textBox_IC0_R3);

                ImageArray_SizeReset();// according to para_Nx,para_Ny
                ImageArray_ValueReset();
            }
        }
        void UpdateGUITextBox_Invoke(ref double value, TextBox T)
        {
            double v = value;
            value += 0.0000001;// to make a little difference for first time parameter write to mcu
            T.BeginInvoke((MethodInvoker)delegate() { T.Text = v.ToString(); });
        }
        void UpdateImageShow_SaveMat(string t)
        {
            //point_now_x = 30;
            //point_now_y = 50;
            //for (int a = 0; a < 100; a++)
            //    for (int b = 0; b < 100; b++)
            //        mImageArrayHL[a, b] = a + b;

            if (point_now_y > 1) point_now_y -= 1;

            object OmImageArrayHL = (object)mImageArrayHL;
            object OmImageArrayHR = (object)mImageArrayHR;
            object OmImageArrayEL = (object)mImageArrayEL;
            object OmImageArrayER = (object)mImageArrayER;

            object Opoint_now_x = (object)(point_now_x);
            object Opoint_now_y = (object)(point_now_y);
            string strIN = "save('input_" + t + ".mat','imHL','imHR','imEL','imER','point_now_x','point_now_y')";
            object OstrIN = (object)strIN;

            string output = null;
            object Ooutput = (object)output;

            mKernelClass.AFM_dip_show_image(
                1, ref Ooutput,
                OmImageArrayHL,
                OmImageArrayHR,
                OmImageArrayEL,
                OmImageArrayER,
                Opoint_now_x,
                Opoint_now_y,
                OstrIN);
            output = Convert.ToString(Ooutput);
            if (output.Length > 1) MY_DEBUG(output);
        }
        void UpdateImageShow()
        {
            //point_now_x = 30;
            //point_now_y = 50;
            //for (int a = 0; a < 100; a++)
            //    for (int b = 0; b < 100; b++)
            //        mImageArrayHL[a, b] = a + b;

            if (point_now_y > 1) point_now_y -= 1;

            object OmImageArrayHL = (object)mImageArrayHL;
            object OmImageArrayHR = (object)mImageArrayHR;
            object OmImageArrayEL = (object)mImageArrayEL;
            object OmImageArrayER = (object)mImageArrayER;

            object Opoint_now_x = (object)(point_now_x);
            object Opoint_now_y = (object)(point_now_y);
            string strIN = "save('input.mat','imHL','imHR','imEL','imER','point_now_x','point_now_y')";
            object OstrIN = (object)strIN;

            string output = null;
            object Ooutput = (object)output;

            mKernelClass.AFM_dip_show_image(
                1, ref Ooutput,
                OmImageArrayHL,
                OmImageArrayHR,
                OmImageArrayEL,
                OmImageArrayER,
                Opoint_now_x,
                Opoint_now_y,
                OstrIN);
            output = Convert.ToString(Ooutput);
            if (output.Length > 1) MY_DEBUG(output);
        }


        public void Function_UpdateUI()
        {
            if (Sys_Inf != null)
                update_UI_label(Sys_Inf);
            if (mSWitchShowImage == true)
                UpdateImageShow();

        }

        private void listBox_Axis_SelectedIndexChanged(object sender, EventArgs e)
        {
            // get selected axis number
            //X,Y,Z,T,XY_Plane,All
            int k = 0;
            for (k = 0; k < listBox_SelectIdlePackage.Items.Count; k++)
                if (listBox_SelectIdlePackage.GetSelected(k))
                    break;
            set_AFM_parameters('a', k);
        }
        private void button_Stop_Click(object sender, EventArgs e)
        {
            //mManipulator.Stop(axis);
        }
        private void button_MoveToEnd_Click(object sender, EventArgs e)
        {
            //mManipulator.MoveAtSpeed(axis, Convert.ToDouble(// textBox_Speed.Text) * (1));
        }

        private void button_MoveToStart_Click(object sender, EventArgs e)
        {
            //mManipulator.MoveAtSpeed(axis, Convert.ToDouble(// textBox_Speed.Text) * (-1));
        }

        private void button_MoveDistanceEnd_Click(object sender, EventArgs e)
        {
            //mManipulator.MoveDistance(axis,Convert.ToDouble(textBox_IC1_R0.Text)*(-1));
        }

        private void button_MoveDistanceStart_Click(object sender, EventArgs e)
        {
            //mManipulator.MoveDistance(axis, Convert.ToDouble(textBox_IC1_R0.Text));
        }


        private void button_SetSpeed_Click(object sender, EventArgs e)
        {

            double data = 0;//Convert.ToDouble(// textBox_Speed.Text);
            data = Math.Abs(data);
            // textBox_Speed.Text = Convert.ToString(data);
            //mManipulator.SetSpeed((sbyte)'A', data);
        }




        private void button_SetPosition_Click(object sender, EventArgs e)
        {

        }

        private void textBox_Postion_TextChanged(object sender, EventArgs e)
        {

        }

        private void button_StopUpdatePosition_Click(object sender, EventArgs e)
        {
            timer_CheckCOM.Stop();
            //this.textBox_Postion.BackColor = System.Drawing.SystemColors.Window;
            // this.textBox_Postion.BackColor = System.Drawing.Color.LightGreen;
        }

        private void button_Reset_Click(object sender, EventArgs e)
        {
            //this.button_Reset.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.working;
            //this.button_Reset.Enabled = false;
            //mThread_ResteHome = new Thread(ThreadFunction_ResteHome);
            //mThread_ResteHome.Start();
            //this.button_Reset.Enabled = true;

            //int m = 1;
            //for (int k = 100; k < 1000; k++)
            //{
            //    //mManipulator.MoveAtSpeed(1, k * m);
            //    Console.WriteLine(k * m);
            //    if (k % 5 == 1)
            //        m *= -1;
            //    Thread.Sleep(30);

            //}
            //mManipulator.Reset(axis);
        }
        void ThreadFunction_ResteHome()
        {
            ////mManipulator.ResetHomePosition(axis);
            //this.Invoke(this.mDelegateFunction);         
        }
        void convert_uint32_to_byte4(UInt32 x, byte[] b)
        {
            //            value+=com_buffer_frame[2]<<24;
            //value+=com_buffer_frame[3]<<16;
            //value+=com_buffer_frame[4]<<8;
            //value+=com_buffer_frame[5];
            byte[] L = new byte[4] { 3, 2, 1, 0 };
            const UInt32 vb = 255;//0xff
            foreach (int k in L)
                b[3 - k] = (byte)((x & (vb << k * 8)) >> k * 8);
        }
        double convert_byte4_to_uint32(byte[] b, int offset)
        {
            double value = 0;
            value += (double)(b[0 + offset]) * 0xffffff;
            value += (double)(b[1 + offset]) * 65536;
            value += (double)(b[2 + offset]) * 256;
            value += (double)(b[3 + offset]);
            return value;
        }
        double convert_byte3_to_uint32(byte[] b, int offset)
        {
            double value = 0;
            value += (double)(b[0 + offset]) * 65536;
            value += (double)(b[1 + offset]) * 256;
            value += (double)(b[2 + offset]);
            return value;
        }
        double convert_byte2_to_int16(byte[] b, int offset)
        {
            double value = 0;
            value += (double)b[0 + offset] * 256;
            value += (double)(b[1 + offset]);
            if (value > 65536 / 2 - 1)
                value -= 65536;
            return value;
        }

        double convert_to_even_int(ref TextBox T)
        {
            double t = Convert.ToDouble(T.Text);
            t -= t % 1;
            t += t % 2;
            T.Text = t.ToString();
            return t;
        }
        private void button_SetParameters_Click(object sender, EventArgs e)
        { button_SetParameters_Click_function(); }
        void button_SetParameters_Click_function()
        {
            if (button_SetParameters.Enabled == false)
                return;
            button_SetParameters.Enabled = false;
            button_SetParameters.Visible = false;
            MY_DEBUG("set parameters start.");
            //pid
            set_AFM_parameters('R', ref para_Sensitivity, textBox_Sensitivity, -100, 100);//0.001, 500)
            set_AFM_parameters('P', ref para_Z_PID_P, textBox_Z_PID_P, 0, 100);
            set_AFM_parameters('I', ref para_Z_PID_I, textBox_Z_PID_I, 0, 100);
            set_AFM_parameters('D', ref para_Z_PID_D, textBox_Z_PID_D, 0, 100);
            // XY resolution

            //  textBox_Nx.Text.to
            double t_para_Nx = para_Nx;
            double t_para_Ny = para_Ny;
            convert_to_even_int(ref textBox_Nx);
            convert_to_even_int(ref textBox_Ny);
            set_AFM_parameters('X', ref para_Nx, textBox_Nx, 16, 512);
            set_AFM_parameters('Y', ref para_Ny, textBox_Ny, 16, 512);
            if (t_para_Nx != para_Nx | t_para_Ny != para_Ny)
            {
                ImageArray_SizeReset();
                ImageArray_ValueReset();
            }

            // xy scan range nm
            set_AFM_parameters('x', ref para_Dx, textBox_Dx, 1, MAX_RANGE_X_NM);
            set_AFM_parameters('y', ref para_Dy, textBox_Dy, 1, MAX_RANGE_Y_NM);
            // start point of XY
            set_AFM_parameters('m', ref para_XL, textBox_XL, 1, MAX_RANGE_X_NM);
            set_AFM_parameters('n', ref para_YL, textBox_YL, 1, MAX_RANGE_Y_NM);
            // 
            set_AFM_parameters('S', ref para_ScanRate, textBox_ScanRate, 0.01, 5);

            // for PRC, use VWset_deltaV_input_mV as nm,
            // for tuning fork, use this as delta_voltage
            set_AFM_parameters('W', ref para_SetDeltaVoltage_mV, textBox_SetDeltaValueNm, 1, 20);

            set_AFM_parameters('N', ref para_NumberOfFrameToScan, textBox_NumberOfFrameToScan, 1, 2000);

            set_AFM_parameters('t', ref para_TF_DC_Gain, textBox_TF_DC_Gain, -5000, 5000);


            // set channel5, Dout2 last will cause self-oscillation stop on madcity lab probe
            send_DR_Value(0, 5, Convert.ToByte(checkBox_IC0_DO2.Checked));

            send_DR_Value(0, 0, (byte)Convert.ToDouble(textBox_IC0_R0.Text));
            send_DR_Value(0, 1, (byte)Convert.ToDouble(textBox_IC0_R1.Text));
            send_DR_Value(0, 2, (byte)Convert.ToDouble(textBox_IC0_R2.Text));
            send_DR_Value(0, 3, (byte)Convert.ToDouble(textBox_IC0_R3.Text));

            trackBar_R01.Value = Convert.ToInt32(para_IC0_DR[0]);

            string t = DateTime.Now.ToString("yyyyMMddHHmmss");
            SaveAFMParaToTextFile(t);
            SaveAFMParaToTextFile(null);
            //Thread.Sleep(14000);
            MY_DEBUG("set parameters done.");
            button_SetParameters.Enabled = true;
            button_SetParameters.Visible = true;
        }
        public void set_AFM_parameters_old(char parameter_name, double data)
        {
            const double para_EPS = 3.814697265625000e-06;
            double Ggain = 16384;//2^17 65536 * 2.0;// keep 5 decimal
            UInt32 value = (UInt32)(data * Ggain);
            //int value = (int)(data * gain);
            byte[] Bvalue = new byte[4] { 0, 0, 0, 0 };
            convert_uint32_to_byte4(value, Bvalue);
            send_Data_Frame_To_Arduino('P', parameter_name, Bvalue);
            Thread.Sleep(20);
        }
        public void set_AFM_parameters(char parameter_name, double data)
        {
            float fdata = Convert.ToSingle(data);
            byte[] valueByte4 = BitConverter.GetBytes(fdata);
            //float x = BitConverter.ToSingle(valueByte4, 0);
            send_Data_Frame_To_Arduino('P', parameter_name, valueByte4);
            Thread.Sleep(50);
        }
        void set_AFM_parameters(char parameter_name, ref double para_store, TextBox T)//,double  low_limit,double up_limit)
        {
            double data = Math.Abs(Convert.ToDouble(T.Text));
            if (checkBox_ForceSetAll.Checked == true || data != para_store)// avoid set the same one
            {
                para_store = data;
                set_AFM_parameters(parameter_name, data);
            }
        }
        void set_AFM_parameters(char parameter_name, ref double para_store, TextBox T, double low_limit, double up_limit)
        {
            double value = //Math.Abs
                (Convert.ToDouble(T.Text));
            // use max 6 number in decimal part
            const int x = 1000000;
            value *= x;
            value = (Int64)value;
            value = value / x;

            value = Math.Max(value, low_limit);
            value = Math.Min(value, up_limit);
            T.Text = Convert.ToString(value);

            set_AFM_parameters(parameter_name, ref para_store, T);
        }
        public void set_output_Position_Value_01(byte axis, double value_01)
        {
            double value_nm = value_01 * MAX_RANGE_AXIS_NM[axis];
            if (value_nm > MAX_RANGE_AXIS_NM[axis])
            {
                value_nm = MAX_RANGE_AXIS_NM[axis];
                MY_DEBUG("set_output_Position_Value_01 input exceed upper limit");
            }
            UInt32 value_position_FF32 = 0;
            if (axis > 2)
                for (axis = 0; axis < 3; axis++)
                {
                    value_position_FF32 = (UInt32)(value_nm * (BIT32MAX / MAX_RANGE_AXIS_NM[axis]));
                    set_output_parameters('F', axis, value_position_FF32);
                    Thread.Sleep(800);
                }
            else// one axis
            {
                value_position_FF32 = (UInt32)(value_nm * (BIT32MAX / MAX_RANGE_AXIS_NM[axis]));
                set_output_parameters('F', axis, value_position_FF32);
            }

        }
        public void set_output_DAC_Value_0_5(byte axis, double value0_5)
        {

            value0_5 = value0_5 * BIT18MAX / 5.0;
            set_output_parameters('D', axis, (UInt32)value0_5);
        }
        public void set_output_parameters(char parameter_name, byte axis, UInt32 value)//,double  low_limit,double up_limit)
        {
            // double data = Math.Abs(Convert.ToDouble(T.Text));
            //double Ggain = 1000.0;// keep 3 decimal
            //int value = (int)(data * Ggain);
            byte[] Bvalue = new byte[4] { 0, 0, 0, 0 };
            convert_uint32_to_byte4(value, Bvalue);
            send_Data_Frame_To_Arduino(parameter_name, (char)axis, Bvalue);
        }


        private void button_SetStepLength_Click(object sender, EventArgs e)
        {
            double sl = Convert.ToDouble(textBox_IC0_R0.Text);
            sl = Math.Abs(sl);
            textBox_IC0_R0.Text = Convert.ToString(sl);
        }


        //////////////////////////////////////////////////////////////////////


        private void button_ConnetComPort_Click(object sender, EventArgs e)
        {
            if (button_ConnetComPort.Text == "disconnect")
                if (serialPort_Arduino.IsOpen == true)
                {
                    try
                    {
                        serialPort_Arduino.Close();
                    }
                    catch
                    { }
                    button_ConnetComPort.Text = "connect";
                    this.button_ConnetComPort.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Signal_off;
                    this.toolTip_Help.SetToolTip(this.button_ConnetComPort, "not connected.");
                    return;
                }


            string com_number = "COM" + textBox_ComPortNO.Text;

            serialPort_Arduino = new SerialPort(com_number, Convert.ToInt32(textBox_BaudRate.Text), Parity.None, 8, StopBits.One);
            serialPort_Arduino.DataReceived += new SerialDataReceivedEventHandler(on_Received_serialPort_DataReceivedHandler);
            serialPort_Arduino.Encoding = Encoding.GetEncoding("Windows-1252");
            serialPort_Arduino.ReceivedBytesThreshold = LENGTH_COM_BUFFER_MCU2PC*100;
            serialPort_Arduino.ReadTimeout = 2;
            serialPort_Arduino.WriteTimeout = 2000;
            serialPort_Arduino.DtrEnable = true;
            serialPort_Arduino.RtsEnable = true;
            // this DtrEnable must be enabled to enable native USB communication on Arduino Due SerialUSB
            // serialPort_Arduino.DtrEnable = true; on programming usb serial will reset mcu
            //serialPort_Arduino.RtsEnable = true;
            bool state = true;
            try
            {
                serialPort_Arduino.Open();
            }
            catch
            {
                state = false;
            }

            if (state == true)
            {

                this.button_ConnetComPort.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Signal_on;
                //this.button_ConnetComPort.Enabled = false;
                this.toolTip_Help.SetToolTip(this.button_ConnetComPort, "Connected.");

                //MessageBox.Show("Initialization successful!");
                // timer_CheckCOM.Start();
                //button_SetParameters_Click_function();

                //reset_MCU_actuator();


                if (button_ConnetComPort.Text == "connect")
                {
                    button_ConnetComPort.Text = "disconnect";
                }
            }
            else
            {
                //this.button_ConnetComPort.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Signal_error;
                //this.button_ConnetComPort.Enabled = false;
                //MessageBox.Show("fail to open serial port");
            }

        }
        //void reset_MCU_actuator()
        //{
        //    set_output_DAC_Value_0_5(0, 0);
        //    set_output_Position_Value_01(0, 0);
        //    set_output_DAC_Value_0_5(0, 0);
        //    set_output_Position_Value_01(0, 0);
        //}
        private void MainWindow_Load(object sender, EventArgs e)
        {

        }

        void CheckRange_TextBox(TextBox T, double limit_down, double limit_up)
        {
            double v = Convert.ToDouble(T.Text);
            v = MIN_MAX(v, limit_down, limit_up);
            v = v - v % 1;// convert to int
            T.Text = Convert.ToString(v);
        }
        private void text_CheckKeys_IC0R0(object sender, System.Windows.Forms.KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)13)
            {
                CheckRange_TextBox(textBox_IC0_R0, 0, 255);
                send_DR_Value(0, 0, Convert.ToByte(textBox_IC0_R0.Text));
            }
        }
        private void text_CheckKeys_IC0R1(object sender, System.Windows.Forms.KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)13)
            {
                CheckRange_TextBox(textBox_IC0_R1, 0, 255);
                send_DR_Value(0, 1, Convert.ToByte(textBox_IC0_R1.Text));
            }
        }
        private void text_CheckKeys_IC0R2(object sender, System.Windows.Forms.KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)13)
            {
                CheckRange_TextBox(textBox_IC0_R2, 0, 255);
                send_DR_Value(0, 2, Convert.ToByte(textBox_IC0_R2.Text));
            }
        }
        private void text_CheckKeys_IC0R3(object sender, System.Windows.Forms.KeyPressEventArgs e)
        {
            if (e.KeyChar == (char)13)
            {
                CheckRange_TextBox(textBox_IC0_R3, 0, 255);
                send_DR_Value(0, 3, Convert.ToByte(textBox_IC0_R3.Text));
            }
        }
        private void checkBox_IC0_DO1_CheckedChanged(object sender, EventArgs e)
        {
            send_DR_Value(0, 4, Convert.ToByte(checkBox_IC0_DO1.Checked));
        }

        private void checkBox_IC0_DO2_CheckedChanged(object sender, EventArgs e)
        {
            send_DR_Value(0, 5, Convert.ToByte(checkBox_IC0_DO2.Checked));
        }

        //AA 55 52 03 f0 55 AA
        //AA 55 52 05 01 55 AA turn on IC0_DO2
        //AA 55 52 05 00 55 AA turn off IC0_DO2
        public void send_DR_Value(byte IC, byte channel, byte value)
        {
            para_IC0_DR[channel] = value;
            send_Data_Frame_To_Arduino('R', IC, channel, value);
        }

        public void send_Data_Frame_To_Arduino(char d0, char d1 = (char)0, char d2 = (char)0, byte d3 = 0, byte d4 = 0, byte d5 = 0)
        { send_Data_Frame_To_Arduino((byte)d0, (byte)d1, (byte)d2, d3, d4, d5); }
        public void send_Data_Frame_To_Arduino(char d0, byte d1 = 0, byte d2 = 0, byte d3 = 0, byte d4 = 0, byte d5 = 0)
        { send_Data_Frame_To_Arduino((byte)d0, (byte)d1, (byte)d2, d3, d4, d5); }
        public void send_Data_Frame_To_Arduino(char d0, char d1, byte[] db4)
        { send_Data_Frame_To_Arduino((byte)d0, (byte)d1, db4[0], db4[1], db4[2], db4[3]); }
        public void send_Data_Frame_To_Arduino
            (byte d0, byte d1 = 0, byte d2 = 0, byte d3 = 0, byte d4 = 0, byte d5 = 0)
        {
            //AA 55 52 00 03 aa 00 00 55 AA 
            byte[] com = new byte[LENGTH_COM_BUFFER_PC2MCU] 
                        { COM_HEADER1, COM_HEADER2, 
                         d0,d1,d2,d3,d4,d5,
                            // (byte)'R', 0, 0, 0,0,0,// 6 byte
                         COM_TAIL1, COM_TAIL2 };
            if (serialPort_Arduino.IsOpen == true)
            {
                try
                {
                    while (mB_serialVirtual_Arduino_busy == true)
                    {
                        Thread.Sleep(5);
                        MY_DEBUG("serial write busy.");
                    }
                    mB_serialVirtual_Arduino_busy = true;
                    serialPort_Arduino.Write(com, 0, LENGTH_COM_BUFFER_PC2MCU);
                    mB_serialVirtual_Arduino_busy = false;

                    Thread.Sleep(15);
                }
                catch
                {
                    //Thread.Sleep(100);
                    //serialPort_Arduino.Write(com, 0, LENGTH_COM_BUFFER_PC2MCU);
                    MY_DEBUG("serialPort_Arduino.Write time out");
                }
            }
            else
                MY_DEBUG("MCU port is not connected.");
            //MessageBox.Show("MCU port is not connected.", "Error");

        }

        private void timerFunction_Appraoch(object sender, EventArgs e)
        {
            if (mApproach_heat_beat_received == -1)
            {
                send_Data_Frame_To_Arduino('C', 'A', 'P');
                MY_DEBUG("retrigger");
            }
        }

        private void serialPort_DataReceivedHandler_com2com(
                            object sender,
                            SerialDataReceivedEventArgs e)
        {
            SerialPort serialPort_echo = (SerialPort)sender;
            //string str_in = serialPort_Arduino.ReadExisting();
            ////serialPort_Arduino.DiscardInBuffer();
            ////MY_DEBUG(str_in);
            //int c = (str_in.Length);
            //if (c < LENGTH_COM_BUFFER_PC2MCU) return; // ignore error

            //byte[] dbx = Encoding.UTF8.GetBytes(str_in);
            ////var regex = new Regex(@"UIM..........U");
            ////MatchCollection m=regex.Matches(str_in);
            int c = -1;
            byte[] db = new byte[LENGTH_COM_BUFFER_MCU2PC];
            try
            {
                c = serialPort_echo.Read(db, 0, LENGTH_COM_BUFFER_MCU2PC);
            }
            catch
            {
                if (c < 1) return; // ignore error
            }

            // whatever received, transfer
            if (checkBox_COM_Transfer.Checked == true)
                if (serialPort_Arduino.IsOpen == true)
                {
                    //while (mB_serialVirtual_Arduino_busy == true)
                    //    Thread.Sleep(10);
                    mB_serialVirtual_Arduino_busy = true;
                    serialPort_Arduino.Write(db, 0, c);
                    mB_serialVirtual_Arduino_busy = false;
                }
        }

        void Updata_UI_Richtext(byte[] db)
        {
            if (mSwitch_ShowComDdata == true)
            {
                try
                {
                    this.richTextBox_serial_read.BeginInvoke((MethodInvoker)delegate()
                    {
                        //richTextBox_serial_read.Text = System.Text.Encoding.UTF8.GetString(db);
                        return;

                        if (richTextBox_serial_read.Text.Length < 2000)
                            richTextBox_serial_read.Text += System.Text.Encoding.UTF8.GetString(db);
                        else
                            richTextBox_serial_read.Text = null;
                        // auto roll down
                        richTextBox_serial_read.SelectionStart = richTextBox_serial_read.Text.Length; //Set the current caret position at the end
                        richTextBox_serial_read.ScrollToCaret(); //Now scroll it automatically
                    });
                }
                catch
                { MY_DEBUG("Updata_UI_Richtext error"); }
            }
        }
        //private void timerFunction_CheckCOM(object sender, EventArgs e)
        //{
        //    return;
        //    // string indata  = serialPort_Arduino.ReadExisting();
        //    // char x= Convert.ToChar((char)0xff);

        //    // //indata[0] = Convert.ToString(x);

        //    // if (indata.Length == 0) return;
        //    //      byte[] db = new byte[128];
        //    // indata.
        //    // char []xx=indata.ToArray();
        //    //      for (int k = 0; k < indata.Length;k++ )
        //    //          db[k] = Convert.ToByte(indata[k]);
        //    //// serialPort_Arduino.Read


        //    //serialPort_Arduino.ReadTimeout = 2;
        //    byte[] db = new byte[2048];

        //    try
        //    {
        //        //  c = serialPort_Arduino.Read(db, 0, serialPort_Arduino.BytesToRead);
        //        string str_in = serialPort_Arduino.ReadExisting();
        //        //MY_DEBUG(str_in);
        //        //c=(str_in.Length);


        //        byte[] db1 = Encoding.UTF8.GetBytes(str_in);
        //        return;

        //        mCounter_ComReadByte = serialPort_Arduino.Read(db, 0, str_in.Length);
        //        serialPort_Arduino.DiscardInBuffer();
        //    }
        //    catch
        //    {
        //    }


        //    if (mCounter_ComReadByte > 0)
        //    {
        //        //  MY_DEBUG(c);
        //        //for (int k = 0; k < 4;k++ )
        //        //MY_DEBUG(db[k]);

        //        if (checkBox_COM_Transfer.Checked == true)
        //            serialVirtual_echo.Write(db, 0, mCounter_ComReadByte);

        //        on_Received_com_frame_anaysis(db, LENGTH_COM_BUFFER_MCU2PC);


        //        //richTextBox_serial_read.Text = "";
        //        //char[] values = indata.ToCharArray();
        //        //foreach (char letter in values)
        //        //{
        //        //    int value = Convert.ToInt32(letter);
        //        //    richTextBox_serial_read.Text += String.Format("{0:X}", value) + '-';
        //        //}
        //        ///*  
        //        //    byte[] db = System.Text.Encoding.UTF8.GetBytes(indata);
        //        //    string hex = BitConverter.ToString(db);

        //        //    hex.Replace('-', ' ');
        //        //    richTextBox_serial_read.Text = hex + "\n";*/
        //        //richTextBox_serial_read.Update();
        //    }
        //}
        // bool mB_serialPort_DataReceivedHandler_busy = false;
        private void on_Received_serialPort_DataReceivedHandler(
                            object sender,
                            SerialDataReceivedEventArgs e)
        {
            //if (mB_serialPort_DataReceivedHandler_busy == true)
            //{
            //    MY_DEBUG("sb");
            //    return;
            //}
            //mB_serialPort_DataReceivedHandler_busy = true;

            SerialPort serialPort_Arduino = (SerialPort)sender;
            //string str_in = serialPort_Arduino.ReadExisting();
            ////serialPort_Arduino.DiscardInBuffer();
            ////MY_DEBUG(str_in);
            //int c = (str_in.Length);
            //if (c < LENGTH_COM_BUFFER_PC2MCU) return; // ignore error

            //byte[] dbx = Encoding.UTF8.GetBytes(str_in);
            ////var regex = new Regex(@"UIM..........U");
            ////MatchCollection m=regex.Matches(str_in);
            mCounter_ComReadByte = -1;
            //int byte_ready=serialPort_Arduino.BytesToRead;
            //if (byte_ready < LENGTH_COM_BUFFER_MCU2PC)
            //    return;

            byte[] db = new byte[LENGTH_COM_BUFFER_MCU2PC * 2];
            try
            {
                //while (mB_serialVirtual_Arduino_busy == true)
                //    Thread.Sleep(1);
                mB_serialVirtual_Arduino_busy = true;
                mCounter_ComReadByte = serialPort_Arduino.Read(db, 0, LENGTH_COM_BUFFER_MCU2PC * 2);
                mB_serialVirtual_Arduino_busy = false;

                //byte[] buffer = Encoding.UTF8.GetBytes(convert);
                // From byte array to string
                //Encoding.UTF8

                //string s = Encoding.UTF8.GetString(db, 0, mCounter_ComReadByte);
                //MY_DEBUG(s);
                //MY_DEBUG(" ");

            }
            catch
            {
                if (mCounter_ComReadByte < LENGTH_COM_BUFFER_MCU2PC) return; // ignore error
            }
            // whatever received, transfer
            if (checkBox_COM_Transfer.Checked == true)
                serialVirtual_echo.Write(db, 0, mCounter_ComReadByte);

            //show received data
            //Updata_UI_Richtext(db);


            // must use this method, otherwise may lose data
            //serialPort_Arduino.DiscardInBuffer();
            int ind = on_Received_com_frame_anaysis(db, LENGTH_COM_BUFFER_MCU2PC * 2);
            //MY_DEBUG("ind:",ind);
            if (mCounter_ComReadByte >= LENGTH_COM_BUFFER_MCU2PC && ind == 0)
            {
                for (int k = 0; k < db.Length - LENGTH_COM_BUFFER_MCU2PC; k++)
                    db[k] = db[k + 16];
                int ind2 = on_Received_com_frame_anaysis(db, LENGTH_COM_BUFFER_MCU2PC * 2);
                //MY_DEBUG("ind2:", ind2);
            }
            //mB_serialPort_DataReceivedHandler_busy = false;
        }

        //void on_Received_com_frame_anaysis(string com_buffer)
        //{
        //    string header = Convert.ToString((char)0x3f)
        //        + Convert.ToString((char)COM_HEADER2) + "AP";
        //    int ind = com_buffer.IndexOf(header);
        //    bool done = true;
        //    if (ind >= 0)
        //    {
        //        done = Convert.ToBoolean(com_buffer[ind + 13]);
        //        if (done == false)
        //            AFM_coarse_positioner_MoveDistance(2, -800);
        //    }

        //}
        int on_Received_com_frame_anaysis(byte[] com_buffer, int length)
        {
            int ind = -1;
            for (int k = 0; k < length - 15; k++)
                if (com_buffer[k] == COM_HEADER1)
                    if (com_buffer[k + 1] == COM_HEADER2)
                        if (com_buffer[k + 14] == COM_TAIL1)
                            if (com_buffer[k + 15] == COM_TAIL2)
                            {
                                ind = k;
                                //break; do not break, continue to search for multi package;
                                if (ind >= 0)// frame found
                                {
                                    if (com_buffer[ind + 2] == 's' & com_buffer[ind + 3] == 'p')// idle state
                                        on_Received_Package_SystemState(com_buffer, ind);
                                    if (com_buffer[ind + 2] == 'I' & com_buffer[ind + 3] == 'D')// indent
                                        on_Received_Package_Indent(com_buffer, ind);
                                    if (com_buffer[ind + 2] == 'I' & com_buffer[ind + 3] == 'M')// image package
                                        on_Received_Package_Image(com_buffer, ind);
                                    if (com_buffer[ind + 2] == 'C' & com_buffer[ind + 3] == 'A' & com_buffer[ind + 4] == 'P')
                                        on_Received_Package_Approach(com_buffer, ind);
                                    if (com_buffer[ind + 2] == 'C' & com_buffer[ind + 3] == 'Z' & com_buffer[ind + 4] == 'E')
                                        //AA 55 43 5A 45 00 00 00 01 00 00 00 02 ff 55 AA
                                        on_Received_Package_ZScannerEngage(com_buffer, ind);
                                }

                            }


            //string str = Encoding.UTF8.GetString(com_buffer);
            //var regex = new Regex(@"UCZE");
            //MatchCollection m=regex.Matches(str);
            //if (m.Count > 0)
            //{
            //    MessageBox.Show(str, "match Z Engage");
            //    ind = -1;
            //}

            return ind;

        }
        //private BitmapImage GetPhoto(byte[] p)
        //{
        //    MemoryStream _stream = new MemoryStream(p);
        //    BitmapImage _bmp = new BitmapImage();
        //    _bmp.SetSource(_stream);
        //    return _bmp;
        //}

        double convert_Temperature2Degree_MCU(double T)
        {
            const double VCC = 2.8;
            const double Sensitivity_VperC = 0.02;
            T = T / 4095.0;//BIT(12);
            T = T * VCC;//--> Volt	
            T = (T - 0.5) / Sensitivity_VperC + 25 - 2.4;// degree
            return T;
        }
        double convert_Temperature2Degree_SEM(double T)
        {
            double VCC = 5.0;
            // const double VCC = 2.8;
            const double Sensitivity_VperC = 0.02;
            T = T / BIT18MAX;
            T = T * VCC;//--> Volt	
            T = (T - 0.5) / Sensitivity_VperC + 25 - 2.4;// degree
            return T;
        }

        void on_Received_Package_SystemState(byte[] com_buffer, int ind)
        {
            // AA 55 73 70 00   05 1E B8    00 FF E0    01 20 41    55 AA

            //value_cantilever, Temperature_SEM, Temperature_MCU

            ind += 2;
            int index = com_buffer[ind + 2];
            double v1 = convert_byte3_to_uint32(com_buffer, ind + 3);
            double v2 = convert_byte3_to_uint32(com_buffer, ind + 3 + 3);
            double v3 = convert_byte3_to_uint32(com_buffer, ind + 3 + 3 + 3);
            if (index == 0)
            {

                double value_cantilever = v1 * 5.0 / BIT18MAX;
                double v_Temperature_SEM = convert_Temperature2Degree_SEM(v2);
                double v_Temperature_MCU = convert_Temperature2Degree_MCU(v3);
                //realtime information of the system
                MY_DEBUG("PRC:\t" + value_cantilever.ToString("f5")
                    + "\t T_sem:\t" + v_Temperature_SEM.ToString("f2")
                     + "\t T_mcu:\t" + v_Temperature_MCU.ToString("f2"));
            }
            if (index == 1)
            {
                double scsg_x = v1;
                double scsg_y = v2;
                double scsg_z = v3;
                MY_DEBUG("scsg_x:\t" + scsg_x.ToString("f2")
                    + "\t scsg_y:\t" + scsg_y.ToString("f2")
                     + "\t scsg_z:\t" + scsg_z.ToString("f2"));
            }
            if (index == 2)
            {
                double value_cantilever = v1;
                double value_cali = v2;
                double scsg_z = v3;
                MY_DEBUG("value_cantilever:\t" + value_cantilever.ToString("f0")
                    + "\t value_cali:\t" + value_cali.ToString("f0")
                     + "\t scsg_z:\t" + scsg_z.ToString("f0"));
            }
        }

        void on_Received_Package_Indent(byte[] com_buffer, int ind)
        {
            // AA 55 73 70 00   05 1E B8    00 FF E0    01 20 41    55 AA
            ind += 2;
            double v_feedforward = convert_byte4_to_uint32(com_buffer, ind + 2);
            double v_Adc = convert_byte3_to_uint32(com_buffer, ind + 2 + 4);
            double v_Dac = convert_byte3_to_uint32(com_buffer, ind + 2 + 4 + 3);

            //v_feedforward *= MAX_RANGE_Z_NM / BIT32MAX;
            //realtime information of the system

            MY_DEBUG("FF:\t" + v_feedforward.ToString("f1")
                + "\tadc:\t" + v_Adc.ToString()
                 + "\tVadc:\t" + (v_Adc / BIT18MAX).ToString("f6")
                + "\tdac:\t" + v_Dac.ToString());
            // MY_DEBUG(textBox_T.Text+" \t"+v_Adc.ToString() + " \t" + (v_Adc / BIT18MAX).ToString("f6"));
            if (mSwitch_IndentTrue_FinishFalse == true)
            {
                mIndentData_index++;
                if (mIndentData.Length / (mIndentData.Rank + 1) < mIndentData_index - 1) { MY_DEBUG("exceed"); return; }

                mIndentData[0, mIndentData_index] = v_feedforward;
                mIndentData[1, mIndentData_index] = v_Adc;
                mIndentData[2, mIndentData_index] = v_Dac;

                if (v_Dac == BIT24MAX)// finished
                {
                    mSwitch_IndentTrue_FinishFalse = false;// tell the subform to save data
                    mIndentData_index--;
                    MY_DEBUG("indent finished.");
                }
            }
        }
        void update_UI_label(string inf)
        {
            if (label_SystemState.InvokeRequired)
            {
                this.label_SystemState.BeginInvoke((MethodInvoker)delegate() { label_SystemState.Text = inf; });
            }
            else
            {
                label_SystemState.Text = inf;
            }

        }
        int indx_store_for_save_image = 0;
        int indy_store_for_save_image = 0;
        void on_Received_Package_Image(byte[] com_buffer, int ind)
        {
            //test point x=127,y=65,height=65536,error=256
            // AA 55 49 4D 00 7F 00 41 01 00 00 00 01 00 55 AA 


            //height=0.1,error=0.8
            //height=1677721  ,error=13421772 
            //height=2500,error=20000
            //AA 55 49 4D FF FB 00 06 19 99 99 CC CC CC 55 AA 

            int indx = (int)convert_byte2_to_int16(com_buffer, ind + 4);// com_buffer[ind + 4] << 8 + com_buffer[ind + 5];
            int indy = (int)convert_byte2_to_int16(com_buffer, ind + 6); //com_buffer[ind + 6] << 8 + com_buffer[ind + 7];
            double vH = convert_byte3_to_uint32(com_buffer, ind + 8);
            double vE = convert_byte3_to_uint32(com_buffer, ind + 8 + 3);
            vE = vE - BIT24MAX / 2;
            vE = vE / BIT24MAX * MAX_RANGE_Z_NM;


            //vH = vH / BIT24MAX;// *MAX_RANGE_Z_NM;
            //// Z axis calibration parameter
            double vH0 = 1.435565217391304e+04;
            double vh_range = 1.519552569169960e+05;

            vH = (vH - vH0) / vh_range;
            vH *= MAX_RANGE_Z_NM;

            //vH = MAX_RANGE_Z_NM - vH;

            Sys_Inf = (
                "RX:" + Convert.ToString(mCounter_ComReadByte)
                + " x:" + Convert.ToString(indx)
                + " y:" + Convert.ToString(indy)
                + " H:" + String.Format("{0:0.0}", vH)//vH.ToString()//Convert.ToString(vH)
                + " E:" + String.Format("{0:0.0}", vE)//vE.ToString()//Convert.ToString(vE)
                );


            Z_position_now = vH;

            //  if (Math.Abs(Math.Abs(indx) - Math.Abs(xold)) != 1) 
            //MY_DEBUG( Sys_Inf);
            //update_UI_label(inf);

            //MY_DEBUG(indx.ToString() + "\t" + indx_store_for_save_image.ToString() + "\t" + indy.ToString() + "\t" + indy_store_for_save_image.ToString());
            // index adjusted
            point_now_x = (indx >= 0) ? indx + 1 : -indx;// +-(1~Nx)
            point_now_y = (indy >= 0) ? indy + 1 : -indy;

            if (
                (indy == para_Ny - 1 & indy_store_for_save_image == para_Ny - 2
                & indx == 0 & indx_store_for_save_image == -1)// positive scan end
                |
                (indy == -1 & indy_store_for_save_image == -2
                & indx == 0 & indx_store_for_save_image == -1)    //negative scan            
                )
            {
                MY_DEBUG("save image: indx " + indx.ToString() + "indy " + indy.ToString());
                SaveImage_StartThread();
                para_NumberOfFrameFinished++;// not used right now
            }

            if (indx == 0 & indx_store_for_save_image == -1)// update y when x finish one line and back
                indy_store_for_save_image = indy;
            indx_store_for_save_image = indx;
            if (indy < 0)
                indy = -indy - 1;

            if (Math.Abs(indx) >= para_Nx || Math.Abs(indy) >= para_Ny)
            {
                MY_DEBUG("indx", indx);
                MY_DEBUG("indy", indy);
                return;
            }

            if (indx >= 0)
            {
                mImageArrayHL[indx, indy] = vH;
                mImageArrayEL[indx, indy] = vE;
            }
            else
            {
                indx = -indx - 1;
                mImageArrayHR[indx, indy] = vH;
                mImageArrayER[indx, indy] = vE;
            }

            //mImageBmpH.SetPixel(indx, indy, Color.FromArgb(vH / 65536, vH / 65536, vH / 65536));
            //pictureBox_Height.Update();

        }

        void on_Received_Package_ZScannerEngage(byte[] com_buffer, int ind)
        {
            //AA 55 43 5A 45 66 61 69 6C 00 00 00 00 00 00 00 55 AA 
            string inf = "";
            for (int k = 0; k < 14; k++)
                inf = inf + Convert.ToString(Convert.ToChar(com_buffer[ind + 2 + k]));

            //MessageBox.Show(inf, "Z Engage");
            if (com_buffer[ind + 2 + 3] == 'd')
            {
                System.Media.SystemSounds.Exclamation.Play();// ok
                MY_DEBUG("Z scanner engage: OK");
            }
            else
            {
                System.Media.SystemSounds.Hand.Play();// fail
                MY_DEBUG("Z scanner engage: fail");
            }
        }
        void on_Received_Package_Approach(byte[] com_buffer, int ind)
        {
            //int adc = (int)convert_byte4_to_uint32(com_buffer, ind + 5);
            double final_adc_value = convert_byte4_to_uint32(com_buffer, ind + 5);
            double fine_step = convert_byte4_to_uint32(com_buffer, ind + 9);
            byte done = com_buffer[ind + 13];
            mApproach_heat_beat_received = done;
            MY_DEBUG(done);
            double max_steps = 2521;

            if (done == 0)// continue to walk
            {
                mApproach_CoarseStepCounter++;
                if (mApproach_CoarseStepCounter > 10000.0 * 1000.0 / (MAX_RANGE_Z_NM / 2.0))// exceed max coarse steps
                {
                    Appraoch_cancel();
                    MessageBox.Show("Coarse approaching failed. Tip-sample distance is too large, coarse step: " + mApproach_CoarseStepCounter.ToString(),
                        "Warning");
                }

                max_steps = fine_step;
                //AFM_coarse_positioner_SetSpeed(20);//20
                //AFM_coarse_positioner_MoveDistance(2, -MAX_RANGE_Z_NM * 0.7);// % BIT18MAX_0D9， use 0.7 only for safety
                mCCoarsePositioner.MoveDistance(mCaxis_z, -MAX_RANGE_Z_NM * 0.7, 20);// move one step down to the sample
                Thread.Sleep(300);
                //Thread.Sleep(1000);// add wait
                if (mApproach_state == true)
                    send_Data_Frame_To_Arduino('C', 'A', 'P');
                else
                {
                    timer_Approach.Stop();
                    MY_DEBUG("Approach canceled.");
                }

                mApproach_heat_beat_received = -1;

            }
            if (done == 2)
            {
                string m = "approach: initial adc value: " + final_adc_value.ToString();
                MY_DEBUG(m);
            }
            if (done == 255)
            {
                Appraoch_cancel();
                const string message = "Vdf at initial position error";
                const string caption = "Error";
                var result = MessageBox.Show(message, caption,
                                             MessageBoxButtons.YesNo,
                                             MessageBoxIcon.Question);
                if (result == DialogResult.No)
                    ;
            }
            if (done == 1)// finished
            {
                mApproach_state = false;
                timer_Approach.Stop();

                double sampling_period_us_of_Approach_Process = 2000;
                double sampling_frequency_of_Approach_Process = 1000000 / sampling_period_us_of_Approach_Process;
                double TIME_APPROACHING_COARSE_STEP = (5);//Second

                double STEP_SIZE_APPROACHING_dac_value = (BIT18MAX / (2.0) /
                    (TIME_APPROACHING_COARSE_STEP * sampling_frequency_of_Approach_Process));
                double STEP_SIZE_APPROACHING_nm = STEP_SIZE_APPROACHING_dac_value * MAX_RANGE_Z_NM / BIT18MAX;
                // double max_steps = 2521;
                double distance_tried = Math.Abs(max_steps - fine_step) * STEP_SIZE_APPROACHING_nm;
                //distance_tried += 10000;// manual adjust
                // final adjust
                //AFM_coarse_positioner_SetSpeed(20);
                //AFM_coarse_positioner_MoveDistance(2, distance_tried);
                //Thread.Sleep(1000);

                string message = "approaching times:" + mApproach_TimesCounter.ToString() +
                    " fine steps: " + Convert.ToString(fine_step) + "\tADC: " + final_adc_value.ToString();
                MY_DEBUG(message);

                mApproach_TimesCounter--;
                if (mApproach_TimesCounter > 0)
                {

                    System.Media.SystemSounds.Exclamation.Play();// ok
                    Thread.Sleep(300);
                    Apporach_start();
                }
                else
                {
                    MY_DEBUG("Approach finished.");
                    System.Media.SystemSounds.Exclamation.Play();// ok
                    Thread.Sleep(500);
                    System.Media.SystemSounds.Hand.Play();
                    Thread.Sleep(500);
                    //var result = MessageBox.Show(message, caption,
                    //                             MessageBoxButtons.YesNo);
                    //if (result == DialogResult.No)
                    //    ;
                }
            }
        }

        //////// coarse positioner start
        //void AFM_coarse_positioner_SetSpeed(int speed)
        //{ AFM_coarse_positioner_MoveDistance(-1, speed); Thread.Sleep(20); }

        //bool serial_lock = false;
        //void AFM_coarse_positioner_MoveDistance(int axis0_2, double d_distance)
        ////X0,Y1,Z2,  d_distance>0, move away from cable terminal
        //{
        //    if (serial_lock == false)
        //    {
        //        serial_lock = true;
        //        byte direction = 0;
        //        int distance = (int)d_distance;
        //        //busy=true;
        //        if (distance < 0)
        //            direction = 255;
        //        else
        //            direction = 1;

        //        if (axis0_2 == -1)
        //        {
        //            direction = 0;//% set speed
        //            axis0_2 = 255;
        //        }
        //        distance = Math.Abs(distance);
        //        if (distance > 65536 * 128)
        //            distance = 65536 * 128;
        //        int d1, d2, d3;
        //        d1 = distance / 65536;
        //        d2 = (distance - d1 * 65536) / 256;
        //        d3 = (distance - d1 * 65536 - d2 * 256);
        //        byte[] rs232_com = new byte[]
        //          {COM_HEADER1, COM_HEADER2,(byte)axis0_2, (byte)direction,
        //          (byte)d1,(byte)d2,(byte)d3,COM_TAIL1, COM_TAIL2};
        //        serialVirtual_Coarse.Write(rs232_com, 0, rs232_com.Length);
        //        Thread.Sleep(10);
        //        serial_lock = false;
        //    }
        //    else
        //        Thread.Sleep(10);
        //}
        /////////// coarse positioner end
        private void trackBar_R1_Scroll(object sender, EventArgs e)
        {
            byte v = Convert.ToByte(trackBar_R1.Value);
            textBox_IC0_R1.Text = Convert.ToString(v);
            send_DR_Value(0, 1, v);
        }

        private void trackBar_R0_Scroll(object sender, EventArgs e)
        {
            byte v = Convert.ToByte(trackBar_R0.Value);
            //v= (byte)(255 - v);
            textBox_IC0_R0.Text = Convert.ToString(v);
            send_DR_Value(0, 0, v);


            /*  byte v = Convert.ToByte(trackBar_R0.Value);
              byte rv = (byte)(255 - v);
              textBox_IC0_R0.Text = Convert.ToString(rv);
              textBox_IC0_R1.Text = Convert.ToString(v);
              send_DR_Value(0, 0, rv);
              Thread.Sleep(100);
              send_DR_Value(0, 1, v);
              Thread.Sleep(100);*/


        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            byte v = Convert.ToByte(trackBar_IC1R3.Value);
            textBox_IC0_R3.Text = Convert.ToString(v);
            send_DR_Value(0, 3, v);
        }


        private void button_Apporach_Click(object sender, EventArgs e)
        { Apporach_start_multi(); }
        public void Apporach_start_multi()
        {
            mApproach_state = true;
            mApproach_TimesCounter = 1;//4
            Apporach_start();
            //on_Received_Package_Approach
        }

        private void button_ManuallyCancelApproach_Click(object sender, EventArgs e)
        {
            Appraoch_cancel();
        }

        public void Appraoch_cancel()
        {
            mApproach_state = false;
            mApproach_TimesCounter = 0;
            timer_Approach.Stop();
            send_Data_Frame_To_Arduino('C', 'A', 'C');
        }

        public void Apporach_start()
        {
            MY_DEBUG("start approach.");

            //AFM_coarse_positioner_SetSpeed(250);
            //AFM_coarse_positioner_MoveDistance(2, MAX_RANGE_Z_NM * 1.21);// % for safety reason, first move up 25*1.2 um

            mCCoarsePositioner.MoveDistance(mCaxis_z, MAX_RANGE_Z_NM * 1.21, 250);// move away up for safety reason
            //Thread.Sleep(1500);
            Thread.Sleep(1500);
            //AFM_coarse_positioner_SetSpeed(10);

            send_Data_Frame_To_Arduino('C', 'A', 'P');
            timer_Approach.Interval = 6512;
            timer_Approach.Start();//trigger function   timerFunction_Appraoch
            mApproach_CoarseStepCounter = 0;


            //if (button_Apporach.Text == "apporach")
            //{
            //    send_Data_Frame_To_Arduino('C','A','P');
            //    button_Apporach.Text = "cancel";
            //}
            //else
            //{

            //    button_Apporach.Text = "apporach";
            //}
        }

        private void button1_Click(object sender, EventArgs e)
        {
            //serialPort_Arduino.DtrEnable = false; //on programming usb serial will reset mcu
            //serialPort_Arduino.DtrEnable = true;// on programming usb serial will reset mcu
            //send_Data_Frame_To_Arduino('r', 's', 't');
            //while (true)
            //{
            //    send_DR_Value(0, 5, 1);
            //    Thread.Sleep(1000);
            //}
        }

        private void checkBox_COM_Transfer_CheckedChanged(object sender, EventArgs e)
        {
            //the checkbox is used as a global switch

            if (checkBox_COM_Transfer.Checked == true)
            {
                if (serialVirtual_echo.IsOpen == false)
                    try
                    {
                        serialVirtual_echo.Open();
                    }
                    catch (Exception ex)
                    {
                        checkBox_COM_Transfer.Checked = false;
                    }

            }
            else
            {
                if (serialVirtual_echo.IsOpen == true)
                    try
                    {
                        serialVirtual_echo.Close();
                    }
                    catch (Exception ex)
                    {
                        checkBox_COM_Transfer.Checked = false;
                    }
            }

        }

        private void button_Z_Engage_Click(object sender, EventArgs e)
        {
            MY_DEBUG("start engage.");
            send_Data_Frame_To_Arduino('C', 'Z', 'E');//AA 55 43 5A 45 00 00 00 55 AA 
            mSwitch_ShowComDdata = false;
        }

        private void button_Z_Withdraw_Click(object sender, EventArgs e)
        {
            //AA 55 43 5A 57 00 00 00 55 AA 
            //AA 55 43 53 52 00 00 00 55 AA 
            send_Data_Frame_To_Arduino_SetSystemIdle_Multi();
            //send_Data_Frame_To_Arduino('C', 'Z', 'W');
        }
        public void send_Data_Frame_To_Arduino_SetSystemIdle_Multi()
        {
            MY_DEBUG("reset.");
            mSwitch_ShowComDdata = true;
            // for (int k = 0; k < 10; k++)
            { send_Data_Frame_To_Arduino('C', 'Z', 'W'); Thread.Sleep(100); }
        }
        private void checkBox_Y_ScanEnable_CheckedChanged(object sender, EventArgs e)
        {
            char ch;
            if (checkBox_Y_ScanEnable.Checked == true)
                ch = 'E';
            else
                ch = 'D';
            send_Data_Frame_To_Arduino('C', 'Y', ch);
        }

        private void button_XY_Scan_Click(object sender, EventArgs e)
        {
            button_XY_Scan_Function();
            mSwitch_ShowComDdata = false;
        }
        void button_XY_Scan_Function()
        {
            //AA 55 43 53 53 00 00 00 55 AA 
            send_Data_Frame_To_Arduino('C', 'S', 'S');
            para_NumberOfFrameFinished = 0;
        }

        private void button_XY_pause_Click(object sender, EventArgs e)
        {
            send_Data_Frame_To_Arduino('C', 'S', 'P');
        }

        private void button_XY_Reset_Click(object sender, EventArgs e)
        {
            //AA 55 43 5A 57 00 00 00 55 AA 
            //AA 55 43 53 52 00 00 00 55 AA 
            send_Data_Frame_To_Arduino_SetSystemIdle_Multi();
            send_Data_Frame_To_Arduino('C', 'S', 'R');
            //with draw Z, stop Zloop PID, and stop xy scan and xy move to XL YL
        }

        private void button_CoarseLiftUp_Click(object sender, EventArgs e)
        {
            send_Data_Frame_To_Arduino('C', 'A', 'C');// with draw Z axis
            Thread.Sleep(10);
            //AFM_coarse_positioner_SetSpeed(2000);
            //AFM_coarse_positioner_MoveDistance(2, 5000000);// 5mm 
            mCCoarsePositioner.MoveDistance(mCaxis_z, 5000000, 2000);// lift up
        }

        private void button2_Click(object sender, EventArgs e)
        {
            MY_DEBUG("tres");
            SpeakVoice("I am test a word");
            SpeakVoice("you are rate");



            //
            //SoundNotice(5);
            // AFM_coarse_positioner_SetSpeed(2500);
            // AFM_coarse_positioner_MoveDistance(1, 800000);// % for safety reason, first move up 25*1.2 um
            //int k = 0;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;
            //SoundNotice(k, 2000); k++;



            //mCIniFile.WriteDouble("Indentation","MaximumDepth(nm)",1/3.0f);
            //double x = mCIniFile.ReadDouble("Indentation", "MaximumDepth(nm)");

            //float x = 10000.5654296875f;
            //byte[] b=BitConverter.GetBytes(x);

            //byte[] db = new byte[LENGTH_COM_BUFFER_MCU2PC * 2];

            //string X = serialPort_Arduino.ReadExisting();


            // MLApp.MLApp matlab = new MLApp.MLApp(); 
            //byte[] x = { 0xff, 0x01 };
            //convert_byte2_to_int16(x, 0);
            //byte[, ,] rgb = new byte[512, 512, 3];
            //double[,] im = new double[512, 512];
            //object Orgb, Oim, Oimin;
            //Orgb = (object)rgb;
            //Oim = (object)im;
            //Oimin = (object)mImageArrayHL;

        }

        ///------------------------------ save image start-----------------------

        private void button_SaveImage_Click(object sender, EventArgs e)
        { SaveImage_StartThread(); }
        void SaveImage_StartThread()
        {
            if (mThread_SaveImage.IsAlive == false)// avoid multi start
            {
                System.Media.SystemSounds.Exclamation.Play();// ok
                Thread.Sleep(300);
                System.Media.SystemSounds.Hand.Play();
                mThread_SaveImage = new Thread(ThreadFunction_SaveImage);// this will delete the previous thread
                mThread_SaveImage.Start();
            }
        }
        void ThreadFunction_SaveImage()
        {
            string t = DateTime.Now.ToString("yyyyMMddHHmmss");
            SaveImageToTextFile(mDataPath, t, "HL", mImageArrayHL);
            SaveImageToTextFile(mDataPath, t, "HR", mImageArrayHR);
            SaveImageToTextFile(mDataPath, t, "EL", mImageArrayEL);
            SaveImageToTextFile(mDataPath, t, "ER", mImageArrayER);
            SaveAFMParaToTextFile(t);

            //UpdateImageShow_SaveMat(t);

            t = null;
            SaveImageToTextFile(mDataPath, t, "HL", mImageArrayHL);
            SaveImageToTextFile(mDataPath, t, "HR", mImageArrayHR);
            SaveImageToTextFile(mDataPath, t, "EL", mImageArrayEL);
            SaveImageToTextFile(mDataPath, t, "ER", mImageArrayER);
            SaveAFMParaToTextFile(t);

        }

        double inc = 0;
        void SaveImageToTextFile(string path, string time, string name, double[,] image)
        {


            string Fpath = path + "AFM" + "_" + name + time + ".txt";
            //para_Nx = 128; para_Ny = 90;
            // StreamWriter writetext = new StreamWriter("write.txt");
            string text = null;
            for (int y = 0; y < para_Ny; y++)
            {
                for (int x = 0; x < para_Nx; x++)
                    text = text + Convert.ToString(image[x, y]) + "\t";
                // text = text + Convert.ToString(Math.Sin((x+inc++) / 20.0) * 100 + Math.Sin(y / 10.0) * 100) + "\t";
                text = text + "\r\n";
            }
            System.IO.File.WriteAllText(Fpath, text);
        }
        void SaveAFMParaToTextFile(string time)
        {
            string path = "AFM" + "_" + "parameter" + time + ".txt";
            string text = Convert.ToString(point_now_x + 1) + "\t%point_now_x\r\n"
                    + Convert.ToString(point_now_y + 1) + "\t%point_now_y\r\n"
                    + Convert.ToString(para_Nx) + "\t%N_x\r\n"
                    + Convert.ToString(para_Ny) + "\t%N_y\r\n"
                    + para_Dx.ToString() + "\t%Dx\r\n"
                    + para_Dy.ToString() + "\t%Dy\r\n"
                    + para_XL.ToString() + "\t%XL\r\n"
                    + para_YL.ToString() + "\t%YL\r\n"
                    + para_ScanRate.ToString() + "\t%ScanRate\r\n"
                    + para_Sensitivity.ToString() + "\t%Sensitivity\r\n"
                    + para_SetDeltaVoltage_mV.ToString() + "\t%SetDeltaVoltage\r\n"
                    + para_Z_PID_P.ToString() + "\t%Z_PID_P\r\n"
                    + para_Z_PID_I.ToString() + "\t%Z_PID_I\r\n"
                    + para_Z_PID_D.ToString() + "\t%Z_PID_D\r\n"
                    + para_NumberOfFrameToScan.ToString() + "\t%NumberOfFrameToScan\r\n"
                    + para_TF_DC_Gain.ToString() + "\t%TF_DC_Gain\r\n"

                    ;
            for (int k = 0; k < 4; k++)
                text += para_IC0_DR[k].ToString() + "\t%para_IC0_DR_" + k.ToString("D") + "\r\n";

            System.IO.File.WriteAllText(path, text);
        }
        public static T[,] GetNew2DArray<T>(int x, int y, T initialValue)
        {
            T[,] nums = new T[x, y];
            for (int i = 0; i < x * y; i++) nums[i % x, i / x] = initialValue;
            return nums;
        }
        private void button_ClearImage_Click(object sender, EventArgs e)
        {
            ImageArray_ValueReset();
        }
        void ImageArray_SizeReset()
        {
            mImageArrayHL = new double[(int)para_Nx, (int)para_Ny];
            mImageArrayEL = new double[(int)para_Nx, (int)para_Ny];
            mImageArrayHR = new double[(int)para_Nx, (int)para_Ny];
            mImageArrayER = new double[(int)para_Nx, (int)para_Ny];
        }
        void ImageArray_ValueReset(double initial_value = -1)
        {
            int L = (int)(para_Nx * para_Ny);

            mImageArrayHL = GetNew2DArray((int)para_Nx, (int)para_Ny, initial_value);// initial value =-1
            mImageArrayHR = GetNew2DArray((int)para_Nx, (int)para_Ny, initial_value);// initial value =-1
            mImageArrayEL = GetNew2DArray((int)para_Nx, (int)para_Ny, initial_value);// initial value =-1
            mImageArrayER = GetNew2DArray((int)para_Nx, (int)para_Ny, initial_value);// initial value =-1

            //Array.Clear(mImageArrayEL, 0, L);
            //Array.Clear(mImageArrayER, 0, L);
            //Array.Clear(mImageArrayHL, 0, L);
            //Array.Clear(mImageArrayHR, 0, L);
        }
        //------------------------------ save image end-----------------------
        private void checkBox_ShowImage_CheckedChanged(object sender, EventArgs e)
        {
            mSWitchShowImage = checkBox_ShowImage.Checked;
        }

        private void button_StartSubWindow_Click(object sender, EventArgs e)
        {
            ParameterSettingForm mParameterSettingForm = new ParameterSettingForm(this);
            mParameterSettingForm.Show();
        }

        private void button_SetScanROI_Click(object sender, EventArgs e)
        {
            //AFM_set_scan_ROI();

            Form_ImageShow_DrawROI mForm_ImageShow_DrawROI = new Form_ImageShow_DrawROI(this);
            mForm_ImageShow_DrawROI.Show();
        }
        public void AFM_set_scan_ROI()
        {


            //double ref para_XL,
            //double ref para_XL,
            //double ref para_XL,
            //double ref para_XL,

            //string in_str = in_str = "out_data=load(\'AFM_parameter.txt\');";//'out_data=load(''AFM_parameter.txt\'');'
            //object Oin_str = (object)in_str;
            //string out_str = null;
            //object Oout_str = (object)out_str;
            double[] in_data = new double[] { para_XL, para_YL, para_Dx, para_Dy, 0 };
            object Oin_data = (object)in_data;
            double[,] out_data = new double[NumberOfParameters, 1];
            object Oout_data = (object)out_data;

            object OmImageArrayHL = (object)mImageArrayHL;
            mKernelClass.AFM_scan_set_ROI(1, ref Oout_data, OmImageArrayHL, Oin_data);

            out_data = (double[,])Oout_data;
            int k = 1;
            para_XL = out_data[k++, 1]; para_YL = out_data[k++, 1]; para_Dx = out_data[k++, 1]; para_Dy = out_data[k++, 1];
            UpdateGUITextBox_Invoke(ref para_Dx, textBox_Dx);//, 1, MAX_RANGE_X_NM);
            UpdateGUITextBox_Invoke(ref para_Dy, textBox_Dy);//, 1, MAX_RANGE_Y_NM);
            UpdateGUITextBox_Invoke(ref para_XL, textBox_XL);//, 1, MAX_RANGE_X_NM);
            UpdateGUITextBox_Invoke(ref para_YL, textBox_YL);//, 1, MAX_RANGE_Y_NM);
        }

        private void button_Start_IndentationWindow_Click(object sender, EventArgs e)
        {

            button_StartIndent_Click(sender, e);
            //Form_Indentation mForm_Indentation = new Form_Indentation(this);
            //if( mForm_Indentation.IsDisposed)
            //    mForm_Indentation = new Form_Indentation(this);
            //mForm_Indentation.Show();
            //mForm_Indentation.BringToFront();

        }

        private void trackBar_R01_Scroll(object sender, EventArgs e)
        {
            byte v = Convert.ToByte(trackBar_R01.Value);
            textBox_IC0_R0.Text = Convert.ToString(v);
            textBox_IC0_R1.Text = Convert.ToString(v);
            send_DR_Value(0, 0, v);
            send_DR_Value(0, 1, v);
        }

        private void checkBox_T_ScanEnable_CheckedChanged(object sender, EventArgs e)
        {
            char ch;
            if (checkBox_T_ScanEnable.Checked == true)
                ch = 'E';
            else
                ch = 'D';
            send_Data_Frame_To_Arduino('C', 'T', ch);
        }

        private void button_MultiTask_Click(object sender, EventArgs e)
        {
            mThread_MultiTask = new Thread(ThreadFunction_MultiTask);// this will delete the previous thread
            mThread_MultiTask.Start();
        }

        void ThreadFunction_MultiTask()
        {
            int N = 1; // number of points
            int P = 1;// P times for each point
            for (int n = 0; n < N; n++)
            {
                for (int p = 0; p < P; p++)
                {
                    Indent_StartThread();

                    int time = (int)(Convert.ToDouble(textBox_TaskTime.Text) * 1000);
                    Thread.Sleep(time);
                    P = Convert.ToInt32(textBox_TaskNumber.Text);
                    N = Convert.ToInt32(textBox_Task_PointsNumber.Text);
                }
                SoundNotice(4, 1000);
                //AFM_coarse_positioner_SetSpeed(250);
                //AFM_coarse_positioner_MoveDistance(1, -20000);// % for safety reason, first move up 25*1.2 um
                SoundNotice(5, 1000);
                //Thread.Sleep(5000);
            }
            SoundNotice(4, 1000);
            SoundNotice(6, 1000);
        }

        private void button_Form_CoarsePositioner_Click(object sender, EventArgs e)
        {
            Form_CoarsePositioner mForm_CoarsePositioner = new Form_CoarsePositioner(this);
            mForm_CoarsePositioner.Show();
        }
    }


}
