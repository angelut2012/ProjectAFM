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

namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form
    {
        //public Indentation()
        //{
        //    InitializeComponent();
        //}
        Thread mThread_Indent;
        Thread mThread_MultiTask;

        bool mSwitch_CancelIndent = false;

        MainWindow pParent;
        public void Form_Indentation(MainWindow pmain)
        {
            // InitializeComponent();

            mThread_Indent = new Thread(ThreadFunction_Indent);
            pParent = pmain;
            //pParent.send_Data_Frame_To_Arduino_SetSystemIdle_Multi();
        }

        private void button_CancelIndent_Click(object sender, EventArgs e)
        {
            string x = pParent.mCParameter.ToString();
            string y = x.ToString();
            mSwitch_CancelIndent = true;
            pParent.send_Data_Frame_To_Arduino_SetSystemIdle_Multi();
        }

        private void button_StartIndent_Click(object sender, EventArgs e)
        {
            Indent_StartThread();
        }

        void Indent_StartThread()
        {
            if (mThread_Indent.IsAlive == false)// avoid multi start
            {               
                mThread_Indent = new Thread(ThreadFunction_Indent);// this will delete the previous thread
                mThread_Indent.Start();
                System.Media.SystemSounds.Exclamation.Play();// ok
                Thread.Sleep(300);
                System.Media.SystemSounds.Hand.Play();

            }
        }

        void ThreadFunction_Indent()
        {
            mSwitch_Indenting1_SavingData2_Idle0 = 1;
            pParent.send_Data_Frame_To_Arduino_SetSystemIdle_Multi();

            pParent.set_AFM_parameters('T', pParent.mCParameter.TriggerForce_nN);
            pParent.set_AFM_parameters('k', pParent.mCParameter.PRCStiffness_nN_per_nm);
            pParent.set_AFM_parameters('s', pParent.mCParameter.PRC_ADCValue_per_nm);
            pParent.set_AFM_parameters('d', pParent.mCParameter.MaxIndentationDepth_nm);
            pParent.set_AFM_parameters('e', pParent.mCParameter.StepSize_nm);
            pParent.set_AFM_parameters('f', pParent.mCParameter.LoopDelay_uS - CParameter.mI_LoopDelay_Min_uS);
            pParent.set_AFM_parameters('g', pParent.mCParameter.NumberOfSamplingPoints);

            pParent.mIndentData = new double[3, 6000];//25*1024
            pParent.mIndentData_index = -1;
            mSwitch_CancelIndent = false;
            pParent.mSwitch_Indenting1_SavingData2_Idle0 = 1;
            Thread.Sleep(500);
            pParent.send_Data_Frame_To_Arduino('C', 'I', 'D');
            while (pParent.mSwitch_Indenting1_SavingData2_Idle0 == 1)
            {
                Thread.Sleep(5);
                if (mSwitch_CancelIndent == true)
                    return;
            }

            mSwitch_Indenting1_SavingData2_Idle0 = 2;
            string paras = pParent.mCParameter.GetParaString()
                + pParent.para_XL.ToString() + "%XL\r\n"
                + pParent.para_YL.ToString() + "%YL\r\n"
                + "\r\n";

            SaveIndentDataToTextFile(mDataPath,
                "_" + pParent.textBox_FileName.Text,
                "_" + pParent.GetCurrentTimeString(),
                paras,
                //"_step_size" + step_size.ToString() +
                //"_start_position" + start_position.ToString() +
                //"_depth" + depth.ToString() +
                //"_time" + pParent.TOC().ToString() + "_",
                pParent.mIndentData,
                pParent.mIndentData.Rank + 1,
                pParent.mIndentData_index
                );

            SaveIndentDataToTextFile(mDataPath, null,
                null,
                paras,
                pParent.mIndentData,
                pParent.mIndentData.Rank + 1,
                pParent.mIndentData_index);
            //button_StartIndent.Enabled = true;

            System.Media.SystemSounds.Hand.Play();
            Thread.Sleep(300);
            System.Media.SystemSounds.Exclamation.Play();// ok
            //MessageBox.Show("done");

            Thread.Sleep(800);
            pParent.send_Data_Frame_To_Arduino_SetSystemIdle_Multi();

            mSwitch_Indenting1_SavingData2_Idle0 = 0;
            //try
            //{
            //    // ShowIndentData();
            //}
            //catch { MessageBox.Show("show indentation data error"); }

        }

        //void ThreadFunction_Indent_old()
        //{
        //    //button_StartIndent.Enabled = false;
        //    double depth = Convert.ToDouble(textBox_Indent_Depth.Text);
        //    double step_size = Convert.ToDouble(textBox_Indent_StepSize.Text);
        //    double start_position = Convert.ToDouble(textBox_Indent_StartPosition.Text);

        //    int length = (int)(depth / step_size) * 2;
        //    pParent.mIndentData = new double[3, length];
        //    pParent.mIndentData_index = 0;

        //    pParent.send_Data_Frame_To_Arduino('C', 'I', 'D');

        //    double v = 0;
        //    const double step = 40;//nm
        //    const int slow_move_time_interval = 15;// ms
        //    while (v < start_position - step)
        //    {
        //        v += step;
        //        pParent.set_output_Position_Value_01(0, v);
        //        Thread.Sleep(slow_move_time_interval);
        //        pParent.MY_DEBUG(v.ToString());
        //    }

        //    double value = start_position;
        //    double direction = 1;
        //    pParent.TIC();
        //    mSwitch_CancelIndent = false;
        //    pParent.set_output_Position_Value_01(0, value);
        //    for (int k = 0; k < length; k++)
        //    {
        //        if (mSwitch_CancelIndent == true) break;

        //        if (k >= length / 2) direction = -1;
        //        value += step_size * direction;
        //        pParent.mIndentData_index = k;
        //        pParent.set_output_Position_Value_01(0, value);
        //        Thread.Sleep(10);// must wait 10 ms for enough data to come back
        //        pParent.MY_DEBUG("indent:" + value.ToString()
        //            + "\tadc:" + pParent.mIndentData[1, k].ToString());
        //    }

        //    ///////////slow withdraw
        //    //v = start_position;
        //    ////const double step = 10;//nm
        //    //while (v > step)
        //    //{
        //    //    v -= step;
        //    //    pParent.set_output_Position_Value_01(0, v);
        //    //    Thread.Sleep(slow_move_time_interval);
        //    //    pParent.MY_DEBUG(v.ToString());
        //    //}
        //    //pParent.set_output_Position_Value_01(0, 0);

        //    if (mSwitch_CancelIndent == true) return;

        //    string paras=pParent.mCParameter.GetParaString();
        //    SaveIndentDataToTextFile(pParent.GetCurrentTimeString(),
        //        pParent.textBox_FileName.Text,
        //        paras,
        //        //"_step_size" + step_size.ToString() +
        //        //"_start_position" + start_position.ToString() +
        //        //"_depth" + depth.ToString() +
        //        //"_time" + pParent.TOC().ToString() + "_",
        //        pParent.mIndentData,
        //        pParent.mIndentData.Rank + 1,
        //        pParent.mIndentData_index
        //        );

        //    SaveIndentDataToTextFile(null,
        //        null,
        //        paras,
        //        pParent.mIndentData,
        //        pParent.mIndentData.Rank + 1,
        //        pParent.mIndentData_index);
        //    //button_StartIndent.Enabled = true;

        //    System.Media.SystemSounds.Hand.Play();
        //    Thread.Sleep(300);
        //    System.Media.SystemSounds.Exclamation.Play();// ok
        //    //MessageBox.Show("done");

        //    Thread.Sleep(800);
        //    pParent.send_Data_Frame_To_Arduino_SetSystemIdle_Multi();

        //    try
        //    {
        //        ShowIndentData();
        //    }
        //    catch { MessageBox.Show("show data error"); }


        //}


        void SaveIndentDataToTextFile(string path, string name, string time, string paras, double[,] data)
        {
            int para_Ny = data.Length / (data.Rank + 1);
            int para_Nx = data.Rank + 1;
            SaveIndentDataToTextFile(path, name, time, paras, data, para_Nx, para_Ny);
        }
        void SaveIndentDataToTextFile(string path, string name, string time, string paras, double[,] data, int para_Nx, int para_Ny)
        {
            string Fpath = path + "IndentData" + name + time + ".txt";
            //para_Nx = 128; para_Ny = 90;
            // StreamWriter writetext = new StreamWriter("write.txt");
            string text = paras;
            //int para_Ny = data.Length/(data.Rank+1);
            //int para_Nx = data.Rank+1;
            for (int y = 0; y < para_Ny; y++)
            {
                for (int x = 0; x < para_Nx; x++)
                    text = text + Convert.ToString(data[x, y]) + "\t";
                // text = text + Convert.ToString(Math.Sin((x+inc++) / 20.0) * 100 + Math.Sin(y / 10.0) * 100) + "\t";
                text = text + "\r\n";
            }
            AFM_Write_StringDatatoText(Fpath, text);
        }
        void AFM_Write_StringDatatoText(string Fpath, string text)
        {
            try
            {
                System.IO.File.WriteAllText(Fpath, text);
            }
            catch
            {
                System.IO.File.WriteAllText(Fpath+"backup", text);
            }
            
        }
        void ShowIndentData()
        {
            string in_str = in_str = "out_data=1;AFM_show_indent_data();";
            object Oin_str = (object)in_str;
            string out_str = null;
            object Oout_str = (object)out_str;
            double in_data = 0;
            object Oin_data = (object)in_data;
            double[] out_data = new double[1];
            object Oout_data = (object)out_data;
            try
            {
                //pParent.mKernelClass.StringEval(2, ref Oout_str, ref Oout_data, Oin_str, Oin_data);
            }
            catch
            {
                pParent.MY_DEBUG("ShowindentData Error in MKernel.");
            }
        }

        //----------------------------------------data capture max
        Thread mThread_FastDataCapture;
        void StartThread_FastDataCapture()
        {
            mThread_FastDataCapture = new Thread(ThreadFunction_FastDataCapture);// this will delete the previous thread
            mThread_FastDataCapture.Start();

        }

        void ThreadFunction_FastDataCapture()
        {

            pParent.mIndentData = new double[3, 20000];//25*1024
            pParent.mIndentData_index = -1;

            mSwitch_CancelIndent = false;
            pParent.mSwitch_Indenting1_SavingData2_Idle0 = 1;

            //pParent.send_Data_Frame_To_Arduino('C', 'I', 'D');
            send_CMD_PC2MCU(CMD_PC2MCU.CMD_PC2MCU_DATA_CAPTURE, 0);
            while (pParent.mSwitch_Indenting1_SavingData2_Idle0 == 1)
            {
                Thread.Sleep(5);
                if (mSwitch_CancelIndent == true)
                    return;
            }

            string paras = pParent.mCParameter.GetParaString()
                + pParent.para_XL.ToString() + "%XL\r\n"
                + pParent.para_YL.ToString() + "%YL\r\n"
                + "\r\n";

            SaveIndentDataToTextFile(mDataPath,
                "_" + pParent.textBox_FileName.Text,
                "_" + pParent.GetCurrentTimeString(),
                paras,
                //"_step_size" + step_size.ToString() +
                //"_start_position" + start_position.ToString() +
                //"_depth" + depth.ToString() +
                //"_time" + pParent.TOC().ToString() + "_",
                pParent.mIndentData,
                pParent.mIndentData.Rank + 1,
                pParent.mIndentData_index
                );

            SaveIndentDataToTextFile(mDataPath, null,
                null,
                paras,
                pParent.mIndentData,
                pParent.mIndentData.Rank + 1,
                pParent.mIndentData_index);
            //button_StartIndent.Enabled = true;

            System.Media.SystemSounds.Hand.Play();
            Thread.Sleep(300);
            System.Media.SystemSounds.Exclamation.Play();// ok
        }
     

    }
}
