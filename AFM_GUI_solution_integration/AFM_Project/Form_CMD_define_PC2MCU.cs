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
    public struct CMD_PC2MCU
    {
	// to be compitable with c#
private 	const int CMD_PC2MCU_BASE =  (32768);//2^15
public  const int CMD_PC2MCU_DATA_CAPTURE	=  (CMD_PC2MCU_BASE + 1);
public  const int CMD_PC2MCU_WAVE_TEST	=  (CMD_PC2MCU_BASE + 2);
public  const int CMD_PC2MCU_SCSG_RANGE_CALIBRATION	=  (CMD_PC2MCU_BASE + 3);
public 	const int CMD_PC2MCU_StartZLoop	=  (CMD_PC2MCU_BASE + 4);
public 	const int CMD_PC2MCU_InitialParameter_ZLoop	=  (CMD_PC2MCU_BASE + 5);
    }

    public partial class MainWindow : Form
    {
        public void send_CMD_PC2MCU(int data_name, double data_value)
        {
            byte cmd0 = (byte)(data_name >> 8);
            byte cmd1 = (byte)((UInt16)data_name & 0x00ff);

            //byte[] db4 = new byte[ ] {0,0,0,0 };
            //convert_uint32_to_byte4((uint)data_value,db4);// use com[2345]as data 32bit
            float fdata = Convert.ToSingle(data_value);
            byte[] db4 = BitConverter.GetBytes(fdata);
            send_Data_Frame_To_Arduino(cmd0, cmd1, db4[0], db4[1], db4[2], db4[3]);
            Thread.Sleep(50);
        }

        public void AFM_InitialParameter_ZLoop()
        {
            send_CMD_PC2MCU(CMD_PC2MCU.CMD_PC2MCU_InitialParameter_ZLoop, 0);
        }

        private void button_ScannerCalibration_Click(object sender, EventArgs e)
        {
            move_CoarseZLiftUp_FineZWidthdraw();
            send_CMD_PC2MCU(CMD_PC2MCU.CMD_PC2MCU_SCSG_RANGE_CALIBRATION, 0);
        }

        //----------------------------------------------------------

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

        void set_AFM_parameters(char parameter_name, ref double para_store, TextBox T)    //,double  low_limit,double up_limit)
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
            T.Invoke((MethodInvoker)delegate()
            {
                T.Text = Convert.ToString(value);}
                );

            set_AFM_parameters(parameter_name, ref para_store, T);
        }

        public void set_output_Position_Value_01(int in_axis, double value_01)
        {
            value_01 = LIMIT_MAX_MIN(value_01, 1, 0);
            UInt32 value_position_FF32 = (UInt32)(value_01 * BIT32MAX);
            set_output_parameters('F', in_axis, value_position_FF32);
        }

        public void set_output_DAC_Value_0_5(byte axis, double value0_5)
        {
            value0_5 = value0_5 * BIT18MAX / 5.0;
            set_output_parameters('D', axis, (UInt32)value0_5);
        }

        public void set_output_parameters(char parameter_name, int axis, UInt32 value)//,double  low_limit,double up_limit)
        {
            // double data = Math.Abs(Convert.ToDouble(T.Text));
            //double Ggain = 1000.0;// keep 3 decimal
            //int value = (int)(data * Ggain);
            byte[] Bvalue = new byte[4] { 0, 0, 0, 0 };
            convert_uint32_to_byte4(value, Bvalue);
            send_Data_Frame_To_Arduino(parameter_name, (char)axis, Bvalue);
        }

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


        byte[] mDataBuffer_ThreadWriteSerial;     // = new byte[LENGTH_COM_FRAME_PC2MCU];
        public void send_Data_Frame_To_Arduino(byte d0, byte d1 = 0, byte d2 = 0, byte d3 = 0, byte d4 = 0, byte d5 = 0)
        {
            //AA 55 52 00 03 aa 00 00 55 AA 
            //byte[]
            mDataBuffer_ThreadWriteSerial = new byte[LENGTH_COM_FRAME_PC2MCU] 
                        { COM_HEADER1, COM_HEADER2, 
                         d0,d1,d2,d3,d4,d5,
                            // (byte)'R', 0, 0, 0,0,0,// 6 byte
                         COM_TAIL1, COM_TAIL2 };
            int wait_count = 10;
            while (mB_serialVirtual_Arduino_busy == true)
            {
                if (wait_count-- < 0)
                {
                    MY_DEBUG("serial write busy, aborted.");
                    break;// to avoid wait forever
                }
                Thread.Sleep(20);
                MY_DEBUG("serial write busy, continue to wait.");
            }

            {
                mThread_WriteSerialData = new Thread(ThreadFunction_WriteSerialData);
                mThread_WriteSerialData.Start();
            }
            //if (serialPort_Arduino.IsOpen == true)
            //{
            //try
            //{

            //mB_serialVirtual_Arduino_busy = true;
            //serialPort_Arduino.Write(com, 0, LENGTH_COM_FRAME_PC2MCU);

            // mThread_WriteSerialData.;// do not block the main thread
            //mB_serialVirtual_Arduino_busy = false;

            //Thread.Sleep(15);
            //}
            //catch
            //{
            //    //Thread.Sleep(100);
            //    //serialPort_Arduino.Write(com, 0, LENGTH_COM_FRAME_PC2MCU);
            //    MY_DEBUG("serialPort_Arduino.Write time out");
            //}

            //MessageBox.Show("MCU port is not connected.", "Error");
        }

        void ThreadFunction_WriteSerialData()
        {
            //MY_DEBUG("Thread Write Serial Data: start");
            if (serialPort_Arduino.IsOpen == true)
            {
                try
                {
                    mB_serialVirtual_Arduino_busy = true;
                    serialPort_Arduino.Write(mDataBuffer_ThreadWriteSerial, 0, LENGTH_COM_FRAME_PC2MCU);
                    mB_serialVirtual_Arduino_busy = false;
                }
                catch
                {
                    //Thread.Sleep(100);
                    //serialPort_Arduino.Write(com, 0, LENGTH_COM_FRAME_PC2MCU);
                    MY_DEBUG("Thread serialPort_Arduino.Write time out");
                }
            }
            else
                MY_DEBUG("Thread, MCU port is not connected.");
            //  MY_DEBUG("Thread Write Serial Data: end");
        }
    }
}