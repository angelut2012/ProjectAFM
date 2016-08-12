using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;

namespace NameSpace_AFM_Project
{
    public partial class ParameterSettingForm : Form
    {
        MainWindow pParent;
        bool mWaveRun = false;
        public ParameterSettingForm(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;
            listBox_ScannerAxis.SelectedIndex = 0;
        }

        private void button_DAC_Output_Click(object sender, EventArgs e)
        {
            //for (k = 0; k < listBox_ScannerAxis.Items.Count; k++)
            //     if (listBox_ScannerAxis.GetSelected(k))
            //         break;
            int axis = listBox_ScannerAxis.SelectedIndex;//Convert.ToInt32(textBox_DAC_Axis.Text);
            double value = Convert.ToDouble(textBox_DAC_Value.Text);
            value = value * 5.0 / 150.0;
            //pParent.set_output_parameters('D', (byte)axis, (UInt32)value);
            pParent.set_output_DAC_Value_0_5((byte)axis, value);
        }

        private void button_SetPosition_OpenLoop_Click(object sender, EventArgs e)
        {
            int axis = listBox_ScannerAxis.SelectedIndex;
            double value_01 = Convert.ToDouble(textBox_Position_Value.Text);
            pParent.set_output_Position_Value_01(axis, value_01);
        }

        private void button_Read_StrainGauge_Continue_Click(object sender, EventArgs e)
        {
            pParent.send_Data_Frame_To_Arduino('G', 's', 'g', 2);
        }

        private void button_Read_StrainGauge_Once_Click(object sender, EventArgs e)
        {
            pParent.send_Data_Frame_To_Arduino('G', 's', 'g', 1);
        }

        private void button_Read_StrainGauge_Stop_Click(object sender, EventArgs e)
        {
            pParent.send_Data_Frame_To_Arduino('G', 's', 'g', 0);
        }

        private void trackBar_PositionZ_Scroll(object sender, EventArgs e)
        {
            double value = (double)trackBar_PositionZ.Value / (double)trackBar_PositionZ.Maximum;
            //byte axis = Convert.ToByte(textBox_Position_Axis.Text);
            int axis = listBox_ScannerAxis.SelectedIndex;
            textBox_Position_Value.Text = value.ToString();
            textBox_DAC_Value.Text = Convert.ToString(value * 150);

            value = Convert.ToDouble(textBox_DAC_Value.Text);
            value = value * 5.0 / 150.0;
            //pParent.set_output_parameters('D', (byte)axis, (UInt32)value);
            pParent.set_output_DAC_Value_0_5((byte)axis, value);
        }

        Thread mThread_wave_generator;
        byte axis_wave;
        private void button_T_debug_Click(object sender, EventArgs e)
        {
            mWaveRun = true;
            axis_wave = (byte)listBox_ScannerAxis.SelectedIndex;
            mThread_wave_generator = new Thread(function_wave_generator);
            mThread_wave_generator.Start();
        }

        void function_wave_generator()
        {
            while (mWaveRun == true)
            {
                int N = Convert.ToInt32(textBox_T_Test_cycles.Text);
                int dt = Convert.ToInt32(textBox_T_test_dt.Text);
                //byte axis = Convert.ToByte(textBox_Position_Axis.Text);
                //for (int k = 0; k < N; k++)
                {
                    //pParent.set_output_DAC_Value_0_5(axis_wave, 5);
                    //Thread.Sleep(dt);
                    //pParent.set_output_DAC_Value_0_5(axis_wave, 0);
                    //Thread.Sleep(dt);

                    pParent.set_output_Position_Value_01(axis_wave + 100, 0.2);
                    Thread.Sleep(dt);
                    pParent.set_output_Position_Value_01(axis_wave + 100, 0.8);
                    Thread.Sleep(dt);


                }
            }
        }
        //for (int k = 50; k > -50; k--)
        //{
        //    pParent.set_output_DAC_Value_0_5(axis, (80.0+k/4.0)*5.0 / 150.0);
        //    Thread.Sleep(dt);
        //    pParent.MY_DEBUG("V\t" + (80 + k/4.0).ToString() + "\tP\t" + pParent.Z_position_now.ToString());

        //}
        //for (int k = -50; k < 50; k++)
        //{
        //    pParent.set_output_DAC_Value_0_5(axis, (80.0+k/4.0)*5.0 / 150.0);
        //    Thread.Sleep(dt);
        //    pParent.MY_DEBUG("V\t" + (80 + k/4.0).ToString() + "\tP\t" + pParent.Z_position_now.ToString());

        //}
        //for (int k = 50; k > -50; k--)
        //{
        //    pParent.set_output_DAC_Value_0_5(axis, (80.0+k/4.0)*5.0 / 150.0);
        //    Thread.Sleep(dt);
        //    pParent.MY_DEBUG("V\t" + (80 + k/4.0).ToString() + "\tP\t" + pParent.Z_position_now.ToString());

        //}
        // pParent.set_output_DAC_Value_0_5(axis, (80.0 ) / 150.0 * 5.0);
        //for (int k = 0; k < N;k++ )
        //{
        //    pParent.set_output_DAC_Value_0_5(axis, 80 / 150 * 5);
        //    Thread.Sleep(dt);
        //    pParent.set_output_DAC_Value_0_5(axis, 90 / 150 * 5);
        //    Thread.Sleep(dt);
        //    pParent.set_output_DAC_Value_0_5(axis, 80 / 150 * 5);
        //    Thread.Sleep(dt);
        //    pParent.set_output_DAC_Value_0_5(axis, 70 / 150 * 5);
        //    Thread.Sleep(dt);
        //}
        //Thread.Sleep(dt*3);
        //pParent.set_output_DAC_Value_0_5(axis, 80/150*5);


        private void button_WaveStop_Click(object sender, EventArgs e)
        {
            mWaveRun = false;
        }

        private void button_SetPositioin_CloseLoop_Click(object sender, EventArgs e)
        {
            //int axis = Convert.ToInt32(textBox_Position_Axis.Text);
            int axis = listBox_ScannerAxis.SelectedIndex;
            double value_01 = Convert.ToDouble(textBox_Position_Value.Text);
            pParent.set_output_Position_Value_01(axis + 100, value_01);
        }

        private void button_P_Click(object sender, EventArgs e)
        {
            int axis = listBox_ScannerAxis.SelectedIndex;
            double value_01 = Convert.ToDouble(textBox_Position_Value.Text);
            value_01 += 0.1;
            value_01 = pParent.LIMIT_MAX_MIN(value_01, 1, 0);
            textBox_Position_Value.Text = value_01.ToString();
            pParent.set_output_Position_Value_01(axis + 100, value_01);
        }

        private void button_N_Click(object sender, EventArgs e)
        {
            int axis = listBox_ScannerAxis.SelectedIndex;
            double value_01 = Convert.ToDouble(textBox_Position_Value.Text);
            value_01 -= 0.1;
            value_01 = pParent.LIMIT_MAX_MIN(value_01, 1, 0);
            textBox_Position_Value.Text = value_01.ToString();
            pParent.set_output_Position_Value_01(axis + 100, value_01);
        }

    }
}