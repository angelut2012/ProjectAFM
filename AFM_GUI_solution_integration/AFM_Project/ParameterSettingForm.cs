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
            listBox_ScannerMode.SelectedIndex = 0;
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
            double value = (double)trackBar_PositionOutput.Value / (double)trackBar_PositionOutput.Maximum;
            double value_01 = value;
            //byte axis = Convert.ToByte(textBox_Position_Axis.Text);
            int axis = listBox_ScannerAxis.SelectedIndex;

            if (listBox_ScannerMode.SelectedIndex == 0)// DAC
            {
                textBox_Position_Value.Text = value.ToString();
                textBox_DAC_Value.Text = Convert.ToString(value * 150);

                value = Convert.ToDouble(textBox_DAC_Value.Text);
                value = value * 5.0 / 150.0;
                //pParent.set_output_parameters('D', (byte)axis, (UInt32)value);
                pParent.set_output_DAC_Value_0_5((byte)axis, value);

            }

            if (listBox_ScannerMode.SelectedIndex == 1)
            {
                pParent.set_output_Position_Value_01(axis, value_01);
            }
            if (listBox_ScannerMode.SelectedIndex == 2)
            {
                pParent.set_output_Position_Value_01(axis + 100, value_01);
            }
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
            double[] p101 = { 0.100000000000000, 0.116000000000000, 0.132000000000000, 0.148000000000000, 0.164000000000000, 0.180000000000000, 0.196000000000000, 0.212000000000000, 0.228000000000000, 0.244000000000000, 0.260000000000000, 0.276000000000000, 0.292000000000000, 0.308000000000000, 0.324000000000000, 0.340000000000000, 0.356000000000000, 0.372000000000000, 0.388000000000000, 0.404000000000000, 0.420000000000000, 0.436000000000000, 0.452000000000000, 0.468000000000000, 0.484000000000000, 0.500000000000000, 0.516000000000000, 0.532000000000000, 0.548000000000000, 0.564000000000000, 0.580000000000000, 0.596000000000000, 0.612000000000000, 0.628000000000000, 0.644000000000000, 0.660000000000000, 0.676000000000000, 0.692000000000000, 0.708000000000000, 0.724000000000000, 0.740000000000000, 0.756000000000000, 0.772000000000000, 0.788000000000000, 0.804000000000000, 0.820000000000000, 0.836000000000000, 0.852000000000000, 0.868000000000000, 0.884000000000000, 0.900000000000000, 0.884000000000000, 0.868000000000000, 0.852000000000000, 0.836000000000000, 0.820000000000000, 0.804000000000000, 0.788000000000000, 0.772000000000000, 0.756000000000000, 0.740000000000000, 0.724000000000000, 0.708000000000000, 0.692000000000000, 0.676000000000000, 0.660000000000000, 0.644000000000000, 0.628000000000000, 0.612000000000000, 0.596000000000000, 0.580000000000000, 0.564000000000000, 0.548000000000000, 0.532000000000000, 0.516000000000000, 0.500000000000000, 0.484000000000000, 0.468000000000000, 0.452000000000000, 0.436000000000000, 0.420000000000000, 0.404000000000000, 0.388000000000000, 0.372000000000000, 0.356000000000000, 0.340000000000000, 0.324000000000000, 0.308000000000000, 0.292000000000000, 0.276000000000000, 0.260000000000000, 0.244000000000000, 0.228000000000000, 0.212000000000000, 0.196000000000000, 0.180000000000000, 0.164000000000000, 0.148000000000000, 0.132000000000000, 0.116000000000000 };
            double[] p2 = { 0.2, 0.8 };

            double[] p = p2;
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

                    //if (listBox_ScannerMode.SelectedIndex == 2)
                    //    pParent.set_output_Position_Value_01(axis_wave + 100, 0.8);                 
                    //if (listBox_ScannerMode.SelectedIndex == 0)
                    //    pParent.set_output_DAC_Value_0_5(axis_wave, 5);
                    //Thread.Sleep(dt);

                    //if (listBox_ScannerMode.SelectedIndex == 2)
                    //    pParent.set_output_Position_Value_01(axis_wave + 100, 0.2);
                    //if (listBox_ScannerMode.SelectedIndex == 0)
                    //    pParent.set_output_DAC_Value_0_5(axis_wave, 0);
                    //Thread.Sleep(dt);



                    //for (int k = 0; k < p.Length; k++)
                    //{
                    //    pParent.set_output_Position_Value_01(axis_wave + 100, p[k]);
                    //    Thread.Sleep(dt);
                    //    if (mWaveRun == false)
                    //        break;
                    //}




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

        private void button_Test_Click(object sender, EventArgs e)
        {
            int dt = Convert.ToInt32(textBox_T_test_dt.Text);
            axis_wave = (byte)listBox_ScannerAxis.SelectedIndex;

            const double SCANNER_RANGE_Z_NM = 9365;
            const double SCANNER_RANGE_X_NM = 45748;
            const double SCANNER_RANGE_Y_NM = 39800;

            double step = 1000.0 / SCANNER_RANGE_X_NM;

           
            pParent.set_output_Position_Value_01(axis_wave + 100, 0.5 + step);
            Thread.Sleep(dt);
            pParent.set_output_Position_Value_01(axis_wave + 100, 0.5);
            Thread.Sleep(dt);
        }

    }
}