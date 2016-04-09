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
        public ParameterSettingForm(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;
        }

        private void button_DAC_Output_Click(object sender, EventArgs e)
        {
            int axis = Convert.ToInt32(textBox_DAC_Axis.Text);
            double value = Convert.ToDouble(textBox_DAC_Value.Text);
            value = value * 5.0 / 150.0;
            //pParent.set_output_parameters('D', (byte)axis, (UInt32)value);
           pParent.set_output_DAC_Value_0_5((byte)axis, value);
        }

        private void button_Position_Output_Click(object sender, EventArgs e)
        {
            int axis = Convert.ToInt32(textBox_Position_Axis.Text);
            double value_nm = Convert.ToDouble(textBox_Position_Value.Text);
            //value = value * 5.0 / 150.0;

            double value_01 = value_nm / pParent.MAX_RANGE_AXIS_NM[axis];
            pParent.set_output_Position_Value_01((byte)axis, value_01);
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
            byte axis = Convert.ToByte(textBox_Position_Axis.Text);
            pParent.set_output_Position_Value_01(axis, value);
            value*=pParent.MAX_RANGE_AXIS_NM[axis];
            textBox_Position_Value.Text = value.ToString();
        }

        private void button_T_debug_Click(object sender, EventArgs e)
        {
            int N = Convert.ToInt32(textBox_T_Test_cycles.Text);
            int dt = Convert.ToInt32(textBox_T_test_dt.Text);
            byte axis = Convert.ToByte(textBox_Position_Axis.Text);

            for (int k = 0;k<N ; k++)
            {
                pParent.set_output_DAC_Value_0_5(axis, 5);
                Thread.Sleep(dt);
                pParent.set_output_DAC_Value_0_5(axis, 0);
                Thread.Sleep(dt);
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
        }

    }
}
