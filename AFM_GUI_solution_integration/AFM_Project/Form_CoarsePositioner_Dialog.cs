using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace NameSpace_AFM_Project
{
    //You can implement functions from the DLL into C# in the following way, suggested by our software developer:    

    //When importing SmarAct DLL functions into C# the calling-convention must be specified as "cdecl",
    //like this:

    public partial class Form_CoarsePositioner_Dialog : Form
    {
        MainWindow pParent;

        public Form_CoarsePositioner_Dialog(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;
            //this is for non-cmd_keys
            //this.KeyPress += new KeyPressEventHandler(Form_CoarsePositioner_KeyPress);// this function is not working well
        }
        // this is for cmd_keys
        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (keyData == Keys.Left)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_x, -GetGuiDistanceNm(), GetGuiFrequency());
            if (keyData == Keys.Right)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_x, GetGuiDistanceNm(), GetGuiFrequency());

            if (keyData == Keys.Up)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_y, GetGuiDistanceNm(), GetGuiFrequency());
            if (keyData == Keys.Down)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_y, -GetGuiDistanceNm(), GetGuiFrequency());

            if (keyData == Keys.PageUp)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_z, GetGuiDistanceNm(), GetGuiFrequency());
            if (keyData == Keys.PageDown)
                pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis_z, -GetGuiDistanceNm(), GetGuiFrequency());
            return base.ProcessCmdKey(ref msg, keyData);
        }

        //void Form_CoarsePositioner_KeyPress(object sender, KeyPressEventArgs e)
        //{
        //    if (e.KeyChar >= 48 && e.KeyChar <= 57)
        //    {
        //        //MessageBox.Show("Form.KeyPress: '" +
        //        //    e.KeyChar.ToString() + "' pressed.");

        //        //switch (e.KeyChar)
        //        //{
        //        //    case (char)49:
        //        //    case (char)52:
        //        //    case (char)55:
        //        //        MessageBox.Show("Form.KeyPress: '" +
        //        //            e.KeyChar.ToString() + "' consumed.");
        //        //        e.Handled = true;
        //        //        break;
        //        //}
        //    }
        //}

        uint GetGuiFrequency()
        {
            uint f = Convert.ToUInt32(textBox_Frequency.Text);
            f = pParent.MIN_MAX(f, 1, 20000);
            textBox_Frequency.Text = f.ToString();
            return f;
        }

        double GetGuiDistanceNm()
        {
            double d = Convert.ToDouble(textBox_Distance.Text);
            double dDir = Math.Sign(d);
            d = Math.Abs(d);
            d = pParent.LIMIT_MAX_MIN(d, 1000, 0.01);
            //d *= dDir;
            textBox_Distance.Text = d.ToString();
            return d * 1000;// convert to nm
        }

        private void button_XN_Click(object sender, EventArgs e)
        {
            click_button_move(button_XN, 0, -1);
        }

        private void button_XP_Click(object sender, EventArgs e)
        {
            click_button_move(button_XP, 0, 1);
        }

        //------------------------
        private void button_YP_Click(object sender, EventArgs e)
        {
            click_button_move(button_YP, 1, 1);
        }

        private void button_YN_Click(object sender, EventArgs e)
        {
            click_button_move(button_YN, 1, -1);
        }

        //------------------
        private void button_ZP_Click(object sender, EventArgs e)
        {
            click_button_move(button_ZP, 2, 1);
        }

        private void button_ZN_Click(object sender, EventArgs e)
        {
            click_button_move(button_ZN, 2, -1);
        }
        // universal function for coarse buttonr move.
        void click_button_move(Button B, uint axis, double direction1_n1)
        {
            //this.d;
            B.Enabled = false;
            pParent.mCCoarsePositioner.MoveDistance(pParent.mCaxis[axis], direction1_n1 * GetGuiDistanceNm(), GetGuiFrequency());

            
                B.Enabled = true;
        }

        private void button_CoarsePositioner_Connect_Click(object sender, EventArgs e)
        {
            pParent.mCCoarsePositioner.Initialize();
            //pParent.mCCoarsePositioner.SetChannelVoltage(0, 150);
        }

        private void button_SensorMode_Disable_Click(object sender, EventArgs e)
        {
            //SetSensorModeDisable();
            //SetSensorModeEnable();
            pParent.mCCoarsePositioner.SetSensorModeDisable();
        }

        private void button_SensorMode_Enable_Click(object sender, EventArgs e)
        {
            pParent.mCCoarsePositioner.SetSensorModeEnable();
        }

        private void button_SensorMode_PowerSave_Click(object sender, EventArgs e)
        {
            pParent.mCCoarsePositioner.SetSensorModePowerSave();
        }
    }
}