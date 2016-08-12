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
    public partial class Form_User_AutoEngageWaiting : Form
    {

        MainWindow pParent;

        public Form_User_AutoEngageWaiting(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;

            TopMost = true;

        }

        private void button_Cancel_Click(object sender, EventArgs e)
        {
            pParent.mState_User_AutoEngage = 3;
        }

        private void timer_AutoEngage_CheckState_Tick(object sender, EventArgs e)
        {
            if (pParent.mState_User_AutoEngage != 1)
                Close();
        }
    }
}
