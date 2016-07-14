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



namespace NameSpace_AFM_Project
{

    public partial class MainWindow : Form
    {
        double para_Nx = 0;
        double para_Ny = 0;
        double para_Dx = 0;
        double para_Dy = 0;
        double para_XL = 0;
        double para_YL = 0;
        double para_ScanRate = 0;
        double para_Sensitivity = 0;
        double para_SetDeltaVoltage = 0;
        double para_Z_PID_P = 0; 
        double para_Z_PID_I = 0;
        double para_Z_PID_D = 0;
    }
}