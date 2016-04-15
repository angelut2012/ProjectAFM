namespace NameSpace_AFM_Project
{
    partial class MainWindow
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            //add  my own code here to do sth before close the window
            mThread_UI_Update_running = false;// stop UI background thread
            SaveAFMParaToTextFile(null);
            //------------------------------------
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainWindow));
            this.listBox_SelectIdlePackage = new System.Windows.Forms.ListBox();
            this.textBox_IC0_R0 = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.timer_CheckCOM = new System.Windows.Forms.Timer(this.components);
            this.toolTip_Help = new System.Windows.Forms.ToolTip(this.components);
            this.textBox_Z_PID_P = new System.Windows.Forms.TextBox();
            this.textBox_Z_PID_I = new System.Windows.Forms.TextBox();
            this.textBox_Z_PID_D = new System.Windows.Forms.TextBox();
            this.button_SetParameters = new System.Windows.Forms.Button();
            this.textBox_ComPortNO = new System.Windows.Forms.TextBox();
            this.button_ConnetComPort = new System.Windows.Forms.Button();
            this.textBox_IC0_R1 = new System.Windows.Forms.TextBox();
            this.textBox_IC0_R3 = new System.Windows.Forms.TextBox();
            this.textBox_IC0_R2 = new System.Windows.Forms.TextBox();
            this.textBox_BaudRate = new System.Windows.Forms.TextBox();
            this.button_XY_Reset = new System.Windows.Forms.Button();
            this.textBox_Nx = new System.Windows.Forms.TextBox();
            this.textBox_Ny = new System.Windows.Forms.TextBox();
            this.textBox_Dy = new System.Windows.Forms.TextBox();
            this.textBox_Dx = new System.Windows.Forms.TextBox();
            this.textBox_YL = new System.Windows.Forms.TextBox();
            this.textBox_XL = new System.Windows.Forms.TextBox();
            this.textBox_ScanRate = new System.Windows.Forms.TextBox();
            this.textBox_Sensitivity = new System.Windows.Forms.TextBox();
            this.textBox_SetDeltaValueNm = new System.Windows.Forms.TextBox();
            this.textBox_NumberOfFrameToScan = new System.Windows.Forms.TextBox();
            this.textBox_TF_DC_Gain = new System.Windows.Forms.TextBox();
            this.textBox_TaskTime = new System.Windows.Forms.TextBox();
            this.textBox_TaskNumber = new System.Windows.Forms.TextBox();
            this.textBox_Task_PointsNumber = new System.Windows.Forms.TextBox();
            this.textBox_T = new System.Windows.Forms.TextBox();
            this.label_End = new System.Windows.Forms.Label();
            this.label_Start = new System.Windows.Forms.Label();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.label7 = new System.Windows.Forms.Label();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.checkBox_COM_Transfer = new System.Windows.Forms.CheckBox();
            this.label12 = new System.Windows.Forms.Label();
            this.checkBox_IC0_DO1 = new System.Windows.Forms.CheckBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.checkBox_IC0_DO2 = new System.Windows.Forms.CheckBox();
            this.serialPort_Arduino = new System.IO.Ports.SerialPort(this.components);
            this.richTextBox_serial_read = new System.Windows.Forms.RichTextBox();
            this.trackBar_R0 = new System.Windows.Forms.TrackBar();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.trackBar_IC1R3 = new System.Windows.Forms.TrackBar();
            this.trackBar_R1 = new System.Windows.Forms.TrackBar();
            this.label11 = new System.Windows.Forms.Label();
            this.button_Apporach = new System.Windows.Forms.Button();
            this.button_CancelApproach = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.button_Z_Withdraw = new System.Windows.Forms.Button();
            this.button_Z_Engage = new System.Windows.Forms.Button();
            this.checkBox_Y_ScanEnable = new System.Windows.Forms.CheckBox();
            this.button_XY_pause = new System.Windows.Forms.Button();
            this.button_XY_Scan = new System.Windows.Forms.Button();
            this.timer_Approach = new System.Windows.Forms.Timer(this.components);
            this.button_CoarseLiftUp = new System.Windows.Forms.Button();
            this.pictureBox_Height = new System.Windows.Forms.PictureBox();
            this.button2 = new System.Windows.Forms.Button();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.label19 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.label21 = new System.Windows.Forms.Label();
            this.label_SystemState = new System.Windows.Forms.Label();
            this.button_SaveImage = new System.Windows.Forms.Button();
            this.button_ClearImage = new System.Windows.Forms.Button();
            this.checkBox_ShowImage = new System.Windows.Forms.CheckBox();
            this.button_StartSubWindow = new System.Windows.Forms.Button();
            this.label22 = new System.Windows.Forms.Label();
            this.button_SetScanROI = new System.Windows.Forms.Button();
            this.button_StartIndent = new System.Windows.Forms.Button();
            this.propertyGrid_AFM_Parameter = new System.Windows.Forms.PropertyGrid();
            this.trackBar_R01 = new System.Windows.Forms.TrackBar();
            this.label23 = new System.Windows.Forms.Label();
            this.checkBox_ForceSetAll = new System.Windows.Forms.CheckBox();
            this.checkBox_T_ScanEnable = new System.Windows.Forms.CheckBox();
            this.label24 = new System.Windows.Forms.Label();
            this.button_CancelIndent = new System.Windows.Forms.Button();
            this.button_MultiTask = new System.Windows.Forms.Button();
            this.label25 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.label27 = new System.Windows.Forms.Label();
            this.label28 = new System.Windows.Forms.Label();
            this.button_Form_CoarsePositioner = new System.Windows.Forms.Button();
            this.button_ShowImage = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.groupBox6.SuspendLayout();
            this.groupBox7.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R0)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_IC1R3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Height)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R01)).BeginInit();
            this.SuspendLayout();
            // 
            // listBox_SelectIdlePackage
            // 
            this.listBox_SelectIdlePackage.BackColor = System.Drawing.SystemColors.Window;
            this.listBox_SelectIdlePackage.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.listBox_SelectIdlePackage.FormattingEnabled = true;
            this.listBox_SelectIdlePackage.ItemHeight = 20;
            this.listBox_SelectIdlePackage.Items.AddRange(new object[] {
            "PRC_Tsem_Tmcu",
            "SCSG_xysz",
            "calibration"});
            this.listBox_SelectIdlePackage.Location = new System.Drawing.Point(433, 619);
            this.listBox_SelectIdlePackage.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.listBox_SelectIdlePackage.Name = "listBox_SelectIdlePackage";
            this.listBox_SelectIdlePackage.Size = new System.Drawing.Size(157, 64);
            this.listBox_SelectIdlePackage.TabIndex = 1;
            this.toolTip_Help.SetToolTip(this.listBox_SelectIdlePackage, "Select an axis to operate.");
            this.listBox_SelectIdlePackage.SelectedIndexChanged += new System.EventHandler(this.listBox_Axis_SelectedIndexChanged);
            // 
            // textBox_IC0_R0
            // 
            this.textBox_IC0_R0.Location = new System.Drawing.Point(111, 755);
            this.textBox_IC0_R0.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_IC0_R0.Name = "textBox_IC0_R0";
            this.textBox_IC0_R0.Size = new System.Drawing.Size(64, 26);
            this.textBox_IC0_R0.TabIndex = 10;
            this.textBox_IC0_R0.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_IC0_R0, "Enter the step length here.");
            this.textBox_IC0_R0.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.text_CheckKeys_IC0R0);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.BackColor = System.Drawing.Color.Transparent;
            this.label2.Location = new System.Drawing.Point(56, 758);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(34, 20);
            this.label2.TabIndex = 11;
            this.label2.Text = "R1:";
            // 
            // timer_CheckCOM
            // 
            this.timer_CheckCOM.Interval = 1;
            // 
            // toolTip_Help
            // 
            this.toolTip_Help.AutomaticDelay = 200;
            // 
            // textBox_Z_PID_P
            // 
            this.textBox_Z_PID_P.Location = new System.Drawing.Point(84, 27);
            this.textBox_Z_PID_P.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Z_PID_P.Name = "textBox_Z_PID_P";
            this.textBox_Z_PID_P.Size = new System.Drawing.Size(55, 26);
            this.textBox_Z_PID_P.TabIndex = 24;
            this.textBox_Z_PID_P.Text = "0.01";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_P, "Enter the step length here.");
            // 
            // textBox_Z_PID_I
            // 
            this.textBox_Z_PID_I.Location = new System.Drawing.Point(84, 68);
            this.textBox_Z_PID_I.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Z_PID_I.Name = "textBox_Z_PID_I";
            this.textBox_Z_PID_I.Size = new System.Drawing.Size(55, 26);
            this.textBox_Z_PID_I.TabIndex = 26;
            this.textBox_Z_PID_I.Text = "0.01";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_I, "Enter the step length here.");
            // 
            // textBox_Z_PID_D
            // 
            this.textBox_Z_PID_D.Location = new System.Drawing.Point(84, 108);
            this.textBox_Z_PID_D.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Z_PID_D.Name = "textBox_Z_PID_D";
            this.textBox_Z_PID_D.Size = new System.Drawing.Size(55, 26);
            this.textBox_Z_PID_D.TabIndex = 25;
            this.textBox_Z_PID_D.Text = "0.0001";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_D, "Enter the step length here.");
            // 
            // button_SetParameters
            // 
            this.button_SetParameters.BackColor = System.Drawing.Color.Transparent;
            this.button_SetParameters.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Set;
            this.button_SetParameters.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.button_SetParameters.Font = new System.Drawing.Font("Microsoft Sans Serif", 8F, System.Drawing.FontStyle.Bold);
            this.button_SetParameters.Location = new System.Drawing.Point(222, 36);
            this.button_SetParameters.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.button_SetParameters.Name = "button_SetParameters";
            this.button_SetParameters.Size = new System.Drawing.Size(88, 86);
            this.button_SetParameters.TabIndex = 11;
            this.toolTip_Help.SetToolTip(this.button_SetParameters, "set parameters");
            this.button_SetParameters.UseVisualStyleBackColor = false;
            this.button_SetParameters.Click += new System.EventHandler(this.button_SetParameters_Click);
            // 
            // textBox_ComPortNO
            // 
            this.textBox_ComPortNO.Location = new System.Drawing.Point(98, 25);
            this.textBox_ComPortNO.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_ComPortNO.Name = "textBox_ComPortNO";
            this.textBox_ComPortNO.Size = new System.Drawing.Size(65, 26);
            this.textBox_ComPortNO.TabIndex = 24;
            this.textBox_ComPortNO.Text = "23";
            this.toolTip_Help.SetToolTip(this.textBox_ComPortNO, "Enter the step length here.");
            // 
            // button_ConnetComPort
            // 
            this.button_ConnetComPort.BackColor = System.Drawing.Color.Transparent;
            this.button_ConnetComPort.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Signal_off;
            this.button_ConnetComPort.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.button_ConnetComPort.Location = new System.Drawing.Point(12, 90);
            this.button_ConnetComPort.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.button_ConnetComPort.Name = "button_ConnetComPort";
            this.button_ConnetComPort.Size = new System.Drawing.Size(117, 84);
            this.button_ConnetComPort.TabIndex = 24;
            this.button_ConnetComPort.Text = "connect";
            this.toolTip_Help.SetToolTip(this.button_ConnetComPort, "Connect...");
            this.button_ConnetComPort.UseVisualStyleBackColor = false;
            this.button_ConnetComPort.Click += new System.EventHandler(this.button_ConnetComPort_Click);
            // 
            // textBox_IC0_R1
            // 
            this.textBox_IC0_R1.Location = new System.Drawing.Point(251, 755);
            this.textBox_IC0_R1.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_IC0_R1.Name = "textBox_IC0_R1";
            this.textBox_IC0_R1.Size = new System.Drawing.Size(64, 26);
            this.textBox_IC0_R1.TabIndex = 27;
            this.textBox_IC0_R1.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_IC0_R1, "Enter the step length here.");
            this.textBox_IC0_R1.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.text_CheckKeys_IC0R1);
            // 
            // textBox_IC0_R3
            // 
            this.textBox_IC0_R3.Location = new System.Drawing.Point(531, 755);
            this.textBox_IC0_R3.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_IC0_R3.Name = "textBox_IC0_R3";
            this.textBox_IC0_R3.Size = new System.Drawing.Size(64, 26);
            this.textBox_IC0_R3.TabIndex = 31;
            this.textBox_IC0_R3.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_IC0_R3, "Enter the step length here.");
            this.textBox_IC0_R3.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.text_CheckKeys_IC0R3);
            // 
            // textBox_IC0_R2
            // 
            this.textBox_IC0_R2.Location = new System.Drawing.Point(391, 755);
            this.textBox_IC0_R2.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_IC0_R2.Name = "textBox_IC0_R2";
            this.textBox_IC0_R2.Size = new System.Drawing.Size(64, 26);
            this.textBox_IC0_R2.TabIndex = 29;
            this.textBox_IC0_R2.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_IC0_R2, "Enter the step length here.");
            this.textBox_IC0_R2.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.text_CheckKeys_IC0R2);
            // 
            // textBox_BaudRate
            // 
            this.textBox_BaudRate.Location = new System.Drawing.Point(98, 53);
            this.textBox_BaudRate.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_BaudRate.Name = "textBox_BaudRate";
            this.textBox_BaudRate.Size = new System.Drawing.Size(65, 26);
            this.textBox_BaudRate.TabIndex = 26;
            this.textBox_BaudRate.Text = "115200";
            this.toolTip_Help.SetToolTip(this.textBox_BaudRate, "Enter the step length here.");
            // 
            // button_XY_Reset
            // 
            this.button_XY_Reset.Location = new System.Drawing.Point(994, 395);
            this.button_XY_Reset.Name = "button_XY_Reset";
            this.button_XY_Reset.Size = new System.Drawing.Size(91, 56);
            this.button_XY_Reset.TabIndex = 48;
            this.button_XY_Reset.Text = "reset";
            this.toolTip_Help.SetToolTip(this.button_XY_Reset, "stop xy scan and xy move to XL YL");
            this.button_XY_Reset.UseVisualStyleBackColor = true;
            this.button_XY_Reset.Click += new System.EventHandler(this.button_XY_Reset_Click);
            // 
            // textBox_Nx
            // 
            this.textBox_Nx.Location = new System.Drawing.Point(184, 173);
            this.textBox_Nx.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Nx.Name = "textBox_Nx";
            this.textBox_Nx.Size = new System.Drawing.Size(55, 26);
            this.textBox_Nx.TabIndex = 30;
            this.textBox_Nx.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_Nx, "Enter the step length here.");
            // 
            // textBox_Ny
            // 
            this.textBox_Ny.Location = new System.Drawing.Point(184, 209);
            this.textBox_Ny.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Ny.Name = "textBox_Ny";
            this.textBox_Ny.Size = new System.Drawing.Size(55, 26);
            this.textBox_Ny.TabIndex = 53;
            this.textBox_Ny.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_Ny, "Enter the step length here.");
            // 
            // textBox_Dy
            // 
            this.textBox_Dy.Location = new System.Drawing.Point(184, 281);
            this.textBox_Dy.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Dy.Name = "textBox_Dy";
            this.textBox_Dy.Size = new System.Drawing.Size(55, 26);
            this.textBox_Dy.TabIndex = 57;
            this.textBox_Dy.Text = "22000";
            this.toolTip_Help.SetToolTip(this.textBox_Dy, "Enter the step length here.");
            // 
            // textBox_Dx
            // 
            this.textBox_Dx.Location = new System.Drawing.Point(184, 245);
            this.textBox_Dx.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Dx.Name = "textBox_Dx";
            this.textBox_Dx.Size = new System.Drawing.Size(55, 26);
            this.textBox_Dx.TabIndex = 55;
            this.textBox_Dx.Text = "22000";
            this.toolTip_Help.SetToolTip(this.textBox_Dx, "Enter the step length here.");
            // 
            // textBox_YL
            // 
            this.textBox_YL.Location = new System.Drawing.Point(184, 353);
            this.textBox_YL.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_YL.Name = "textBox_YL";
            this.textBox_YL.Size = new System.Drawing.Size(55, 26);
            this.textBox_YL.TabIndex = 61;
            this.textBox_YL.Text = "5000";
            this.toolTip_Help.SetToolTip(this.textBox_YL, "Enter the step length here.");
            // 
            // textBox_XL
            // 
            this.textBox_XL.Location = new System.Drawing.Point(184, 317);
            this.textBox_XL.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_XL.Name = "textBox_XL";
            this.textBox_XL.Size = new System.Drawing.Size(55, 26);
            this.textBox_XL.TabIndex = 59;
            this.textBox_XL.Text = "5000";
            this.toolTip_Help.SetToolTip(this.textBox_XL, "Enter the step length here.");
            // 
            // textBox_ScanRate
            // 
            this.textBox_ScanRate.Location = new System.Drawing.Point(184, 389);
            this.textBox_ScanRate.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_ScanRate.Name = "textBox_ScanRate";
            this.textBox_ScanRate.Size = new System.Drawing.Size(55, 26);
            this.textBox_ScanRate.TabIndex = 63;
            this.textBox_ScanRate.Text = "0.5";
            this.toolTip_Help.SetToolTip(this.textBox_ScanRate, "Enter the step length here.");
            // 
            // textBox_Sensitivity
            // 
            this.textBox_Sensitivity.Location = new System.Drawing.Point(184, 425);
            this.textBox_Sensitivity.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Sensitivity.Name = "textBox_Sensitivity";
            this.textBox_Sensitivity.Size = new System.Drawing.Size(55, 26);
            this.textBox_Sensitivity.TabIndex = 65;
            this.textBox_Sensitivity.Text = "35.3";
            this.toolTip_Help.SetToolTip(this.textBox_Sensitivity, "Enter the step length here.");
            // 
            // textBox_SetDeltaValueNm
            // 
            this.textBox_SetDeltaValueNm.Location = new System.Drawing.Point(184, 461);
            this.textBox_SetDeltaValueNm.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_SetDeltaValueNm.Name = "textBox_SetDeltaValueNm";
            this.textBox_SetDeltaValueNm.Size = new System.Drawing.Size(55, 26);
            this.textBox_SetDeltaValueNm.TabIndex = 67;
            this.textBox_SetDeltaValueNm.Text = "5";
            this.toolTip_Help.SetToolTip(this.textBox_SetDeltaValueNm, "Enter the step length here.");
            // 
            // textBox_NumberOfFrameToScan
            // 
            this.textBox_NumberOfFrameToScan.Location = new System.Drawing.Point(183, 499);
            this.textBox_NumberOfFrameToScan.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_NumberOfFrameToScan.Name = "textBox_NumberOfFrameToScan";
            this.textBox_NumberOfFrameToScan.Size = new System.Drawing.Size(55, 26);
            this.textBox_NumberOfFrameToScan.TabIndex = 75;
            this.textBox_NumberOfFrameToScan.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_NumberOfFrameToScan, "Enter the step length here.");
            // 
            // textBox_TF_DC_Gain
            // 
            this.textBox_TF_DC_Gain.Location = new System.Drawing.Point(246, 589);
            this.textBox_TF_DC_Gain.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_TF_DC_Gain.Name = "textBox_TF_DC_Gain";
            this.textBox_TF_DC_Gain.Size = new System.Drawing.Size(55, 26);
            this.textBox_TF_DC_Gain.TabIndex = 84;
            this.textBox_TF_DC_Gain.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_TF_DC_Gain, "Enter the step length here.");
            // 
            // textBox_TaskTime
            // 
            this.textBox_TaskTime.Location = new System.Drawing.Point(611, 113);
            this.textBox_TaskTime.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_TaskTime.Name = "textBox_TaskTime";
            this.textBox_TaskTime.Size = new System.Drawing.Size(88, 26);
            this.textBox_TaskTime.TabIndex = 87;
            this.textBox_TaskTime.Text = "20";
            this.toolTip_Help.SetToolTip(this.textBox_TaskTime, "Enter the step length here.");
            // 
            // textBox_TaskNumber
            // 
            this.textBox_TaskNumber.Location = new System.Drawing.Point(611, 174);
            this.textBox_TaskNumber.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_TaskNumber.Name = "textBox_TaskNumber";
            this.textBox_TaskNumber.Size = new System.Drawing.Size(88, 26);
            this.textBox_TaskNumber.TabIndex = 89;
            this.textBox_TaskNumber.Text = "11";
            this.toolTip_Help.SetToolTip(this.textBox_TaskNumber, "Enter the step length here.");
            // 
            // textBox_Task_PointsNumber
            // 
            this.textBox_Task_PointsNumber.Location = new System.Drawing.Point(611, 252);
            this.textBox_Task_PointsNumber.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_Task_PointsNumber.Name = "textBox_Task_PointsNumber";
            this.textBox_Task_PointsNumber.Size = new System.Drawing.Size(88, 26);
            this.textBox_Task_PointsNumber.TabIndex = 91;
            this.textBox_Task_PointsNumber.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_Task_PointsNumber, "Enter the step length here.");
            // 
            // textBox_T
            // 
            this.textBox_T.Location = new System.Drawing.Point(611, 332);
            this.textBox_T.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBox_T.Name = "textBox_T";
            this.textBox_T.Size = new System.Drawing.Size(88, 26);
            this.textBox_T.TabIndex = 93;
            this.textBox_T.Text = "25";
            this.toolTip_Help.SetToolTip(this.textBox_T, "Enter the step length here.");
            // 
            // label_End
            // 
            this.label_End.AutoSize = true;
            this.label_End.BackColor = System.Drawing.Color.Transparent;
            this.label_End.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label_End.Location = new System.Drawing.Point(1047, 840);
            this.label_End.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label_End.Name = "label_End";
            this.label_End.Size = new System.Drawing.Size(50, 25);
            this.label_End.TabIndex = 22;
            this.label_End.Text = "End";
            // 
            // label_Start
            // 
            this.label_Start.AutoSize = true;
            this.label_Start.BackColor = System.Drawing.Color.Transparent;
            this.label_Start.Font = new System.Drawing.Font("Microsoft Sans Serif", 15.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label_Start.Location = new System.Drawing.Point(1105, 840);
            this.label_Start.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label_Start.Name = "label_Start";
            this.label_Start.Size = new System.Drawing.Size(57, 25);
            this.label_Start.TabIndex = 23;
            this.label_Start.Text = "Start";
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackColor = System.Drawing.Color.Transparent;
            this.pictureBox1.Image = global::NameSpace_AFM_Project.Properties.Resources.motor_pic;
            this.pictureBox1.Location = new System.Drawing.Point(921, 827);
            this.pictureBox1.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(99, 50);
            this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pictureBox1.TabIndex = 4;
            this.pictureBox1.TabStop = false;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.Color.Transparent;
            this.label1.Location = new System.Drawing.Point(15, 27);
            this.label1.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(59, 20);
            this.label1.TabIndex = 27;
            this.label1.Text = "PID_P:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.BackColor = System.Drawing.Color.Transparent;
            this.label5.Location = new System.Drawing.Point(15, 68);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(54, 20);
            this.label5.TabIndex = 28;
            this.label5.Text = "PID_I:";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.BackColor = System.Drawing.Color.Transparent;
            this.label6.Location = new System.Drawing.Point(15, 111);
            this.label6.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(61, 20);
            this.label6.TabIndex = 29;
            this.label6.Text = "PID_D:";
            // 
            // groupBox6
            // 
            this.groupBox6.BackColor = System.Drawing.Color.Transparent;
            this.groupBox6.Controls.Add(this.textBox_Z_PID_P);
            this.groupBox6.Controls.Add(this.label1);
            this.groupBox6.Controls.Add(this.label6);
            this.groupBox6.Controls.Add(this.textBox_Z_PID_D);
            this.groupBox6.Controls.Add(this.textBox_Z_PID_I);
            this.groupBox6.Controls.Add(this.label5);
            this.groupBox6.Location = new System.Drawing.Point(51, 14);
            this.groupBox6.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox6.Size = new System.Drawing.Size(150, 146);
            this.groupBox6.TabIndex = 21;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Z_PID";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.BackColor = System.Drawing.Color.Transparent;
            this.label7.Location = new System.Drawing.Point(8, 28);
            this.label7.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(82, 20);
            this.label7.TabIndex = 25;
            this.label7.Text = "COM Port:";
            // 
            // groupBox7
            // 
            this.groupBox7.BackColor = System.Drawing.Color.Transparent;
            this.groupBox7.Controls.Add(this.checkBox_COM_Transfer);
            this.groupBox7.Controls.Add(this.label12);
            this.groupBox7.Controls.Add(this.textBox_BaudRate);
            this.groupBox7.Controls.Add(this.label7);
            this.groupBox7.Controls.Add(this.button_ConnetComPort);
            this.groupBox7.Controls.Add(this.textBox_ComPortNO);
            this.groupBox7.Location = new System.Drawing.Point(984, 14);
            this.groupBox7.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox7.Size = new System.Drawing.Size(209, 180);
            this.groupBox7.TabIndex = 26;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "Com Connection";
            // 
            // checkBox_COM_Transfer
            // 
            this.checkBox_COM_Transfer.AutoSize = true;
            this.checkBox_COM_Transfer.Location = new System.Drawing.Point(137, 121);
            this.checkBox_COM_Transfer.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_COM_Transfer.Name = "checkBox_COM_Transfer";
            this.checkBox_COM_Transfer.Size = new System.Drawing.Size(64, 24);
            this.checkBox_COM_Transfer.TabIndex = 44;
            this.checkBox_COM_Transfer.Text = "trans";
            this.checkBox_COM_Transfer.UseVisualStyleBackColor = true;
            this.checkBox_COM_Transfer.CheckedChanged += new System.EventHandler(this.checkBox_COM_Transfer_CheckedChanged);
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.BackColor = System.Drawing.Color.Transparent;
            this.label12.Location = new System.Drawing.Point(8, 56);
            this.label12.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(83, 20);
            this.label12.TabIndex = 27;
            this.label12.Text = "Baud rate:";
            // 
            // checkBox_IC0_DO1
            // 
            this.checkBox_IC0_DO1.AutoSize = true;
            this.checkBox_IC0_DO1.Location = new System.Drawing.Point(616, 756);
            this.checkBox_IC0_DO1.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_IC0_DO1.Name = "checkBox_IC0_DO1";
            this.checkBox_IC0_DO1.Size = new System.Drawing.Size(61, 24);
            this.checkBox_IC0_DO1.TabIndex = 26;
            this.checkBox_IC0_DO1.Text = "DO1";
            this.checkBox_IC0_DO1.UseVisualStyleBackColor = true;
            this.checkBox_IC0_DO1.CheckedChanged += new System.EventHandler(this.checkBox_IC0_DO1_CheckedChanged);
            // 
            // groupBox4
            // 
            this.groupBox4.BackColor = System.Drawing.Color.Transparent;
            this.groupBox4.Location = new System.Drawing.Point(51, 736);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.groupBox4.Size = new System.Drawing.Size(795, 57);
            this.groupBox4.TabIndex = 20;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "IC0";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.BackColor = System.Drawing.Color.Transparent;
            this.label3.Location = new System.Drawing.Point(196, 758);
            this.label3.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(34, 20);
            this.label3.TabIndex = 28;
            this.label3.Text = "R2:";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.BackColor = System.Drawing.Color.Transparent;
            this.label4.Location = new System.Drawing.Point(476, 758);
            this.label4.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(34, 20);
            this.label4.TabIndex = 32;
            this.label4.Text = "R4:";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.BackColor = System.Drawing.Color.Transparent;
            this.label8.Location = new System.Drawing.Point(336, 758);
            this.label8.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(34, 20);
            this.label8.TabIndex = 30;
            this.label8.Text = "R3:";
            // 
            // checkBox_IC0_DO2
            // 
            this.checkBox_IC0_DO2.AutoSize = true;
            this.checkBox_IC0_DO2.Location = new System.Drawing.Point(698, 756);
            this.checkBox_IC0_DO2.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_IC0_DO2.Name = "checkBox_IC0_DO2";
            this.checkBox_IC0_DO2.Size = new System.Drawing.Size(61, 24);
            this.checkBox_IC0_DO2.TabIndex = 33;
            this.checkBox_IC0_DO2.Text = "DO2";
            this.checkBox_IC0_DO2.UseVisualStyleBackColor = true;
            this.checkBox_IC0_DO2.CheckedChanged += new System.EventHandler(this.checkBox_IC0_DO2_CheckedChanged);
            // 
            // serialPort_Arduino
            // 
            this.serialPort_Arduino.PortName = "COM_Arduino";
            // 
            // richTextBox_serial_read
            // 
            this.richTextBox_serial_read.Location = new System.Drawing.Point(799, 840);
            this.richTextBox_serial_read.Name = "richTextBox_serial_read";
            this.richTextBox_serial_read.Size = new System.Drawing.Size(99, 72);
            this.richTextBox_serial_read.TabIndex = 34;
            this.richTextBox_serial_read.Text = "";
            // 
            // trackBar_R0
            // 
            this.trackBar_R0.LargeChange = 1;
            this.trackBar_R0.Location = new System.Drawing.Point(433, 866);
            this.trackBar_R0.Maximum = 255;
            this.trackBar_R0.Name = "trackBar_R0";
            this.trackBar_R0.Size = new System.Drawing.Size(121, 42);
            this.trackBar_R0.TabIndex = 35;
            this.trackBar_R0.TickFrequency = 2;
            this.trackBar_R0.Scroll += new System.EventHandler(this.trackBar_R0_Scroll);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.BackColor = System.Drawing.Color.Transparent;
            this.label9.Location = new System.Drawing.Point(396, 877);
            this.label9.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(30, 20);
            this.label9.TabIndex = 36;
            this.label9.Text = "R1";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.BackColor = System.Drawing.Color.Transparent;
            this.label10.Location = new System.Drawing.Point(582, 876);
            this.label10.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(50, 20);
            this.label10.TabIndex = 38;
            this.label10.Text = "R_C0";
            // 
            // trackBar_IC1R3
            // 
            this.trackBar_IC1R3.LargeChange = 1;
            this.trackBar_IC1R3.Location = new System.Drawing.Point(633, 866);
            this.trackBar_IC1R3.Maximum = 255;
            this.trackBar_IC1R3.Name = "trackBar_IC1R3";
            this.trackBar_IC1R3.Size = new System.Drawing.Size(121, 42);
            this.trackBar_IC1R3.TabIndex = 37;
            this.trackBar_IC1R3.TickFrequency = 2;
            this.trackBar_IC1R3.Scroll += new System.EventHandler(this.trackBar1_Scroll);
            // 
            // trackBar_R1
            // 
            this.trackBar_R1.LargeChange = 1;
            this.trackBar_R1.Location = new System.Drawing.Point(633, 823);
            this.trackBar_R1.Maximum = 255;
            this.trackBar_R1.Name = "trackBar_R1";
            this.trackBar_R1.Size = new System.Drawing.Size(121, 42);
            this.trackBar_R1.TabIndex = 39;
            this.trackBar_R1.TickFrequency = 2;
            this.trackBar_R1.Scroll += new System.EventHandler(this.trackBar_R1_Scroll);
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.BackColor = System.Drawing.Color.Transparent;
            this.label11.Location = new System.Drawing.Point(596, 833);
            this.label11.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(30, 20);
            this.label11.TabIndex = 40;
            this.label11.Text = "R2";
            // 
            // button_Apporach
            // 
            this.button_Apporach.Location = new System.Drawing.Point(799, 237);
            this.button_Apporach.Name = "button_Apporach";
            this.button_Apporach.Size = new System.Drawing.Size(91, 56);
            this.button_Apporach.TabIndex = 41;
            this.button_Apporach.Text = "apporach";
            this.button_Apporach.UseVisualStyleBackColor = true;
            this.button_Apporach.Click += new System.EventHandler(this.button_Apporach_Click);
            // 
            // button_CancelApproach
            // 
            this.button_CancelApproach.Location = new System.Drawing.Point(896, 237);
            this.button_CancelApproach.Name = "button_CancelApproach";
            this.button_CancelApproach.Size = new System.Drawing.Size(91, 56);
            this.button_CancelApproach.TabIndex = 42;
            this.button_CancelApproach.Text = "cancel";
            this.button_CancelApproach.UseVisualStyleBackColor = true;
            this.button_CancelApproach.Click += new System.EventHandler(this.button_ManuallyCancelApproach_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(863, 736);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(60, 37);
            this.button1.TabIndex = 43;
            this.button1.Text = "reset mcu";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // button_Z_Withdraw
            // 
            this.button_Z_Withdraw.Location = new System.Drawing.Point(896, 317);
            this.button_Z_Withdraw.Name = "button_Z_Withdraw";
            this.button_Z_Withdraw.Size = new System.Drawing.Size(91, 56);
            this.button_Z_Withdraw.TabIndex = 45;
            this.button_Z_Withdraw.Text = "withdraw";
            this.button_Z_Withdraw.UseVisualStyleBackColor = true;
            this.button_Z_Withdraw.Click += new System.EventHandler(this.button_Z_Withdraw_Click);
            // 
            // button_Z_Engage
            // 
            this.button_Z_Engage.Location = new System.Drawing.Point(799, 317);
            this.button_Z_Engage.Name = "button_Z_Engage";
            this.button_Z_Engage.Size = new System.Drawing.Size(91, 56);
            this.button_Z_Engage.TabIndex = 44;
            this.button_Z_Engage.Text = "engage";
            this.button_Z_Engage.UseVisualStyleBackColor = true;
            this.button_Z_Engage.Click += new System.EventHandler(this.button_Z_Engage_Click);
            // 
            // checkBox_Y_ScanEnable
            // 
            this.checkBox_Y_ScanEnable.AutoSize = true;
            this.checkBox_Y_ScanEnable.Checked = true;
            this.checkBox_Y_ScanEnable.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBox_Y_ScanEnable.Location = new System.Drawing.Point(777, 457);
            this.checkBox_Y_ScanEnable.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_Y_ScanEnable.Name = "checkBox_Y_ScanEnable";
            this.checkBox_Y_ScanEnable.Size = new System.Drawing.Size(77, 24);
            this.checkBox_Y_ScanEnable.TabIndex = 45;
            this.checkBox_Y_ScanEnable.Text = "Y scan";
            this.checkBox_Y_ScanEnable.UseVisualStyleBackColor = true;
            this.checkBox_Y_ScanEnable.CheckedChanged += new System.EventHandler(this.checkBox_Y_ScanEnable_CheckedChanged);
            // 
            // button_XY_pause
            // 
            this.button_XY_pause.Location = new System.Drawing.Point(896, 395);
            this.button_XY_pause.Name = "button_XY_pause";
            this.button_XY_pause.Size = new System.Drawing.Size(91, 56);
            this.button_XY_pause.TabIndex = 47;
            this.button_XY_pause.Text = "pause";
            this.button_XY_pause.UseVisualStyleBackColor = true;
            this.button_XY_pause.Click += new System.EventHandler(this.button_XY_pause_Click);
            // 
            // button_XY_Scan
            // 
            this.button_XY_Scan.Location = new System.Drawing.Point(799, 395);
            this.button_XY_Scan.Name = "button_XY_Scan";
            this.button_XY_Scan.Size = new System.Drawing.Size(91, 56);
            this.button_XY_Scan.TabIndex = 46;
            this.button_XY_Scan.Text = "scan";
            this.button_XY_Scan.UseVisualStyleBackColor = true;
            this.button_XY_Scan.Click += new System.EventHandler(this.button_XY_Scan_Click);
            // 
            // timer_Approach
            // 
            this.timer_Approach.Interval = 6000;
            this.timer_Approach.Tick += new System.EventHandler(this.timerFunction_Appraoch);
            // 
            // button_CoarseLiftUp
            // 
            this.button_CoarseLiftUp.Location = new System.Drawing.Point(1014, 237);
            this.button_CoarseLiftUp.Name = "button_CoarseLiftUp";
            this.button_CoarseLiftUp.Size = new System.Drawing.Size(91, 56);
            this.button_CoarseLiftUp.TabIndex = 49;
            this.button_CoarseLiftUp.Text = "lift up";
            this.button_CoarseLiftUp.UseVisualStyleBackColor = true;
            this.button_CoarseLiftUp.Click += new System.EventHandler(this.button_CoarseLiftUp_Click);
            // 
            // pictureBox_Height
            // 
            this.pictureBox_Height.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
            this.pictureBox_Height.Location = new System.Drawing.Point(931, 528);
            this.pictureBox_Height.Name = "pictureBox_Height";
            this.pictureBox_Height.Size = new System.Drawing.Size(262, 254);
            this.pictureBox_Height.TabIndex = 50;
            this.pictureBox_Height.TabStop = false;
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(639, 440);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 47);
            this.button2.TabIndex = 51;
            this.button2.Text = "button2";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.BackColor = System.Drawing.Color.Transparent;
            this.label13.Location = new System.Drawing.Point(34, 176);
            this.label13.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(31, 20);
            this.label13.TabIndex = 30;
            this.label13.Text = "Nx:";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.BackColor = System.Drawing.Color.Transparent;
            this.label14.Location = new System.Drawing.Point(34, 209);
            this.label14.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(31, 20);
            this.label14.TabIndex = 52;
            this.label14.Text = "Ny:";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.BackColor = System.Drawing.Color.Transparent;
            this.label15.Location = new System.Drawing.Point(34, 281);
            this.label15.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(64, 20);
            this.label15.TabIndex = 56;
            this.label15.Text = "Dy(nm):";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.BackColor = System.Drawing.Color.Transparent;
            this.label16.Location = new System.Drawing.Point(34, 248);
            this.label16.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(64, 20);
            this.label16.TabIndex = 54;
            this.label16.Text = "Dx(nm):";
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.BackColor = System.Drawing.Color.Transparent;
            this.label17.Location = new System.Drawing.Point(34, 353);
            this.label17.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(61, 20);
            this.label17.TabIndex = 60;
            this.label17.Text = "yL(nm):";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.BackColor = System.Drawing.Color.Transparent;
            this.label18.Location = new System.Drawing.Point(34, 320);
            this.label18.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(61, 20);
            this.label18.TabIndex = 58;
            this.label18.Text = "xL(nm):";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.BackColor = System.Drawing.Color.Transparent;
            this.label19.Location = new System.Drawing.Point(34, 389);
            this.label19.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(79, 20);
            this.label19.TabIndex = 62;
            this.label19.Text = "scan rate:";
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.BackColor = System.Drawing.Color.Transparent;
            this.label20.Location = new System.Drawing.Point(33, 431);
            this.label20.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(112, 20);
            this.label20.TabIndex = 64;
            this.label20.Text = "DTSsensitivity:";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.BackColor = System.Drawing.Color.Transparent;
            this.label21.Location = new System.Drawing.Point(5, 464);
            this.label21.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(162, 20);
            this.label21.TabIndex = 66;
            this.label21.Text = "WorkingDistance(nm)";
            // 
            // label_SystemState
            // 
            this.label_SystemState.AutoSize = true;
            this.label_SystemState.BackColor = System.Drawing.Color.Transparent;
            this.label_SystemState.Location = new System.Drawing.Point(19, 619);
            this.label_SystemState.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label_SystemState.Name = "label_SystemState";
            this.label_SystemState.Size = new System.Drawing.Size(106, 20);
            this.label_SystemState.TabIndex = 68;
            this.label_SystemState.Text = "system ready.";
            // 
            // button_SaveImage
            // 
            this.button_SaveImage.Location = new System.Drawing.Point(994, 457);
            this.button_SaveImage.Name = "button_SaveImage";
            this.button_SaveImage.Size = new System.Drawing.Size(91, 51);
            this.button_SaveImage.TabIndex = 69;
            this.button_SaveImage.Text = "save image";
            this.button_SaveImage.UseVisualStyleBackColor = true;
            this.button_SaveImage.Click += new System.EventHandler(this.button_SaveImage_Click);
            // 
            // button_ClearImage
            // 
            this.button_ClearImage.Location = new System.Drawing.Point(896, 459);
            this.button_ClearImage.Name = "button_ClearImage";
            this.button_ClearImage.Size = new System.Drawing.Size(91, 51);
            this.button_ClearImage.TabIndex = 70;
            this.button_ClearImage.Text = "clear image";
            this.button_ClearImage.UseVisualStyleBackColor = true;
            this.button_ClearImage.Click += new System.EventHandler(this.button_ClearImage_Click);
            // 
            // checkBox_ShowImage
            // 
            this.checkBox_ShowImage.AutoSize = true;
            this.checkBox_ShowImage.Location = new System.Drawing.Point(777, 484);
            this.checkBox_ShowImage.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_ShowImage.Name = "checkBox_ShowImage";
            this.checkBox_ShowImage.Size = new System.Drawing.Size(112, 24);
            this.checkBox_ShowImage.TabIndex = 72;
            this.checkBox_ShowImage.Text = "show image";
            this.checkBox_ShowImage.UseVisualStyleBackColor = true;
            this.checkBox_ShowImage.CheckedChanged += new System.EventHandler(this.checkBox_ShowImage_CheckedChanged);
            // 
            // button_StartSubWindow
            // 
            this.button_StartSubWindow.Location = new System.Drawing.Point(246, 221);
            this.button_StartSubWindow.Name = "button_StartSubWindow";
            this.button_StartSubWindow.Size = new System.Drawing.Size(98, 32);
            this.button_StartSubWindow.TabIndex = 73;
            this.button_StartSubWindow.Text = "subform";
            this.button_StartSubWindow.UseVisualStyleBackColor = true;
            this.button_StartSubWindow.Click += new System.EventHandler(this.button_StartSubWindow_Click);
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.BackColor = System.Drawing.Color.Transparent;
            this.label22.Location = new System.Drawing.Point(33, 504);
            this.label22.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(63, 20);
            this.label22.TabIndex = 74;
            this.label22.Text = "Frames";
            // 
            // button_SetScanROI
            // 
            this.button_SetScanROI.Location = new System.Drawing.Point(246, 271);
            this.button_SetScanROI.Name = "button_SetScanROI";
            this.button_SetScanROI.Size = new System.Drawing.Size(94, 32);
            this.button_SetScanROI.TabIndex = 76;
            this.button_SetScanROI.Text = "ROI";
            this.button_SetScanROI.UseVisualStyleBackColor = true;
            this.button_SetScanROI.Click += new System.EventHandler(this.button_SetScanROI_Click);
            // 
            // button_StartIndent
            // 
            this.button_StartIndent.Location = new System.Drawing.Point(799, 14);
            this.button_StartIndent.Name = "button_StartIndent";
            this.button_StartIndent.Size = new System.Drawing.Size(100, 67);
            this.button_StartIndent.TabIndex = 77;
            this.button_StartIndent.Text = "start indentation";
            this.button_StartIndent.UseVisualStyleBackColor = true;
            this.button_StartIndent.Click += new System.EventHandler(this.button_Start_IndentationWindow_Click);
            // 
            // propertyGrid_AFM_Parameter
            // 
            this.propertyGrid_AFM_Parameter.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.propertyGrid_AFM_Parameter.Location = new System.Drawing.Point(346, 19);
            this.propertyGrid_AFM_Parameter.Name = "propertyGrid_AFM_Parameter";
            this.propertyGrid_AFM_Parameter.Size = new System.Drawing.Size(244, 545);
            this.propertyGrid_AFM_Parameter.TabIndex = 78;
            // 
            // trackBar_R01
            // 
            this.trackBar_R01.LargeChange = 1;
            this.trackBar_R01.Location = new System.Drawing.Point(433, 818);
            this.trackBar_R01.Maximum = 255;
            this.trackBar_R01.Name = "trackBar_R01";
            this.trackBar_R01.Size = new System.Drawing.Size(121, 42);
            this.trackBar_R01.TabIndex = 79;
            this.trackBar_R01.TickFrequency = 2;
            this.trackBar_R01.Scroll += new System.EventHandler(this.trackBar_R01_Scroll);
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.BackColor = System.Drawing.Color.Transparent;
            this.label23.Location = new System.Drawing.Point(382, 830);
            this.label23.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(39, 20);
            this.label23.TabIndex = 80;
            this.label23.Text = "R12";
            // 
            // checkBox_ForceSetAll
            // 
            this.checkBox_ForceSetAll.AutoSize = true;
            this.checkBox_ForceSetAll.Checked = true;
            this.checkBox_ForceSetAll.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBox_ForceSetAll.Location = new System.Drawing.Point(222, 136);
            this.checkBox_ForceSetAll.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_ForceSetAll.Name = "checkBox_ForceSetAll";
            this.checkBox_ForceSetAll.Size = new System.Drawing.Size(83, 24);
            this.checkBox_ForceSetAll.TabIndex = 81;
            this.checkBox_ForceSetAll.Text = "force all";
            this.checkBox_ForceSetAll.UseVisualStyleBackColor = true;
            // 
            // checkBox_T_ScanEnable
            // 
            this.checkBox_T_ScanEnable.AutoSize = true;
            this.checkBox_T_ScanEnable.Location = new System.Drawing.Point(777, 518);
            this.checkBox_T_ScanEnable.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.checkBox_T_ScanEnable.Name = "checkBox_T_ScanEnable";
            this.checkBox_T_ScanEnable.Size = new System.Drawing.Size(85, 24);
            this.checkBox_T_ScanEnable.TabIndex = 82;
            this.checkBox_T_ScanEnable.Text = "TF scan";
            this.checkBox_T_ScanEnable.UseVisualStyleBackColor = true;
            this.checkBox_T_ScanEnable.CheckedChanged += new System.EventHandler(this.checkBox_T_ScanEnable_CheckedChanged);
            // 
            // label24
            // 
            this.label24.AutoSize = true;
            this.label24.BackColor = System.Drawing.Color.Transparent;
            this.label24.Location = new System.Drawing.Point(131, 592);
            this.label24.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(89, 20);
            this.label24.TabIndex = 83;
            this.label24.Text = "TF DC gain";
            // 
            // button_CancelIndent
            // 
            this.button_CancelIndent.Location = new System.Drawing.Point(799, 104);
            this.button_CancelIndent.Name = "button_CancelIndent";
            this.button_CancelIndent.Size = new System.Drawing.Size(100, 65);
            this.button_CancelIndent.TabIndex = 85;
            this.button_CancelIndent.Text = "cancel indent";
            this.button_CancelIndent.UseVisualStyleBackColor = true;
            this.button_CancelIndent.Click += new System.EventHandler(this.button_CancelIndent_Click);
            // 
            // button_MultiTask
            // 
            this.button_MultiTask.Location = new System.Drawing.Point(611, 14);
            this.button_MultiTask.Name = "button_MultiTask";
            this.button_MultiTask.Size = new System.Drawing.Size(100, 67);
            this.button_MultiTask.TabIndex = 86;
            this.button_MultiTask.Text = "start task";
            this.button_MultiTask.UseVisualStyleBackColor = true;
            this.button_MultiTask.Click += new System.EventHandler(this.button_MultiTask_Click);
            // 
            // label25
            // 
            this.label25.AutoSize = true;
            this.label25.BackColor = System.Drawing.Color.Transparent;
            this.label25.Location = new System.Drawing.Point(619, 88);
            this.label25.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(95, 20);
            this.label25.TabIndex = 88;
            this.label25.Text = "task time (s)";
            // 
            // label26
            // 
            this.label26.AutoSize = true;
            this.label26.BackColor = System.Drawing.Color.Transparent;
            this.label26.Location = new System.Drawing.Point(619, 149);
            this.label26.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(97, 20);
            this.label26.TabIndex = 90;
            this.label26.Text = "task number";
            // 
            // label27
            // 
            this.label27.AutoSize = true;
            this.label27.BackColor = System.Drawing.Color.Transparent;
            this.label27.Location = new System.Drawing.Point(619, 227);
            this.label27.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(166, 20);
            this.label27.TabIndex = 92;
            this.label27.Text = "task  number of points";
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.BackColor = System.Drawing.Color.Transparent;
            this.label28.Location = new System.Drawing.Point(619, 307);
            this.label28.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(45, 20);
            this.label28.TabIndex = 94;
            this.label28.Text = "temp";
            // 
            // button_Form_CoarsePositioner
            // 
            this.button_Form_CoarsePositioner.Location = new System.Drawing.Point(246, 174);
            this.button_Form_CoarsePositioner.Name = "button_Form_CoarsePositioner";
            this.button_Form_CoarsePositioner.Size = new System.Drawing.Size(98, 30);
            this.button_Form_CoarsePositioner.TabIndex = 95;
            this.button_Form_CoarsePositioner.Text = "coarse position";
            this.button_Form_CoarsePositioner.UseVisualStyleBackColor = true;
            this.button_Form_CoarsePositioner.Click += new System.EventHandler(this.button_Form_CoarsePositioner_Click);
            // 
            // button_ShowImage
            // 
            this.button_ShowImage.Location = new System.Drawing.Point(246, 317);
            this.button_ShowImage.Name = "button_ShowImage";
            this.button_ShowImage.Size = new System.Drawing.Size(94, 32);
            this.button_ShowImage.TabIndex = 96;
            this.button_ShowImage.Text = "show image";
            this.button_ShowImage.UseVisualStyleBackColor = true;
            this.button_ShowImage.Click += new System.EventHandler(this.button_ShowImage_Click);
            // 
            // MainWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(48)))), ((int)(((byte)(203)))), ((int)(((byte)(197)))));
            this.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.back_ground_blue;
            this.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.ClientSize = new System.Drawing.Size(1196, 902);
            this.Controls.Add(this.button_ShowImage);
            this.Controls.Add(this.button_Form_CoarsePositioner);
            this.Controls.Add(this.label28);
            this.Controls.Add(this.textBox_T);
            this.Controls.Add(this.label27);
            this.Controls.Add(this.textBox_Task_PointsNumber);
            this.Controls.Add(this.label26);
            this.Controls.Add(this.textBox_TaskNumber);
            this.Controls.Add(this.label25);
            this.Controls.Add(this.textBox_TaskTime);
            this.Controls.Add(this.button_MultiTask);
            this.Controls.Add(this.button_CancelIndent);
            this.Controls.Add(this.label24);
            this.Controls.Add(this.textBox_TF_DC_Gain);
            this.Controls.Add(this.checkBox_T_ScanEnable);
            this.Controls.Add(this.checkBox_ForceSetAll);
            this.Controls.Add(this.label23);
            this.Controls.Add(this.trackBar_R01);
            this.Controls.Add(this.propertyGrid_AFM_Parameter);
            this.Controls.Add(this.button_StartIndent);
            this.Controls.Add(this.button_SetScanROI);
            this.Controls.Add(this.label22);
            this.Controls.Add(this.textBox_NumberOfFrameToScan);
            this.Controls.Add(this.button_StartSubWindow);
            this.Controls.Add(this.checkBox_ShowImage);
            this.Controls.Add(this.button_ClearImage);
            this.Controls.Add(this.button_SaveImage);
            this.Controls.Add(this.label_SystemState);
            this.Controls.Add(this.label21);
            this.Controls.Add(this.textBox_SetDeltaValueNm);
            this.Controls.Add(this.label20);
            this.Controls.Add(this.textBox_Sensitivity);
            this.Controls.Add(this.label19);
            this.Controls.Add(this.textBox_ScanRate);
            this.Controls.Add(this.label17);
            this.Controls.Add(this.textBox_YL);
            this.Controls.Add(this.label18);
            this.Controls.Add(this.textBox_XL);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.textBox_Dy);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.textBox_Dx);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.textBox_Ny);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.textBox_Nx);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.pictureBox_Height);
            this.Controls.Add(this.button_CoarseLiftUp);
            this.Controls.Add(this.button_XY_Reset);
            this.Controls.Add(this.button_XY_pause);
            this.Controls.Add(this.button_XY_Scan);
            this.Controls.Add(this.checkBox_Y_ScanEnable);
            this.Controls.Add(this.button_Z_Withdraw);
            this.Controls.Add(this.button_Z_Engage);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.button_CancelApproach);
            this.Controls.Add(this.button_Apporach);
            this.Controls.Add(this.button_SetParameters);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.trackBar_R1);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.trackBar_IC1R3);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.trackBar_R0);
            this.Controls.Add(this.richTextBox_serial_read);
            this.Controls.Add(this.checkBox_IC0_DO2);
            this.Controls.Add(this.checkBox_IC0_DO1);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.textBox_IC0_R3);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.textBox_IC0_R2);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox_IC0_R1);
            this.Controls.Add(this.listBox_SelectIdlePackage);
            this.Controls.Add(this.groupBox7);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.label_Start);
            this.Controls.Add(this.label_End);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBox_IC0_R0);
            this.Controls.Add(this.groupBox4);
            this.DoubleBuffered = true;
            this.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.MaximizeBox = false;
            this.Name = "MainWindow";
            this.Text = "AFM";
            this.Load += new System.EventHandler(this.MainWindow_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R0)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_IC1R3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Height)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_R01)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ListBox listBox_SelectIdlePackage;
        private System.Windows.Forms.TextBox textBox_IC0_R0;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Timer timer_CheckCOM;
        private System.Windows.Forms.ToolTip toolTip_Help;
        private System.Windows.Forms.Label label_End;
        private System.Windows.Forms.Label label_Start;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox textBox_Z_PID_P;
        private System.Windows.Forms.TextBox textBox_Z_PID_I;
        private System.Windows.Forms.TextBox textBox_Z_PID_D;
        private System.Windows.Forms.Button button_SetParameters;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox textBox_ComPortNO;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.Button button_ConnetComPort;
        private System.Windows.Forms.CheckBox checkBox_IC0_DO1;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBox_IC0_R1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBox_IC0_R3;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox textBox_IC0_R2;
        private System.Windows.Forms.CheckBox checkBox_IC0_DO2;
        private System.IO.Ports.SerialPort serialPort_Arduino;
        private System.Windows.Forms.RichTextBox richTextBox_serial_read;
        private System.Windows.Forms.TrackBar trackBar_R0;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TrackBar trackBar_IC1R3;
        private System.Windows.Forms.TrackBar trackBar_R1;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox textBox_BaudRate;
        private System.Windows.Forms.Button button_Apporach;
        private System.Windows.Forms.Button button_CancelApproach;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.CheckBox checkBox_COM_Transfer;
        private System.Windows.Forms.Button button_Z_Withdraw;
        private System.Windows.Forms.Button button_Z_Engage;
        private System.Windows.Forms.CheckBox checkBox_Y_ScanEnable;
        private System.Windows.Forms.Button button_XY_pause;
        private System.Windows.Forms.Button button_XY_Scan;
        private System.Windows.Forms.Button button_XY_Reset;
        private System.Windows.Forms.Timer timer_Approach;
        private System.Windows.Forms.Button button_CoarseLiftUp;
        private System.Windows.Forms.PictureBox pictureBox_Height;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.TextBox textBox_Nx;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TextBox textBox_Ny;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TextBox textBox_Dy;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.TextBox textBox_Dx;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.TextBox textBox_YL;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.TextBox textBox_XL;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.TextBox textBox_ScanRate;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.TextBox textBox_Sensitivity;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.TextBox textBox_SetDeltaValueNm;
        private System.Windows.Forms.Label label_SystemState;
        private System.Windows.Forms.Button button_SaveImage;
        private System.Windows.Forms.Button button_ClearImage;
        private System.Windows.Forms.CheckBox checkBox_ShowImage;
        private System.Windows.Forms.Button button_StartSubWindow;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.TextBox textBox_NumberOfFrameToScan;
        private System.Windows.Forms.Button button_SetScanROI;
        private System.Windows.Forms.Button button_StartIndent;
        private System.Windows.Forms.PropertyGrid propertyGrid_AFM_Parameter;
        private System.Windows.Forms.TrackBar trackBar_R01;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.CheckBox checkBox_ForceSetAll;
        private System.Windows.Forms.CheckBox checkBox_T_ScanEnable;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.TextBox textBox_TF_DC_Gain;
        private System.Windows.Forms.Button button_CancelIndent;
        private System.Windows.Forms.Button button_MultiTask;
        private System.Windows.Forms.TextBox textBox_TaskTime;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.TextBox textBox_TaskNumber;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.TextBox textBox_Task_PointsNumber;
        private System.Windows.Forms.TextBox textBox_T;
        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.Button button_Form_CoarsePositioner;
        private System.Windows.Forms.Button button_ShowImage;
    }
}

