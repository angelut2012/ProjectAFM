﻿namespace NameSpace_AFM_Project
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
            this.timer_CheckCOM = new System.Windows.Forms.Timer(this.components);
            this.toolTip_Help = new System.Windows.Forms.ToolTip(this.components);
            this.textBox_Z_PID_P = new System.Windows.Forms.TextBox();
            this.textBox_Z_PID_I = new System.Windows.Forms.TextBox();
            this.textBox_Z_PID_D = new System.Windows.Forms.TextBox();
            this.button_SetParameters = new System.Windows.Forms.Button();
            this.textBox_ComPortNO = new System.Windows.Forms.TextBox();
            this.button_ConnetComPort = new System.Windows.Forms.Button();
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
            this.button_Update_ParaPID = new System.Windows.Forms.Button();
            this.button_ScannerCalibration = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.button_Z_HybridWithdraw = new System.Windows.Forms.Button();
            this.button_XY_Scan = new System.Windows.Forms.Button();
            this.button_Z_HybridEngage = new System.Windows.Forms.Button();
            this.button_XY_pause = new System.Windows.Forms.Button();
            this.button_ClearImage = new System.Windows.Forms.Button();
            this.button_Z_Engage_Test = new System.Windows.Forms.Button();
            this.button_Z_Withdraw = new System.Windows.Forms.Button();
            this.checkBox_Y_ScanEnable = new System.Windows.Forms.CheckBox();
            this.label7 = new System.Windows.Forms.Label();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.checkBox_COM_Transfer = new System.Windows.Forms.CheckBox();
            this.label12 = new System.Windows.Forms.Label();
            this.serialPort_Arduino = new System.IO.Ports.SerialPort(this.components);
            this.button_Apporach = new System.Windows.Forms.Button();
            this.button_CancelApproach = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.timer_Approach = new System.Windows.Forms.Timer(this.components);
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
            this.button_SaveImage = new System.Windows.Forms.Button();
            this.checkBox_ShowImage = new System.Windows.Forms.CheckBox();
            this.button_StartSubWindow = new System.Windows.Forms.Button();
            this.label22 = new System.Windows.Forms.Label();
            this.button_SetScanROI = new System.Windows.Forms.Button();
            this.button_StartIndent = new System.Windows.Forms.Button();
            this.propertyGrid_AFM_Parameter = new System.Windows.Forms.PropertyGrid();
            this.checkBox_ForceSetAll = new System.Windows.Forms.CheckBox();
            this.label24 = new System.Windows.Forms.Label();
            this.button_CancelIndent = new System.Windows.Forms.Button();
            this.button_MultiTask = new System.Windows.Forms.Button();
            this.label25 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.label27 = new System.Windows.Forms.Label();
            this.label28 = new System.Windows.Forms.Label();
            this.button_Form_CoarsePositioner = new System.Windows.Forms.Button();
            this.button_ShowImage = new System.Windows.Forms.Button();
            this.radioButton_Topography = new System.Windows.Forms.RadioButton();
            this.pictureBox_AFM_Real_Image = new System.Windows.Forms.PictureBox();
            this.button_ExportData = new System.Windows.Forms.Button();
            this.groupBox_AFMImage = new System.Windows.Forms.GroupBox();
            this.checkBox_User_AutoStartScanAfterEngage = new System.Windows.Forms.CheckBox();
            this.button_User_AutoEngage = new System.Windows.Forms.Button();
            this.button_User_ImageScan_StartStop = new System.Windows.Forms.Button();
            this.radioButton_Deflection = new System.Windows.Forms.RadioButton();
            this.button_Coarse_MoveToHomePosition = new System.Windows.Forms.Button();
            this.label_Speed_Z = new System.Windows.Forms.Label();
            this.label_Encoder_Z = new System.Windows.Forms.Label();
            this.button_CantileverCalibration = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label_Speed_Y = new System.Windows.Forms.Label();
            this.button_SEMImage_VideoRecord = new System.Windows.Forms.Button();
            this.label_VideoRecord = new System.Windows.Forms.Label();
            this.label_ScreenShot = new System.Windows.Forms.Label();
            this.button_SEMImage_ScreenShot = new System.Windows.Forms.Button();
            this.groupBox_SEMPreview = new System.Windows.Forms.GroupBox();
            this.checkBox_ReferenceMarks = new System.Windows.Forms.CheckBox();
            this.pictureBox_SEMImage = new System.Windows.Forms.PictureBox();
            this.label_Encoder_Y = new System.Windows.Forms.Label();
            this.label_Speed_X = new System.Windows.Forms.Label();
            this.label_Axis_Z = new System.Windows.Forms.Label();
            this.label_Encoder_X = new System.Windows.Forms.Label();
            this.label_Axis_Y = new System.Windows.Forms.Label();
            this.label_Speed = new System.Windows.Forms.Label();
            this.label_Axis_X = new System.Windows.Forms.Label();
            this.label_Encoder = new System.Windows.Forms.Label();
            this.label_Axis = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.button_Optical = new System.Windows.Forms.Button();
            this.groupBox_Setup = new System.Windows.Forms.GroupBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label_Servicing = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.button_Servicing = new System.Windows.Forms.Button();
            this.label10 = new System.Windows.Forms.Label();
            this.groupBox_SystemStatus = new System.Windows.Forms.GroupBox();
            this.button_SEM = new System.Windows.Forms.Button();
            this.label23 = new System.Windows.Forms.Label();
            this.label_Controller = new System.Windows.Forms.Label();
            this.label_Joystick = new System.Windows.Forms.Label();
            this.label_Microscope = new System.Windows.Forms.Label();
            this.tabPage_TopoScan = new System.Windows.Forms.TabPage();
            this.comboBox_ParaImageQuality = new System.Windows.Forms.ComboBox();
            this.comboBox_ParaLines = new System.Windows.Forms.ComboBox();
            this.comboBox_ParaPixels = new System.Windows.Forms.ComboBox();
            this.comboBox_ParaScanArea = new System.Windows.Forms.ComboBox();
            this.label34 = new System.Windows.Forms.Label();
            this.label33 = new System.Windows.Forms.Label();
            this.label32 = new System.Windows.Forms.Label();
            this.label31 = new System.Windows.Forms.Label();
            this.button_AdvancedSettings = new System.Windows.Forms.Button();
            this.tabPage_Indentation = new System.Windows.Forms.TabPage();
            this.tabPage_Memory = new System.Windows.Forms.TabPage();
            this.tabControl = new System.Windows.Forms.TabControl();
            this.tabPage_backup = new System.Windows.Forms.TabPage();
            this.tabPage_Control = new System.Windows.Forms.TabPage();
            this.button_StartTask = new System.Windows.Forms.Button();
            this.button_WaveTest = new System.Windows.Forms.Button();
            this.button_DataCapture = new System.Windows.Forms.Button();
            this.tabPage_DataExport = new System.Windows.Forms.TabPage();
            this.label30 = new System.Windows.Forms.Label();
            this.textBox_FileName = new System.Windows.Forms.TextBox();
            this.label29 = new System.Windows.Forms.Label();
            this.textBox_DataPath = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.button_CoarseWithdraw = new System.Windows.Forms.Button();
            this.timer_Update_CoarseReadout = new System.Windows.Forms.Timer(this.components);
            this.timer_ScaningImageShow_RealTime = new System.Windows.Forms.Timer(this.components);
            this.label_SystemState = new System.Windows.Forms.Label();
            this.groupBox6.SuspendLayout();
            this.groupBox7.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_AFM_Real_Image)).BeginInit();
            this.groupBox_AFMImage.SuspendLayout();
            this.groupBox_SEMPreview.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_SEMImage)).BeginInit();
            this.groupBox_Setup.SuspendLayout();
            this.groupBox_SystemStatus.SuspendLayout();
            this.tabPage_TopoScan.SuspendLayout();
            this.tabPage_Indentation.SuspendLayout();
            this.tabControl.SuspendLayout();
            this.tabPage_backup.SuspendLayout();
            this.tabPage_Control.SuspendLayout();
            this.tabPage_DataExport.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // listBox_SelectIdlePackage
            // 
            this.listBox_SelectIdlePackage.BackColor = System.Drawing.SystemColors.Window;
            this.listBox_SelectIdlePackage.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.listBox_SelectIdlePackage.FormattingEnabled = true;
            this.listBox_SelectIdlePackage.Items.AddRange(new object[] {
            "PRC_Tsem_Tmcu",
            "SCSG_XYZ",
            "calibration",
            "drift_XPZ"});
            this.listBox_SelectIdlePackage.Location = new System.Drawing.Point(46, 24);
            this.listBox_SelectIdlePackage.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.listBox_SelectIdlePackage.Name = "listBox_SelectIdlePackage";
            this.listBox_SelectIdlePackage.Size = new System.Drawing.Size(86, 56);
            this.listBox_SelectIdlePackage.TabIndex = 1;
            this.toolTip_Help.SetToolTip(this.listBox_SelectIdlePackage, "Select an axis to operate.");
            this.listBox_SelectIdlePackage.SelectedIndexChanged += new System.EventHandler(this.listBox_Axis_SelectedIndexChanged);
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
            this.textBox_Z_PID_P.Location = new System.Drawing.Point(182, 276);
            this.textBox_Z_PID_P.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Z_PID_P.Name = "textBox_Z_PID_P";
            this.textBox_Z_PID_P.Size = new System.Drawing.Size(54, 20);
            this.textBox_Z_PID_P.TabIndex = 24;
            this.textBox_Z_PID_P.Text = "0.4";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_P, "Enter the step length here.");
            // 
            // textBox_Z_PID_I
            // 
            this.textBox_Z_PID_I.Location = new System.Drawing.Point(182, 302);
            this.textBox_Z_PID_I.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Z_PID_I.Name = "textBox_Z_PID_I";
            this.textBox_Z_PID_I.Size = new System.Drawing.Size(54, 20);
            this.textBox_Z_PID_I.TabIndex = 26;
            this.textBox_Z_PID_I.Text = "0.3";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_I, "Enter the step length here.");
            // 
            // textBox_Z_PID_D
            // 
            this.textBox_Z_PID_D.Location = new System.Drawing.Point(182, 328);
            this.textBox_Z_PID_D.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Z_PID_D.Name = "textBox_Z_PID_D";
            this.textBox_Z_PID_D.Size = new System.Drawing.Size(54, 20);
            this.textBox_Z_PID_D.TabIndex = 25;
            this.textBox_Z_PID_D.Text = "0.00005";
            this.toolTip_Help.SetToolTip(this.textBox_Z_PID_D, "Enter the step length here.");
            // 
            // button_SetParameters
            // 
            this.button_SetParameters.BackColor = System.Drawing.Color.Transparent;
            this.button_SetParameters.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.button_SetParameters.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button_SetParameters.Location = new System.Drawing.Point(163, 190);
            this.button_SetParameters.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_SetParameters.Name = "button_SetParameters";
            this.button_SetParameters.Size = new System.Drawing.Size(81, 41);
            this.button_SetParameters.TabIndex = 11;
            this.button_SetParameters.Text = "Para Update";
            this.toolTip_Help.SetToolTip(this.button_SetParameters, "set parameters");
            this.button_SetParameters.UseVisualStyleBackColor = false;
            this.button_SetParameters.Click += new System.EventHandler(this.button_SetParameters_Click);
            // 
            // textBox_ComPortNO
            // 
            this.textBox_ComPortNO.Location = new System.Drawing.Point(64, 17);
            this.textBox_ComPortNO.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_ComPortNO.Name = "textBox_ComPortNO";
            this.textBox_ComPortNO.Size = new System.Drawing.Size(44, 20);
            this.textBox_ComPortNO.TabIndex = 24;
            this.textBox_ComPortNO.Text = "23";
            this.toolTip_Help.SetToolTip(this.textBox_ComPortNO, "Enter the step length here.");
            // 
            // button_ConnetComPort
            // 
            this.button_ConnetComPort.BackColor = System.Drawing.Color.White;
            this.button_ConnetComPort.BackgroundImage = global::NameSpace_AFM_Project.Properties.Resources.Signal_off;
            this.button_ConnetComPort.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.button_ConnetComPort.Location = new System.Drawing.Point(700, 23);
            this.button_ConnetComPort.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_ConnetComPort.Name = "button_ConnetComPort";
            this.button_ConnetComPort.Size = new System.Drawing.Size(65, 65);
            this.button_ConnetComPort.TabIndex = 24;
            this.button_ConnetComPort.Text = "connect";
            this.toolTip_Help.SetToolTip(this.button_ConnetComPort, "Connect...");
            this.button_ConnetComPort.UseVisualStyleBackColor = false;
            this.button_ConnetComPort.Click += new System.EventHandler(this.button_ConnetComPort_Click);
            // 
            // textBox_BaudRate
            // 
            this.textBox_BaudRate.Location = new System.Drawing.Point(64, 33);
            this.textBox_BaudRate.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_BaudRate.Name = "textBox_BaudRate";
            this.textBox_BaudRate.Size = new System.Drawing.Size(44, 20);
            this.textBox_BaudRate.TabIndex = 26;
            this.textBox_BaudRate.Text = "115200";
            this.toolTip_Help.SetToolTip(this.textBox_BaudRate, "Enter the step length here.");
            // 
            // button_XY_Reset
            // 
            this.button_XY_Reset.Location = new System.Drawing.Point(14, 118);
            this.button_XY_Reset.Margin = new System.Windows.Forms.Padding(2);
            this.button_XY_Reset.Name = "button_XY_Reset";
            this.button_XY_Reset.Size = new System.Drawing.Size(67, 35);
            this.button_XY_Reset.TabIndex = 48;
            this.button_XY_Reset.Text = "reset";
            this.toolTip_Help.SetToolTip(this.button_XY_Reset, "stop xy scan and xy move to XL YL");
            this.button_XY_Reset.UseVisualStyleBackColor = true;
            this.button_XY_Reset.Click += new System.EventHandler(this.button_XY_Reset_Click);
            // 
            // textBox_Nx
            // 
            this.textBox_Nx.Location = new System.Drawing.Point(104, 11);
            this.textBox_Nx.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Nx.Name = "textBox_Nx";
            this.textBox_Nx.Size = new System.Drawing.Size(38, 20);
            this.textBox_Nx.TabIndex = 30;
            this.textBox_Nx.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_Nx, "Enter the step length here.");
            // 
            // textBox_Ny
            // 
            this.textBox_Ny.Location = new System.Drawing.Point(104, 35);
            this.textBox_Ny.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Ny.Name = "textBox_Ny";
            this.textBox_Ny.Size = new System.Drawing.Size(38, 20);
            this.textBox_Ny.TabIndex = 53;
            this.textBox_Ny.Text = "128";
            this.toolTip_Help.SetToolTip(this.textBox_Ny, "Enter the step length here.");
            // 
            // textBox_Dy
            // 
            this.textBox_Dy.Location = new System.Drawing.Point(104, 84);
            this.textBox_Dy.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Dy.Name = "textBox_Dy";
            this.textBox_Dy.Size = new System.Drawing.Size(38, 20);
            this.textBox_Dy.TabIndex = 57;
            this.textBox_Dy.Text = "22000";
            this.toolTip_Help.SetToolTip(this.textBox_Dy, "Enter the step length here.");
            // 
            // textBox_Dx
            // 
            this.textBox_Dx.Location = new System.Drawing.Point(104, 60);
            this.textBox_Dx.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Dx.Name = "textBox_Dx";
            this.textBox_Dx.Size = new System.Drawing.Size(38, 20);
            this.textBox_Dx.TabIndex = 55;
            this.textBox_Dx.Text = "22000";
            this.toolTip_Help.SetToolTip(this.textBox_Dx, "Enter the step length here.");
            // 
            // textBox_YL
            // 
            this.textBox_YL.Location = new System.Drawing.Point(104, 128);
            this.textBox_YL.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_YL.Name = "textBox_YL";
            this.textBox_YL.Size = new System.Drawing.Size(38, 20);
            this.textBox_YL.TabIndex = 61;
            this.textBox_YL.Text = "5000";
            this.toolTip_Help.SetToolTip(this.textBox_YL, "Enter the step length here.");
            // 
            // textBox_XL
            // 
            this.textBox_XL.Location = new System.Drawing.Point(104, 106);
            this.textBox_XL.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_XL.Name = "textBox_XL";
            this.textBox_XL.Size = new System.Drawing.Size(38, 20);
            this.textBox_XL.TabIndex = 59;
            this.textBox_XL.Text = "5000";
            this.toolTip_Help.SetToolTip(this.textBox_XL, "Enter the step length here.");
            // 
            // textBox_ScanRate
            // 
            this.textBox_ScanRate.Location = new System.Drawing.Point(104, 152);
            this.textBox_ScanRate.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_ScanRate.Name = "textBox_ScanRate";
            this.textBox_ScanRate.Size = new System.Drawing.Size(38, 20);
            this.textBox_ScanRate.TabIndex = 63;
            this.textBox_ScanRate.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_ScanRate, "Enter the step length here.");
            // 
            // textBox_Sensitivity
            // 
            this.textBox_Sensitivity.Location = new System.Drawing.Point(103, 176);
            this.textBox_Sensitivity.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Sensitivity.Name = "textBox_Sensitivity";
            this.textBox_Sensitivity.Size = new System.Drawing.Size(38, 20);
            this.textBox_Sensitivity.TabIndex = 65;
            this.textBox_Sensitivity.Text = "82";
            this.toolTip_Help.SetToolTip(this.textBox_Sensitivity, "Enter the step length here.");
            // 
            // textBox_SetDeltaValueNm
            // 
            this.textBox_SetDeltaValueNm.Location = new System.Drawing.Point(104, 201);
            this.textBox_SetDeltaValueNm.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_SetDeltaValueNm.Name = "textBox_SetDeltaValueNm";
            this.textBox_SetDeltaValueNm.Size = new System.Drawing.Size(38, 20);
            this.textBox_SetDeltaValueNm.TabIndex = 67;
            this.textBox_SetDeltaValueNm.Text = "20";
            this.toolTip_Help.SetToolTip(this.textBox_SetDeltaValueNm, "Enter the step length here.");
            // 
            // textBox_NumberOfFrameToScan
            // 
            this.textBox_NumberOfFrameToScan.Location = new System.Drawing.Point(103, 223);
            this.textBox_NumberOfFrameToScan.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_NumberOfFrameToScan.Name = "textBox_NumberOfFrameToScan";
            this.textBox_NumberOfFrameToScan.Size = new System.Drawing.Size(38, 20);
            this.textBox_NumberOfFrameToScan.TabIndex = 75;
            this.textBox_NumberOfFrameToScan.Text = "10";
            this.toolTip_Help.SetToolTip(this.textBox_NumberOfFrameToScan, "Enter the step length here.");
            // 
            // textBox_TF_DC_Gain
            // 
            this.textBox_TF_DC_Gain.Location = new System.Drawing.Point(98, 149);
            this.textBox_TF_DC_Gain.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_TF_DC_Gain.Name = "textBox_TF_DC_Gain";
            this.textBox_TF_DC_Gain.Size = new System.Drawing.Size(38, 20);
            this.textBox_TF_DC_Gain.TabIndex = 84;
            this.textBox_TF_DC_Gain.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_TF_DC_Gain, "Enter the step length here.");
            // 
            // textBox_TaskTime
            // 
            this.textBox_TaskTime.Location = new System.Drawing.Point(12, 276);
            this.textBox_TaskTime.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_TaskTime.Name = "textBox_TaskTime";
            this.textBox_TaskTime.Size = new System.Drawing.Size(60, 20);
            this.textBox_TaskTime.TabIndex = 87;
            this.textBox_TaskTime.Text = "20";
            this.toolTip_Help.SetToolTip(this.textBox_TaskTime, "Enter the step length here.");
            // 
            // textBox_TaskNumber
            // 
            this.textBox_TaskNumber.Location = new System.Drawing.Point(12, 317);
            this.textBox_TaskNumber.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_TaskNumber.Name = "textBox_TaskNumber";
            this.textBox_TaskNumber.Size = new System.Drawing.Size(60, 20);
            this.textBox_TaskNumber.TabIndex = 89;
            this.textBox_TaskNumber.Text = "11";
            this.toolTip_Help.SetToolTip(this.textBox_TaskNumber, "Enter the step length here.");
            // 
            // textBox_Task_PointsNumber
            // 
            this.textBox_Task_PointsNumber.Location = new System.Drawing.Point(12, 367);
            this.textBox_Task_PointsNumber.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_Task_PointsNumber.Name = "textBox_Task_PointsNumber";
            this.textBox_Task_PointsNumber.Size = new System.Drawing.Size(60, 20);
            this.textBox_Task_PointsNumber.TabIndex = 91;
            this.textBox_Task_PointsNumber.Text = "1";
            this.toolTip_Help.SetToolTip(this.textBox_Task_PointsNumber, "Enter the step length here.");
            // 
            // textBox_T
            // 
            this.textBox_T.Location = new System.Drawing.Point(12, 419);
            this.textBox_T.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.textBox_T.Name = "textBox_T";
            this.textBox_T.Size = new System.Drawing.Size(60, 20);
            this.textBox_T.TabIndex = 93;
            this.textBox_T.Text = "25";
            this.toolTip_Help.SetToolTip(this.textBox_T, "Enter the step length here.");
            // 
            // button_Update_ParaPID
            // 
            this.button_Update_ParaPID.BackColor = System.Drawing.Color.Transparent;
            this.button_Update_ParaPID.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.button_Update_ParaPID.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button_Update_ParaPID.Location = new System.Drawing.Point(143, 376);
            this.button_Update_ParaPID.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_Update_ParaPID.Name = "button_Update_ParaPID";
            this.button_Update_ParaPID.Size = new System.Drawing.Size(81, 41);
            this.button_Update_ParaPID.TabIndex = 86;
            this.button_Update_ParaPID.Text = "PID Update";
            this.toolTip_Help.SetToolTip(this.button_Update_ParaPID, "set parameters");
            this.button_Update_ParaPID.UseVisualStyleBackColor = false;
            this.button_Update_ParaPID.Click += new System.EventHandler(this.button_Update_ParaPID_Click);
            // 
            // button_ScannerCalibration
            // 
            this.button_ScannerCalibration.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_ScannerCalibration.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_ScannerCalibration.BackgroundImage")));
            this.button_ScannerCalibration.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.button_ScannerCalibration.Location = new System.Drawing.Point(127, 23);
            this.button_ScannerCalibration.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_ScannerCalibration.Name = "button_ScannerCalibration";
            this.button_ScannerCalibration.Size = new System.Drawing.Size(65, 65);
            this.button_ScannerCalibration.TabIndex = 0;
            this.toolTip_Help.SetToolTip(this.button_ScannerCalibration, "to calibrate the range of SCSG sensor on XYZ fine scanner.");
            this.button_ScannerCalibration.UseVisualStyleBackColor = false;
            this.button_ScannerCalibration.Click += new System.EventHandler(this.button_ScannerCalibration_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.Color.Transparent;
            this.label1.Location = new System.Drawing.Point(136, 276);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(41, 13);
            this.label1.TabIndex = 27;
            this.label1.Text = "PID_P:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.BackColor = System.Drawing.Color.Transparent;
            this.label5.Location = new System.Drawing.Point(136, 302);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(37, 13);
            this.label5.TabIndex = 28;
            this.label5.Text = "PID_I:";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.BackColor = System.Drawing.Color.Transparent;
            this.label6.Location = new System.Drawing.Point(136, 330);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(42, 13);
            this.label6.TabIndex = 29;
            this.label6.Text = "PID_D:";
            // 
            // groupBox6
            // 
            this.groupBox6.BackColor = System.Drawing.Color.Transparent;
            this.groupBox6.Controls.Add(this.button_Z_HybridWithdraw);
            this.groupBox6.Controls.Add(this.button_XY_Scan);
            this.groupBox6.Controls.Add(this.button_Z_HybridEngage);
            this.groupBox6.Controls.Add(this.button_XY_Reset);
            this.groupBox6.Controls.Add(this.button_XY_pause);
            this.groupBox6.Controls.Add(this.button_ClearImage);
            this.groupBox6.Controls.Add(this.button_Z_Engage_Test);
            this.groupBox6.Controls.Add(this.button_Z_Withdraw);
            this.groupBox6.Controls.Add(this.checkBox_Y_ScanEnable);
            this.groupBox6.Location = new System.Drawing.Point(365, 77);
            this.groupBox6.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox6.Size = new System.Drawing.Size(214, 269);
            this.groupBox6.TabIndex = 21;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Scanning";
            // 
            // button_Z_HybridWithdraw
            // 
            this.button_Z_HybridWithdraw.Location = new System.Drawing.Point(118, 204);
            this.button_Z_HybridWithdraw.Margin = new System.Windows.Forms.Padding(2);
            this.button_Z_HybridWithdraw.Name = "button_Z_HybridWithdraw";
            this.button_Z_HybridWithdraw.Size = new System.Drawing.Size(67, 35);
            this.button_Z_HybridWithdraw.TabIndex = 71;
            this.button_Z_HybridWithdraw.Text = "hybrid withdraw";
            this.button_Z_HybridWithdraw.UseVisualStyleBackColor = true;
            this.button_Z_HybridWithdraw.Click += new System.EventHandler(this.button_Z_HybridWithdraw_Click);
            // 
            // button_XY_Scan
            // 
            this.button_XY_Scan.Location = new System.Drawing.Point(14, 16);
            this.button_XY_Scan.Margin = new System.Windows.Forms.Padding(2);
            this.button_XY_Scan.Name = "button_XY_Scan";
            this.button_XY_Scan.Size = new System.Drawing.Size(67, 35);
            this.button_XY_Scan.TabIndex = 46;
            this.button_XY_Scan.Text = "start XY scan";
            this.button_XY_Scan.UseVisualStyleBackColor = true;
            this.button_XY_Scan.Click += new System.EventHandler(this.button_XY_Scan_Click);
            // 
            // button_Z_HybridEngage
            // 
            this.button_Z_HybridEngage.Location = new System.Drawing.Point(14, 204);
            this.button_Z_HybridEngage.Margin = new System.Windows.Forms.Padding(2);
            this.button_Z_HybridEngage.Name = "button_Z_HybridEngage";
            this.button_Z_HybridEngage.Size = new System.Drawing.Size(96, 35);
            this.button_Z_HybridEngage.TabIndex = 72;
            this.button_Z_HybridEngage.Text = "hybrid engage scan";
            this.button_Z_HybridEngage.UseVisualStyleBackColor = true;
            this.button_Z_HybridEngage.Click += new System.EventHandler(this.button_Z_HybridEngage_Click);
            // 
            // button_XY_pause
            // 
            this.button_XY_pause.Location = new System.Drawing.Point(14, 66);
            this.button_XY_pause.Margin = new System.Windows.Forms.Padding(2);
            this.button_XY_pause.Name = "button_XY_pause";
            this.button_XY_pause.Size = new System.Drawing.Size(67, 35);
            this.button_XY_pause.TabIndex = 47;
            this.button_XY_pause.Text = "pause";
            this.button_XY_pause.UseVisualStyleBackColor = true;
            this.button_XY_pause.Click += new System.EventHandler(this.button_XY_pause_Click);
            // 
            // button_ClearImage
            // 
            this.button_ClearImage.Location = new System.Drawing.Point(118, 118);
            this.button_ClearImage.Margin = new System.Windows.Forms.Padding(2);
            this.button_ClearImage.Name = "button_ClearImage";
            this.button_ClearImage.Size = new System.Drawing.Size(67, 35);
            this.button_ClearImage.TabIndex = 70;
            this.button_ClearImage.Text = "clear image";
            this.button_ClearImage.UseVisualStyleBackColor = true;
            this.button_ClearImage.Click += new System.EventHandler(this.button_ClearImage_Click);
            // 
            // button_Z_Engage_Test
            // 
            this.button_Z_Engage_Test.Location = new System.Drawing.Point(118, 16);
            this.button_Z_Engage_Test.Margin = new System.Windows.Forms.Padding(2);
            this.button_Z_Engage_Test.Name = "button_Z_Engage_Test";
            this.button_Z_Engage_Test.Size = new System.Drawing.Size(67, 35);
            this.button_Z_Engage_Test.TabIndex = 44;
            this.button_Z_Engage_Test.Text = "test engage";
            this.button_Z_Engage_Test.UseVisualStyleBackColor = true;
            this.button_Z_Engage_Test.Click += new System.EventHandler(this.button_Z_EngageTest_Click);
            // 
            // button_Z_Withdraw
            // 
            this.button_Z_Withdraw.Location = new System.Drawing.Point(118, 67);
            this.button_Z_Withdraw.Margin = new System.Windows.Forms.Padding(2);
            this.button_Z_Withdraw.Name = "button_Z_Withdraw";
            this.button_Z_Withdraw.Size = new System.Drawing.Size(67, 35);
            this.button_Z_Withdraw.TabIndex = 45;
            this.button_Z_Withdraw.Text = "withdraw";
            this.button_Z_Withdraw.UseVisualStyleBackColor = true;
            this.button_Z_Withdraw.Click += new System.EventHandler(this.button_Z_Withdraw_Click);
            // 
            // checkBox_Y_ScanEnable
            // 
            this.checkBox_Y_ScanEnable.AutoSize = true;
            this.checkBox_Y_ScanEnable.Checked = true;
            this.checkBox_Y_ScanEnable.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBox_Y_ScanEnable.Location = new System.Drawing.Point(14, 171);
            this.checkBox_Y_ScanEnable.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.checkBox_Y_ScanEnable.Name = "checkBox_Y_ScanEnable";
            this.checkBox_Y_ScanEnable.Size = new System.Drawing.Size(76, 17);
            this.checkBox_Y_ScanEnable.TabIndex = 45;
            this.checkBox_Y_ScanEnable.Text = "Area Scan";
            this.checkBox_Y_ScanEnable.UseVisualStyleBackColor = true;
            this.checkBox_Y_ScanEnable.CheckedChanged += new System.EventHandler(this.checkBox_Y_ScanEnable_CheckedChanged);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.BackColor = System.Drawing.Color.Transparent;
            this.label7.Location = new System.Drawing.Point(4, 19);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(56, 13);
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
            this.groupBox7.Controls.Add(this.textBox_ComPortNO);
            this.groupBox7.Location = new System.Drawing.Point(395, 365);
            this.groupBox7.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox7.Size = new System.Drawing.Size(112, 87);
            this.groupBox7.TabIndex = 26;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "Com Connection";
            // 
            // checkBox_COM_Transfer
            // 
            this.checkBox_COM_Transfer.AutoSize = true;
            this.checkBox_COM_Transfer.Location = new System.Drawing.Point(10, 57);
            this.checkBox_COM_Transfer.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.checkBox_COM_Transfer.Name = "checkBox_COM_Transfer";
            this.checkBox_COM_Transfer.Size = new System.Drawing.Size(49, 17);
            this.checkBox_COM_Transfer.TabIndex = 44;
            this.checkBox_COM_Transfer.Text = "trans";
            this.checkBox_COM_Transfer.UseVisualStyleBackColor = true;
            this.checkBox_COM_Transfer.CheckedChanged += new System.EventHandler(this.checkBox_COM_Transfer_CheckedChanged);
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.BackColor = System.Drawing.Color.Transparent;
            this.label12.Location = new System.Drawing.Point(4, 35);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(56, 13);
            this.label12.TabIndex = 27;
            this.label12.Text = "Baud rate:";
            // 
            // serialPort_Arduino
            // 
            this.serialPort_Arduino.PortName = "COM_Arduino";
            // 
            // button_Apporach
            // 
            this.button_Apporach.Location = new System.Drawing.Point(10, 20);
            this.button_Apporach.Margin = new System.Windows.Forms.Padding(2);
            this.button_Apporach.Name = "button_Apporach";
            this.button_Apporach.Size = new System.Drawing.Size(63, 35);
            this.button_Apporach.TabIndex = 41;
            this.button_Apporach.Text = "Start";
            this.button_Apporach.UseVisualStyleBackColor = true;
            this.button_Apporach.Click += new System.EventHandler(this.button_Apporach_Click);
            // 
            // button_CancelApproach
            // 
            this.button_CancelApproach.Location = new System.Drawing.Point(89, 20);
            this.button_CancelApproach.Margin = new System.Windows.Forms.Padding(2);
            this.button_CancelApproach.Name = "button_CancelApproach";
            this.button_CancelApproach.Size = new System.Drawing.Size(58, 35);
            this.button_CancelApproach.TabIndex = 42;
            this.button_CancelApproach.Text = "Cancel";
            this.button_CancelApproach.UseVisualStyleBackColor = true;
            this.button_CancelApproach.Click += new System.EventHandler(this.button_ManuallyCancelApproach_Click);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(16, 175);
            this.button1.Margin = new System.Windows.Forms.Padding(2);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(88, 33);
            this.button1.TabIndex = 43;
            this.button1.Text = "reset mcu";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // timer_Approach
            // 
            this.timer_Approach.Interval = 6000;
            this.timer_Approach.Tick += new System.EventHandler(this.timerFunction_Appraoch);
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(2, 74);
            this.button2.Margin = new System.Windows.Forms.Padding(2);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(34, 23);
            this.button2.TabIndex = 51;
            this.button2.Text = "test";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.BackColor = System.Drawing.Color.Transparent;
            this.label13.Location = new System.Drawing.Point(3, 13);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(64, 13);
            this.label13.TabIndex = 30;
            this.label13.Text = "Scan Points";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.BackColor = System.Drawing.Color.Transparent;
            this.label14.Location = new System.Drawing.Point(3, 37);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(60, 13);
            this.label14.TabIndex = 52;
            this.label14.Text = "Scan Lines";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.BackColor = System.Drawing.Color.Transparent;
            this.label15.Location = new System.Drawing.Point(3, 85);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(88, 13);
            this.label15.TabIndex = 56;
            this.label15.Text = "Scan Size Y(nm):";
            // 
            // label16
            // 
            this.label16.AutoSize = true;
            this.label16.BackColor = System.Drawing.Color.Transparent;
            this.label16.Location = new System.Drawing.Point(3, 61);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(88, 13);
            this.label16.TabIndex = 54;
            this.label16.Text = "Scan Size X(nm):";
            // 
            // label17
            // 
            this.label17.AutoSize = true;
            this.label17.BackColor = System.Drawing.Color.Transparent;
            this.label17.Location = new System.Drawing.Point(3, 133);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(66, 13);
            this.label17.TabIndex = 60;
            this.label17.Text = "Y offset(nm):";
            // 
            // label18
            // 
            this.label18.AutoSize = true;
            this.label18.BackColor = System.Drawing.Color.Transparent;
            this.label18.Location = new System.Drawing.Point(3, 109);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(66, 13);
            this.label18.TabIndex = 58;
            this.label18.Text = "X offset(nm):";
            // 
            // label19
            // 
            this.label19.AutoSize = true;
            this.label19.BackColor = System.Drawing.Color.Transparent;
            this.label19.Location = new System.Drawing.Point(3, 157);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(77, 13);
            this.label19.TabIndex = 62;
            this.label19.Text = "Scan Rate(Hz)";
            // 
            // label20
            // 
            this.label20.AutoSize = true;
            this.label20.BackColor = System.Drawing.Color.Transparent;
            this.label20.Location = new System.Drawing.Point(2, 181);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(54, 13);
            this.label20.TabIndex = 64;
            this.label20.Text = "Sensitivity";
            // 
            // label21
            // 
            this.label21.AutoSize = true;
            this.label21.BackColor = System.Drawing.Color.Transparent;
            this.label21.Location = new System.Drawing.Point(3, 204);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(70, 13);
            this.label21.TabIndex = 66;
            this.label21.Text = "Set Point(nm)";
            // 
            // button_SaveImage
            // 
            this.button_SaveImage.Location = new System.Drawing.Point(122, 96);
            this.button_SaveImage.Margin = new System.Windows.Forms.Padding(2);
            this.button_SaveImage.Name = "button_SaveImage";
            this.button_SaveImage.Size = new System.Drawing.Size(100, 43);
            this.button_SaveImage.TabIndex = 69;
            this.button_SaveImage.Text = "save image";
            this.button_SaveImage.UseVisualStyleBackColor = true;
            this.button_SaveImage.Click += new System.EventHandler(this.button_SaveImage_Click);
            // 
            // checkBox_ShowImage
            // 
            this.checkBox_ShowImage.AutoSize = true;
            this.checkBox_ShowImage.Location = new System.Drawing.Point(50, 254);
            this.checkBox_ShowImage.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.checkBox_ShowImage.Name = "checkBox_ShowImage";
            this.checkBox_ShowImage.Size = new System.Drawing.Size(82, 17);
            this.checkBox_ShowImage.TabIndex = 72;
            this.checkBox_ShowImage.Text = "show image";
            this.checkBox_ShowImage.UseVisualStyleBackColor = true;
            this.checkBox_ShowImage.CheckedChanged += new System.EventHandler(this.checkBox_ShowImage_CheckedChanged);
            // 
            // button_StartSubWindow
            // 
            this.button_StartSubWindow.BackColor = System.Drawing.Color.White;
            this.button_StartSubWindow.Location = new System.Drawing.Point(614, 23);
            this.button_StartSubWindow.Margin = new System.Windows.Forms.Padding(2);
            this.button_StartSubWindow.Name = "button_StartSubWindow";
            this.button_StartSubWindow.Size = new System.Drawing.Size(65, 65);
            this.button_StartSubWindow.TabIndex = 73;
            this.button_StartSubWindow.Text = "subform";
            this.button_StartSubWindow.UseVisualStyleBackColor = false;
            this.button_StartSubWindow.Click += new System.EventHandler(this.button_StartSubWindow_Click);
            // 
            // label22
            // 
            this.label22.AutoSize = true;
            this.label22.BackColor = System.Drawing.Color.Transparent;
            this.label22.Location = new System.Drawing.Point(3, 229);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(69, 13);
            this.label22.TabIndex = 74;
            this.label22.Text = "Scan Frames";
            // 
            // button_SetScanROI
            // 
            this.button_SetScanROI.BackColor = System.Drawing.Color.White;
            this.button_SetScanROI.Location = new System.Drawing.Point(528, 23);
            this.button_SetScanROI.Margin = new System.Windows.Forms.Padding(2);
            this.button_SetScanROI.Name = "button_SetScanROI";
            this.button_SetScanROI.Size = new System.Drawing.Size(65, 65);
            this.button_SetScanROI.TabIndex = 76;
            this.button_SetScanROI.Text = "ROI";
            this.button_SetScanROI.UseVisualStyleBackColor = false;
            this.button_SetScanROI.Click += new System.EventHandler(this.button_SetScanROI_Click);
            // 
            // button_StartIndent
            // 
            this.button_StartIndent.Location = new System.Drawing.Point(93, 525);
            this.button_StartIndent.Margin = new System.Windows.Forms.Padding(2);
            this.button_StartIndent.Name = "button_StartIndent";
            this.button_StartIndent.Size = new System.Drawing.Size(68, 45);
            this.button_StartIndent.TabIndex = 77;
            this.button_StartIndent.Text = "start indentation";
            this.button_StartIndent.UseVisualStyleBackColor = true;
            this.button_StartIndent.Click += new System.EventHandler(this.button_Start_IndentationWindow_Click);
            // 
            // propertyGrid_AFM_Parameter
            // 
            this.propertyGrid_AFM_Parameter.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.propertyGrid_AFM_Parameter.Location = new System.Drawing.Point(48, 56);
            this.propertyGrid_AFM_Parameter.Margin = new System.Windows.Forms.Padding(2);
            this.propertyGrid_AFM_Parameter.Name = "propertyGrid_AFM_Parameter";
            this.propertyGrid_AFM_Parameter.Size = new System.Drawing.Size(289, 450);
            this.propertyGrid_AFM_Parameter.TabIndex = 78;
            // 
            // checkBox_ForceSetAll
            // 
            this.checkBox_ForceSetAll.AutoSize = true;
            this.checkBox_ForceSetAll.Checked = true;
            this.checkBox_ForceSetAll.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBox_ForceSetAll.Location = new System.Drawing.Point(176, 115);
            this.checkBox_ForceSetAll.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.checkBox_ForceSetAll.Name = "checkBox_ForceSetAll";
            this.checkBox_ForceSetAll.Size = new System.Drawing.Size(63, 17);
            this.checkBox_ForceSetAll.TabIndex = 81;
            this.checkBox_ForceSetAll.Text = "force all";
            this.checkBox_ForceSetAll.UseVisualStyleBackColor = true;
            // 
            // label24
            // 
            this.label24.AutoSize = true;
            this.label24.BackColor = System.Drawing.Color.Transparent;
            this.label24.Location = new System.Drawing.Point(27, 154);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(61, 13);
            this.label24.TabIndex = 83;
            this.label24.Text = "TF DC gain";
            // 
            // button_CancelIndent
            // 
            this.button_CancelIndent.Location = new System.Drawing.Point(194, 527);
            this.button_CancelIndent.Margin = new System.Windows.Forms.Padding(2);
            this.button_CancelIndent.Name = "button_CancelIndent";
            this.button_CancelIndent.Size = new System.Drawing.Size(68, 43);
            this.button_CancelIndent.TabIndex = 85;
            this.button_CancelIndent.Text = "cancel indent";
            this.button_CancelIndent.UseVisualStyleBackColor = true;
            this.button_CancelIndent.Click += new System.EventHandler(this.button_CancelIndent_Click);
            // 
            // button_MultiTask
            // 
            this.button_MultiTask.Location = new System.Drawing.Point(100, 308);
            this.button_MultiTask.Margin = new System.Windows.Forms.Padding(2);
            this.button_MultiTask.Name = "button_MultiTask";
            this.button_MultiTask.Size = new System.Drawing.Size(68, 45);
            this.button_MultiTask.TabIndex = 86;
            this.button_MultiTask.Text = "Multi Indent";
            this.button_MultiTask.UseVisualStyleBackColor = true;
            this.button_MultiTask.Click += new System.EventHandler(this.button_MultiTask_Click);
            // 
            // label25
            // 
            this.label25.AutoSize = true;
            this.label25.BackColor = System.Drawing.Color.Transparent;
            this.label25.Location = new System.Drawing.Point(18, 261);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(63, 13);
            this.label25.TabIndex = 88;
            this.label25.Text = "task time (s)";
            // 
            // label26
            // 
            this.label26.AutoSize = true;
            this.label26.BackColor = System.Drawing.Color.Transparent;
            this.label26.Location = new System.Drawing.Point(18, 300);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(65, 13);
            this.label26.TabIndex = 90;
            this.label26.Text = "task number";
            // 
            // label27
            // 
            this.label27.AutoSize = true;
            this.label27.BackColor = System.Drawing.Color.Transparent;
            this.label27.Location = new System.Drawing.Point(7, 350);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(111, 13);
            this.label27.TabIndex = 92;
            this.label27.Text = "task  number of points";
            // 
            // label28
            // 
            this.label28.AutoSize = true;
            this.label28.BackColor = System.Drawing.Color.Transparent;
            this.label28.Location = new System.Drawing.Point(18, 404);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(30, 13);
            this.label28.TabIndex = 94;
            this.label28.Text = "temp";
            // 
            // button_Form_CoarsePositioner
            // 
            this.button_Form_CoarsePositioner.BackColor = System.Drawing.Color.White;
            this.button_Form_CoarsePositioner.Location = new System.Drawing.Point(356, 23);
            this.button_Form_CoarsePositioner.Margin = new System.Windows.Forms.Padding(2);
            this.button_Form_CoarsePositioner.Name = "button_Form_CoarsePositioner";
            this.button_Form_CoarsePositioner.Size = new System.Drawing.Size(65, 65);
            this.button_Form_CoarsePositioner.TabIndex = 95;
            this.button_Form_CoarsePositioner.Text = "coarse position";
            this.button_Form_CoarsePositioner.UseVisualStyleBackColor = false;
            this.button_Form_CoarsePositioner.Click += new System.EventHandler(this.button_Form_CoarsePositioner_Click);
            // 
            // button_ShowImage
            // 
            this.button_ShowImage.BackColor = System.Drawing.Color.White;
            this.button_ShowImage.Location = new System.Drawing.Point(442, 23);
            this.button_ShowImage.Margin = new System.Windows.Forms.Padding(2);
            this.button_ShowImage.Name = "button_ShowImage";
            this.button_ShowImage.Size = new System.Drawing.Size(65, 65);
            this.button_ShowImage.TabIndex = 96;
            this.button_ShowImage.Text = "show image";
            this.button_ShowImage.UseVisualStyleBackColor = false;
            this.button_ShowImage.Click += new System.EventHandler(this.button_ShowImage_Click);
            // 
            // radioButton_Topography
            // 
            this.radioButton_Topography.AutoSize = true;
            this.radioButton_Topography.Location = new System.Drawing.Point(10, 30);
            this.radioButton_Topography.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.radioButton_Topography.Name = "radioButton_Topography";
            this.radioButton_Topography.Size = new System.Drawing.Size(82, 17);
            this.radioButton_Topography.TabIndex = 2;
            this.radioButton_Topography.TabStop = true;
            this.radioButton_Topography.Text = "Topography";
            this.radioButton_Topography.UseVisualStyleBackColor = true;
            // 
            // pictureBox_AFM_Real_Image
            // 
            this.pictureBox_AFM_Real_Image.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pictureBox_AFM_Real_Image.BackgroundImage")));
            this.pictureBox_AFM_Real_Image.InitialImage = null;
            this.pictureBox_AFM_Real_Image.Location = new System.Drawing.Point(20, 38);
            this.pictureBox_AFM_Real_Image.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.pictureBox_AFM_Real_Image.Name = "pictureBox_AFM_Real_Image";
            this.pictureBox_AFM_Real_Image.Size = new System.Drawing.Size(570, 570);
            this.pictureBox_AFM_Real_Image.TabIndex = 1;
            this.pictureBox_AFM_Real_Image.TabStop = false;
            // 
            // button_ExportData
            // 
            this.button_ExportData.Location = new System.Drawing.Point(122, 24);
            this.button_ExportData.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_ExportData.Name = "button_ExportData";
            this.button_ExportData.Size = new System.Drawing.Size(98, 50);
            this.button_ExportData.TabIndex = 0;
            this.button_ExportData.Text = "Export Data";
            this.button_ExportData.UseVisualStyleBackColor = true;
            // 
            // groupBox_AFMImage
            // 
            this.groupBox_AFMImage.BackColor = System.Drawing.Color.LightGray;
            this.groupBox_AFMImage.Controls.Add(this.checkBox_User_AutoStartScanAfterEngage);
            this.groupBox_AFMImage.Controls.Add(this.button_User_AutoEngage);
            this.groupBox_AFMImage.Controls.Add(this.button_User_ImageScan_StartStop);
            this.groupBox_AFMImage.Controls.Add(this.button_CancelIndent);
            this.groupBox_AFMImage.Controls.Add(this.button_StartIndent);
            this.groupBox_AFMImage.Controls.Add(this.propertyGrid_AFM_Parameter);
            this.groupBox_AFMImage.Controls.Add(this.groupBox6);
            this.groupBox_AFMImage.Controls.Add(this.groupBox7);
            this.groupBox_AFMImage.Location = new System.Drawing.Point(294, 168);
            this.groupBox_AFMImage.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_AFMImage.Name = "groupBox_AFMImage";
            this.groupBox_AFMImage.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_AFMImage.Size = new System.Drawing.Size(610, 640);
            this.groupBox_AFMImage.TabIndex = 103;
            this.groupBox_AFMImage.TabStop = false;
            this.groupBox_AFMImage.Text = "AFM Imaging Result";
            // 
            // checkBox_User_AutoStartScanAfterEngage
            // 
            this.checkBox_User_AutoStartScanAfterEngage.AutoSize = true;
            this.checkBox_User_AutoStartScanAfterEngage.Checked = true;
            this.checkBox_User_AutoStartScanAfterEngage.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBox_User_AutoStartScanAfterEngage.Location = new System.Drawing.Point(379, 522);
            this.checkBox_User_AutoStartScanAfterEngage.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.checkBox_User_AutoStartScanAfterEngage.Name = "checkBox_User_AutoStartScanAfterEngage";
            this.checkBox_User_AutoStartScanAfterEngage.Size = new System.Drawing.Size(159, 17);
            this.checkBox_User_AutoStartScanAfterEngage.TabIndex = 45;
            this.checkBox_User_AutoStartScanAfterEngage.Text = "auto start scan after engage";
            this.checkBox_User_AutoStartScanAfterEngage.UseVisualStyleBackColor = true;
            // 
            // button_User_AutoEngage
            // 
            this.button_User_AutoEngage.Location = new System.Drawing.Point(379, 478);
            this.button_User_AutoEngage.Margin = new System.Windows.Forms.Padding(2);
            this.button_User_AutoEngage.Name = "button_User_AutoEngage";
            this.button_User_AutoEngage.Size = new System.Drawing.Size(63, 35);
            this.button_User_AutoEngage.TabIndex = 44;
            this.button_User_AutoEngage.Text = "auto engage";
            this.button_User_AutoEngage.UseVisualStyleBackColor = true;
            this.button_User_AutoEngage.Click += new System.EventHandler(this.button_User_AutoEngage_Click);
            // 
            // button_User_ImageScan_StartStop
            // 
            this.button_User_ImageScan_StartStop.Location = new System.Drawing.Point(492, 478);
            this.button_User_ImageScan_StartStop.Margin = new System.Windows.Forms.Padding(2);
            this.button_User_ImageScan_StartStop.Name = "button_User_ImageScan_StartStop";
            this.button_User_ImageScan_StartStop.Size = new System.Drawing.Size(67, 35);
            this.button_User_ImageScan_StartStop.TabIndex = 73;
            this.button_User_ImageScan_StartStop.Text = "start";
            this.button_User_ImageScan_StartStop.UseVisualStyleBackColor = true;
            this.button_User_ImageScan_StartStop.Click += new System.EventHandler(this.button_User_ImageScan_StartStop_Click);
            // 
            // radioButton_Deflection
            // 
            this.radioButton_Deflection.AutoSize = true;
            this.radioButton_Deflection.Location = new System.Drawing.Point(10, 55);
            this.radioButton_Deflection.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.radioButton_Deflection.Name = "radioButton_Deflection";
            this.radioButton_Deflection.Size = new System.Drawing.Size(73, 17);
            this.radioButton_Deflection.TabIndex = 2;
            this.radioButton_Deflection.TabStop = true;
            this.radioButton_Deflection.Text = "Deflection";
            this.radioButton_Deflection.UseVisualStyleBackColor = true;
            // 
            // button_Coarse_MoveToHomePosition
            // 
            this.button_Coarse_MoveToHomePosition.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_Coarse_MoveToHomePosition.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_Coarse_MoveToHomePosition.BackgroundImage")));
            this.button_Coarse_MoveToHomePosition.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.button_Coarse_MoveToHomePosition.Font = new System.Drawing.Font("SimSun", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.button_Coarse_MoveToHomePosition.Location = new System.Drawing.Point(1189, 23);
            this.button_Coarse_MoveToHomePosition.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_Coarse_MoveToHomePosition.Name = "button_Coarse_MoveToHomePosition";
            this.button_Coarse_MoveToHomePosition.Size = new System.Drawing.Size(65, 65);
            this.button_Coarse_MoveToHomePosition.TabIndex = 97;
            this.button_Coarse_MoveToHomePosition.TextAlign = System.Drawing.ContentAlignment.BottomCenter;
            this.button_Coarse_MoveToHomePosition.UseVisualStyleBackColor = false;
            this.button_Coarse_MoveToHomePosition.Click += new System.EventHandler(this.button_Coarse_MoveToHomePosition_Click);
            // 
            // label_Speed_Z
            // 
            this.label_Speed_Z.AutoSize = true;
            this.label_Speed_Z.Location = new System.Drawing.Point(194, 199);
            this.label_Speed_Z.Name = "label_Speed_Z";
            this.label_Speed_Z.Size = new System.Drawing.Size(38, 13);
            this.label_Speed_Z.TabIndex = 2;
            this.label_Speed_Z.Text = "Speed";
            // 
            // label_Encoder_Z
            // 
            this.label_Encoder_Z.AutoSize = true;
            this.label_Encoder_Z.Location = new System.Drawing.Point(112, 199);
            this.label_Encoder_Z.Name = "label_Encoder_Z";
            this.label_Encoder_Z.Size = new System.Drawing.Size(13, 13);
            this.label_Encoder_Z.TabIndex = 2;
            this.label_Encoder_Z.Text = "0";
            this.label_Encoder_Z.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // button_CantileverCalibration
            // 
            this.button_CantileverCalibration.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_CantileverCalibration.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_CantileverCalibration.BackgroundImage")));
            this.button_CantileverCalibration.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.button_CantileverCalibration.Location = new System.Drawing.Point(19, 23);
            this.button_CantileverCalibration.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_CantileverCalibration.Name = "button_CantileverCalibration";
            this.button_CantileverCalibration.Size = new System.Drawing.Size(65, 65);
            this.button_CantileverCalibration.TabIndex = 0;
            this.button_CantileverCalibration.UseVisualStyleBackColor = false;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(23, 109);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(56, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "Calibration";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(24, 93);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(54, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "Cantilever";
            // 
            // label_Speed_Y
            // 
            this.label_Speed_Y.AutoSize = true;
            this.label_Speed_Y.Location = new System.Drawing.Point(196, 175);
            this.label_Speed_Y.Name = "label_Speed_Y";
            this.label_Speed_Y.Size = new System.Drawing.Size(29, 13);
            this.label_Speed_Y.TabIndex = 2;
            this.label_Speed_Y.Text = "Auto";
            // 
            // button_SEMImage_VideoRecord
            // 
            this.button_SEMImage_VideoRecord.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_SEMImage_VideoRecord.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_SEMImage_VideoRecord.BackgroundImage")));
            this.button_SEMImage_VideoRecord.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.button_SEMImage_VideoRecord.Location = new System.Drawing.Point(852, 176);
            this.button_SEMImage_VideoRecord.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_SEMImage_VideoRecord.Name = "button_SEMImage_VideoRecord";
            this.button_SEMImage_VideoRecord.Size = new System.Drawing.Size(60, 65);
            this.button_SEMImage_VideoRecord.TabIndex = 6;
            this.button_SEMImage_VideoRecord.UseVisualStyleBackColor = false;
            this.button_SEMImage_VideoRecord.Click += new System.EventHandler(this.button_SEMImage_VideoRecord_Click);
            // 
            // label_VideoRecord
            // 
            this.label_VideoRecord.AutoSize = true;
            this.label_VideoRecord.Location = new System.Drawing.Point(846, 245);
            this.label_VideoRecord.Name = "label_VideoRecord";
            this.label_VideoRecord.Size = new System.Drawing.Size(72, 13);
            this.label_VideoRecord.TabIndex = 6;
            this.label_VideoRecord.Text = "Video Record";
            // 
            // label_ScreenShot
            // 
            this.label_ScreenShot.AutoSize = true;
            this.label_ScreenShot.Location = new System.Drawing.Point(846, 154);
            this.label_ScreenShot.Name = "label_ScreenShot";
            this.label_ScreenShot.Size = new System.Drawing.Size(66, 13);
            this.label_ScreenShot.TabIndex = 6;
            this.label_ScreenShot.Text = "Screen Shot";
            // 
            // button_SEMImage_ScreenShot
            // 
            this.button_SEMImage_ScreenShot.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_SEMImage_ScreenShot.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_SEMImage_ScreenShot.BackgroundImage")));
            this.button_SEMImage_ScreenShot.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.button_SEMImage_ScreenShot.Location = new System.Drawing.Point(849, 77);
            this.button_SEMImage_ScreenShot.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_SEMImage_ScreenShot.Name = "button_SEMImage_ScreenShot";
            this.button_SEMImage_ScreenShot.Size = new System.Drawing.Size(60, 65);
            this.button_SEMImage_ScreenShot.TabIndex = 0;
            this.button_SEMImage_ScreenShot.UseVisualStyleBackColor = false;
            this.button_SEMImage_ScreenShot.Click += new System.EventHandler(this.button_SEMImage_ScreenShot_Click);
            // 
            // groupBox_SEMPreview
            // 
            this.groupBox_SEMPreview.Controls.Add(this.button_SEMImage_VideoRecord);
            this.groupBox_SEMPreview.Controls.Add(this.label_VideoRecord);
            this.groupBox_SEMPreview.Controls.Add(this.checkBox_ReferenceMarks);
            this.groupBox_SEMPreview.Controls.Add(this.button_SEMImage_ScreenShot);
            this.groupBox_SEMPreview.Controls.Add(this.pictureBox_AFM_Real_Image);
            this.groupBox_SEMPreview.Controls.Add(this.label_ScreenShot);
            this.groupBox_SEMPreview.Location = new System.Drawing.Point(930, 168);
            this.groupBox_SEMPreview.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_SEMPreview.Name = "groupBox_SEMPreview";
            this.groupBox_SEMPreview.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_SEMPreview.Size = new System.Drawing.Size(638, 640);
            this.groupBox_SEMPreview.TabIndex = 104;
            this.groupBox_SEMPreview.TabStop = false;
            this.groupBox_SEMPreview.Text = "SEM/Optical Image Preview";
            // 
            // checkBox_ReferenceMarks
            // 
            this.checkBox_ReferenceMarks.AutoSize = true;
            this.checkBox_ReferenceMarks.Location = new System.Drawing.Point(830, 283);
            this.checkBox_ReferenceMarks.Margin = new System.Windows.Forms.Padding(2);
            this.checkBox_ReferenceMarks.Name = "checkBox_ReferenceMarks";
            this.checkBox_ReferenceMarks.Size = new System.Drawing.Size(108, 17);
            this.checkBox_ReferenceMarks.TabIndex = 4;
            this.checkBox_ReferenceMarks.Text = "Reference Marks";
            this.checkBox_ReferenceMarks.UseVisualStyleBackColor = true;
            // 
            // pictureBox_SEMImage
            // 
            this.pictureBox_SEMImage.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("pictureBox_SEMImage.BackgroundImage")));
            this.pictureBox_SEMImage.Location = new System.Drawing.Point(1560, 637);
            this.pictureBox_SEMImage.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.pictureBox_SEMImage.Name = "pictureBox_SEMImage";
            this.pictureBox_SEMImage.Size = new System.Drawing.Size(600, 600);
            this.pictureBox_SEMImage.TabIndex = 1;
            this.pictureBox_SEMImage.TabStop = false;
            // 
            // label_Encoder_Y
            // 
            this.label_Encoder_Y.AutoSize = true;
            this.label_Encoder_Y.Location = new System.Drawing.Point(112, 175);
            this.label_Encoder_Y.Name = "label_Encoder_Y";
            this.label_Encoder_Y.Size = new System.Drawing.Size(13, 13);
            this.label_Encoder_Y.TabIndex = 2;
            this.label_Encoder_Y.Text = "0";
            this.label_Encoder_Y.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label_Speed_X
            // 
            this.label_Speed_X.AutoSize = true;
            this.label_Speed_X.Location = new System.Drawing.Point(196, 152);
            this.label_Speed_X.Name = "label_Speed_X";
            this.label_Speed_X.Size = new System.Drawing.Size(29, 13);
            this.label_Speed_X.TabIndex = 2;
            this.label_Speed_X.Text = "Auto";
            // 
            // label_Axis_Z
            // 
            this.label_Axis_Z.AutoSize = true;
            this.label_Axis_Z.Location = new System.Drawing.Point(28, 199);
            this.label_Axis_Z.Name = "label_Axis_Z";
            this.label_Axis_Z.Size = new System.Drawing.Size(14, 13);
            this.label_Axis_Z.TabIndex = 2;
            this.label_Axis_Z.Text = "Z";
            // 
            // label_Encoder_X
            // 
            this.label_Encoder_X.AutoSize = true;
            this.label_Encoder_X.ImageAlign = System.Drawing.ContentAlignment.MiddleLeft;
            this.label_Encoder_X.Location = new System.Drawing.Point(112, 152);
            this.label_Encoder_X.Name = "label_Encoder_X";
            this.label_Encoder_X.Size = new System.Drawing.Size(13, 13);
            this.label_Encoder_X.TabIndex = 2;
            this.label_Encoder_X.Text = "0";
            this.label_Encoder_X.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label_Axis_Y
            // 
            this.label_Axis_Y.AutoSize = true;
            this.label_Axis_Y.Location = new System.Drawing.Point(28, 175);
            this.label_Axis_Y.Name = "label_Axis_Y";
            this.label_Axis_Y.Size = new System.Drawing.Size(14, 13);
            this.label_Axis_Y.TabIndex = 2;
            this.label_Axis_Y.Text = "Y";
            // 
            // label_Speed
            // 
            this.label_Speed.AutoSize = true;
            this.label_Speed.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.label_Speed.Location = new System.Drawing.Point(190, 130);
            this.label_Speed.Name = "label_Speed";
            this.label_Speed.Size = new System.Drawing.Size(54, 17);
            this.label_Speed.TabIndex = 2;
            this.label_Speed.Text = "Speed";
            // 
            // label_Axis_X
            // 
            this.label_Axis_X.AutoSize = true;
            this.label_Axis_X.Location = new System.Drawing.Point(28, 152);
            this.label_Axis_X.Name = "label_Axis_X";
            this.label_Axis_X.Size = new System.Drawing.Size(14, 13);
            this.label_Axis_X.TabIndex = 2;
            this.label_Axis_X.Text = "X";
            // 
            // label_Encoder
            // 
            this.label_Encoder.AutoSize = true;
            this.label_Encoder.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.label_Encoder.Location = new System.Drawing.Point(94, 130);
            this.label_Encoder.Name = "label_Encoder";
            this.label_Encoder.Size = new System.Drawing.Size(68, 17);
            this.label_Encoder.TabIndex = 2;
            this.label_Encoder.Text = "Encoder";
            // 
            // label_Axis
            // 
            this.label_Axis.AutoSize = true;
            this.label_Axis.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.label_Axis.Location = new System.Drawing.Point(21, 130);
            this.label_Axis.Name = "label_Axis";
            this.label_Axis.Size = new System.Drawing.Size(37, 17);
            this.label_Axis.TabIndex = 2;
            this.label_Axis.Text = "Axis";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.BackColor = System.Drawing.SystemColors.Highlight;
            this.label4.Location = new System.Drawing.Point(159, 32);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(13, 13);
            this.label4.TabIndex = 2;
            this.label4.Text = "  ";
            // 
            // button_Optical
            // 
            this.button_Optical.Location = new System.Drawing.Point(20, 78);
            this.button_Optical.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_Optical.Name = "button_Optical";
            this.button_Optical.Size = new System.Drawing.Size(88, 35);
            this.button_Optical.TabIndex = 1;
            this.button_Optical.Text = "Optical";
            this.button_Optical.UseVisualStyleBackColor = true;
            // 
            // groupBox_Setup
            // 
            this.groupBox_Setup.BackColor = System.Drawing.Color.Transparent;
            this.groupBox_Setup.Controls.Add(this.label11);
            this.groupBox_Setup.Controls.Add(this.label_Servicing);
            this.groupBox_Setup.Controls.Add(this.label8);
            this.groupBox_Setup.Controls.Add(this.label9);
            this.groupBox_Setup.Controls.Add(this.button_Servicing);
            this.groupBox_Setup.Controls.Add(this.button_Coarse_MoveToHomePosition);
            this.groupBox_Setup.Controls.Add(this.button_ConnetComPort);
            this.groupBox_Setup.Controls.Add(this.button_ScannerCalibration);
            this.groupBox_Setup.Controls.Add(this.button_CantileverCalibration);
            this.groupBox_Setup.Controls.Add(this.label2);
            this.groupBox_Setup.Controls.Add(this.button_StartSubWindow);
            this.groupBox_Setup.Controls.Add(this.button_SetScanROI);
            this.groupBox_Setup.Controls.Add(this.button_ShowImage);
            this.groupBox_Setup.Controls.Add(this.label3);
            this.groupBox_Setup.Controls.Add(this.button_Form_CoarsePositioner);
            this.groupBox_Setup.Location = new System.Drawing.Point(294, 13);
            this.groupBox_Setup.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_Setup.Name = "groupBox_Setup";
            this.groupBox_Setup.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_Setup.Size = new System.Drawing.Size(1274, 137);
            this.groupBox_Setup.TabIndex = 105;
            this.groupBox_Setup.TabStop = false;
            this.groupBox_Setup.Text = "Setup";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(1205, 95);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(39, 13);
            this.label11.TabIndex = 108;
            this.label11.Text = "HOME";
            // 
            // label_Servicing
            // 
            this.label_Servicing.AutoSize = true;
            this.label_Servicing.Location = new System.Drawing.Point(242, 93);
            this.label_Servicing.Name = "label_Servicing";
            this.label_Servicing.Size = new System.Drawing.Size(51, 13);
            this.label_Servicing.TabIndex = 5;
            this.label_Servicing.Text = "Servicing";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(131, 109);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(56, 13);
            this.label8.TabIndex = 3;
            this.label8.Text = "Calibration";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(136, 93);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(47, 13);
            this.label9.TabIndex = 4;
            this.label9.Text = "Scanner";
            // 
            // button_Servicing
            // 
            this.button_Servicing.BackColor = System.Drawing.SystemColors.HighlightText;
            this.button_Servicing.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("button_Servicing.BackgroundImage")));
            this.button_Servicing.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
            this.button_Servicing.Location = new System.Drawing.Point(235, 23);
            this.button_Servicing.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_Servicing.Name = "button_Servicing";
            this.button_Servicing.Size = new System.Drawing.Size(65, 65);
            this.button_Servicing.TabIndex = 0;
            this.button_Servicing.UseVisualStyleBackColor = false;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.BackColor = System.Drawing.SystemColors.Highlight;
            this.label10.Location = new System.Drawing.Point(159, 65);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(13, 13);
            this.label10.TabIndex = 2;
            this.label10.Text = "  ";
            // 
            // groupBox_SystemStatus
            // 
            this.groupBox_SystemStatus.Controls.Add(this.label_Speed_Z);
            this.groupBox_SystemStatus.Controls.Add(this.label_Encoder_Z);
            this.groupBox_SystemStatus.Controls.Add(this.label_Speed_Y);
            this.groupBox_SystemStatus.Controls.Add(this.label_Encoder_Y);
            this.groupBox_SystemStatus.Controls.Add(this.label_Speed_X);
            this.groupBox_SystemStatus.Controls.Add(this.label_Axis_Z);
            this.groupBox_SystemStatus.Controls.Add(this.label_Encoder_X);
            this.groupBox_SystemStatus.Controls.Add(this.label_Axis_Y);
            this.groupBox_SystemStatus.Controls.Add(this.label_Speed);
            this.groupBox_SystemStatus.Controls.Add(this.label_Axis_X);
            this.groupBox_SystemStatus.Controls.Add(this.label_Encoder);
            this.groupBox_SystemStatus.Controls.Add(this.label_Axis);
            this.groupBox_SystemStatus.Controls.Add(this.label4);
            this.groupBox_SystemStatus.Controls.Add(this.button_Optical);
            this.groupBox_SystemStatus.Controls.Add(this.label10);
            this.groupBox_SystemStatus.Controls.Add(this.button_SEM);
            this.groupBox_SystemStatus.Controls.Add(this.label23);
            this.groupBox_SystemStatus.Controls.Add(this.label_Controller);
            this.groupBox_SystemStatus.Controls.Add(this.label_Joystick);
            this.groupBox_SystemStatus.Controls.Add(this.label_Microscope);
            this.groupBox_SystemStatus.Location = new System.Drawing.Point(12, 13);
            this.groupBox_SystemStatus.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_SystemStatus.Name = "groupBox_SystemStatus";
            this.groupBox_SystemStatus.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.groupBox_SystemStatus.Size = new System.Drawing.Size(260, 225);
            this.groupBox_SystemStatus.TabIndex = 101;
            this.groupBox_SystemStatus.TabStop = false;
            this.groupBox_SystemStatus.Text = "System Status";
            // 
            // button_SEM
            // 
            this.button_SEM.Location = new System.Drawing.Point(20, 28);
            this.button_SEM.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_SEM.Name = "button_SEM";
            this.button_SEM.Size = new System.Drawing.Size(88, 35);
            this.button_SEM.TabIndex = 0;
            this.button_SEM.Text = "SEM";
            this.button_SEM.UseVisualStyleBackColor = true;
            this.button_SEM.Click += new System.EventHandler(this.button_SEM_Click);
            // 
            // label23
            // 
            this.label23.AutoSize = true;
            this.label23.BackColor = System.Drawing.SystemColors.Highlight;
            this.label23.Location = new System.Drawing.Point(159, 95);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(13, 13);
            this.label23.TabIndex = 2;
            this.label23.Text = "  ";
            // 
            // label_Controller
            // 
            this.label_Controller.AutoSize = true;
            this.label_Controller.Location = new System.Drawing.Point(177, 32);
            this.label_Controller.Name = "label_Controller";
            this.label_Controller.Size = new System.Drawing.Size(51, 13);
            this.label_Controller.TabIndex = 2;
            this.label_Controller.Text = "Controller";
            // 
            // label_Joystick
            // 
            this.label_Joystick.AutoSize = true;
            this.label_Joystick.Location = new System.Drawing.Point(177, 95);
            this.label_Joystick.Name = "label_Joystick";
            this.label_Joystick.Size = new System.Drawing.Size(45, 13);
            this.label_Joystick.TabIndex = 2;
            this.label_Joystick.Text = "Joystick";
            // 
            // label_Microscope
            // 
            this.label_Microscope.AutoSize = true;
            this.label_Microscope.Location = new System.Drawing.Point(177, 65);
            this.label_Microscope.Name = "label_Microscope";
            this.label_Microscope.Size = new System.Drawing.Size(62, 13);
            this.label_Microscope.TabIndex = 2;
            this.label_Microscope.Text = "Microscope";
            // 
            // tabPage_TopoScan
            // 
            this.tabPage_TopoScan.BackColor = System.Drawing.Color.LightGray;
            this.tabPage_TopoScan.Controls.Add(this.comboBox_ParaImageQuality);
            this.tabPage_TopoScan.Controls.Add(this.comboBox_ParaLines);
            this.tabPage_TopoScan.Controls.Add(this.comboBox_ParaPixels);
            this.tabPage_TopoScan.Controls.Add(this.comboBox_ParaScanArea);
            this.tabPage_TopoScan.Controls.Add(this.label34);
            this.tabPage_TopoScan.Controls.Add(this.label33);
            this.tabPage_TopoScan.Controls.Add(this.label32);
            this.tabPage_TopoScan.Controls.Add(this.label31);
            this.tabPage_TopoScan.Controls.Add(this.button_AdvancedSettings);
            this.tabPage_TopoScan.Location = new System.Drawing.Point(4, 22);
            this.tabPage_TopoScan.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_TopoScan.Name = "tabPage_TopoScan";
            this.tabPage_TopoScan.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_TopoScan.Size = new System.Drawing.Size(252, 460);
            this.tabPage_TopoScan.TabIndex = 0;
            this.tabPage_TopoScan.Text = "TopoScan";
            // 
            // comboBox_ParaImageQuality
            // 
            this.comboBox_ParaImageQuality.BackColor = System.Drawing.Color.LightGray;
            this.comboBox_ParaImageQuality.FormattingEnabled = true;
            this.comboBox_ParaImageQuality.Items.AddRange(new object[] {
            "Standard",
            "Speed Priority",
            "High quality"});
            this.comboBox_ParaImageQuality.Location = new System.Drawing.Point(96, 124);
            this.comboBox_ParaImageQuality.Name = "comboBox_ParaImageQuality";
            this.comboBox_ParaImageQuality.Size = new System.Drawing.Size(132, 21);
            this.comboBox_ParaImageQuality.TabIndex = 27;
            // 
            // comboBox_ParaLines
            // 
            this.comboBox_ParaLines.BackColor = System.Drawing.Color.LightGray;
            this.comboBox_ParaLines.FormattingEnabled = true;
            this.comboBox_ParaLines.Items.AddRange(new object[] {
            "512",
            "256",
            "128",
            "64",
            "32",
            "16"});
            this.comboBox_ParaLines.Location = new System.Drawing.Point(96, 90);
            this.comboBox_ParaLines.Name = "comboBox_ParaLines";
            this.comboBox_ParaLines.Size = new System.Drawing.Size(132, 21);
            this.comboBox_ParaLines.TabIndex = 28;
            // 
            // comboBox_ParaPixels
            // 
            this.comboBox_ParaPixels.BackColor = System.Drawing.Color.LightGray;
            this.comboBox_ParaPixels.FormattingEnabled = true;
            this.comboBox_ParaPixels.Items.AddRange(new object[] {
            "512",
            "256",
            "128",
            "64",
            "32",
            "16"});
            this.comboBox_ParaPixels.Location = new System.Drawing.Point(96, 56);
            this.comboBox_ParaPixels.Name = "comboBox_ParaPixels";
            this.comboBox_ParaPixels.Size = new System.Drawing.Size(132, 21);
            this.comboBox_ParaPixels.TabIndex = 27;
            // 
            // comboBox_ParaScanArea
            // 
            this.comboBox_ParaScanArea.BackColor = System.Drawing.Color.LightGray;
            this.comboBox_ParaScanArea.FormattingEnabled = true;
            this.comboBox_ParaScanArea.Items.AddRange(new object[] {
            "50um × 50um",
            "40um × 40um",
            "30um × 30um",
            "20um × 20um",
            "10um × 10um",
            "5um × 5um",
            "2um × 2um",
            "1um × 1um"});
            this.comboBox_ParaScanArea.Location = new System.Drawing.Point(96, 22);
            this.comboBox_ParaScanArea.Name = "comboBox_ParaScanArea";
            this.comboBox_ParaScanArea.Size = new System.Drawing.Size(132, 21);
            this.comboBox_ParaScanArea.TabIndex = 26;
            // 
            // label34
            // 
            this.label34.AutoSize = true;
            this.label34.Location = new System.Drawing.Point(17, 128);
            this.label34.Name = "label34";
            this.label34.Size = new System.Drawing.Size(71, 13);
            this.label34.TabIndex = 25;
            this.label34.Text = "Image Quality";
            // 
            // label33
            // 
            this.label33.AutoSize = true;
            this.label33.Location = new System.Drawing.Point(17, 94);
            this.label33.Name = "label33";
            this.label33.Size = new System.Drawing.Size(32, 13);
            this.label33.TabIndex = 24;
            this.label33.Text = "Lines";
            // 
            // label32
            // 
            this.label32.AutoSize = true;
            this.label32.Location = new System.Drawing.Point(17, 60);
            this.label32.Name = "label32";
            this.label32.Size = new System.Drawing.Size(34, 13);
            this.label32.TabIndex = 23;
            this.label32.Text = "Pixels";
            // 
            // label31
            // 
            this.label31.AutoSize = true;
            this.label31.Location = new System.Drawing.Point(17, 26);
            this.label31.Name = "label31";
            this.label31.Size = new System.Drawing.Size(57, 13);
            this.label31.TabIndex = 22;
            this.label31.Text = "Scan Area";
            // 
            // button_AdvancedSettings
            // 
            this.button_AdvancedSettings.Location = new System.Drawing.Point(17, 169);
            this.button_AdvancedSettings.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.button_AdvancedSettings.Name = "button_AdvancedSettings";
            this.button_AdvancedSettings.Size = new System.Drawing.Size(81, 41);
            this.button_AdvancedSettings.TabIndex = 0;
            this.button_AdvancedSettings.Text = "Advanced Settings";
            this.button_AdvancedSettings.UseVisualStyleBackColor = true;
            // 
            // tabPage_Indentation
            // 
            this.tabPage_Indentation.BackColor = System.Drawing.Color.LightGray;
            this.tabPage_Indentation.Controls.Add(this.button_MultiTask);
            this.tabPage_Indentation.Location = new System.Drawing.Point(4, 22);
            this.tabPage_Indentation.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_Indentation.Name = "tabPage_Indentation";
            this.tabPage_Indentation.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_Indentation.Size = new System.Drawing.Size(252, 460);
            this.tabPage_Indentation.TabIndex = 1;
            this.tabPage_Indentation.Text = "Indentation";
            // 
            // tabPage_Memory
            // 
            this.tabPage_Memory.Location = new System.Drawing.Point(4, 22);
            this.tabPage_Memory.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_Memory.Name = "tabPage_Memory";
            this.tabPage_Memory.Size = new System.Drawing.Size(252, 460);
            this.tabPage_Memory.TabIndex = 2;
            this.tabPage_Memory.Text = "Memory";
            this.tabPage_Memory.UseVisualStyleBackColor = true;
            // 
            // tabControl
            // 
            this.tabControl.Controls.Add(this.tabPage_backup);
            this.tabControl.Controls.Add(this.tabPage_Control);
            this.tabControl.Controls.Add(this.tabPage_TopoScan);
            this.tabControl.Controls.Add(this.tabPage_Indentation);
            this.tabControl.Controls.Add(this.tabPage_Memory);
            this.tabControl.Controls.Add(this.tabPage_DataExport);
            this.tabControl.Location = new System.Drawing.Point(12, 322);
            this.tabControl.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabControl.Name = "tabControl";
            this.tabControl.SelectedIndex = 0;
            this.tabControl.Size = new System.Drawing.Size(260, 486);
            this.tabControl.TabIndex = 98;
            this.tabControl.Tag = "";
            // 
            // tabPage_backup
            // 
            this.tabPage_backup.Controls.Add(this.label15);
            this.tabPage_backup.Controls.Add(this.textBox_T);
            this.tabPage_backup.Controls.Add(this.button_Update_ParaPID);
            this.tabPage_backup.Controls.Add(this.textBox_Dy);
            this.tabPage_backup.Controls.Add(this.textBox_Z_PID_P);
            this.tabPage_backup.Controls.Add(this.textBox_TaskTime);
            this.tabPage_backup.Controls.Add(this.checkBox_ForceSetAll);
            this.tabPage_backup.Controls.Add(this.label25);
            this.tabPage_backup.Controls.Add(this.label16);
            this.tabPage_backup.Controls.Add(this.textBox_TaskNumber);
            this.tabPage_backup.Controls.Add(this.button_SetParameters);
            this.tabPage_backup.Controls.Add(this.label1);
            this.tabPage_backup.Controls.Add(this.label5);
            this.tabPage_backup.Controls.Add(this.label26);
            this.tabPage_backup.Controls.Add(this.textBox_Z_PID_I);
            this.tabPage_backup.Controls.Add(this.textBox_Dx);
            this.tabPage_backup.Controls.Add(this.textBox_Z_PID_D);
            this.tabPage_backup.Controls.Add(this.label28);
            this.tabPage_backup.Controls.Add(this.label6);
            this.tabPage_backup.Controls.Add(this.textBox_XL);
            this.tabPage_backup.Controls.Add(this.textBox_Task_PointsNumber);
            this.tabPage_backup.Controls.Add(this.label27);
            this.tabPage_backup.Controls.Add(this.textBox_Sensitivity);
            this.tabPage_backup.Controls.Add(this.label14);
            this.tabPage_backup.Controls.Add(this.label20);
            this.tabPage_backup.Controls.Add(this.label18);
            this.tabPage_backup.Controls.Add(this.label13);
            this.tabPage_backup.Controls.Add(this.textBox_Ny);
            this.tabPage_backup.Controls.Add(this.textBox_YL);
            this.tabPage_backup.Controls.Add(this.textBox_ScanRate);
            this.tabPage_backup.Controls.Add(this.textBox_Nx);
            this.tabPage_backup.Controls.Add(this.label19);
            this.tabPage_backup.Controls.Add(this.textBox_NumberOfFrameToScan);
            this.tabPage_backup.Controls.Add(this.label17);
            this.tabPage_backup.Controls.Add(this.textBox_SetDeltaValueNm);
            this.tabPage_backup.Controls.Add(this.label21);
            this.tabPage_backup.Controls.Add(this.label22);
            this.tabPage_backup.Location = new System.Drawing.Point(4, 22);
            this.tabPage_backup.Name = "tabPage_backup";
            this.tabPage_backup.Size = new System.Drawing.Size(252, 460);
            this.tabPage_backup.TabIndex = 5;
            this.tabPage_backup.Text = "backup";
            this.tabPage_backup.UseVisualStyleBackColor = true;
            // 
            // tabPage_Control
            // 
            this.tabPage_Control.Controls.Add(this.button_StartTask);
            this.tabPage_Control.Controls.Add(this.button_WaveTest);
            this.tabPage_Control.Controls.Add(this.button_DataCapture);
            this.tabPage_Control.Controls.Add(this.listBox_SelectIdlePackage);
            this.tabPage_Control.Controls.Add(this.button2);
            this.tabPage_Control.Controls.Add(this.button1);
            this.tabPage_Control.Controls.Add(this.checkBox_ShowImage);
            this.tabPage_Control.Controls.Add(this.textBox_TF_DC_Gain);
            this.tabPage_Control.Controls.Add(this.label24);
            this.tabPage_Control.Location = new System.Drawing.Point(4, 22);
            this.tabPage_Control.Name = "tabPage_Control";
            this.tabPage_Control.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage_Control.Size = new System.Drawing.Size(252, 460);
            this.tabPage_Control.TabIndex = 4;
            this.tabPage_Control.Text = "backup1";
            this.tabPage_Control.UseVisualStyleBackColor = true;
            // 
            // button_StartTask
            // 
            this.button_StartTask.Location = new System.Drawing.Point(143, 270);
            this.button_StartTask.Name = "button_StartTask";
            this.button_StartTask.Size = new System.Drawing.Size(97, 34);
            this.button_StartTask.TabIndex = 87;
            this.button_StartTask.Text = "start task";
            this.button_StartTask.UseVisualStyleBackColor = true;
            this.button_StartTask.Click += new System.EventHandler(this.button_StartTask_Click);
            // 
            // button_WaveTest
            // 
            this.button_WaveTest.Location = new System.Drawing.Point(139, 221);
            this.button_WaveTest.Margin = new System.Windows.Forms.Padding(2);
            this.button_WaveTest.Name = "button_WaveTest";
            this.button_WaveTest.Size = new System.Drawing.Size(101, 32);
            this.button_WaveTest.TabIndex = 86;
            this.button_WaveTest.Text = "WaveTest";
            this.button_WaveTest.UseVisualStyleBackColor = true;
            this.button_WaveTest.Click += new System.EventHandler(this.button_WaveTest_Click);
            // 
            // button_DataCapture
            // 
            this.button_DataCapture.Location = new System.Drawing.Point(139, 176);
            this.button_DataCapture.Margin = new System.Windows.Forms.Padding(2);
            this.button_DataCapture.Name = "button_DataCapture";
            this.button_DataCapture.Size = new System.Drawing.Size(101, 32);
            this.button_DataCapture.TabIndex = 85;
            this.button_DataCapture.Text = "data capture";
            this.button_DataCapture.UseVisualStyleBackColor = true;
            this.button_DataCapture.Click += new System.EventHandler(this.button_DataCapture_Click);
            // 
            // tabPage_DataExport
            // 
            this.tabPage_DataExport.Controls.Add(this.label30);
            this.tabPage_DataExport.Controls.Add(this.textBox_FileName);
            this.tabPage_DataExport.Controls.Add(this.label29);
            this.tabPage_DataExport.Controls.Add(this.textBox_DataPath);
            this.tabPage_DataExport.Controls.Add(this.radioButton_Topography);
            this.tabPage_DataExport.Controls.Add(this.button_ExportData);
            this.tabPage_DataExport.Controls.Add(this.button_SaveImage);
            this.tabPage_DataExport.Controls.Add(this.radioButton_Deflection);
            this.tabPage_DataExport.Location = new System.Drawing.Point(4, 22);
            this.tabPage_DataExport.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_DataExport.Name = "tabPage_DataExport";
            this.tabPage_DataExport.Padding = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.tabPage_DataExport.Size = new System.Drawing.Size(252, 460);
            this.tabPage_DataExport.TabIndex = 3;
            this.tabPage_DataExport.Text = "Data Export";
            this.tabPage_DataExport.UseVisualStyleBackColor = true;
            // 
            // label30
            // 
            this.label30.AutoSize = true;
            this.label30.Location = new System.Drawing.Point(6, 210);
            this.label30.Name = "label30";
            this.label30.Size = new System.Drawing.Size(52, 13);
            this.label30.TabIndex = 72;
            this.label30.Text = "file_name";
            // 
            // textBox_FileName
            // 
            this.textBox_FileName.Location = new System.Drawing.Point(0, 226);
            this.textBox_FileName.Name = "textBox_FileName";
            this.textBox_FileName.Size = new System.Drawing.Size(242, 20);
            this.textBox_FileName.TabIndex = 71;
            this.textBox_FileName.Text = "Sample";
            // 
            // label29
            // 
            this.label29.AutoSize = true;
            this.label29.Location = new System.Drawing.Point(6, 154);
            this.label29.Name = "label29";
            this.label29.Size = new System.Drawing.Size(32, 13);
            this.label29.TabIndex = 3;
            this.label29.Text = "Path:";
            // 
            // textBox_DataPath
            // 
            this.textBox_DataPath.Location = new System.Drawing.Point(3, 180);
            this.textBox_DataPath.Name = "textBox_DataPath";
            this.textBox_DataPath.Size = new System.Drawing.Size(242, 20);
            this.textBox_DataPath.TabIndex = 70;
            this.textBox_DataPath.Text = "D:\\AFM_Data";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.button_CoarseWithdraw);
            this.groupBox1.Controls.Add(this.button_CancelApproach);
            this.groupBox1.Controls.Add(this.button_Apporach);
            this.groupBox1.Location = new System.Drawing.Point(12, 245);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(260, 70);
            this.groupBox1.TabIndex = 106;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Auto Approach";
            // 
            // button_CoarseWithdraw
            // 
            this.button_CoarseWithdraw.Location = new System.Drawing.Point(162, 20);
            this.button_CoarseWithdraw.Margin = new System.Windows.Forms.Padding(2);
            this.button_CoarseWithdraw.Name = "button_CoarseWithdraw";
            this.button_CoarseWithdraw.Size = new System.Drawing.Size(86, 35);
            this.button_CoarseWithdraw.TabIndex = 43;
            this.button_CoarseWithdraw.Text = "move to safe  position";
            this.button_CoarseWithdraw.UseVisualStyleBackColor = true;
            this.button_CoarseWithdraw.Click += new System.EventHandler(this.button_CoarseWithdraw_Click);
            // 
            // timer_Update_CoarseReadout
            // 
            this.timer_Update_CoarseReadout.Enabled = true;
            this.timer_Update_CoarseReadout.Interval = 300;
            this.timer_Update_CoarseReadout.Tick += new System.EventHandler(this.timer_Update_CoarseReadout_Tick);
            // 
            // timer_ScaningImageShow_RealTime
            // 
            this.timer_ScaningImageShow_RealTime.Interval = 10000;
            this.timer_ScaningImageShow_RealTime.Tick += new System.EventHandler(this.timer_ScaningImageShow_RealTime_Tick);
            // 
            // label_SystemState
            // 
            this.label_SystemState.AutoSize = true;
            this.label_SystemState.Location = new System.Drawing.Point(19, 874);
            this.label_SystemState.Name = "label_SystemState";
            this.label_SystemState.Size = new System.Drawing.Size(73, 13);
            this.label_SystemState.TabIndex = 107;
            this.label_SystemState.Text = "System ready.";
            // 
            // MainWindow
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.BackColor = System.Drawing.Color.LightGray;
            this.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.ClientSize = new System.Drawing.Size(1664, 1012);
            this.Controls.Add(this.label_SystemState);
            this.Controls.Add(this.groupBox_SEMPreview);
            this.Controls.Add(this.pictureBox_SEMImage);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.groupBox_AFMImage);
            this.Controls.Add(this.groupBox_Setup);
            this.Controls.Add(this.groupBox_SystemStatus);
            this.Controls.Add(this.tabControl);
            this.DoubleBuffered = true;
            this.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(3, 4, 3, 4);
            this.MaximizeBox = false;
            this.Name = "MainWindow";
            this.Text = "AFM";
            this.Load += new System.EventHandler(this.MainWindow_Load);
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_AFM_Real_Image)).EndInit();
            this.groupBox_AFMImage.ResumeLayout(false);
            this.groupBox_AFMImage.PerformLayout();
            this.groupBox_SEMPreview.ResumeLayout(false);
            this.groupBox_SEMPreview.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_SEMImage)).EndInit();
            this.groupBox_Setup.ResumeLayout(false);
            this.groupBox_Setup.PerformLayout();
            this.groupBox_SystemStatus.ResumeLayout(false);
            this.groupBox_SystemStatus.PerformLayout();
            this.tabPage_TopoScan.ResumeLayout(false);
            this.tabPage_TopoScan.PerformLayout();
            this.tabPage_Indentation.ResumeLayout(false);
            this.tabControl.ResumeLayout(false);
            this.tabPage_backup.ResumeLayout(false);
            this.tabPage_backup.PerformLayout();
            this.tabPage_Control.ResumeLayout(false);
            this.tabPage_Control.PerformLayout();
            this.tabPage_DataExport.ResumeLayout(false);
            this.tabPage_DataExport.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ListBox listBox_SelectIdlePackage;
        private System.Windows.Forms.Timer timer_CheckCOM;
        private System.Windows.Forms.ToolTip toolTip_Help;
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
        private System.IO.Ports.SerialPort serialPort_Arduino;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox textBox_BaudRate;
        private System.Windows.Forms.Button button_Apporach;
        private System.Windows.Forms.Button button_CancelApproach;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.CheckBox checkBox_COM_Transfer;
        private System.Windows.Forms.Button button_Z_Withdraw;
        private System.Windows.Forms.Button button_Z_Engage_Test;
        private System.Windows.Forms.CheckBox checkBox_Y_ScanEnable;
        private System.Windows.Forms.Button button_XY_pause;
        private System.Windows.Forms.Button button_XY_Scan;
        private System.Windows.Forms.Button button_XY_Reset;
        private System.Windows.Forms.Timer timer_Approach;
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
        private System.Windows.Forms.Button button_SaveImage;
        private System.Windows.Forms.Button button_ClearImage;
        private System.Windows.Forms.CheckBox checkBox_ShowImage;
        private System.Windows.Forms.Button button_StartSubWindow;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.TextBox textBox_NumberOfFrameToScan;
        private System.Windows.Forms.Button button_SetScanROI;
        private System.Windows.Forms.Button button_StartIndent;
        private System.Windows.Forms.PropertyGrid propertyGrid_AFM_Parameter;
        private System.Windows.Forms.CheckBox checkBox_ForceSetAll;
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
        private System.Windows.Forms.RadioButton radioButton_Topography;
        private System.Windows.Forms.PictureBox pictureBox_AFM_Real_Image;
        private System.Windows.Forms.Button button_ExportData;
        private System.Windows.Forms.GroupBox groupBox_AFMImage;
        private System.Windows.Forms.RadioButton radioButton_Deflection;
        private System.Windows.Forms.Button button_Coarse_MoveToHomePosition;
        private System.Windows.Forms.Label label_Speed_Z;
        private System.Windows.Forms.Label label_Encoder_Z;
        private System.Windows.Forms.Button button_CantileverCalibration;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label_Speed_Y;
        private System.Windows.Forms.Button button_SEMImage_VideoRecord;
        private System.Windows.Forms.Label label_VideoRecord;
        private System.Windows.Forms.Label label_ScreenShot;
        private System.Windows.Forms.Button button_SEMImage_ScreenShot;
        private System.Windows.Forms.GroupBox groupBox_SEMPreview;
        private System.Windows.Forms.CheckBox checkBox_ReferenceMarks;
        private System.Windows.Forms.PictureBox pictureBox_SEMImage;
        private System.Windows.Forms.Label label_Encoder_Y;
        private System.Windows.Forms.Label label_Speed_X;
        private System.Windows.Forms.Label label_Axis_Z;
        private System.Windows.Forms.Label label_Encoder_X;
        private System.Windows.Forms.Label label_Axis_Y;
        private System.Windows.Forms.Label label_Speed;
        private System.Windows.Forms.Label label_Axis_X;
        private System.Windows.Forms.Label label_Encoder;
        private System.Windows.Forms.Label label_Axis;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Button button_Optical;
        private System.Windows.Forms.GroupBox groupBox_Setup;
        private System.Windows.Forms.Label label_Servicing;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Button button_Servicing;
        private System.Windows.Forms.Button button_ScannerCalibration;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.GroupBox groupBox_SystemStatus;
        private System.Windows.Forms.Button button_SEM;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.Label label_Controller;
        private System.Windows.Forms.Label label_Joystick;
        private System.Windows.Forms.Label label_Microscope;
        private System.Windows.Forms.TabPage tabPage_TopoScan;
        private System.Windows.Forms.Button button_AdvancedSettings;
        private System.Windows.Forms.TabPage tabPage_Indentation;
        private System.Windows.Forms.TabPage tabPage_Memory;
        private System.Windows.Forms.TabControl tabControl;
        private System.Windows.Forms.TabPage tabPage_DataExport;
        private System.Windows.Forms.TabPage tabPage_Control;
        private System.Windows.Forms.TextBox textBox_DataPath;
        private System.Windows.Forms.Label label29;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button button_CoarseWithdraw;
        private System.Windows.Forms.Button button_DataCapture;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.TextBox textBox_FileName;
        private System.Windows.Forms.Button button_WaveTest;
        private System.Windows.Forms.Button button_StartTask;
        private System.Windows.Forms.Timer timer_Update_CoarseReadout;
        private System.Windows.Forms.TabPage tabPage_backup;
        private System.Windows.Forms.ComboBox comboBox_ParaImageQuality;
        private System.Windows.Forms.ComboBox comboBox_ParaLines;
        private System.Windows.Forms.ComboBox comboBox_ParaPixels;
        private System.Windows.Forms.ComboBox comboBox_ParaScanArea;
        private System.Windows.Forms.Label label34;
        private System.Windows.Forms.Label label33;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.Label label31;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Timer timer_ScaningImageShow_RealTime;
        private System.Windows.Forms.Button button_Update_ParaPID;
        private System.Windows.Forms.Label label_SystemState;
        private System.Windows.Forms.Button button_Z_HybridWithdraw;
        private System.Windows.Forms.Button button_Z_HybridEngage;
        private System.Windows.Forms.Button button_User_ImageScan_StartStop;
        private System.Windows.Forms.Button button_User_AutoEngage;
        private System.Windows.Forms.CheckBox checkBox_User_AutoStartScanAfterEngage;
    }
}

