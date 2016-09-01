namespace NameSpace_AFM_Project
{
    partial class ParameterSettingForm
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
            this.button_DAC_Output = new System.Windows.Forms.Button();
            this.textBox_DAC_Value = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.button_SetPosition_OpenLoop = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox_Position_Value = new System.Windows.Forms.TextBox();
            this.button_Read_StrainGauge_Stop = new System.Windows.Forms.Button();
            this.button_Read_StrainGauge_Once = new System.Windows.Forms.Button();
            this.button_Read_StrainGauge_Continue = new System.Windows.Forms.Button();
            this.trackBar_PositionOutput = new System.Windows.Forms.TrackBar();
            this.button_T_debug = new System.Windows.Forms.Button();
            this.textBox_T_Test_cycles = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.textBox_T_test_dt = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.button_WaveStop = new System.Windows.Forms.Button();
            this.button_SetPositioin_CloseLoop = new System.Windows.Forms.Button();
            this.listBox_ScannerAxis = new System.Windows.Forms.ListBox();
            this.button_N = new System.Windows.Forms.Button();
            this.button_P = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.listBox_ScannerMode = new System.Windows.Forms.ListBox();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_PositionOutput)).BeginInit();
            this.SuspendLayout();
            // 
            // button_DAC_Output
            // 
            this.button_DAC_Output.Location = new System.Drawing.Point(25, 12);
            this.button_DAC_Output.Name = "button_DAC_Output";
            this.button_DAC_Output.Size = new System.Drawing.Size(70, 32);
            this.button_DAC_Output.TabIndex = 0;
            this.button_DAC_Output.Text = "DAC";
            this.button_DAC_Output.UseVisualStyleBackColor = true;
            this.button_DAC_Output.Click += new System.EventHandler(this.button_DAC_Output_Click);
            // 
            // textBox_DAC_Value
            // 
            this.textBox_DAC_Value.Location = new System.Drawing.Point(413, 63);
            this.textBox_DAC_Value.Name = "textBox_DAC_Value";
            this.textBox_DAC_Value.Size = new System.Drawing.Size(57, 20);
            this.textBox_DAC_Value.TabIndex = 2;
            this.textBox_DAC_Value.Text = "0";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(321, 66);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(86, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Voltage: 0-150 V";
            // 
            // button_SetPosition_OpenLoop
            // 
            this.button_SetPosition_OpenLoop.Location = new System.Drawing.Point(101, 12);
            this.button_SetPosition_OpenLoop.Name = "button_SetPosition_OpenLoop";
            this.button_SetPosition_OpenLoop.Size = new System.Drawing.Size(70, 32);
            this.button_SetPosition_OpenLoop.TabIndex = 5;
            this.button_SetPosition_OpenLoop.Text = "openloop";
            this.button_SetPosition_OpenLoop.UseVisualStyleBackColor = true;
            this.button_SetPosition_OpenLoop.Click += new System.EventHandler(this.button_SetPosition_OpenLoop_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(283, 22);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(115, 13);
            this.label3.TabIndex = 9;
            this.label3.Text = "normalized position0~1";
            // 
            // textBox_Position_Value
            // 
            this.textBox_Position_Value.Location = new System.Drawing.Point(413, 24);
            this.textBox_Position_Value.Name = "textBox_Position_Value";
            this.textBox_Position_Value.Size = new System.Drawing.Size(57, 20);
            this.textBox_Position_Value.TabIndex = 7;
            this.textBox_Position_Value.Text = "0";
            // 
            // button_Read_StrainGauge_Stop
            // 
            this.button_Read_StrainGauge_Stop.Location = new System.Drawing.Point(225, 320);
            this.button_Read_StrainGauge_Stop.Name = "button_Read_StrainGauge_Stop";
            this.button_Read_StrainGauge_Stop.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Stop.TabIndex = 10;
            this.button_Read_StrainGauge_Stop.Text = "read SG stop";
            this.button_Read_StrainGauge_Stop.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Stop.Click += new System.EventHandler(this.button_Read_StrainGauge_Stop_Click);
            // 
            // button_Read_StrainGauge_Once
            // 
            this.button_Read_StrainGauge_Once.Location = new System.Drawing.Point(120, 320);
            this.button_Read_StrainGauge_Once.Name = "button_Read_StrainGauge_Once";
            this.button_Read_StrainGauge_Once.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Once.TabIndex = 11;
            this.button_Read_StrainGauge_Once.Text = "read SG once";
            this.button_Read_StrainGauge_Once.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Once.Click += new System.EventHandler(this.button_Read_StrainGauge_Once_Click);
            // 
            // button_Read_StrainGauge_Continue
            // 
            this.button_Read_StrainGauge_Continue.Location = new System.Drawing.Point(12, 320);
            this.button_Read_StrainGauge_Continue.Name = "button_Read_StrainGauge_Continue";
            this.button_Read_StrainGauge_Continue.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Continue.TabIndex = 12;
            this.button_Read_StrainGauge_Continue.Text = "read SG continue";
            this.button_Read_StrainGauge_Continue.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Continue.Click += new System.EventHandler(this.button_Read_StrainGauge_Continue_Click);
            // 
            // trackBar_PositionOutput
            // 
            this.trackBar_PositionOutput.Location = new System.Drawing.Point(3, 179);
            this.trackBar_PositionOutput.Maximum = 100000;
            this.trackBar_PositionOutput.Name = "trackBar_PositionOutput";
            this.trackBar_PositionOutput.Size = new System.Drawing.Size(496, 42);
            this.trackBar_PositionOutput.TabIndex = 13;
            this.trackBar_PositionOutput.Scroll += new System.EventHandler(this.trackBar_PositionZ_Scroll);
            // 
            // button_T_debug
            // 
            this.button_T_debug.Location = new System.Drawing.Point(158, 231);
            this.button_T_debug.Name = "button_T_debug";
            this.button_T_debug.Size = new System.Drawing.Size(111, 32);
            this.button_T_debug.TabIndex = 14;
            this.button_T_debug.Text = "square wave";
            this.button_T_debug.UseVisualStyleBackColor = true;
            this.button_T_debug.Click += new System.EventHandler(this.button_T_debug_Click);
            // 
            // textBox_T_Test_cycles
            // 
            this.textBox_T_Test_cycles.Location = new System.Drawing.Point(78, 238);
            this.textBox_T_Test_cycles.Name = "textBox_T_Test_cycles";
            this.textBox_T_Test_cycles.Size = new System.Drawing.Size(57, 20);
            this.textBox_T_Test_cycles.TabIndex = 15;
            this.textBox_T_Test_cycles.Text = "0";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(41, 245);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(31, 13);
            this.label5.TabIndex = 16;
            this.label5.Text = "times";
            // 
            // textBox_T_test_dt
            // 
            this.textBox_T_test_dt.Location = new System.Drawing.Point(78, 280);
            this.textBox_T_test_dt.Name = "textBox_T_test_dt";
            this.textBox_T_test_dt.Size = new System.Drawing.Size(57, 20);
            this.textBox_T_test_dt.TabIndex = 17;
            this.textBox_T_test_dt.Text = "1000";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(40, 287);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(32, 13);
            this.label6.TabIndex = 18;
            this.label6.Text = "dt ms";
            // 
            // button_WaveStop
            // 
            this.button_WaveStop.Location = new System.Drawing.Point(158, 273);
            this.button_WaveStop.Name = "button_WaveStop";
            this.button_WaveStop.Size = new System.Drawing.Size(111, 32);
            this.button_WaveStop.TabIndex = 20;
            this.button_WaveStop.Text = "stop";
            this.button_WaveStop.UseVisualStyleBackColor = true;
            this.button_WaveStop.Click += new System.EventHandler(this.button_WaveStop_Click);
            // 
            // button_SetPositioin_CloseLoop
            // 
            this.button_SetPositioin_CloseLoop.Location = new System.Drawing.Point(177, 12);
            this.button_SetPositioin_CloseLoop.Name = "button_SetPositioin_CloseLoop";
            this.button_SetPositioin_CloseLoop.Size = new System.Drawing.Size(70, 32);
            this.button_SetPositioin_CloseLoop.TabIndex = 21;
            this.button_SetPositioin_CloseLoop.Text = "closed loop";
            this.button_SetPositioin_CloseLoop.UseVisualStyleBackColor = true;
            this.button_SetPositioin_CloseLoop.Click += new System.EventHandler(this.button_SetPositioin_CloseLoop_Click);
            // 
            // listBox_ScannerAxis
            // 
            this.listBox_ScannerAxis.FormattingEnabled = true;
            this.listBox_ScannerAxis.Items.AddRange(new object[] {
            "Z",
            "Y",
            "X",
            "ALL"});
            this.listBox_ScannerAxis.Location = new System.Drawing.Point(25, 63);
            this.listBox_ScannerAxis.Name = "listBox_ScannerAxis";
            this.listBox_ScannerAxis.Size = new System.Drawing.Size(82, 56);
            this.listBox_ScannerAxis.TabIndex = 22;
            // 
            // button_N
            // 
            this.button_N.Location = new System.Drawing.Point(337, 110);
            this.button_N.Name = "button_N";
            this.button_N.Size = new System.Drawing.Size(70, 32);
            this.button_N.TabIndex = 24;
            this.button_N.Text = "-";
            this.button_N.UseVisualStyleBackColor = true;
            this.button_N.Click += new System.EventHandler(this.button_N_Click);
            // 
            // button_P
            // 
            this.button_P.Location = new System.Drawing.Point(257, 110);
            this.button_P.Name = "button_P";
            this.button_P.Size = new System.Drawing.Size(70, 32);
            this.button_P.TabIndex = 23;
            this.button_P.Text = "+";
            this.button_P.UseVisualStyleBackColor = true;
            this.button_P.Click += new System.EventHandler(this.button_P_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(257, 66);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(58, 13);
            this.label1.TabIndex = 25;
            this.label1.Text = "DAC value";
            // 
            // listBox_ScannerMode
            // 
            this.listBox_ScannerMode.FormattingEnabled = true;
            this.listBox_ScannerMode.Items.AddRange(new object[] {
            "DAC",
            "OpenLoop",
            "ClosedLoop"});
            this.listBox_ScannerMode.Location = new System.Drawing.Point(120, 63);
            this.listBox_ScannerMode.Name = "listBox_ScannerMode";
            this.listBox_ScannerMode.Size = new System.Drawing.Size(82, 56);
            this.listBox_ScannerMode.TabIndex = 26;
            // 
            // ParameterSettingForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(513, 432);
            this.Controls.Add(this.listBox_ScannerMode);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.button_N);
            this.Controls.Add(this.button_P);
            this.Controls.Add(this.listBox_ScannerAxis);
            this.Controls.Add(this.button_SetPositioin_CloseLoop);
            this.Controls.Add(this.button_WaveStop);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.textBox_T_test_dt);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBox_T_Test_cycles);
            this.Controls.Add(this.button_T_debug);
            this.Controls.Add(this.trackBar_PositionOutput);
            this.Controls.Add(this.button_Read_StrainGauge_Continue);
            this.Controls.Add(this.button_Read_StrainGauge_Once);
            this.Controls.Add(this.button_Read_StrainGauge_Stop);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox_Position_Value);
            this.Controls.Add(this.button_SetPosition_OpenLoop);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBox_DAC_Value);
            this.Controls.Add(this.button_DAC_Output);
            this.Name = "ParameterSettingForm";
            this.Text = "ParameterSettingForm";
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_PositionOutput)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button button_DAC_Output;
        private System.Windows.Forms.TextBox textBox_DAC_Value;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button button_SetPosition_OpenLoop;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBox_Position_Value;
        private System.Windows.Forms.Button button_Read_StrainGauge_Stop;
        private System.Windows.Forms.Button button_Read_StrainGauge_Once;
        private System.Windows.Forms.Button button_Read_StrainGauge_Continue;
        private System.Windows.Forms.TrackBar trackBar_PositionOutput;
        private System.Windows.Forms.Button button_T_debug;
        private System.Windows.Forms.TextBox textBox_T_Test_cycles;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBox_T_test_dt;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button button_WaveStop;
        private System.Windows.Forms.Button button_SetPositioin_CloseLoop;
        private System.Windows.Forms.ListBox listBox_ScannerAxis;
        private System.Windows.Forms.Button button_N;
        private System.Windows.Forms.Button button_P;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ListBox listBox_ScannerMode;
    }
}