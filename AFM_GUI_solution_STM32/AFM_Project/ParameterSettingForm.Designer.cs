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
            this.textBox_DAC_Axis = new System.Windows.Forms.TextBox();
            this.textBox_DAC_Value = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.button_Position_Output = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.textBox_Position_Value = new System.Windows.Forms.TextBox();
            this.textBox_Position_Axis = new System.Windows.Forms.TextBox();
            this.button_Read_StrainGauge_Stop = new System.Windows.Forms.Button();
            this.button_Read_StrainGauge_Once = new System.Windows.Forms.Button();
            this.button_Read_StrainGauge_Continue = new System.Windows.Forms.Button();
            this.trackBar_PositionZ = new System.Windows.Forms.TrackBar();
            this.button_T_debug = new System.Windows.Forms.Button();
            this.textBox_T_Test_cycles = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.textBox_T_test_dt = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_PositionZ)).BeginInit();
            this.SuspendLayout();
            // 
            // button_DAC_Output
            // 
            this.button_DAC_Output.Location = new System.Drawing.Point(35, 33);
            this.button_DAC_Output.Name = "button_DAC_Output";
            this.button_DAC_Output.Size = new System.Drawing.Size(70, 32);
            this.button_DAC_Output.TabIndex = 0;
            this.button_DAC_Output.Text = "set voltage";
            this.button_DAC_Output.UseVisualStyleBackColor = true;
            this.button_DAC_Output.Click += new System.EventHandler(this.button_DAC_Output_Click);
            // 
            // textBox_DAC_Axis
            // 
            this.textBox_DAC_Axis.Location = new System.Drawing.Point(130, 40);
            this.textBox_DAC_Axis.Name = "textBox_DAC_Axis";
            this.textBox_DAC_Axis.Size = new System.Drawing.Size(29, 20);
            this.textBox_DAC_Axis.TabIndex = 1;
            this.textBox_DAC_Axis.Text = "0";
            // 
            // textBox_DAC_Value
            // 
            this.textBox_DAC_Value.Location = new System.Drawing.Point(189, 40);
            this.textBox_DAC_Value.Name = "textBox_DAC_Value";
            this.textBox_DAC_Value.Size = new System.Drawing.Size(57, 20);
            this.textBox_DAC_Value.TabIndex = 2;
            this.textBox_DAC_Value.Text = "0";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(127, 21);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(46, 13);
            this.label1.TabIndex = 3;
            this.label1.Text = "axis: 0-4";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(186, 21);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(76, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "value: 0-150 V";
            // 
            // button_Position_Output
            // 
            this.button_Position_Output.Location = new System.Drawing.Point(35, 98);
            this.button_Position_Output.Name = "button_Position_Output";
            this.button_Position_Output.Size = new System.Drawing.Size(70, 32);
            this.button_Position_Output.TabIndex = 5;
            this.button_Position_Output.Text = "set position";
            this.button_Position_Output.UseVisualStyleBackColor = true;
            this.button_Position_Output.Click += new System.EventHandler(this.button_Position_Output_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(186, 91);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 13);
            this.label3.TabIndex = 9;
            this.label3.Text = "value: nm";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(127, 91);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(46, 13);
            this.label4.TabIndex = 8;
            this.label4.Text = "axis: 0-4";
            // 
            // textBox_Position_Value
            // 
            this.textBox_Position_Value.Location = new System.Drawing.Point(189, 110);
            this.textBox_Position_Value.Name = "textBox_Position_Value";
            this.textBox_Position_Value.Size = new System.Drawing.Size(57, 20);
            this.textBox_Position_Value.TabIndex = 7;
            this.textBox_Position_Value.Text = "0";
            // 
            // textBox_Position_Axis
            // 
            this.textBox_Position_Axis.Location = new System.Drawing.Point(130, 110);
            this.textBox_Position_Axis.Name = "textBox_Position_Axis";
            this.textBox_Position_Axis.Size = new System.Drawing.Size(29, 20);
            this.textBox_Position_Axis.TabIndex = 6;
            this.textBox_Position_Axis.Text = "0";
            // 
            // button_Read_StrainGauge_Stop
            // 
            this.button_Read_StrainGauge_Stop.Location = new System.Drawing.Point(35, 292);
            this.button_Read_StrainGauge_Stop.Name = "button_Read_StrainGauge_Stop";
            this.button_Read_StrainGauge_Stop.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Stop.TabIndex = 10;
            this.button_Read_StrainGauge_Stop.Text = "read SG stop";
            this.button_Read_StrainGauge_Stop.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Stop.Click += new System.EventHandler(this.button_Read_StrainGauge_Stop_Click);
            // 
            // button_Read_StrainGauge_Once
            // 
            this.button_Read_StrainGauge_Once.Location = new System.Drawing.Point(35, 254);
            this.button_Read_StrainGauge_Once.Name = "button_Read_StrainGauge_Once";
            this.button_Read_StrainGauge_Once.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Once.TabIndex = 11;
            this.button_Read_StrainGauge_Once.Text = "read SG once";
            this.button_Read_StrainGauge_Once.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Once.Click += new System.EventHandler(this.button_Read_StrainGauge_Once_Click);
            // 
            // button_Read_StrainGauge_Continue
            // 
            this.button_Read_StrainGauge_Continue.Location = new System.Drawing.Point(35, 216);
            this.button_Read_StrainGauge_Continue.Name = "button_Read_StrainGauge_Continue";
            this.button_Read_StrainGauge_Continue.Size = new System.Drawing.Size(102, 32);
            this.button_Read_StrainGauge_Continue.TabIndex = 12;
            this.button_Read_StrainGauge_Continue.Text = "read SG continue";
            this.button_Read_StrainGauge_Continue.UseVisualStyleBackColor = true;
            this.button_Read_StrainGauge_Continue.Click += new System.EventHandler(this.button_Read_StrainGauge_Continue_Click);
            // 
            // trackBar_PositionZ
            // 
            this.trackBar_PositionZ.Location = new System.Drawing.Point(12, 136);
            this.trackBar_PositionZ.Maximum = 14000;
            this.trackBar_PositionZ.Name = "trackBar_PositionZ";
            this.trackBar_PositionZ.Size = new System.Drawing.Size(1805, 42);
            this.trackBar_PositionZ.TabIndex = 13;
            this.trackBar_PositionZ.Scroll += new System.EventHandler(this.trackBar_PositionZ_Scroll);
            // 
            // button_T_debug
            // 
            this.button_T_debug.Location = new System.Drawing.Point(499, 98);
            this.button_T_debug.Name = "button_T_debug";
            this.button_T_debug.Size = new System.Drawing.Size(111, 32);
            this.button_T_debug.TabIndex = 14;
            this.button_T_debug.Text = "square wave";
            this.button_T_debug.UseVisualStyleBackColor = true;
            this.button_T_debug.Click += new System.EventHandler(this.button_T_debug_Click);
            // 
            // textBox_T_Test_cycles
            // 
            this.textBox_T_Test_cycles.Location = new System.Drawing.Point(336, 110);
            this.textBox_T_Test_cycles.Name = "textBox_T_Test_cycles";
            this.textBox_T_Test_cycles.Size = new System.Drawing.Size(57, 20);
            this.textBox_T_Test_cycles.TabIndex = 15;
            this.textBox_T_Test_cycles.Text = "20";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(333, 91);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(31, 13);
            this.label5.TabIndex = 16;
            this.label5.Text = "times";
            // 
            // textBox_T_test_dt
            // 
            this.textBox_T_test_dt.Location = new System.Drawing.Point(422, 110);
            this.textBox_T_test_dt.Name = "textBox_T_test_dt";
            this.textBox_T_test_dt.Size = new System.Drawing.Size(57, 20);
            this.textBox_T_test_dt.TabIndex = 17;
            this.textBox_T_test_dt.Text = "1000";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(431, 91);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(32, 13);
            this.label6.TabIndex = 18;
            this.label6.Text = "dt ms";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(264, 110);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(70, 13);
            this.label7.TabIndex = 19;
            this.label7.Text = "---------------------";
            // 
            // ParameterSettingForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1829, 639);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.textBox_T_test_dt);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.textBox_T_Test_cycles);
            this.Controls.Add(this.button_T_debug);
            this.Controls.Add(this.trackBar_PositionZ);
            this.Controls.Add(this.button_Read_StrainGauge_Continue);
            this.Controls.Add(this.button_Read_StrainGauge_Once);
            this.Controls.Add(this.button_Read_StrainGauge_Stop);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.textBox_Position_Value);
            this.Controls.Add(this.textBox_Position_Axis);
            this.Controls.Add(this.button_Position_Output);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.textBox_DAC_Value);
            this.Controls.Add(this.textBox_DAC_Axis);
            this.Controls.Add(this.button_DAC_Output);
            this.Name = "ParameterSettingForm";
            this.Text = "ParameterSettingForm";
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_PositionZ)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button button_DAC_Output;
        private System.Windows.Forms.TextBox textBox_DAC_Axis;
        private System.Windows.Forms.TextBox textBox_DAC_Value;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button button_Position_Output;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBox_Position_Value;
        private System.Windows.Forms.TextBox textBox_Position_Axis;
        private System.Windows.Forms.Button button_Read_StrainGauge_Stop;
        private System.Windows.Forms.Button button_Read_StrainGauge_Once;
        private System.Windows.Forms.Button button_Read_StrainGauge_Continue;
        private System.Windows.Forms.TrackBar trackBar_PositionZ;
        private System.Windows.Forms.Button button_T_debug;
        private System.Windows.Forms.TextBox textBox_T_Test_cycles;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox textBox_T_test_dt;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
    }
}