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
            this.button_StartIndent = new System.Windows.Forms.Button();
            this.textBox_Indent_StepSize = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.textBox_Indent_Depth = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.textBox_Indent_StartPosition = new System.Windows.Forms.TextBox();
            this.button_CancelIndent = new System.Windows.Forms.Button();
            this.label4 = new System.Windows.Forms.Label();
            this.textBox_FileName = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // button_StartIndent
            // 
            this.button_StartIndent.Location = new System.Drawing.Point(48, 50);
            this.button_StartIndent.Name = "button_StartIndent";
            this.button_StartIndent.Size = new System.Drawing.Size(85, 41);
            this.button_StartIndent.TabIndex = 0;
            this.button_StartIndent.Text = "start indent";
            this.button_StartIndent.UseVisualStyleBackColor = true;
            this.button_StartIndent.Click += new System.EventHandler(this.button_StartIndent_Click);
            // 
            // textBox_Indent_StepSize
            // 
            this.textBox_Indent_StepSize.Location = new System.Drawing.Point(332, 181);
            this.textBox_Indent_StepSize.Name = "textBox_Indent_StepSize";
            this.textBox_Indent_StepSize.Size = new System.Drawing.Size(63, 20);
            this.textBox_Indent_StepSize.TabIndex = 1;
            this.textBox_Indent_StepSize.Text = "10";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(258, 184);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(68, 13);
            this.label1.TabIndex = 2;
            this.label1.Text = "step size: nm";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(258, 250);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(54, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "depth: nm";
            // 
            // textBox_Indent_Depth
            // 
            this.textBox_Indent_Depth.Location = new System.Drawing.Point(332, 247);
            this.textBox_Indent_Depth.Name = "textBox_Indent_Depth";
            this.textBox_Indent_Depth.Size = new System.Drawing.Size(63, 20);
            this.textBox_Indent_Depth.TabIndex = 3;
            this.textBox_Indent_Depth.Text = "1000";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(258, 220);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(47, 13);
            this.label3.TabIndex = 6;
            this.label3.Text = "start: nm";
            // 
            // textBox_Indent_StartPosition
            // 
            this.textBox_Indent_StartPosition.Location = new System.Drawing.Point(332, 217);
            this.textBox_Indent_StartPosition.Name = "textBox_Indent_StartPosition";
            this.textBox_Indent_StartPosition.Size = new System.Drawing.Size(63, 20);
            this.textBox_Indent_StartPosition.TabIndex = 5;
            this.textBox_Indent_StartPosition.Text = "0";
            // 
            // button_CancelIndent
            // 
            this.button_CancelIndent.Location = new System.Drawing.Point(170, 50);
            this.button_CancelIndent.Name = "button_CancelIndent";
            this.button_CancelIndent.Size = new System.Drawing.Size(85, 41);
            this.button_CancelIndent.TabIndex = 7;
            this.button_CancelIndent.Text = "cancel";
            this.button_CancelIndent.UseVisualStyleBackColor = true;
            this.button_CancelIndent.Click += new System.EventHandler(this.button_CancelIndent_Click);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(45, 127);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(52, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "file name:";
            // 
            // textBox_FileName
            // 
            this.textBox_FileName.Location = new System.Drawing.Point(119, 124);
            this.textBox_FileName.Name = "textBox_FileName";
            this.textBox_FileName.Size = new System.Drawing.Size(63, 20);
            this.textBox_FileName.TabIndex = 8;
            this.textBox_FileName.Text = "SiO2";
            // 
            // Form_Indentation
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(488, 365);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.textBox_FileName);
            this.Controls.Add(this.button_CancelIndent);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox_Indent_StartPosition);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.textBox_Indent_Depth);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.textBox_Indent_StepSize);
            this.Controls.Add(this.button_StartIndent);
            this.Name = "Form_Indentation";
            this.Text = "Indentation";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button button_StartIndent;
        private System.Windows.Forms.TextBox textBox_Indent_StepSize;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox textBox_Indent_Depth;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox textBox_Indent_StartPosition;
        private System.Windows.Forms.Button button_CancelIndent;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox textBox_FileName;
    }
}