namespace NameSpace_AFM_Project
{
    partial class Form_CoarsePositioner
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
            this.button_XN = new System.Windows.Forms.Button();
            this.button_XP = new System.Windows.Forms.Button();
            this.button_YP = new System.Windows.Forms.Button();
            this.button_YN = new System.Windows.Forms.Button();
            this.button_ZN = new System.Windows.Forms.Button();
            this.button_ZP = new System.Windows.Forms.Button();
            this.textBox_Distance = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.textBox_Frequency = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.button_CoarsePositioner_Connect = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // button_XN
            // 
            this.button_XN.Location = new System.Drawing.Point(12, 108);
            this.button_XN.Name = "button_XN";
            this.button_XN.Size = new System.Drawing.Size(100, 100);
            this.button_XN.TabIndex = 0;
            this.button_XN.Text = "<";
            this.button_XN.UseVisualStyleBackColor = true;
            this.button_XN.Click += new System.EventHandler(this.button_XN_Click);
            // 
            // button_XP
            // 
            this.button_XP.Location = new System.Drawing.Point(226, 108);
            this.button_XP.Name = "button_XP";
            this.button_XP.Size = new System.Drawing.Size(100, 100);
            this.button_XP.TabIndex = 1;
            this.button_XP.Text = ">";
            this.button_XP.UseVisualStyleBackColor = true;
            this.button_XP.Click += new System.EventHandler(this.button_XP_Click);
            // 
            // button_YP
            // 
            this.button_YP.Location = new System.Drawing.Point(121, 4);
            this.button_YP.Name = "button_YP";
            this.button_YP.Size = new System.Drawing.Size(100, 100);
            this.button_YP.TabIndex = 2;
            this.button_YP.Text = "^";
            this.button_YP.UseVisualStyleBackColor = true;
            this.button_YP.Click += new System.EventHandler(this.button_YP_Click);
            // 
            // button_YN
            // 
            this.button_YN.Location = new System.Drawing.Point(121, 214);
            this.button_YN.Name = "button_YN";
            this.button_YN.Size = new System.Drawing.Size(100, 100);
            this.button_YN.TabIndex = 3;
            this.button_YN.Text = "v";
            this.button_YN.UseVisualStyleBackColor = true;
            this.button_YN.Click += new System.EventHandler(this.button_YN_Click);
            // 
            // button_ZN
            // 
            this.button_ZN.Location = new System.Drawing.Point(370, 214);
            this.button_ZN.Name = "button_ZN";
            this.button_ZN.Size = new System.Drawing.Size(100, 100);
            this.button_ZN.TabIndex = 4;
            this.button_ZN.Text = "v";
            this.button_ZN.UseVisualStyleBackColor = true;
            this.button_ZN.Click += new System.EventHandler(this.button_ZN_Click);
            // 
            // button_ZP
            // 
            this.button_ZP.Location = new System.Drawing.Point(370, 12);
            this.button_ZP.Name = "button_ZP";
            this.button_ZP.Size = new System.Drawing.Size(100, 100);
            this.button_ZP.TabIndex = 5;
            this.button_ZP.Text = "^";
            this.button_ZP.UseVisualStyleBackColor = true;
            this.button_ZP.Click += new System.EventHandler(this.button_ZP_Click);
            // 
            // textBox_Distance
            // 
            this.textBox_Distance.Location = new System.Drawing.Point(205, 346);
            this.textBox_Distance.Name = "textBox_Distance";
            this.textBox_Distance.Size = new System.Drawing.Size(58, 20);
            this.textBox_Distance.TabIndex = 6;
            this.textBox_Distance.Text = "5";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(113, 350);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(71, 13);
            this.label1.TabIndex = 7;
            this.label1.Text = "step size (um)";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(325, 350);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(76, 13);
            this.label2.TabIndex = 8;
            this.label2.Text = "frequency (Hz)";
            // 
            // textBox_Frequency
            // 
            this.textBox_Frequency.Location = new System.Drawing.Point(414, 346);
            this.textBox_Frequency.Name = "textBox_Frequency";
            this.textBox_Frequency.Size = new System.Drawing.Size(41, 20);
            this.textBox_Frequency.TabIndex = 9;
            this.textBox_Frequency.Text = "1000";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            this.label3.Location = new System.Drawing.Point(405, 154);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(31, 31);
            this.label3.TabIndex = 10;
            this.label3.Text = "Z";
            // 
            // button_CoarsePositioner_Connect
            // 
            this.button_CoarsePositioner_Connect.Location = new System.Drawing.Point(12, 334);
            this.button_CoarsePositioner_Connect.Name = "button_CoarsePositioner_Connect";
            this.button_CoarsePositioner_Connect.Size = new System.Drawing.Size(100, 42);
            this.button_CoarsePositioner_Connect.TabIndex = 11;
            this.button_CoarsePositioner_Connect.Text = "connect";
            this.button_CoarsePositioner_Connect.UseVisualStyleBackColor = true;
            this.button_CoarsePositioner_Connect.Click += new System.EventHandler(this.button_CoarsePositioner_Connect_Click);
            // 
            // Form_CoarsePositioner
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(485, 390);
            this.Controls.Add(this.button_CoarsePositioner_Connect);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.textBox_Frequency);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.textBox_Distance);
            this.Controls.Add(this.button_ZP);
            this.Controls.Add(this.button_ZN);
            this.Controls.Add(this.button_YN);
            this.Controls.Add(this.button_YP);
            this.Controls.Add(this.button_XP);
            this.Controls.Add(this.button_XN);
            this.KeyPreview = true;
            this.Name = "Form_CoarsePositioner";
            this.Text = "Form_CoarsePositioner";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button button_XN;
        private System.Windows.Forms.Button button_XP;
        private System.Windows.Forms.Button button_YP;
        private System.Windows.Forms.Button button_YN;
        private System.Windows.Forms.Button button_ZN;
        private System.Windows.Forms.Button button_ZP;
        private System.Windows.Forms.TextBox textBox_Distance;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox textBox_Frequency;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Button button_CoarsePositioner_Connect;
    }
}