namespace NameSpace_AFM_Project
{
    partial class Form_Indentation_PostPorcess
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
            this.Button_test = new System.Windows.Forms.Button();
            this.zgraph1 = new ZedGraph.ZedGraphControl();
            this.SuspendLayout();
            // 
            // Button_test
            // 
            this.Button_test.Location = new System.Drawing.Point(481, 12);
            this.Button_test.Name = "Button_test";
            this.Button_test.Size = new System.Drawing.Size(67, 29);
            this.Button_test.TabIndex = 0;
            this.Button_test.Text = "Test";
            this.Button_test.UseVisualStyleBackColor = true;
            this.Button_test.Click += new System.EventHandler(this.Button_test_Click);
            // 
            // zgraph1
            // 
            this.zgraph1.IsShowPointValues = false;
            this.zgraph1.Location = new System.Drawing.Point(12, 12);
            this.zgraph1.Name = "zgraph1";
            this.zgraph1.PointValueFormat = "G";
            this.zgraph1.Size = new System.Drawing.Size(453, 408);
            this.zgraph1.TabIndex = 1;
            // 
            // Form_SubForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(561, 440);
            this.Controls.Add(this.zgraph1);
            this.Controls.Add(this.Button_test);
            this.Name = "Form_SubForm";
            this.Text = "Form_SubForm";
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button Button_test;
        private ZedGraph.ZedGraphControl zgraph1;
    }
}