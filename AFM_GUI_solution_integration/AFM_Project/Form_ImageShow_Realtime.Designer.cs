namespace NameSpace_AFM_Project
{
    partial class Form_ImageShow_Realtime
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
            this.components = new System.ComponentModel.Container();
            this.zedGraphControl_Height = new ZedGraph.ZedGraphControl();
            this.zedGraphControl_Error2 = new ZedGraph.ZedGraphControl();
            this.button1 = new System.Windows.Forms.Button();
            this.pictureBox_Height = new System.Windows.Forms.PictureBox();
            this.zedGraphControl_Height2 = new ZedGraph.ZedGraphControl();
            this.zedGraphControl_Error = new ZedGraph.ZedGraphControl();
            this.timer_UpdateUI_Show = new System.Windows.Forms.Timer(this.components);
            this.pictureBox_Error = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Height)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Error)).BeginInit();
            this.SuspendLayout();
            // 
            // zedGraphControl_Height
            // 
            this.zedGraphControl_Height.IsShowPointValues = false;
            this.zedGraphControl_Height.Location = new System.Drawing.Point(12, 12);
            this.zedGraphControl_Height.Name = "zedGraphControl_Height";
            this.zedGraphControl_Height.PointValueFormat = "G";
            this.zedGraphControl_Height.Size = new System.Drawing.Size(644, 285);
            this.zedGraphControl_Height.TabIndex = 0;
            // 
            // zedGraphControl_Error2
            // 
            this.zedGraphControl_Error2.IsShowPointValues = false;
            this.zedGraphControl_Error2.Location = new System.Drawing.Point(662, 307);
            this.zedGraphControl_Error2.Name = "zedGraphControl_Error2";
            this.zedGraphControl_Error2.PointValueFormat = "G";
            this.zedGraphControl_Error2.Size = new System.Drawing.Size(644, 285);
            this.zedGraphControl_Error2.TabIndex = 1;
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(1312, 12);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(37, 23);
            this.button1.TabIndex = 2;
            this.button1.Text = "button1";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // pictureBox_Height
            // 
            this.pictureBox_Height.Location = new System.Drawing.Point(1380, 307);
            this.pictureBox_Height.Name = "pictureBox_Height";
            this.pictureBox_Height.Size = new System.Drawing.Size(358, 308);
            this.pictureBox_Height.TabIndex = 3;
            this.pictureBox_Height.TabStop = false;
            // 
            // zedGraphControl_Height2
            // 
            this.zedGraphControl_Height2.IsShowPointValues = false;
            this.zedGraphControl_Height2.Location = new System.Drawing.Point(12, 307);
            this.zedGraphControl_Height2.Name = "zedGraphControl_Height2";
            this.zedGraphControl_Height2.PointValueFormat = "G";
            this.zedGraphControl_Height2.Size = new System.Drawing.Size(644, 285);
            this.zedGraphControl_Height2.TabIndex = 4;
            // 
            // zedGraphControl_Error
            // 
            this.zedGraphControl_Error.IsShowPointValues = false;
            this.zedGraphControl_Error.Location = new System.Drawing.Point(662, 12);
            this.zedGraphControl_Error.Name = "zedGraphControl_Error";
            this.zedGraphControl_Error.PointValueFormat = "G";
            this.zedGraphControl_Error.Size = new System.Drawing.Size(644, 285);
            this.zedGraphControl_Error.TabIndex = 5;
            // 
            // timer_UpdateUI_Show
            // 
            this.timer_UpdateUI_Show.Interval = 500;
            this.timer_UpdateUI_Show.Tick += new System.EventHandler(this.timer_UpdateUI_Show_Tick);
            // 
            // pictureBox_Error
            // 
            this.pictureBox_Error.Location = new System.Drawing.Point(1380, -7);
            this.pictureBox_Error.Name = "pictureBox_Error";
            this.pictureBox_Error.Size = new System.Drawing.Size(358, 308);
            this.pictureBox_Error.TabIndex = 6;
            this.pictureBox_Error.TabStop = false;
            // 
            // Form_ImageShow_Realtime
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1348, 289);
            this.Controls.Add(this.pictureBox_Error);
            this.Controls.Add(this.zedGraphControl_Error);
            this.Controls.Add(this.zedGraphControl_Height2);
            this.Controls.Add(this.pictureBox_Height);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.zedGraphControl_Error2);
            this.Controls.Add(this.zedGraphControl_Height);
            this.Name = "Form_ImageShow_Realtime";
            this.Text = "Form_ImageShow_Realtime";
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Height)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox_Error)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private ZedGraph.ZedGraphControl zedGraphControl_Height;
        private ZedGraph.ZedGraphControl zedGraphControl_Error2;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.PictureBox pictureBox_Height;
        private ZedGraph.ZedGraphControl zedGraphControl_Height2;
        private ZedGraph.ZedGraphControl zedGraphControl_Error;
        private System.Windows.Forms.Timer timer_UpdateUI_Show;
        private System.Windows.Forms.PictureBox pictureBox_Error;
    }
}