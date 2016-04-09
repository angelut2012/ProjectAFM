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
    public partial class Form_ImageShow_DrawROI : Form
    {
        MainWindow pParent;
        public Form_ImageShow_DrawROI(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;
            pictureBox1.Image = new Bitmap("SetScanROI.bmp");
            //pictureBox1.Image = convert_ByteArrayToBitmap(pParent.mImageArrayHL);
        }

        public Bitmap convert_ByteArrayToBitmap(byte[,] byteArray)
        {
            ImageConverter ic = new ImageConverter();
            Image img = (Image)ic.ConvertFrom(byteArray);
            Bitmap bitmap1 = new Bitmap(img);
            return bitmap1;
        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }
        private Point RectStartPoint;
        private Rectangle Rect = new Rectangle();
        private Brush selectionBrush = new SolidBrush(Color.FromArgb(200, 0, 0, 220));
        private Pen pen = new Pen(Color.FromName("red"));
        // Start Rectangle
        //
        private void pictureBox1_MouseDown(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            // Determine the initial rectangle coordinates...
            RectStartPoint = e.Location;
            Invalidate();
        }

        // Draw Rectangle
        //
        private void pictureBox1_MouseMove(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left)
                return;
            Point tempEndPoint = e.Location;
            Rect.Location = new Point(
                Math.Min(RectStartPoint.X, tempEndPoint.X),
                Math.Min(RectStartPoint.Y, tempEndPoint.Y));
            Rect.Size = new Size(
                Math.Abs(RectStartPoint.X - tempEndPoint.X),
                Math.Abs(RectStartPoint.Y - tempEndPoint.Y));
            pictureBox1.Invalidate();
        }

        // Draw Area
        //
        private void pictureBox1_Paint(object sender, System.Windows.Forms.PaintEventArgs e)
        {
            // Draw the rectangle...
            //if (pictureBox1.Image != null)
            {
                if (Rect != null && Rect.Width > 0 && Rect.Height > 0)
                {
                    //e.Graphics.FillRectangle(selectionBrush, Rect);
                    e.Graphics.DrawRectangle(pen,Rect);
                }
            }
        }

        private void pictureBox1_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right)
            {
                if (Rect.Contains(e.Location))
                {
                    //Debug.WriteLine("Right click");
                }
            }
        }
    }
}
