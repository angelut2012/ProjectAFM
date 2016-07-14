using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using ZedGraph;
using System.Threading;
//using System.Windows.Media;

namespace NameSpace_AFM_Project
{
    public partial class Form_ImageShow_Realtime : Form
    {
        MainWindow pParent;

        public Form_ImageShow_Realtime(MainWindow pmain)
        {
            InitializeComponent();
            pParent = pmain;
        }

        public void StartUpdate()
        {
            timer_UpdateUI_Show.Start();
        }

        Image ConvertArray2Image(double[,] mImageArray)
        {
            //Get image data from gridview column.
            //byte[] imageData = new byte[512 * 512];
            ////for (int k = 0; k < mImageArray.Length; k++)
            ////    imageData[k] = (byte)mImageArray[k,1];
            ////Initialize image variable
            //Image newImage;
            ////Read image data into a memory stream
            //using (MemoryStream ms = new MemoryStream(imageData, 0, imageData.Length))
            //{
            //    ms.Write(imageData, 0, imageData.Length);

            //    //Set image variable value using memory stream.
            //    newImage = Image.FromStream(ms, true);
            //}
            //Bitmap myBitmap = new Bitmap("SetScanROI.bmp");

            Bitmap myBitmap = new Bitmap(mImageArray.GetLength(0), mImageArray.GetLength(1));
            for (int x = 0; x < myBitmap.Width; x++)
            {
                for (int y = 0; y < myBitmap.Height; y++)
                {
                    //myBitmap.SetPixel(x, y, Color.Black);
                    Color myRgbColor = new Color();
                    int v = (int)mImageArray[x, y];
                    v = Math.Abs(v);
                    //v = x + y;
                    int r = v % 255;
                    int g = v % 100;
                    int b = v % 50;
                    myRgbColor = Color.FromArgb(r, g, b);
                    myBitmap.SetPixel(x, y, myRgbColor);
                }
            }

            if (pictureBox_Height.Image != null)
                pictureBox_Height.Image.Dispose();
            Image newImage = myBitmap.Clone(new Rectangle(0, 0, myBitmap.Width, myBitmap.Height), System.Drawing.Imaging.PixelFormat.DontCare);

            return newImage;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            // UpdateUI_ShowImageLine(); 
            int x = 100;
            pParent.MY_DEBUG("st");
            while (x-- > 0)
                pictureBox_Height.Image = ConvertArray2Image(pParent.mImageArrayHR);   // time =170 ms

            pParent.MY_DEBUG("end");
        }

        Thread mThread_UpdateUI_ShowImageLine;

        bool mLock_UpdateUI_ShowImageLine = false;
        public void UpdateUI_ShowImageLine()
        {
            //timer_UpdateUI_Show.Enabled = false;
            if (mLock_UpdateUI_ShowImageLine == true) return;
            mLock_UpdateUI_ShowImageLine = true;
            mThread_UpdateUI_ShowImageLine = new Thread(ThreadFunction_UpdateUI_ShowImageLine);
            mThread_UpdateUI_ShowImageLine.Start();
        }
        void ThreadFunction_UpdateUI_ShowImageLine()
        {
            try
            {
                pParent.MY_DEBUG("show line:", pParent.point_now_y);
                ShowLine(pParent.mImageArrayHL, pParent.point_now_y, zedGraphControl_Height, true, Color.Red);
                ShowLine(pParent.mImageArrayHR, pParent.point_now_y, zedGraphControl_Height2, true, Color.Blue);

                ShowLine(pParent.mImageArrayEL, pParent.point_now_y, zedGraphControl_Error, true, Color.Red);
                ShowLine(pParent.mImageArrayER, pParent.point_now_y, zedGraphControl_Error2, true, Color.Blue);

                //pictureBox_Height.Image = ConvertArray2Image(pParent.mImageArrayHL);// time =170 ms
                //pictureBox_Error.Image = ConvertArray2Image(pParent.mImageArrayEL);// time =170 ms
            }
            catch
            {
                //return;
                pParent.MY_DEBUG("show line error.");
            }
            mLock_UpdateUI_ShowImageLine = false;
            //timer_UpdateUI_Show.Enabled = true;
        }

        public double[,] Calculate_Line(double[,] mImageArray, int point_now_y, int fit_order = 1, int index_base_point = 1)
        {
            //ref double[,] line_show, 
            try
            {
                int L = mImageArray.GetLength(0);
                double[,] line_in = new double[1, L];
                if (point_now_y > 1) point_now_y--;    // show last line 

                point_now_y = pParent.MIN_MAX(point_now_y, 1, mImageArray.GetLength(1));
                for (int k = 0; k < L; k++)
                    line_in[0, k] = mImageArray[k, point_now_y - 1];

                object Oline_show = null;     // (object)line_show;
                object Oline_in = (object)line_in;
                object Ofit_order = (object)fit_order;
                object Oindex_base_point = (object)index_base_point;

                pParent.mKernelClass.AFM_line_for_show(1, ref Oline_show, Oline_in, Ofit_order, Oindex_base_point);

                double[,] line_show = (double[,])Oline_show;
                return line_show;
            }
            catch
            {
                double[,] line_show = new double[1, 512];
                return line_show;
            }
        }

        public void ShowLine(double[,] mImageArray, int point_now_y, ZedGraph.ZedGraphControl zg, bool clear, Color in_color)
        {
            double[,] line_show = Calculate_Line(mImageArray, point_now_y);

            line_show[1, line_show.Length] = line_show[1, line_show.Length - 1];// delete last point
            line_show[1, 1] = line_show[1, 2];    // delete last point

            double[] line_x = new double[line_show.GetLength(1)];
            double[] line_y = new double[line_show.GetLength(1)];
            for (int k = 0; k < line_show.GetLength(1); k++)
            {
                line_x[k] = k;
                line_y[k] = line_show[1, k + 1];
            }

            if (clear == true) zg.GraphPane.CurveList.Clear();
            zg.GraphPane.AddCurve("", line_x, line_y, in_color, SymbolType.Star);
            //((LineItem)zg.GraphPane.CurveList[0]).Line.Width = 3.0F;

            //   zg..AddCurve("", time, data, Color.Red, SymbolType.Plus);
            zg.AxisChange();

            zg.IsShowPointValues = true;
            //zg.GraphPane.Title = "Nano Force Sensor Readout";
            zg.GraphPane.XAxis.Title = "";
            zg.GraphPane.YAxis.Title = "nm";

            zg.Invalidate();
        }

        private void timer_UpdateUI_Show_Tick(object sender, EventArgs e)
        {
            UpdateUI_ShowImageLine();
        }
    }
}