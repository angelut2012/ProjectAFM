﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using System.IO;
using ZedGraph;
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
        //Image ConvertArray2Image(double[,]mImageArray)
        //{
        //    //Get image data from gridview column.
        //    byte[] imageData = new byte[512 * 512];
        //    //for (int k = 0; k < mImageArray.Length; k++)
        //    //    imageData[k] = (byte)mImageArray[k,1];
        //    //Initialize image variable
        //    Image newImage;
        //    //Read image data into a memory stream
        //    using (MemoryStream ms = new MemoryStream(imageData, 0, imageData.Length))
        //    {
        //        ms.Write(imageData, 0, imageData.Length);

        //        //Set image variable value using memory stream.
        //        newImage = Image.FromStream(ms, true);
        //    }

        //    return newImage;
        //}

        private void button1_Click(object sender, EventArgs e)
        { UpdateUI_ShowImageLine(); }
        public void UpdateUI_ShowImageLine()
        {
            try
             {
            ShowLine(pParent.mImageArrayHL, pParent.point_now_y, zedGraphControl_Height, true, Color.Red);
            ShowLine(pParent.mImageArrayHR, pParent.point_now_y, zedGraphControl_Height2, true, Color.Blue);


            ShowLine(pParent.mImageArrayEL, pParent.point_now_y, zedGraphControl_Error, true, Color.Red);
            ShowLine(pParent.mImageArrayER, pParent.point_now_y, zedGraphControl_Error2, true, Color.Blue);

            //pictureBox_Height.Image = ConvertArray2Image(pParent.mImageArrayHR);

             }
            catch
            {
                return;
            }

        }
        public double[,] Calculate_Line(double[,] mImageArray, int point_now_y, int fit_order = 1, int index_base_point = 1)
        {
            //ref double[,] line_show, 
            try
            {
                int L = mImageArray.GetLength(1);
                double[,] line_in = new double[1, L];
                if (point_now_y > 1) point_now_y--;

                if (point_now_y > 1) point_now_y--;// twice

                if (point_now_y < 1) point_now_y = 0;
                point_now_y = Math.Min(point_now_y, mImageArray.GetLength(1));
                for (int k = 0; k < L; k++)
                    line_in[0, k] = mImageArray[k, point_now_y];



                object Oline_show = null;// (object)line_show;
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

        public void ShowLine(double[,]mImageArray,int point_now_y,ZedGraph.ZedGraphControl zg,bool clear,Color in_color )
        {
            
            double[,] line_show = Calculate_Line(mImageArray, point_now_y);
            
                line_show[1, line_show.Length] = line_show[1, line_show.Length - 1];// delete last point
                line_show[1, 1] = line_show[1, 2];// delete last point
 
            double[] line_x = new double[line_show.GetLength(1)];
            double[] line_y = new double[line_show.GetLength(1)];
            for (int k = 0; k < line_show.GetLength(1); k++)
            {
                line_x[k] = k;
                line_y[k] = line_show[1, k+1];
            }

            if (clear==true)zg.GraphPane.CurveList.Clear();
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
