using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Drawing.Imaging;
using MathNet.Numerics;

namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form
    {

        public static void WriteColumn<T>(ref T[,] matrix, T[] array, int row)
        {
            var columns = matrix.GetLength(0);
            //var array = new T[columns];
            for (int i = 0; i < columns; ++i)
                matrix[i,row] = array[i];
            //return array;
        }
        public static T[] ReadColumn<T>(T[,] matrix, int row)
        {
            var columns = matrix.GetLength(0);
            var array = new T[columns];
            for (int i = 0; i < columns; ++i)
                array[i] = matrix[i,row];
            return array;
        }

        public static void WriteRow<T>(ref T[,] matrix, T[] array, int row)
        {
            var columns = matrix.GetLength(1);
            //var array = new T[columns];
            for (int i = 0; i < columns; ++i)
                matrix[row, i] = array[i];
            //return array;
        }
        public static T[] ReadRow<T>(T[,] matrix, int row)
        {
            var columns = matrix.GetLength(1);
            var array = new T[columns];
            for (int i = 0; i < columns; ++i)
                array[i] = matrix[row, i];
            return array;
        }

        public void Math_AFM_line_polyfit_adjust(ref double[] rawdata, int order)
        {
            int rawdata_length = rawdata.Length;
            double[] xdata = new double[rawdata_length];
            for (int i = 0; i < rawdata_length; i++)
                xdata[i] = i + 1;

            Double[] p = Fit.Polynomial(xdata, rawdata, order, MathNet.Numerics.LinearRegression.DirectRegressionMethod.QR);

            for (int n = 0; n < rawdata_length; n++)
            {
                double temp = 0;
                for (int k = 0; k <= order; k++)
                    temp = temp + p[k] * Math.Pow(xdata[n], k);

                rawdata[n] -= temp;
            }
        }

        private double[] DataLineNormalization(double[] data)
        {
            int data_length = data.Length;
            double[] norm_data = new double[data_length];
            for (int k = 0; k < data_length; k++)
            {
                norm_data[k] = 255 * (data[k] - data.Min()) / (data.Max() - data.Min());    // y=(x-MinValue)/(MaxValue-MinValue)        
            }
            return norm_data;
        }

        private double[,] DataPlaneNormalization(double[,] data)
        {
            int data_length = data.GetLength(0);      // int data_length = data.Length;        

            double data_max = data.Cast<double>().Max();
            double data_min = data.Cast<double>().Min();

            double[,] norm_data = new double[data_length, data_length];

            for (int k = 0; k < data_length; k++)
            {
                for (int m = 0; m < data_length; m++)
                {
                    norm_data[m, k] = 255 * (data[m, k] - data_max) / (data_max - data_min);    // y=(x-MinValue)/(MaxValue-MinValue)        
                }
            }
            return norm_data;
        }

        //mImageArrayHL
        void AFM_ScaningImageShow_RealTime(double[,] mImageArray)
        {
            //mImageArrayHL = new double[(int)para_Nx__dimension_0, (int)para_Ny__dimension_1];
            int ix = mImageArray.GetLength(0);
            int iy = mImageArray.GetLength(1);

            double[,] temp_img = new double[ix, iy];
            double[] temp_row = new double[ix];
            double[] normalized_data = new double[ix];

            for (int y = 0; y < iy; y++)
            {
                for (int x = 0; x < ix; x++)
                {
                    temp_row[x] = mImageArray[x,y];//RawImage[m, n];
                }

                Math_AFM_line_polyfit_adjust(ref temp_row, 2);
                for (int k = 0; k < ix; k++)
                {
                    temp_img[k, y] = temp_row[k];
                }
            }

            //    double[] normalized_data = new double[128];
            //  normalized_data = DataNormalization(ydata);

            //// test image
            //for (int k = 0; k < para_Ny; k++)
            //    for (int m = 0; m < para_Nx; m++)
            //        mImageArrayHL[m, k] = -1;
            //point_now_y++;
            //point_now_y %= (int)para_Ny;
            //for (int k = 0; k < point_now_y; k++)
            //    for (int m = 0; m < para_Nx; m++)
            //        mImageArrayHL[m, k] = 100.0 * Math.Sin(m / 10.0) + Math.Cos((k + m) / 30.0) + k / 20.0;

            //--------------------------------
            //object OmImageArrayHL = (object)mImageArrayHL;     // Height image
            //int line_now = 0;
            //double[] para = new double[4];
            //para[0] = line_now;
            //para[1] = 0;// do not show image in matlab
            //para[2] = 1;// line fit order
            //para[3] = 1;// // save data
            //object Opara = (object)para;

            //double[,] out_r = new double[(int)para_Nx, (int)para_Ny];
            //double[,] out_g = new double[(int)para_Nx, (int)para_Ny];
            //double[,] out_b = new double[(int)para_Nx, (int)para_Ny];
            //object Oout_r = (object)out_r;
            //object Oout_g = (object)out_g;
            //object Oout_b = (object)out_b;
            //MY_DEBUG("start  ");
            //try
            //{
            ////    mKernelClass.AFM_convert_height2RGB(3, ref Oout_r, ref Oout_g, ref Oout_b, OmImageArrayHL, Opara);    // time =5s

            //    //MY_DEBUG("end  ");
            //    //out_r = (double[,])Oout_r;
            //    //out_g = (double[,])Oout_g;
            //    //out_b = (double[,])Oout_b;
            //}
            //catch
            //{
            //    MY_DEBUG("AFM_convert_height2RGB error.");
            //    return;
            //}
            //double[,,] t = new double[100, 200,300];
            //int s0 = t.GetLength(0);
            //int s1 = t.GetLength(1);
            //int s2 = t.GetLength(2);

            int p = 0;
            var data = new byte[(int)(ix * iy * 4)];

            temp_img = DataPlaneNormalization(temp_img);

            for (int y = 0; y < iy; y++)
            {
                for (int x = 0; x < ix; x++)
                {
                   // temp_row[x] = mImageArray[x, y];//RawImage[m, n];
                    data[p++] = 0;    // Convert.ToByte(Math.Abs(temp_img[m1, k1]));
                    data[p++] = Convert.ToByte(Math.Abs(temp_img[x, y]));
                    data[p++] = Convert.ToByte(Math.Abs(temp_img[x, y]));
                    data[p++] = 0;
                }
            }

            //for (int k1 = 0; k1 < iy; k1++)
            //{
            //    //for (int m1 = 1; m1 <= ix; m1++)
            //    //{
            //    data[p++] = Convert.ToByte(normalized_data[k1]);
            //    data[p++] = Convert.ToByte(normalized_data[k1]);
            //    data[p++] = Convert.ToByte(normalized_data[k1]);
            //    data[p++] = 0;
            //    //   }
            //}

            Bitmap image = null;
            unsafe
            {
                fixed (byte* ptr = data)
                {
                    // Craete a bitmap wit a raw pointer to the data
                    using (image = new Bitmap(ix, iy, ix * 4, PixelFormat.Format32bppRgb, new IntPtr(ptr)))
                    {
                        // And save it.
                        // image.Save(Path.ChangeExtension(fileName, ".bmp"));
                        int scale = pictureBox_AFM_Real_Image.Width;
                        mAFMImage_show = image;
                        Bitmap mAFMImage_show_scale = new Bitmap(mAFMImage_show, pictureBox_AFM_Real_Image.Width, pictureBox_AFM_Real_Image.Height);
                        //mSEMImage_show_scale = new Bitmap(mSEMImage_show, pictureBox_SEMImage.Width, pictureBox_SEMImage.Height);
                        pictureBox_AFM_Real_Image.Image = mAFMImage_show_scale;
                    }
                }
            }
        }
        
    }
}
