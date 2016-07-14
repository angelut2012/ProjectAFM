using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using DeckLinkAPI;
using System.IO;
using System.Runtime.InteropServices;
using System.Drawing.Imaging;
//using OpenCvSharp;
//using Emgu.CV;
//using Emgu.CV.Structure;
//using Emgu.CV.CvEnum;

namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form, IDeckLinkInputCallback
    {
        public Bitmap mSEMImage_show;
        Bitmap mSEMImage_show_scale;
        bool mbSave_SEMImage = false;
        int mSEMImage_SaveCount = 0;

        public IDeckLinkIterator _deckLinkIterator;
        public List<IDeckLink> _deckLinkList = new List<IDeckLink>();
        public IDeckLink _currentDevice = null;
        public IDeckLinkInput _deckLinkInput = null;

        //public int _width = 1280;
        //public int _height = 720;

        //const int DECK_WIDTH = 1920;    //800;
        //const int DECK_HEIGHT = 1080;   //600;
        const int DECK_WIDTH_OFF = 240;
        //int mImageSize = DECK_WIDTH * DECK_HEIGHT;      // * 2;
        int frameCount = 0;

        // private WriteableBitmap _writeableBitmap = null;

        //IntPtr _tempRGBData;
        //byte[] _tempRGBDataBytes;

        // DispatcherTimer _timer = new DispatcherTimer();

        //public void MY_DEBUG(string inf)
        //{
        //    if (string.IsNullOrEmpty(inf) == false)
        //        System.Diagnostics.Debug.WriteLine(inf);
        //}
        //------------------------------------------------
        public void SEM_Image_StartCapture()
        {
            _deckLinkInput.StartStreams();
        }

        public void SEM_Image_StopCapture()
        {
            _deckLinkInput.StopStreams();
        }

        public void SEM_Image_Initialize()
        {
            _deckLinkIterator = new CDeckLinkIterator();

            IDeckLink dl = null;
            while (true)
            {
                _deckLinkIterator.Next(out dl);

                if (dl == null)
                {
                    break;
                }
                else
                {
                    _deckLinkList.Add(dl);
                }
            }

            foreach (IDeckLink device in _deckLinkList)
            {
                String name;
                device.GetModelName(out name);
                MY_DEBUG("" + name);
            }

            _currentDevice = _deckLinkList[0];
            _deckLinkInput = (IDeckLinkInput)_currentDevice;

            _deckLinkInput.SetCallback(this);

            _BMDDisplayModeSupport result;
            IDeckLinkDisplayMode resultDisplayMode;
            _deckLinkInput.DoesSupportVideoMode(_BMDDisplayMode.bmdModeHD1080p2398,
                _BMDPixelFormat.bmdFormat8BitYUV,
                _BMDVideoInputFlags.bmdVideoInputFlagDefault,
                out result,
                out resultDisplayMode);

            _deckLinkInput.EnableVideoInput(_BMDDisplayMode.bmdModeHD1080p2398,
                 _BMDPixelFormat.bmdFormat8BitYUV,
                _BMDVideoInputFlags.bmdVideoInputFlagDefault);

            SEM_Image_StartCapture();
        }

        //----------------------------------------------------



        //private void button2_Click(object sender, EventArgs e)
        //{
        //    _deckLinkInput.StopStreams();
        //}

        //private void button1_Click(object sender, EventArgs e)
        //{
        //    // _deckLinkInput.FlushStreams();

        //    _deckLinkInput.StartStreams();

        //    // _deckLinkInput.PauseStreams();
        //    // _deckLinkInput.FlushStreams();
        //}


        //void SaveImageToTextFile(byte[] image)
        //{
        //    string Fpath = "c:\\x.txt";
        //    //para_Nx = 128; para_Ny = 90;
        //    // StreamWriter writetext = new StreamWriter("write.txt");
        //    string text = null;     // new string(' ', 800 * 600);

        //    System.IO.File.WriteAllBytes("c:\\xbb_new.txt", image);

        //    //for (int x = 0; x < image.GetLength(0); x++)
        //    //{
        //    //    string t = Convert.ToString(image[x]) + "\t";
        //    //    text += t;
        //    //    // System.IO.File.AppendText(text);
        //    //}
        //    //System.IO.File.WriteAllText(Fpath, text);
        //}

        public Bitmap convertRGB2Bitmap_Show(byte[] imageData, int width, int height)
        {
            // Need to copy our 8 bit greyscale image into a 32bit layout.
            // Choosing 32bit rather than 24 bit as its easier to calculate stride etc.
            // This will be slow enough and isn't the most efficient method.
            var data = new byte[width * height * 4];

            int p = 0;

            for (var i = 0; i < width * height; i++)
            {
                var value = imageData[i];

                // Greyscale image so r, g, b, get the same
                // intensity value.
                data[p++] = value;
                data[p++] = value;
                data[p++] = value;
                data[p++] = 0;  // Alpha isn't actually used
            }

            Bitmap image = null;
            unsafe
            {
                fixed (byte* ptr = data)
                {
                    // Craete a bitmap wit a raw pointer to the data
                    using (image = new Bitmap(width, height, width * 4, PixelFormat.Format32bppRgb, new IntPtr(ptr)))
                    {
                        // And save it.
                        // image.Save(Path.ChangeExtension(fileName, ".bmp"));
                        int scale = pictureBox_SEMImage.Width;
                        Rectangle rect = new Rectangle(239, 0, 1440, 1080);
                        mSEMImage_show = image.Clone(rect, PixelFormat.Format32bppRgb);
                        if (mbSave_SEMImage == true)
                        {
                            mSEMImage_SaveCount++;
                            mSEMImage_show.Save("C:\\SEMImage" + mSEMImage_SaveCount.ToString() + ".bmp");
                            mbSave_SEMImage = false;
                        }
                        //mSEMImage_show_scale = new Bitmap(mSEMImage_show, width / scale, height / scale);
                        mSEMImage_show_scale = new Bitmap(mSEMImage_show, pictureBox_SEMImage.Width, pictureBox_SEMImage.Height);
                        pictureBox_SEMImage.Image = mSEMImage_show_scale;
                    }
                }
            }
            return image;
        }

        void convert_byteYUV_To_byteRGB(byte[] data_YUV, out byte[] data_RGB, int DECK_WIDTH, int DECK_HEIGHT)
        {
            data_RGB = new byte[DECK_HEIGHT * DECK_WIDTH * 3];
            int j;
            int u, y, r, c;
            try
            {
                for (r = 0; r < DECK_HEIGHT; r++)
                    for (c = 0; c < DECK_WIDTH; c += 2)
                    {
                        j = (r * DECK_WIDTH + c) * 2;    //DECK_WIDTH_OFF
                        u = data_YUV[j];
                        y = data_YUV[j + 1];

                        data_RGB[r * DECK_WIDTH + c] = (byte)(1.0 * y + 1.772 * (u - 128) + 0);        // b

                        y = data_YUV[j + 3];
                        data_RGB[r * DECK_WIDTH + c + 1] = (byte)(1.0 * y + 1.772 * (u - 128) + 0);
                    }
            }
            catch (Exception e)
            { }
        }

        public void VideoInputFrameArrived(IDeckLinkVideoInputFrame video, IDeckLinkAudioInputPacket audio)
        {
            // _deckLinkInput.PauseStreams();
            //get image data
            IntPtr pData;    // =new int[ DECK_WIDTH * DECK_HEIGHT * 2];
            video.GetBytes(out pData);

            // MY_DEBUG("video frame arrived!! " + frameCount + "\n");
            frameCount++;

            byte[] data_YUV = new byte[DECK_WIDTH * DECK_HEIGHT * 2];
            System.Runtime.InteropServices.Marshal.Copy(pData, data_YUV, 0, DECK_WIDTH * DECK_HEIGHT * 2);
            byte[] data_RGB;    // = new byte[mImageSize*3];
            convert_byteYUV_To_byteRGB(data_YUV, out data_RGB, DECK_WIDTH, DECK_HEIGHT);
            convertRGB2Bitmap_Show(data_RGB, DECK_WIDTH, DECK_HEIGHT);
            System.Runtime.InteropServices.Marshal.ReleaseComObject(video);
        }

        //static byte[] PadLines(byte[] bytes, int rows, int columns)
        //{
        //    var currentStride = columns; // 3
        //    var newStride = columns;  // 4
        //    var newBytes = new byte[newStride * rows];
        //    for (var i = 0; i < rows; i++)
        //        Buffer.BlockCopy(bytes, currentStride * i, newBytes, newStride * i, currentStride);
        //    return newBytes;
        //}

        public void VideoInputFormatChanged(_BMDVideoInputFormatChangedEvents events,
            IDeckLinkDisplayMode displayMode,
            _BMDDetectedVideoInputFormatFlags flags)
        {
            MY_DEBUG("video format changed!!");
        }
    }
}