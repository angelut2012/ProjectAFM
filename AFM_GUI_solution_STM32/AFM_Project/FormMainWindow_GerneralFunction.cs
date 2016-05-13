using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using System.Threading;

using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Collections.Generic;
//using CLRWrapper;

namespace NameSpace_AFM_Project
{

    public partial class MainWindow : Form
    {

        //-------
        public void DOS_Command(string cmdstr)
        { System.Diagnostics.Process.Start("cmd.exe", cmdstr); }
        public void SpeakVoice(string words)
        {
            //C:\Program Files (x86)\Reference Assemblies\Microsoft\Framework\.NETFramework\v4.0
            System.Speech.Synthesis.SpeechSynthesizer mVoiceSpeaker = new System.Speech.Synthesis.SpeechSynthesizer();
            mVoiceSpeaker.SpeakAsync(words);
            //mVoiceSpeaker.Speak(words);
        }

        public void PlaySimpleSound(string file_name)
        {//@"c:\Windows\Media\chimes.wav"
            System.Media.SoundPlayer simpleSound = new System.Media.SoundPlayer(file_name);
            simpleSound.Play();
        }
        public void SoundNotice(int k, int sleep_ms)
        { SoundNotice(k); Thread.Sleep(sleep_ms); }
        public void SoundNotice(int k)
        {
            if (k == 0) System.Media.SystemSounds.Exclamation.Play();// ok
            if (k == 1) System.Media.SystemSounds.Hand.Play();// fail
            if (k == 2) System.Media.SystemSounds.Beep.Play();
            if (k == 3) System.Media.SystemSounds.Asterisk.Play();
            if (k == 4) System.Media.SystemSounds.Question.Play();
            if (k == 5) PlaySimpleSound(@"camera.wav");
            if (k == 6) PlaySimpleSound(@"chimes.wav");

        }


        static T Swap<T>(ref T lhs, ref T rhs) where T : System.IComparable<T>
        {
            T temp;
            temp = lhs;
            lhs = rhs;
            rhs = temp;
            return temp;
        }

        public uint MIN_MAX(uint v, uint down, uint up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public int MIN_MAX(int v, int down, int up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public double MIN_MAX(double v, double down, double up)
        {
            //v = Math.Max(v, down);
            //v = Math.Min(v, up);
            if (v > up) v = up;
            if (v < down) v = down;
            return v;
        }
        public void MY_DEBUG(string inf)
        {
            if (string.IsNullOrEmpty(inf) == false)
                System.Diagnostics.Debug.WriteLine(inf);
            //string str = "The quick brown fox jumped over the gentleman.";
            //{
            //    byte[] bytes = Encoding.ASCII.GetBytes(inf);
            //    unsafe
            //    {
            //        fixed (byte* p = bytes)
            //        {
            //            sbyte* sp = (sbyte*)p;
            //            //SP is now what you want
            //            mCCoarsePositioner.MY_DEBUG_CLR(sp);
            //        }
            //    }

            //}
        }
        public void MY_DEBUG(string inf, int x) { MY_DEBUG(inf + Convert.ToString(x)); }
        public void MY_DEBUG(string inf, double x) { MY_DEBUG(inf + Convert.ToString(x)); }
        public void MY_DEBUG(int x) { MY_DEBUG(Convert.ToString(x)); }

        public string GetCurrentTimeString() { return DateTime.Now.ToString("yyyyMMddHHmmss"); }
        public long GetCurrentTimeLong() { return DateTime.Now.ToFileTimeUtc(); }
        public long TIC() { return TIC_TOC(0); }
        public long TOC() { return TIC_TOC(1); }// 1e-7 S
        private long TIC_TOC_t;
        private long TIC_TOC(int s)
        {
            long t = 0;
            if (s == 0)
                t = TIC_TOC_t = GetCurrentTimeLong();// tic
            else
                t = GetCurrentTimeLong() - TIC_TOC_t;
            return t;
        }

    }
}