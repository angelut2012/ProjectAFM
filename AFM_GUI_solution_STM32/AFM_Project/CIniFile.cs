using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;

namespace NameSpace_AFM_Project
{
    public class CIniFile
    {
        private string filePath;
         
        [DllImport("kernel32")]
        private static extern long WritePrivateProfileString(string section,
        string key,
        string val,
        string filePath);
 
        [DllImport("kernel32")]
        private static extern int GetPrivateProfileString(string section,
        string key,
        string def,
        StringBuilder retVal,
        int size,
        string filePath);

        public CIniFile(string filePath)
        {
            this.filePath = filePath;
        }
        public void WriteDouble(string section, string key, double value)
        {
            string str = value.ToString();
            WriteString(section, key, str.ToLower());
        } 
        public void WriteString(string section, string key, string value)
        {
            WritePrivateProfileString(section, key, value.ToLower(), this.filePath);
        }

        public string ReadString(string section, string key)
        {
            StringBuilder SB = new StringBuilder(255);
            int i = GetPrivateProfileString(section, key, "", SB, 255, this.filePath);
            return SB.ToString();
        }
        public double ReadDouble(string section, string key)
        {
            string str=ReadString(section, key);
            return Convert.ToDouble(str);
        }
         
        public string FilePath
        {
            get { return this.filePath; }
            set { this.filePath = value; }
        }
    }
}
