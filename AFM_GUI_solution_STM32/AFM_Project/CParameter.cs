using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.ComponentModel;
using CSE;
using System.Globalization;
using System.CodeDom.Compiler;
using Microsoft.CSharp;
using Microsoft.VisualBasic;

namespace NameSpace_AFM_Project
{
    [DefaultPropertyAttribute("AFM paremeters")]
    public class CParameter
    {
        const int mI_MaxStep = 4096;//6000;
        public const int mI_LoopDelay_Min_uS = 104;

        enum ParaName { Sun, Mon, Tue, Wed, Thu, Fri, Sat };
        // indentation parameters, input from GUI
        double mI_MaxDepth = 6000;//MAX_RANGE_Z_NM for savety reason only
        double mI_step_size_nm = 0.1;
        double mI_StiffnessPRC_nN_per_nm = 40.0;//30~40N/m, 40 nN/nm
        double mI_PRC_ADCValue_per_nm = 37.5270000321971;// prc6
        //89.602534853964968;//68.2331;//102 86.9;//

        double mI_TriggerForce_nN = 1500;//mI_StiffnessPRC_nN_per_nm * 100;// 100nm
        double mI_LoopDelay_uS = mI_LoopDelay_Min_uS;// 
        uint mI_AllNumberOfSamplingPoints = mI_MaxStep;

        string mSampleName = "Sample";
        string mProbeName = "Probe_x";

        const int NumOfPara = 20;
        object[,] mO;//= new object[NumOfPara];
        public string GetParaString()
        {
            int k = 0;
            mO[k, 0] = (object)mSampleName; mO[k, 1] = "SampleName"; k++;
            mO[k, 0] = (object)mProbeName; mO[k, 1] = "ProbeName"; k++;
            mO[k, 0] = (object)mI_MaxDepth; mO[k, 1] = "MaxDepth"; k++;
            mO[k, 0] = (object)mI_step_size_nm; mO[k, 1] = "step_size_nm"; k++;
            mO[k, 0] = (object)mI_StiffnessPRC_nN_per_nm; mO[k, 1] = "StiffnessPRC_nN_per_nm"; k++;
            mO[k, 0] = (object)mI_PRC_ADCValue_per_nm; mO[k, 1] = "PRC_ADCValue_per_nm"; k++;
            mO[k, 0] = (object)mI_TriggerForce_nN; mO[k, 1] = "TriggerForce_nN"; k++;
            mO[k, 0] = (object)mI_LoopDelay_uS; mO[k, 1] = "LoopDelay_uS"; k++;
            mO[k, 0] = (object)mI_AllNumberOfSamplingPoints; mO[k, 1] = "NumberOfSamplingPoints"; k++;

            string text = null;
            for (int j = 0; j < mO.Length / mO.Rank; j++)
                text += mO[j, 0].ToString() + "%" + mO[j, 1].ToString() + "\r\n";
            return text;
        }

        ///--------------
        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(1)]
        public double TriggerForce_nN
        {
            get
            {
                return this.mI_TriggerForce_nN;
            }
            set
            {
                this.mI_TriggerForce_nN = value;
            }
        }
        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(25000)]
        public double MaxIndentationDepth_nm
        {
            get { return this.mI_MaxDepth; }
            set { this.mI_MaxDepth = value; }
        }

        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(25000)]
        public double StepSize_nm
        {
            get { return this.mI_step_size_nm; }
            set { this.mI_step_size_nm = value; }
        }

        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(40)]
        public double PRCStiffness_nN_per_nm
        {
            get { return this.mI_StiffnessPRC_nN_per_nm; }
            set { this.mI_StiffnessPRC_nN_per_nm = value; }
        }
        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(86.9)]
        public double PRC_ADCValue_per_nm
        {
            get { return this.mI_PRC_ADCValue_per_nm; }
            set { this.mI_PRC_ADCValue_per_nm = value; }
        }

        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("parameters for AFM indentation"),
        DefaultValueAttribute(1)]
        public double LoopDelay_uS
        {
            get { return this.mI_LoopDelay_uS; }
            set
            {
                this.mI_LoopDelay_uS = value;
                if (this.mI_LoopDelay_uS < mI_LoopDelay_Min_uS)
                    this.mI_LoopDelay_uS = mI_LoopDelay_Min_uS;
            }
        }

        [CategoryAttribute("AFM indentation"),
        DescriptionAttribute("number of points to sample and store and send back"),
        DefaultValueAttribute(500)]
        public uint NumberOfSamplingPoints
        {
            get { return this.mI_AllNumberOfSamplingPoints; }
            set
            {
                if (mI_AllNumberOfSamplingPoints > mI_MaxStep) value = mI_MaxStep;
                this.mI_AllNumberOfSamplingPoints = value;
            }
        }

        //-------------------
        [DescriptionAttribute("the name of the sample"),
        DefaultValueAttribute("Sample")]
        public string SampleName
        {
            get { return this.mSampleName; }
            set { this.mSampleName = value; }
        }

        [DescriptionAttribute("the name of the sample"),
        DefaultValueAttribute("Probe_x")]
        public string ProbeName
        {
            get { return this.mProbeName; }
            set { this.mProbeName = value; }
        }

        public CParameter()
        {
            mO = new object[NumOfPara, 2];
            for (int k = 0; k < NumOfPara; k++)
            { mO[k, 0] = ""; mO[k, 1] = ""; }

            //int x = 2;
            //CsEval.EvalEnvironment = this;
            //object result = CsEval.Eval("3 + mI_MaxDepth / 2");

            // string x=GetParaString();
        }

        void test()
        {
            object[] Oin = new object[2];
            Oin[0] = (object)this;

            double[] data = new double[100];
            data[0] = 100;
            data[1] = 20;

            Oin[1] = (object)data;

            object output = Eval("Oin[1]=Oin[0];", ref Oin);
        }
        //  Eval > Evaluates C# sourcelanguage

        public static object Eval(string sCSCode, ref object[] Oin)
        {
            CSharpCodeProvider c = new CSharpCodeProvider();
            ICodeCompiler icc = c.CreateCompiler();
            CompilerParameters cp = new CompilerParameters();

            cp.ReferencedAssemblies.Add("system.dll");
            cp.ReferencedAssemblies.Add("system.xml.dll");
            cp.ReferencedAssemblies.Add("system.data.dll");
            cp.ReferencedAssemblies.Add("system.windows.forms.dll");
            cp.ReferencedAssemblies.Add("system.drawing.dll");

            cp.CompilerOptions = "/t:library";
            cp.GenerateInMemory = true;

            StringBuilder sb = new StringBuilder("");
            sb.Append("using System;\n");
            sb.Append("using System.Xml;\n");
            sb.Append("using System.Data;\n");
            sb.Append("using System.Data.SqlClient;\n");
            sb.Append("using System.Windows.Forms;\n");
            sb.Append("using System.Drawing;\n");

            sb.Append("namespace CSCodeEvaler{ \n");
            sb.Append("public class CSCodeEvaler{ \n");
            sb.Append("public object EvalCode(ref object [] Oin){\n");
            //sb.Append(sCSCode + "; \n");
            sb.Append("Oin[1]=1000*100; \n");
            //sb.Append("return " + sCSCode + "; \n");
            sb.Append("return (object)Oin; \n");
            sb.Append("} \n");
            sb.Append("} \n");
            sb.Append("}\n");

            CompilerResults cr = icc.CompileAssemblyFromSource(cp, sb.ToString());
            if (cr.Errors.Count > 0)
            {
                //MessageBox.Show("ERROR: " + cr.Errors[0].ErrorText,
                //   "Error evaluating cs code", MessageBoxButtons.OK,
                //   MessageBoxIcon.Error);
                return null;
            }

            System.Reflection.Assembly a = cr.CompiledAssembly;
            object o = a.CreateInstance("CSCodeEvaler.CSCodeEvaler");

            Type t = o.GetType();
            System.Reflection.MethodInfo mi = t.GetMethod("EvalCode");

            // Oin[1] = "test";
            object s = mi.Invoke(o, Oin);
            //object s = mi.Invoke(o,  Oin);
            return s;
        }
    }
}