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
//using CLRWrapper;


using System;
using System.IO;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;

using System.CodeDom.Compiler;
using Microsoft.CSharp;
using Microsoft.VisualBasic;
using System.Reflection;

using Westwind.RemoteLoader;
namespace NameSpace_AFM_Project
{
    public partial class MainWindow : Form
    {

        public void CallDynamicCode(string filename="c:\\Online_code.cs")
        {
            // ** Create an AppDomain
            AppDomainSetup loSetup = new AppDomainSetup();
            loSetup.ApplicationBase = AppDomain.CurrentDomain.BaseDirectory;
            AppDomain loAppDomain = AppDomain.CreateDomain("NameSpace_AFM_Project", null, loSetup);


            string lcCode = File.ReadAllText(filename);
            //string lcCode = File.ReadAllText("c:\\Online_code.cs");
            ICodeCompiler loCompiler = new CSharpCodeProvider().CreateCompiler();
            CompilerParameters loParameters = new CompilerParameters();

            // *** Start by adding any referenced assemblies
            loParameters.ReferencedAssemblies.Add("System.dll");
            loParameters.ReferencedAssemblies.Add("System.Windows.Forms.dll");
            loParameters.ReferencedAssemblies.Add("Remoteloader.dll");

            // *** Load the resulting assembly into memory
            loParameters.GenerateInMemory = false;
            loParameters.OutputAssembly = "AFM_DynamicCode.dll";

            // *** Now compile the whole thing
            CompilerResults loCompiled = loCompiler.CompileAssemblyFromSource(loParameters, lcCode);

            if (loCompiled.Errors.HasErrors)
            {
                string lcErrorMsg = "";

                // *** Create Error String
                lcErrorMsg = loCompiled.Errors.Count.ToString() + " Errors:";
                for (int x = 0; x < loCompiled.Errors.Count; x++)
                    lcErrorMsg = lcErrorMsg + "\r\nLine: " + loCompiled.Errors[x].Line.ToString() + " - " +
                        loCompiled.Errors[x].ErrorText;

                MessageBox.Show(lcErrorMsg + "\r\n\r\n" + lcCode, "Compiler Demo", MessageBoxButtons.OK, MessageBoxIcon.Error);

                return;
            }

            // create the factory class in the secondary app-domain
            RemoteLoaderFactory factory =
                (RemoteLoaderFactory)loAppDomain.CreateInstance("RemoteLoader",
                "Westwind.RemoteLoader.RemoteLoaderFactory").Unwrap();

            // with the help of this factory, we can now create a real 'LiveClass' instance
            object loObject = factory.Create("AFM_DynamicCode.dll", "NameSpace_AFM_Project.DynamicClass", null);

            // *** Cast the object to the remote interface to avoid loading type info
            IRemoteInterface loRemote = (IRemoteInterface)loObject;

            if (loObject == null)
            {
                MessageBox.Show("Couldn't load class.");
                return;
            }

            object[] loCodeParms = new object[1];
            loCodeParms[0] = this;// "West Wind Technologies";

            try
            {
                // *** Indirectly call the remote interface
                object loResult = loRemote.Invoke("DynamicCode", loCodeParms);


                DateTime ltNow = (DateTime)loResult;
                MessageBox.Show("Method Call Result:\r\n\r\n" + loResult.ToString(), "Compiler Demo", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception loError)
            {
                MessageBox.Show(loError.Message, "Compiler Demo", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }

            loRemote = null;
            AppDomain.Unload(loAppDomain);
            loAppDomain = null;
            File.Delete("AFM_DynamicCode.dll");

        }
    }
}