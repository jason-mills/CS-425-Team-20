using C3PO.Model;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace C3PO.ViewModel.Commands
{
    internal class ViewResultsCommand : CommandBase
    {
        private readonly SettingsParser settings;
        private readonly int SW_SHOWMAXIMIZED = 3;
        private string pythonPath;

        [DllImport("user32.dll", SetLastError =true)]
        static extern bool ShowWindow(IntPtr hWnd, int nComdShow);

        [DllImport("user32.dll", EntryPoint = "FindWindow", SetLastError = true)]
        static extern IntPtr FindWindowByCaption(IntPtr ZeroOnly, string lpWindowName);

        public ViewResultsCommand(SettingsParser settings)
        {
            this.settings = settings;
            pythonPath = GetPythonPath();
        }

        public override void Execute(object? parameter)
        {
            // Get file to display
            string fName = "final.xyz";
            if (parameter != null)
            {
                fName = FileSelect((string)parameter);
            }

            // Declaring and configuring process-running object
            string fPath = settings.dir + "\\" + fName;
            string exeDir = System.IO.Directory.GetCurrentDirectory() + "\\bin\\RenderDriver.py";

            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = pythonPath + "python.exe",
                    Arguments = exeDir + " \"" + fPath + "\"",
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            // Start process
            p.Start();

            IntPtr hWnd= IntPtr.Zero;
            //while(!p.HasExited && hWnd.Equals(IntPtr.Zero))
            //{
            //    hWnd = FindWindowByCaption(IntPtr.Zero, "Open3D");
            //}

            //ShowWindow(hWnd, SW_SHOWMAXIMIZED);

            //p.WaitForExit();

            //string error = p.StandardError.ReadToEnd();
            //string output = p.StandardOutput.ReadToEnd();
        }

        public string FileSelect(string sel)
        {
            if(sel.Equals("0"))
            {
                return settings.outPrefix + settings.outputFormat;
            }
            return sel + settings.inputFormat;
        }

        public string GetPythonPath()
        {
            var paths = Environment.GetEnvironmentVariable("path")?.Split(';');
            string pyPath = "";

            foreach(string p in paths)
            {
                string[] pSplit = p.ToLower().Split('\\');
                if (pSplit[pSplit.Length - 1].Contains("python"))
                {
                    pyPath = p;
                }
            }

            if (pyPath.Length > 0 && pyPath[pyPath.Length - 1] != '\\')
            {
                pyPath += "\\";
            }

            return pyPath;
        }
    }
}
