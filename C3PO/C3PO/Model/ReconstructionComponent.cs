using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using C3PO.Model;

namespace C3PO.Model
{
    internal class ReconstructionComponent : IComponent
    {
        private readonly CancellationToken ct;
        private readonly SettingsParser settingsParser;

        public ReconstructionComponent()
        {
            ct = new CancellationToken();
            settingsParser = new SettingsParser();
        }
        public ReconstructionComponent(CancellationToken ct, SettingsParser settings)
        {
            this.ct = ct;
            this.settingsParser = settings;
        }

        public bool StartOp()
        {
            // Create output directory if doesn't exist
            if (!System.IO.Directory.Exists(settingsParser.dir))
            {
                System.IO.Directory.CreateDirectory(settingsParser.dir);
            }

            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            string sourcePath = settingsParser.dir;
            string sourcePrefix = settingsParser.inPrefix;
            // string order = GetICPOrder();
            string order = settingsParser.regOrder;
            string interMode = settingsParser.interMode.ToString();
            string pyPath = GetPythonPath();
            // QUESTION FOR JASON: Can file extension be xyz or has to be .xyz?
            string args = $"--input_directory_path={sourcePath} " +
                $"--input_file_extension={settingsParser.inputFormat} " +
                $"--output_file_extension={settingsParser.outputFormat} " +
                $"--output_directory_path={settingsParser.dir} " +
                $"--output_file_base_name={settingsParser.outPrefix} " +
                $"--is_user_scan=True " +
                //$"--is_user_scan={settingsParser.isUserScan} " +
                $"--run_interactive_mode={interMode} ";
            if(settingsParser.isUserScan == true)
            {
                args += $"--file_order={order} " + $"--input_file_base_name={sourcePrefix} ";
            }
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = pyPath + "python.exe",
                    Arguments = ".\\bin\\EditorDriver.py " + args,
                    UseShellExecute = false,
                    CreateNoWindow = true,
                }
            };

            // Start process
            bool result;
            try
            {
                result = p.Start();
            }
            catch
            {
                return false;
            }

            // Test for finished process or cancellation
            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            bool pc2ImageResult = PC2Image();

            return result && pc2ImageResult;
        }

        public bool StopOp()
        {
            throw new NotImplementedException();
        }

        public string GetICPOrder()
        {
            string order = "";
            int turnRadius = settingsParser.turnRadius;

            for (int i = 0, rad = 0; rad < 360; i++, rad += turnRadius)
            {
                order += i.ToString() + ",";
            }

            order = order.Remove(order.Length - 1);
            return order;
        }

        public bool PC2Image()
        {
            // Get path variables
            string pyPath = GetPythonPath();
            string dir = settingsParser.dir;
            string prefixPath = dir + "\\" + settingsParser.outPrefix;
            string iPath = settingsParser.outPrefix + settingsParser.inputFormat;
            string oPath = prefixPath + ".png";
            string exeDir = System.IO.Directory.GetCurrentDirectory() + "\\bin\\pc2png.py";

            // Set up process
            Process p = new Process()
            {
                StartInfo =
                {
                    FileName = pyPath + "python.exe",
                    Arguments = $"{exeDir} --file {iPath} --out {oPath}",
                    UseShellExecute = true
                }
            };

            // Start scanning process
            bool result;
            try
            {
                result = p.Start();
            }
            catch
            {
                return false;
            }

            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            return result;
        }

        public bool CheckConnection()
        {
            return true;
        }

        public string GetPythonPath()
        {
            var paths = Environment.GetEnvironmentVariable("path")?.Split(';');
            string pyPath = "";

            foreach (string p in paths)
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
