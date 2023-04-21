using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace C3PO.Model
{
    internal class ScannerComponent : IComponent
    {
        CancellationToken ct;
        SettingsParser settings;

        public ScannerComponent()
        {
            ct = new CancellationToken();
            settings = new SettingsParser();
        }
        public ScannerComponent(CancellationToken ct, SettingsParser settings)
        {
            this.ct = ct;
            this.settings = settings;
        }

        public bool StartOp()
        {
            // Configure process for performing a scan
            string path = System.IO.Directory.GetCurrentDirectory();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = path + "\\bin\\Scanning.exe",
                    Arguments = "1 30"
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

            // Wait for scanning to finish
            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            // Test if process didn't run correctly
            if (!result)
            {
                return false;
            }

            // Configure process for generating PLY files
            /*p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "python.exe",
                    Arguments = path + "\\bin\\xyzToPly.py " + path + "\\output " + " temp " + path + "\\output out"
                }
            };

            // Start process to generate PLY files
            try
            {
                result = p.Start();
            }
            catch
            {
                return false;
            }

            // Test if process didn't run correctly
            if (!result)
            {
                return false;
            }

            // Wait for process to finish
            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }*/

            return true;
        }

        public bool StopOp()
        {
            throw new NotImplementedException();
        }
    }
}
