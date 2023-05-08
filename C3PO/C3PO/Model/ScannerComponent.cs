/* ScannerComponent.cs:
 *  The model class responsible for handling interactions with the
 *  scanning component of the C3PO system.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Text;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace C3PO.Model
{
    internal class ScannerComponent : IComponent
    {
        private CancellationToken ct;
        private SettingsParser settings;

        public ScannerComponent()
        {
            ct = new CancellationToken();
            settings = new SettingsParser();
        }
        public ScannerComponent(SettingsParser settings)
        {
            ct = new CancellationToken();
            this.settings = settings;
        }
        public ScannerComponent(CancellationToken ct, SettingsParser settings)
        {
            this.ct = ct;
            this.settings = settings;
        }

        public bool StartOp()
        {
            // Create output directory if doesn't exist
            if(!System.IO.Directory.Exists(settings.dir))
            {
                System.IO.Directory.CreateDirectory(settings.dir);
            }

            // Configure process for performing a scan
            CopyJsonToSub();
            string path = System.IO.Directory.GetCurrentDirectory();
            string args = $"{settings.scansPerAngle} {settings.turnRadius} {settings.dir}\\ {settings.dir}\\";
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = path + "\\bin\\Scanning.exe",
                    Arguments = args
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

            return true;
        }

        public bool StopOp()
        {
            throw new NotImplementedException();
        }

        public List<string> GetScansFromDir()
        {
            List<string> scans = new List<string>();
            string dir = settings.dir;
            string prefix = settings.inPrefix;
            string postfix = settings.outPrefix;

            DirectoryInfo directoryInfo = new DirectoryInfo(dir);
            foreach(FileInfo file in directoryInfo.GetFiles()) {
                string fpath = file.FullName;
                string fname = fpath.Substring(fpath.LastIndexOf("\\") + 1);

                if(Regex.IsMatch(fname, @$"^{prefix}[0-9]*{settings.inputFormat}"))
                {
                    scans.Add(fname);
                }
            }

            return scans;
        }

        public bool CheckConnection()
        {
            // Variables
            string binPath = System.IO.Directory.GetCurrentDirectory() + "\\bin";

            // Check arduino connection
            if(!System.IO.File.Exists(binPath + "\\check_arduino.exe"))
            {
                return false;
            }

            Process p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = binPath + "\\check_arduino.exe",
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            p.Start();
            p.WaitForExit();

            if(p.ExitCode != 0)
            {
                return false;
            }

            // Check camera connection
            if (!System.IO.File.Exists(binPath + "\\check_camera.exe"))
            {
                return false;
            }

            p.StartInfo.FileName = binPath + "\\check_camera.exe";
            p.Start();
            p.WaitForExit();

            if(p.ExitCode != 0 )
            {
                return false;
            }

            return true;
        }

        private void CopyJsonToSub()
        {
            string path = System.IO.Directory.GetCurrentDirectory().ToString();
            System.IO.File.Copy(path + "\\resources\\test.json", $"{settings.dir}\\test.json");
        }
    }
}
