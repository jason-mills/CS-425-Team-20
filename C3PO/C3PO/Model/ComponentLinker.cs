using C3PO.ViewModel;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Management.Automation.Language;
using System.Security.Policy;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;

namespace C3PO.Model
{
    class ComponentLinker
    {
        private readonly IComponent _reconstructComp;
        private readonly IComponent _scanComp;
        private DateTime _startTime;
        private string _endTimeStr;
        public string EndTimeStr
        {
            get
            {
                return _endTimeStr;
            }
            private set
            {
                _endTimeStr = value;
            }
        }

        /* Constructors */
        public ComponentLinker()
        {
            _reconstructComp = new ReconstructionComponent();
            _scanComp = new ScannerComponent();

            _startTime = new DateTime();
            _endTimeStr = "";
        }

        public bool StartScan(CancellationToken ct)
        {
            _startTime = DateTime.Now;
            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = path + "\\bin\\Scanning.exe"
                }
            };

            //// Start process
            p.Start();
            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "python.exe",
                    Arguments = path + "\\bin\\xyzToPly.py " + path + "\\output " + " temp " + path + "\\output out"
                }
            };

            //// Start process
            p.Start();

            while (!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            return true;
        }

        public bool StartReconstruction(CancellationToken ct)
        {
            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "\\bin\\reconstruct.exe",
                    Arguments = "--dir=" + path + "\\output --prefix=out --out=" + path + "\\output\\final.ply"
                }
            };
            //var p = new Process()
            //{
            //    StartInfo = new ProcessStartInfo()
            //    {
            //        FileName = "C:\\Users\\froil\\source\\repos\\PointCloudReconstruction\\Debug\\reconstruct.exe",
            //        Arguments = "--dir=C:\\Users\\froil\\Downloads\\bunny\\data --prefix=bun --out=C:\\Users\\froil\\Downloads\\bunny\\data\\out.ply"
            //    }
            //};

            // Start process
            p.Start();
            //p.WaitForExit();
            while(!p.HasExited)
            {
                if (ct.IsCancellationRequested)
                {
                    p.Kill();
                    return false;
                }
            }

            return true;
        }

        public bool Finish()
        {
            _endTimeStr = (DateTime.Now - _startTime).ToString();
            return true;
        }

        public String GetTimeSpanned()
        {
            return _endTimeStr;
        }
    }
}
