using C3PO.ViewModel;
using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Management.Automation.Language;
using System.Security.Policy;
using System.Text;
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

        public bool StartScan()
        {
            //// Declaring and configuring process-running object
            //var p = new Process()
            //{
            //    StartInfo = new ProcessStartInfo()
            //    {
            //        FileName = "C:\\dev\\CS-425-Team-20\\jason-mills\\Scanning\\x64\\Debug\\Scanning.exe"
            //    }
            //};

            //// Start process
            //p.Start();
            //p.WaitForExit();
            _startTime = DateTime.Now;
            return true;
        }

        public bool StartReconstruction()
        {
            //// Declaring and configuring process-running object
            //var p = new Process()
            //{
            //    StartInfo = new ProcessStartInfo()
            //    {
            //        FileName = "C:\\dev\\CS-425-Team-20\\jason-mills\\reconstruct.exe",
            //        Arguments = "--dir=C:\\dev\\CS-425-Team-20\\jason-mills\\Scanning\\Scanning\\src\\scans\\ --prefix=bun --out=C:\\Users\\froil\\Downloads\\bunny\\data\\out.ply"
            //    }
            //};
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\Users\\froil\\source\\repos\\PointCloudReconstruction\\Debug\\reconstruct.exe",
                    Arguments = "--dir=C:\\Users\\froil\\Downloads\\bunny\\data --prefix=bun --out=C:\\Users\\froil\\Downloads\\bunny\\data\\out.ply"
                }
            };

            // Start process
            p.Start();
            p.WaitForExit();

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
