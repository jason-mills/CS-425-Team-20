using C3PO.ViewModel;
using System;
using System.Collections.Generic;
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
        private IComponent _reconstructComp;
        private IComponent _scanComp;

        /* Constructors */
        public ComponentLinker()
        {
            _reconstructComp = new ReconstructionComponent();
            _scanComp = new ScannerComponent();
        }

        public bool StartScan()
        {
            // Declaring and configuring process-running object
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\dev\\CS-425-Team-20\\jason-mills\\Scanning\\x64\\Debug\\Scanning.exe"
                }
            };

            // Start process
            p.Start();
            p.WaitForExit();

            return true;
        }

        public bool StartReconstruction()
        {
            // Declaring and configuring process-running object
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\dev\\CS-425-Team-20\\jason-mills\\reconstruct.exe",
                    Arguments = "--dir=C:\\dev\\CS-425-Team-20\\jason-mills\\Scanning\\Scanning\\src\\scans\\ --prefix=bun --out=C:\\Users\\froil\\Downloads\\bunny\\data\\out.ply"
                }
            };

            // Start process
            p.Start();
            p.WaitForExit();

            return true;
        }

        public bool Finish()
        {
            return true;
        }
    }
}
