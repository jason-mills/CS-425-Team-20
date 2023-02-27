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
            // Call hardware software to start scan
            return true;
        }

        public bool StartReconstruction()
        {
            // Declaring and configuring process-running object
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\Users\\froil\\source\\repos\\C3PO\\C3PO\\Resources\\bin\\reconstruct.exe",
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
            return true;
        }
    }
}
