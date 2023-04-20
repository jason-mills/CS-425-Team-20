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
        private CancellationToken ct;
        private SettingsParser settingsParser;

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
            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            string sourcePath = settingsParser.dir;
            string sourcePrefix = settingsParser.inPrefix;
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = path + "\\bin\\reconstruct.exe",
                    Arguments = "--dir=" + sourcePath + " --prefix=" + sourcePrefix + " --out=" + sourcePath + "\\final.ply"
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

        public bool StopOp()
        {
            throw new NotImplementedException();
        }
    }
}
