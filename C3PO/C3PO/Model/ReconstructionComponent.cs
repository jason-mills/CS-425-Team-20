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
            string order = GetICPOrder();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "python",
                    Arguments = ".\\bin\\icp.py " + sourcePath + " " + sourcePrefix + " .xyz .\\output\\final " + "\"" + order + "\"" + " True"
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

        public string GetICPOrder()
        {
            string order = "";
            int turnRadius = settingsParser.turnRadius;

            for (int i = 0, rad = 0; rad < 360; i++, rad += turnRadius)
            {
                order += i.ToString() + " ";
            }

            order = order.Remove(order.Length - 1);
            return order;
        }
    }
}
