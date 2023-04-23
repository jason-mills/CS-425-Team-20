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
                    Arguments = ".\\bin\\icp.py " + sourcePath + " " + sourcePrefix + " .xyz .\\output\\final " + order + " True"
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
            string dir = settingsParser.dir;
            string prefixPath = dir + "\\" + settingsParser.outPrefix;
            string iPath = prefixPath + ".xyz";
            string oPath = prefixPath + ".png";
            string exeDir = System.IO.Directory.GetCurrentDirectory() + "\\bin\\pc2png.py";

            // Set up process
            Process p = new Process()
            {
                StartInfo =
                {
                    FileName = "python",
                    Arguments = $"{exeDir} --file {iPath} --out {oPath}"
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
    }
}
