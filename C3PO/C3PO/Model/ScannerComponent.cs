﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
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
            // Configure process for performing a scan
            string path = System.IO.Directory.GetCurrentDirectory();

            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = path + "\\bin\\Scanning.exe",
                    Arguments = "1 15"
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

        public List<string> GetScansFromDir()
        {
            List<string> scans = new List<string>();
            string dir = settings.dir;
            string prefix = settings.inPrefix;
            string postfix = settings.outPrefix;

            foreach(string fpath in System.IO.Directory.GetFiles(dir)) {
                string fname = fpath.Substring(fpath.LastIndexOf("\\") + 1);

                if(Regex.IsMatch(fname, @$"^{prefix}[0-9]*.xyz"))
                {
                    scans.Add(fname);
                }
            }

            return scans;
        }
    }
}
