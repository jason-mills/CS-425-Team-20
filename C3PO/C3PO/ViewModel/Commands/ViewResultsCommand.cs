﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class ViewResultsCommand : CommandBase
    {
        private string _prefix;
        private readonly int SW_SHOWMAXIMIZED = 3;

        [DllImport("user32.dll", SetLastError =true)]
        static extern bool ShowWindow(IntPtr hWnd, int nComdShow);

        [DllImport("user32.dll", EntryPoint = "FindWindow", SetLastError = true)]
        static extern IntPtr FindWindowByCaption(IntPtr ZeroOnly, string lpWindowName);

        public ViewResultsCommand()
        {
            _prefix = "out";
        }

        public override void Execute(object? parameter)
        {
            // Get file to display
            string fName = "Final.ply";
            if (parameter != null)
            {
                fName = FileSelect((string)parameter);
            }
            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "python.exe",
                    Arguments = path + "\\bin\\render.py " + path + "\\output\\" + fName,
                    UseShellExecute = false,
                    CreateNoWindow = true
                }
            };

            // Start process
            p.Start();

            IntPtr hWnd= IntPtr.Zero;
            while(!p.HasExited && hWnd.Equals(IntPtr.Zero))
            {
                hWnd = FindWindowByCaption(IntPtr.Zero, "Open3D");
            }

            ShowWindow(hWnd, SW_SHOWMAXIMIZED);

            p.WaitForExit();
        }

        public string FileSelect(string sel)
        {
            if(sel.Equals("0"))
            {
                return "Final.ply";
            }
            return _prefix + Int32.Parse(sel) + ".ply";
        }
    }
}
