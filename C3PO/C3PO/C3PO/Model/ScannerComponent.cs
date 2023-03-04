using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.Model
{
    internal class ScannerComponent : IComponent
    {
        public Task<int> StartOpAsync()
        {
            // Task to return upon function completion
            var tcs = new TaskCompletionSource<int>();

            // Declaring and configuring process-running object
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\dev\\CS-425-Team-20\\jason-mills\\Scanning\\x64\\Debug\\Scanning.exe"
                }
            };

            // Set function to perform upon completion of process
            p.Exited += (sender, args) =>
            {
                tcs.SetResult(p.ExitCode);
                p.Dispose();
            };

            // Start process
            p.Start();

            return tcs.Task;
        }

        public Task<int> StopOpAsync()
        {
            throw new NotImplementedException();
        }
    }
}
