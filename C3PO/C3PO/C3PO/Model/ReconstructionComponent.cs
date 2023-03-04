using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using C3PO.Model;

namespace C3PO.Model
{
    internal class ReconstructionComponent : IComponent
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
                    FileName = "C:\\Users\\froil\\source\\repos\\C3PO\\C3PO\\Resources\\bin\\reconstruct.exe"
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
