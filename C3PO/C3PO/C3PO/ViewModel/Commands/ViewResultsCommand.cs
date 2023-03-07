using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class ViewResultsCommand : CommandBase
    {
        public ViewResultsCommand() { }

        public override void Execute(object? parameter)
        {
            // Declaring and configuring process-running object
            string path = System.IO.Directory.GetCurrentDirectory();
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "python.exe ",
                    Arguments = path + "\\render.py " + path + "\\Final.ply",
                }
            };

            // Start process
            p.Start();
            p.WaitForExit();
        }
    }
}
