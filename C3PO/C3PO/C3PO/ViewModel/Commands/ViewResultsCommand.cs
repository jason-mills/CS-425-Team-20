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
            var p = new Process()
            {
                StartInfo = new ProcessStartInfo()
                {
                    FileName = "C:\\dev\\vcpkg\\packages\\python3_x64-windows\\tools\\python3\\python.exe ",
                    Arguments = "C:\\Users\\froil\\source\\repos\\CS-425-Team-20\\jason-mills\\Open3D\\render.py " + @"""C:\\Users\\froil\\source\\repos\\CS-425-Team-20\\jason-mills\\Open3D\\PLY Files\\bowl_1_0.ply""",
                }
            };

            // Start process
            p.Start();
            p.WaitForExit();
        }
    }
}
