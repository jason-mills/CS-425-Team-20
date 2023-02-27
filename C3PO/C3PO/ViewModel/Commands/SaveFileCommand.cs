/* SaveFileCommand.cs
 *  Contains the class responsible for housing command
 *  functionality for saving scanned files.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 02, 2023
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Win32;
using System.IO;

namespace C3PO.ViewModel.Commands
{
    internal class SaveFileCommand : CommandBase
    {
        public SaveFileCommand() { }

        public override void Execute(object? parameter)
        {
            // Configure save file dialogue for saving a file
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog();
            saveFileDialog.FileName = "c3po_results";
            saveFileDialog.DefaultExt = ".stl";
            saveFileDialog.Filter = "Standard Tessellation Language|*.stl";

            // Display file dialogue to user
            bool? result = saveFileDialog.ShowDialog();

            // Test if user successfully selected a path to save to
            if(result == true)
            {
                string filename = saveFileDialog.FileName;
                string ogFilePath = AppContext.BaseDirectory + "\\input.stl";
                if (File.Exists(ogFilePath))
                {
                    File.Move(ogFilePath, filename);
                }
            }
        }
    }
}
