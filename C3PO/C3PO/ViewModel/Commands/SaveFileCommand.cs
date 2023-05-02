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
using C3PO.Model;
using System.CodeDom;

namespace C3PO.ViewModel.Commands
{
    internal class SaveFileCommand : CommandBase
    {
        private SettingsParser settings;

        public SaveFileCommand(SettingsParser settings)
        {
            this.settings = settings;
        }

        public override void Execute(object? parameter)
        {
            // Configure save file dialogue for saving a file
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog();
            saveFileDialog.FileName = "c3po_results";
            saveFileDialog.DefaultExt = $"{settings.outputFormat}";
            saveFileDialog.Filter = $"Meshes|*{settings.outputFormat}";

            // Display file dialogue to user
            bool? result = saveFileDialog.ShowDialog();

            // Test if user successfully selected a path to save to
            if(result == true)
            {
                string filename = saveFileDialog.FileName;
                string ogFilePath = $"{settings.dir}\\{settings.outPrefix}{settings.outputFormat}";
                if (File.Exists(ogFilePath))
                {
                    File.Move(ogFilePath, filename);
                }
            }
        }
    }
}
