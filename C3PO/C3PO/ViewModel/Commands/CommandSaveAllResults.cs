using Microsoft.PowerShell.Commands;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;

namespace C3PO.ViewModel.Commands
{
    public class CommandSaveAllResults : CommandBase
    {
        private string _prefix;
        public CommandSaveAllResults()
        {
            _prefix = "out";
        }

        public override void Execute(object? parameter)
        {
            // Configure folder dialogue
            FolderBrowserDialog fbd = new FolderBrowserDialog();

            // Display folder dialogue
            DialogResult result = fbd.ShowDialog();

            // Test if user properly selected folder
            if(result != DialogResult.OK)
            {
                return;
            }

            // Save all scans
            string path = System.IO.Directory.GetCurrentDirectory();
            string destFolder = fbd.SelectedPath;
            foreach (string pathFile in Directory.GetFiles(path + "\\output"))
            {
                string destFile = pathFile.Substring(pathFile.LastIndexOf("\\"));
                File.Move(pathFile, destFolder + destFile);
            }
        }
    }
}
