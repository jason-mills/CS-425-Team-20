using Microsoft.PowerShell.Commands;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using C3PO.Model;

namespace C3PO.ViewModel.Commands
{
    public class CommandSaveAllResults : CommandBase
    {
        private SettingsParser settings;

        public CommandSaveAllResults(SettingsParser settings)
        {
            this.settings = settings;
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
            string path = settings.dir;
            string destFolder = fbd.SelectedPath;
            foreach (string pathFile in Directory.GetFiles(path))
            {
                string destFile = pathFile.Substring(pathFile.LastIndexOf("\\"));
                try
                {
                    File.Move(pathFile, destFolder + destFile);
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
            }
        }
    }
}
