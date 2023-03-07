using C3PO.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    class CommandExportSettings : CommandBase
    {
        private SettingsViewModel svm;
        private SettingsParser parser;
        private string _importFPath;
        public string ImportFPath
        {
            get
            {
                return _importFPath;
            }
            set
            {
                _importFPath = value;
            }
        }

        public CommandExportSettings(SettingsViewModel svm, string? fpath = null)
        {
            this.svm = svm;
            parser = new SettingsParser(svm);
            _importFPath = fpath is null ?
                System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.json"
                : fpath;
        }
        public override void Execute(object? parameter)
        {
            // Configure save file dialogue for saving a file
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog();
            saveFileDialog.FileName = "Settings";
            saveFileDialog.DefaultExt = ".xml";
            saveFileDialog.Filter = "Extensible Markup Language|*.xml";

            // Display file dialogue to user
            bool? result = saveFileDialog.ShowDialog();

            // Test if user successfully selected a path to save to
            if (result == true)
            {
                string filename = saveFileDialog.FileName;
                parser.ExportSettingsFile(filename, _importFPath);
            }
        }
    }
}
