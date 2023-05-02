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

        public CommandExportSettings(SettingsParser parser, string? fpath = null)
        {
            this.parser = parser;
            _importFPath = fpath is null ?
                System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.xaml"
                : fpath;
        }
        public override void Execute(object? parameter)
        {
            // Configure save file dialogue for saving a file
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog();
            saveFileDialog.FileName = "Settings";
            saveFileDialog.DefaultExt = ".xaml";
            saveFileDialog.Filter = "Extensible Markup Language|*.xaml";

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
