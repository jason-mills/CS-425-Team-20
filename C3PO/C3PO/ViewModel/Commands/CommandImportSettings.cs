using C3PO.Model;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace C3PO.ViewModel.Commands
{
    internal class CommandImportSettings : CommandBase
    {
        private readonly SettingsParser parser;
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

        public CommandImportSettings(SettingsParser parser, string? fpath = null)
        {
            this.parser = parser;

            _importFPath = fpath is null ? 
                System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.json" 
                : fpath;
        }

        public override void Execute(object? parameter)
        {
            OpenFileDialog ofd = new OpenFileDialog();

            bool? result = ofd.ShowDialog();

            if(result == true )
            {
                string filename = ofd.FileName;
                parser.ImportSettingsFile(filename);
            }
        }
    }
}
