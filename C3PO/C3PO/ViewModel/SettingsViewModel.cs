/* SettingsViewModel.cs:
 *  ViewModel for the SettingsWindow.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 02, 2023
 */

using C3PO.Model;
using C3PO.Stores;
using C3PO.ViewModel.Commands;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Management.Automation.Language;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace C3PO.ViewModel
{
    internal class SettingsViewModel : ViewModelBase
    {
        public ObservableCollection<string> UsbNames
        {
            get
            {
                return new ObservableCollection<string>((new USBDevices()).GetNames());
            }
        }
        private ObservableCollection<string> _noOfPartitionsOpt = new ObservableCollection<string> { "30", "45", "90" };
        public ObservableCollection<string> NoOfPartitionsOpt
        {
            get
            {
                return _noOfPartitionsOpt;
            }
        }
        private int _partitions;
        public int Partitions
        {
            get
            {
                return _partitions;
            }
            set
            {
                _partitions = (int)value;
                settings.turnRadius = (int)value;
                UpdatePartitionIndex();
                OnPropertyChanged(nameof(Partitions));
            }
        }
        private int _selectionPartitionIndex;
        public int SelectionPartitionIndex
        {
            get
            {
                return _selectionPartitionIndex;
            }
            set
            {
                _selectionPartitionIndex = value;
                OnPropertyChanged(nameof(SelectionPartitionIndex));
            }
        }
        private int _scansPerAngle;
        public int ScansPerAngle
        {
            get
            {
                return _scansPerAngle;
            }
            set
            {
                _scansPerAngle = value;
                OnPropertyChanged(nameof(ScansPerAngle));
            }
        }
        public SettingsParser settings;
        //public SettingsParser Settings
        //{
        //    get
        //    {
        //        return _settings;
        //    }
        //}
        //private bool _interMode;
        public bool InterMode
        {
            get
            {
                return settings.interMode;
            }
            set
            {
                settings.interMode = value;
            }
        }
        public string SPrefix
        {
            get
            {
                return settings.inPrefix;
            }
        }
        public string OPrefix
        {
            get
            {
                return settings.outPrefix;
            }
        }

        public ICommand BackBtnCommand { get; }
        public ICommand ImportSettingsCommand { get; }
        public ICommand ExportSettingsCommand { get; }
        public ICommand InputRotationsCommand { get; }
        public ICommand InputRegOrder { get; }
        public ICommand UpdateSPrefixCommand { get; }
        public ICommand UpdateOPrefixCommand { get; }

        public SettingsViewModel(NavigationStore ns)
        {
            _partitions = Int32.Parse(_noOfPartitionsOpt[_selectionPartitionIndex]);
            _scansPerAngle = 1;

            settings = new SettingsParser();
            settings.ImportSettingsFile();
            UpdatePartitionIndex();


            BackBtnCommand = new SettingsBackButtonCommand(ns);
            ImportSettingsCommand = new CommandImportSettings(settings);
            ExportSettingsCommand = new CommandExportSettings(settings);
            InputRotationsCommand = new RotationsChangedCommand(this);
            InputRegOrder = new CommandUpdateRegOrder(this);
            UpdateSPrefixCommand = new CommandUpdateSourcePrefix(settings);
            UpdateOPrefixCommand = new CommandUpdateOutPrefix(settings);
        }

        public void UpdatePartitionIndex()
        {
            int counter = 0;
            foreach(var par in NoOfPartitionsOpt)
            {
                if (par.Equals(Partitions.ToString())) {
                    SelectionPartitionIndex = counter;
                    break;
                }
                counter++;
            }
        }
    }
}
