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
        private ObservableCollection<string> _outputFormats = new ObservableCollection<string> { ".stl", ".obj", ".ply" };
        public ObservableCollection<string> OutputFormats
        {
            get
            {
                return _outputFormats;
            }
        }
        public string OutputFormat
        {
            get
            {
                return settings.outputFormat;
            }
            set
            {
                settings.outputFormat = value;
                OnPropertyChanged(nameof(OutputFormat));
            }
        }
        private ObservableCollection<string> _algoOpts = new ObservableCollection<string> { "point-to-plane", "point-to-point", "multiway" };
        public ObservableCollection<string> AlgoOpts
        {
            get
            {
                return _algoOpts;
            }
        }
        public string SelectedAlgo
        {
            get
            {
                return settings.algo;
            }
            set
            {
                settings.algo = value;
                OnPropertyChanged(nameof(SelectedAlgo));
            }
        }
        private ObservableCollection<string> _noOfPartitionsOpt = new ObservableCollection<string> { "15", "30", "45", "90" };
        public ObservableCollection<string> NoOfPartitionsOpt
        {
            get
            {
                return _noOfPartitionsOpt;
            }
        }
        public int TurnRadius
        {
            get
            {
                return settings.turnRadius;
            }
            set
            {
                settings.turnRadius = (int)value;
                UpdatePartitionIndex();
                OnPropertyChanged(nameof(TurnRadius));
                OnPropertyChanged(nameof(RegOrder));
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
        public int ScansPerAngle
        {
            get
            {
                return settings.scansPerAngle;
            }
            set
            {
                settings.scansPerAngle = value;
                OnPropertyChanged(nameof(ScansPerAngle));
            }
        }
        public SettingsParser settings;
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
        private bool _autoOut;
        public bool AutoOut
        {
            get
            {
                return _autoOut;
            }
            set
            {
                _autoOut = value;
                UpdateOutputName();
                OnPropertyChanged(nameof(AutoOut));
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
            set
            {
                settings.outPrefix = value;
                OnPropertyChanged(nameof(OPrefix));
            }
        }
        public string RegOrder
        {
            get
            {
                settings.regOrder = settings.GenDefaultRegOrder();
                return settings.regOrder;
            }
            set
            {
                settings.regOrder = value;
                OnPropertyChanged(nameof(RegOrder));
            }
        }
        public double VoxelMult
        {
            get
            {
                return settings.voxelMult;
            }
            set
            {
                settings.voxelMult = value;
                OnPropertyChanged(nameof(VoxelMult));
            }
        }
        public int IcpIters
        {
            get
            {
                return settings.icpIters;
            }
            set
            {
                settings.icpIters = value;
                OnPropertyChanged(nameof(IcpIters));
            }
        }
        private string[] _inputFormats = new string[] { ".xyz", ".ply", ".xyzn", ".xyzrgb", "pts", "pcd" };
        public string[] InputFormats
        {
            get
            {
                return _inputFormats;
            }
        }
        public string InputFormat
        {
            get
            {
                return settings.inputFormat;
            }
            set
            {
                settings.inputFormat = value;
                OnPropertyChanged(nameof(InputFormat));
            }
        }
        private bool _genQR;
        public bool GenQR
        {
            get
            {
                return settings.genQr;
            }
            set
            {
                settings.genQr = value;
                OnPropertyChanged(nameof(GenQR));
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
            _autoOut = false;
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
                if (par.Equals(TurnRadius.ToString())) {
                    SelectionPartitionIndex = counter;
                    break;
                }
                counter++;
            }
        }

        public void UpdateOutputName()
        {
            if(_autoOut == false)
            {
                OPrefix = "final";
            }
            else
            {
                string date = DateTime.Now.ToString("dd-MM-yyyy_HH-mm-ss");
                OPrefix = date;
            }
        }
    }
}
