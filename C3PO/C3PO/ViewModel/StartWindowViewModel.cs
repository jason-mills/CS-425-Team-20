/* StartWindowViewModel.cs:
 *  The class that is the viewmodel for the StartWindow
 *  view. Its function is to provide the information
 *  for the view to display and how it should operate.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *  date: February 2, 2023
 */

using C3PO.Model;
using C3PO.Stores;
using C3PO.ViewModel.Commands;
using Microsoft.VisualStudio.PlatformUI;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Navigation;

namespace C3PO.ViewModel
{
    public enum ScanStates
    {
        Start,
        Scan,
        Reconstruct,
        Finish,
        Display,
        Empty,
        GuiTutorial,
        HardwareTutorial
    }

    public class StartWindowViewModel : ViewModelBase
    {
        /*
         * Attributes
         */
        private SettingsViewModel settingsVM;
        private MetadataParser metadataParser;
        private ComponentLinker _componentLinker;
        public ComponentLinker ComponentLinker
        {
            get
            {
                return _componentLinker;
            }
        }
        private ObservableCollection<string> _scannerButtons;
        public ObservableCollection<string> ScannerButtons { 
            get
            {
                return _scannerButtons; 
            }
            protected set
            { 
                _scannerButtons = value;
                OnPropertyChanged(nameof(ScannerButtons));
            } 
        }
        private NavigationStore _navigateStore;
        public NavigationStore NavigateStore
        {
            get
            {
                return _navigateStore;
            }
        }
        private UserControl _resultsUC;
        public UserControl ResultsUC
        {
            get
            {
                return _resultsUC;
            }
            set
            {
                _resultsUC = value;
                OnPropertyChanged(nameof(ResultsUC));
            }
        }
        public ObservableCollection<string> UsbNames
        {
            get
            {
                return new ObservableCollection<string>((new USBDevices()).GetNames());
            }
        }
        private string _resultPanelHeader;
        public string ResultPanelHeader
        {
            get
            {
                return _resultPanelHeader;
            }
            set
            {
                _resultPanelHeader = value;
                OnPropertyChanged(nameof(ResultPanelHeader));
            }
        }
        private ScanStates _linkerState;
        public ScanStates LinkerState
        {
            get
            {
                return _linkerState;
            }
            set
            {
                _linkerState = value;
                LinkerUpdate();
            }
        }
        public ObservableCollection<string> _rawResults;
        public ObservableCollection<string> RawResults
        {
            get
            {
                return _rawResults;
            }
            set
            {
                _rawResults = value;
            }
        }
        public ObservableCollection<ScanMetadata> Metadata { get; set; }
        private string _cloudImagePath;
        public string CloudImagePath
        {
            get
            {
                return _cloudImagePath;
            }
            set
            {
                _cloudImagePath = value;
                OnPropertyChanged(nameof(CloudImagePath));
            }
        }
        public string ConnectionStatus
        {
            get
            {
                return GetConnectionStatus();
            }
        }
        public string ConnectionColor
        {
            get
            {
                return GetConnectionColor();
            }
        }

        /*
         * Commands
         */
        public ICommand ScannerBtnCommand { get; }
        public ICommand SettingsBtnCommand { get; }
        public ICommand SaveBtnCommand { get; }
        public ICommand ViewResultsBtnCommand { get; }
        public ICommand GoToHomePageCommand { get; }
        public ICommand SaveAllResultsBtnCommand { get; }
        public ICommand SaveFinalToCloudCommand { get; }
        public ICommand SaveAllFilesToCloudCommand { get; }
        public ICommand GuiTutorialBtnCommand { get; }
        public ICommand BackResultsBtnCommand { get; }
        public ICommand HwTutorialBtnCommand { get; }

        /*
         * Constructors
         */
        public StartWindowViewModel(NavigationStore navigateStore)
        {
            _rawResults = new ObservableCollection<string>() { "1", "2", "3" };
            _navigateStore = navigateStore;
            settingsVM = new SettingsViewModel(navigateStore);
            _componentLinker = new ComponentLinker(settingsVM.settings);
            Metadata = new ObservableCollection<ScanMetadata>();
            _cloudImagePath = "";
            _linkerState = ScanStates.Empty;
            _resultsUC = new C3PO.View.ScanResultsEmpty();
            _resultsUC.DataContext = new ScanResultsEmptyViewModel();

            _scannerButtons = new ObservableCollection<string>();
            ScannerButtons.Add("Scan");
            ScannerButtons.Add("Load Scans");

            _resultPanelHeader = "No Scanned Content Available";

            ScannerBtnCommand = new ScannerButtonsClickedCommand(this, settingsVM.settings);
            SettingsBtnCommand = new SettingsBtnClickedCommand(NavigateStore, settingsVM);
            NavigateStore.PreviousViewModel = this;
            SaveBtnCommand = new SaveFileCommand();
            SaveAllResultsBtnCommand = new CommandSaveAllResults();
            ViewResultsBtnCommand = new ViewResultsCommand(settingsVM.settings);
            GoToHomePageCommand = new NavigateUriCommand("https://sites.google.com/nevada.unr.edu/team-20-c3po/home?authuser=1");
            SaveFinalToCloudCommand = new CommandUploadToCloud(settingsVM.settings, "final");
            SaveAllFilesToCloudCommand = new CommandUploadToCloud(settingsVM.settings, "all");
            GuiTutorialBtnCommand = new CommandUpdateResultsPanel(this);
            BackResultsBtnCommand = new CommandUpdateResultsPanel(this);
            HwTutorialBtnCommand = new CommandUpdateResultsPanel(this);
        }

        /*
         * Methods
         */

        public void ScanButtonsSet(ObservableCollection<string> btns)
        {
            this.ScannerButtons = btns;
        }

        public void LinkerUpdate()
        {
            if(_linkerState == ScanStates.Reconstruct)
            {
                ResultPanelHeader = "Reconstructing model... please wait...";
            }
            else if(_linkerState == ScanStates.Scan)
            {
                ResultPanelHeader = "Scanning... please wait...";
            }
            else if(_linkerState == ScanStates.Finish)
            {
                ResultPanelHeader = "Scan Results";
            }
            else if(_linkerState == ScanStates.Display)
            {
                ResultPanelHeader = "Displaying Model";
            }
            else if(_linkerState == ScanStates.Empty)
            {
                ResultPanelHeader = "No Scanned Content Available";
            }
            else if(_linkerState == ScanStates.GuiTutorial)
            {
                ResultPanelHeader = "GUI Tutorial";
            }
            else if(_linkerState == ScanStates.HardwareTutorial)
            {
                ResultPanelHeader = "Hardware Tutorial";
            }
        }

        public void StartScan(CancellationToken ct)
        {
            Application.Current.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Scan;
            }));
            bool result = _componentLinker.StartScan();
            
            if (result)
            {
                FinishScan();
            }
            else
            {
                LinkerState = ScanStates.Empty;
            }
        }

        public void FinishScan()
        {
            Application.Current?.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Reconstruct;
            }));
            bool result = _componentLinker.StartReconstruction();
            if (result)
            {
                FinishReconstruct();
            }
            else
            {
                LinkerState = ScanStates.Empty;
            }
        }

        public void FinishReconstruct()
        {
            _componentLinker.Finish();
            Application.Current.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Finish;
                ResultsUC = new C3PO.View.ScanResults();
                ObservableCollection<string> newBtns = new ObservableCollection<string>();
                newBtns.Add("Scan");
                newBtns.Add("Load Scans");
                ScanButtonsSet(newBtns);
                UpdateMetadata();
                UpdatePartitionList();
                UpdateCloudImagePath();
            }));
        }

        public void UpdateMetadata()
        {
            metadataParser.LoadMetadata(settingsVM.settings.dir);

            Metadata = new ObservableCollection<ScanMetadata>();
            Metadata.Add(new ScanMetadata("Time Spanned", _componentLinker.TimeSpanned.ToString(@"hh\:mm\:ss")));
            Metadata.Add(new ScanMetadata("Partitions", "0"));
            Metadata.Add(new ScanMetadata("Scans Taken", "0"));
            Metadata.Add(new ScanMetadata("Avg. Fitness Score", "0"));
            Metadata.Add(new ScanMetadata("Points Captured", "0"));
        }

        public void UpdatePartitionList()
        {
            ScannerComponent sc = new ScannerComponent(settingsVM.settings);
            List<int> orderedNums = new List<int>();
            List<string> orderedFiles = new List<string>();
            string prefix = settingsVM.settings.inPrefix;

            foreach(string fname in sc.GetScansFromDir())
            {
                int startIndex = fname.IndexOfAny("0123456789".ToCharArray());
                orderedNums.Add(
                    int.Parse(fname[startIndex..fname.LastIndexOf('.')])
                    );
            }

            orderedNums.Sort();

            foreach(int num in orderedNums)
            {
                orderedFiles.Add($"{prefix}{num}");
            }

            RawResults = new ObservableCollection<string>(orderedFiles);
        }

        public string UpdateCloudImagePath()
        {
            SettingsParser settings = settingsVM.settings;
            CloudImagePath = $"{settings.dir}\\{settings.outPrefix}.png";
            return _cloudImagePath;
        }

        public string GetConnectionStatus()
        {
            bool connectionStatus = _componentLinker.CheckConnections();

            if(connectionStatus)
            {
                return "Connected";
            }

            return "Disconnected";
        }

        public string GetConnectionColor()
        {
            bool connectionStatus = _componentLinker.CheckConnections();

            if(connectionStatus)
            {
                return "LawnGreen";
            }

            return "Red";
        }
    }
}
