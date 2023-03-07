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
        Empty
    }

    public class StartWindowViewModel : ViewModelBase
    {
        /*
         * Attributes
         */
        private ViewModelBase settingsVM;
        private ComponentLinker componentLinker;
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

        /*
         * Commands
         */
        public ICommand ScannerBtnCommand { get; }
        public ICommand SettingsBtnCommand { get; }
        public ICommand SaveBtnCommand { get; }
        public ICommand ViewResultsBtnCommand { get; }
        public ICommand GoToHomePageCommand { get; }

        /*
         * Constructors
         */
        public StartWindowViewModel(NavigationStore navigateStore)
        {
            _navigateStore = navigateStore;
            settingsVM = new SettingsViewModel(navigateStore);

            _resultsUC = new C3PO.View.ScanResultsEmpty();
            _resultsUC.DataContext = new ScanResultsEmptyViewModel();

            _scannerButtons = new ObservableCollection<string>();
            ScannerButtons.Add("Scan");

            _resultPanelHeader = "No Scanned Content Available";

            ScannerBtnCommand = new ScannerButtonsClickedCommand(this);
            SettingsBtnCommand = new SettingsBtnClickedCommand(NavigateStore, settingsVM);
            NavigateStore.PreviousViewModel = this;
            SaveBtnCommand = new SaveFileCommand();
            ViewResultsBtnCommand = new ViewResultsCommand();
            GoToHomePageCommand = new NavigateUriCommand("https://sites.google.com/nevada.unr.edu/team-20-c3po/home?authuser=1");

            componentLinker = new ComponentLinker();
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
        }

        public void StartScan(CancellationToken ct)
        {
            Application.Current.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Scan;
            }));
            bool result = componentLinker.StartScan(ct);
            if (result)
            {
                FinishScan(ct);
            }
            else
            {
                LinkerState = ScanStates.Empty;
            }
        }

        public void FinishScan(CancellationToken ct)
        {
            Application.Current?.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Reconstruct;
            }));
            bool result = componentLinker.StartReconstruction(ct);
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
            Application.Current.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Finish;
                ResultsUC = new C3PO.View.ScanResults();
            }));
            componentLinker.Finish();
        }

        //public void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        //{
        //    ProcessStartInfo uriStartInfo = new ProcessStartInfo(e.Uri.AbsoluteUri);
        //    uriStartInfo.UseShellExecute = true;
        //    Process.Start(uriStartInfo);
        //    e.Handled = true;
        //}
    }
}
