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
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace C3PO.ViewModel
{
    public enum ScanStates
    {
        Start,
        Scan,
        Reconstruct,
        Finish,
        Display
    }

    public class StartWindowViewModel : ViewModelBase
    {
        /*
         * Attributes
         */
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

        /*
         * Constructors
         */
        public StartWindowViewModel(NavigationStore navigateStore)
        {
            _navigateStore = navigateStore;

            _resultsUC = new C3PO.View.ScanResultsEmpty();
            _resultsUC.DataContext = new ScanResultsEmptyViewModel();

            _scannerButtons = new ObservableCollection<string>();
            ScannerButtons.Add("Scan");

            _resultPanelHeader = "No Scanned Content Available";

            ScannerBtnCommand = new ScannerButtonsClickedCommand(this);
            SettingsBtnCommand = new SettingsBtnClickedCommand(NavigateStore);
            NavigateStore.PreviousViewModel = this;
            SaveBtnCommand = new SaveFileCommand();
            ViewResultsBtnCommand = new ViewResultsCommand();

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
        }

        public void StartScan()
        {
            Application.Current.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Scan;
            }));
            componentLinker.StartScan();
            FinishScan();
        }

        public void FinishScan()
        {
            Application.Current?.Dispatcher.Invoke(new Action(() =>
            {
                LinkerState = ScanStates.Reconstruct;
            }));
            //componentLinker.StartReconstruction();
            FinishReconstruct();
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
    }
}
