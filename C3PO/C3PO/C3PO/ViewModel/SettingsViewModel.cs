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

        public ICommand BackBtnCommand { get; }
        public ICommand ExportSettingsCommand { get; }

        public SettingsViewModel(NavigationStore ns)
        {
            BackBtnCommand = new SettingsBackButtonCommand(ns);
            ExportSettingsCommand = new SaveFileCommand();
        }
    }
}
