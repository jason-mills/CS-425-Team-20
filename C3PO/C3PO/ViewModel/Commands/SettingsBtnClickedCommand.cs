/* SettingsBtnClickedCommand.cs:
 *  Class that contains the command functionality
 *  for when the user clicks on the settings command.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 02, 2023
 */

using C3PO.Stores;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    public class SettingsBtnClickedCommand : CommandBase
    {
        private NavigationStore _navigationStore;
        private ViewModelBase _settingsViewModel;

        public SettingsBtnClickedCommand(NavigationStore navigationStore, ViewModelBase svm) 
        {
            _navigationStore = navigationStore;
            _settingsViewModel = svm;
        }
        public override void Execute(object? sender)
        {
            //_navigationStore.CurrentViewModel = new SettingsViewModel(_navigationStore);
            _navigationStore.CurrentViewModel = _settingsViewModel;
        }
    }
}
