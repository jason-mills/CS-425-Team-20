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

        public SettingsBtnClickedCommand(NavigationStore navigationStore) 
        {
            _navigationStore = navigationStore;
        }
        public override void Execute(object? sender)
        {
            _navigationStore.CurrentViewModel = new SettingsViewModel(_navigationStore);
        }
    }
}
