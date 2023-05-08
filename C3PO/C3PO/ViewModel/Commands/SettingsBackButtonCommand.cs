/* SettingsBackButtonCommand.cs:
 *  The command class for returning to the main page from the settings page
 *  when the user tells the UI to go back.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using C3PO.Stores;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class SettingsBackButtonCommand : CommandBase
    {
        private NavigationStore _navigationStore;

        public SettingsBackButtonCommand(NavigationStore ns)
        {
            _navigationStore= ns;
        }

        public override void Execute(object? parameter)
        {
            _navigationStore.CurrentViewModel = _navigationStore.PreviousViewModel;
        }
    }
}
