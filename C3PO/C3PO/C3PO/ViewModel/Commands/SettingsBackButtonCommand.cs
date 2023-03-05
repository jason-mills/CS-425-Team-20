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
