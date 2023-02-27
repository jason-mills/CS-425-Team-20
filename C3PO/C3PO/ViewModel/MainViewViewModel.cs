using C3PO.Stores;
using System;
using System.Collections.Generic;
/* MainViewViewModel.cs:
 *  ViewModel for parent window of application.
 *  Holds functionality for application-wide processes.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 02, 2023
 */

using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel
{
    public class MainViewViewModel : ViewModelBase
    {
        private readonly NavigationStore _navigationStore;

        public ViewModelBase CurrentViewModel => _navigationStore.CurrentViewModel;

        public MainViewViewModel(NavigationStore nstore)
        {
            _navigationStore = nstore;
            _navigationStore.CurrentViewModelChanged += OnCurrentViewModelChanged;
        }

        private void OnCurrentViewModelChanged()
        {
            OnPropertyChanged(nameof(CurrentViewModel));
        }
    }
}
