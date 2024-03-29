﻿/* NavigationStore.cs:
 *  The store class responsible for storing and updating the global page that is
 *  in view for the user.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using C3PO.ViewModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.Stores
{
    public class NavigationStore
    {
        private ViewModelBase _currentViewModel;

        public event Action CurrentViewModelChanged;

        public ViewModelBase CurrentViewModel
        {
            get
            {
                return _currentViewModel;
            }
            set
            {
                _currentViewModel = value;
                OnCurrentViewModelChanged();
            }
        }
        private ViewModelBase _previousViewModel;
        public ViewModelBase PreviousViewModel
        {
            get
            {
                return _previousViewModel;
            }
            set
            {
                _previousViewModel = value;
            }
        }

        private void OnCurrentViewModelChanged()
        {
            CurrentViewModelChanged?.Invoke();
        }
    }
}
