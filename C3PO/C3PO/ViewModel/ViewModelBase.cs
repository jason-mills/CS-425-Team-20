/* ViewModelBase.cs:
 *  The view model class used as the parent class for all view models.
 *  It handles property changes.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel
{
    public abstract class ViewModelBase : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
