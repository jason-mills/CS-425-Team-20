/* ScanMetadata.cs:
 *  Model class responsible for storing and updating the metadata values obtained
 *  for the C3PO system.
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

namespace C3PO.Model
{
    public class ScanMetadata : INotifyPropertyChanged
    {
        private string _dataName;
        public string DataName
        {
            get
            {
                return _dataName;
            }
            set
            {
                this._dataName = value;
                OnPropertyChanged(nameof(_dataName));
            }
        }

        private string _dataValue;
        public string DataValue
        {
            get
            {
                return _dataValue;
            }
            set
            {
                this._dataValue = value;
                OnPropertyChanged(nameof(_dataValue));
            }
        }

        public ScanMetadata(string name, string value)
        {
            _dataName = name;
            _dataValue = value;
        }

        public event PropertyChangedEventHandler? PropertyChanged;
        protected virtual void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
