/* ScannerButtonsClickedCommand.cs:
 *  Class for the command that modifies the viewmodel's
 *  displayed scanning buttons.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 1, 2023
 */

using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace C3PO.ViewModel.Commands
{
    internal class ScannerButtonsClickedCommand : CommandBase
    {
        private StartWindowViewModel vm;
        public ScannerButtonsClickedCommand(StartWindowViewModel vm)
        {
            this.vm = vm;
        }

        private void ScanBtnClicked()
        {
            Thread thread = new Thread(() => vm.StartScan());
            thread.Start();
        }

        public override void Execute(object? sender)
        {
            sender = sender as string;
            // Get new buttons
            ObservableCollection<string> newButtons = new ObservableCollection<string>();
            if (sender.Equals("Scan") || sender.Equals("Resume"))
            {
                newButtons.Add("Pause");
                newButtons.Add("Cancel");
                ScanBtnClicked();
            }
            else if (sender.Equals("Pause"))
            {
                newButtons.Add("Resume");
                newButtons.Add("Cancel");
            }
            else if (sender.Equals("Cancel"))
            {
                newButtons.Add("Scan");
            }

            vm.ScanButtonsSet(newButtons);
        }
    }
}
