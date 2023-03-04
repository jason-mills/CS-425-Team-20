/* CommandBase.cs:
 *  Abstract parent class for all commands to
 *  inherit and implement.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *  date: February 1, 2023
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace C3PO.ViewModel.Commands
{
    public abstract class CommandBase : ICommand
    {
        public event EventHandler? CanExecuteChanged;

        public virtual bool CanExecute(object? parameter)
        {
            return true;
        }

        public abstract void Execute(object? parameter);

        protected void OnCanExecuteChanged()
        {
            CanExecuteChanged?.Invoke(this, new EventArgs());
        }
    }
}
