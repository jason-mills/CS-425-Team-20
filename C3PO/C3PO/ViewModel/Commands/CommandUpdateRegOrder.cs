/* CommandUpdateRegOrder.cs:
 *  The command class used when the user tells the UI to update the setting
 *  for the order to reconstruct the partitions.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class CommandUpdateRegOrder: CommandBase
    {
        SettingsViewModel vm;

        public CommandUpdateRegOrder(SettingsViewModel vm)
        {
            this.vm = vm;
        }

        public override void Execute(object? parameter)
        {
            if(parameter == null)
            {
                return;
            }

            string newOrder = (string)parameter;

            vm.RegOrder = newOrder;
        }
    }
}
