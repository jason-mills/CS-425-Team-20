/* RotationsChangedCommand.cs:
 *  The command class used when the user tells the UI to update the settings
 *  for the scans per angle in the settings.
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
    internal class RotationsChangedCommand : CommandBase
    {
        SettingsViewModel vm;
        public RotationsChangedCommand(SettingsViewModel vm)
        {
            this.vm = vm;
        }
        public override void Execute(object? parameter)
        {
            String rotStr = parameter?.ToString() ?? "1";
            vm.ScansPerAngle = Int32.Parse(rotStr);
        }
    }
}
