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
