using C3PO.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    public class CommandReconstruct : CommandBase
    {
        StartWindowViewModel vm;
        SettingsParser settings;
        CancellationToken ct;

        public CommandReconstruct(StartWindowViewModel vm, SettingsParser settings)
        {
            this.vm = vm;
            this.settings = settings;
            ct =  new CancellationToken();
        }

        public override void Execute(object? parameter)
        {
            // Reset results panel
            vm.ResultsUC = new C3PO.View.ScanResultsEmpty();

            // Update output directory
            //settings.UpdateDir();

            // Run reconstruction
            Task t1 = Task.Factory.StartNew(() =>
            {
                vm.FinishScan();
            }, ct);
        }
    }
}
