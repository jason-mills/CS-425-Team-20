using System;
using System.Collections.Generic;
using System.Linq;
using System.Management.Automation.Language;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace C3PO.ViewModel.Commands
{
    internal class CommandUpdateResultsPanel : CommandBase
    {
        private StartWindowViewModel vm;
        private UserControl previousPanel;
        private static ScanStates previousState;

        public CommandUpdateResultsPanel(StartWindowViewModel vm)
        {
            this.vm = vm;
            previousPanel = new UserControl();
        }

        public override void Execute(object? parameter)
        {
            if(parameter == null)
            {
                return;
            }

            string resultType = (string)parameter;

            if(resultType == null)
            {
                return;
            }
            else if (resultType == "back")
            {
                vm.ResultsUC = previousPanel;
                vm.LinkerState = previousState;
                vm.LinkerUpdate();

                previousState = 0;
                previousPanel = new UserControl();
            }
            else if(vm.LinkerState == previousState)
            {
                return;
            }
            else if(resultType == "GuiTutorial")
            {
                previousState = (ScanStates)((int)vm.LinkerState);
                previousPanel = vm.ResultsUC;
                vm.LinkerState = ScanStates.GuiTutorial;
                vm.ResultsUC = new C3PO.View.GuiTutorial();
            }
            else if(resultType == "Results")
            {
                previousState = vm.LinkerState;
                previousPanel = vm.ResultsUC;
                vm.LinkerState = ScanStates.Finish;
                vm.ResultsUC = new C3PO.View.ScanResults();
            }
            else if(resultType == "HwTutorial")
            {
                previousState = (ScanStates)((int)vm.LinkerState);
                previousPanel = vm.ResultsUC;
                vm.LinkerState = ScanStates.HardwareTutorial;
                vm.ResultsUC = new C3PO.View.HardwareTutorial();
            }
        }
    }
}
