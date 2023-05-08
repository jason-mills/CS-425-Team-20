/* NaviateUriCommand.cs:
 *  The command class used when the user tells the UI to switch to a new page.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Navigation;

namespace C3PO.ViewModel.Commands
{
    internal class NavigateUriCommand : CommandBase
    {
        private string uri;

        public NavigateUriCommand(string uri)
        {
            this.uri = uri;
        }
        public override void Execute(object? parameter)
        {
            RequestNavigateEventArgs? args = parameter is null ? null : (RequestNavigateEventArgs)parameter;

            if(args != null)
            {
                return;
            }

            ProcessStartInfo uriStartInfo = new ProcessStartInfo(uri);
            uriStartInfo.UseShellExecute = true;
            Process.Start(uriStartInfo);
        }
    }
}
