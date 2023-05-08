/* CommandUpdateSourcePrefix.cs:
 *  The command class used when the user tells the UI to update the setting
 *  for the source prefix to use when searching for scans.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using C3PO.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class CommandUpdateSourcePrefix : CommandBase
    {
        private SettingsParser parser;

        public CommandUpdateSourcePrefix(SettingsParser parser)
        {
            this.parser = parser;
        }

        public override void Execute(object? parameter)
        {
            if(parameter == null) return;

            string prefix = (string)parameter;
            parser.inPrefix = prefix;
        }
    }
}
