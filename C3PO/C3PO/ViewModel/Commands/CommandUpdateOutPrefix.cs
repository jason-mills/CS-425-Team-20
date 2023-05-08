/* CommandUpdateOutPrefix.cs:
 *  The command class used to update the output prefix setting variable.
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
    internal class CommandUpdateOutPrefix : CommandBase
    {
        private SettingsParser parser;

        public CommandUpdateOutPrefix(SettingsParser parser)
        {
            this.parser = parser;
        }

        public override void Execute(object? parameter)
        {
            if (parameter == null) return;

            string prefix = (string)parameter;
            parser.outPrefix = prefix;
        }
    }
}
