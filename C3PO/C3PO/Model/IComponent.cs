/* IComponent.cs:
 *  Interface class for describing the functionality that all
 *  components of the C3PO system must implement.
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

namespace C3PO.Model
{
    internal interface IComponent
    {
        public bool StartOp();
        public bool StopOp();

        public bool CheckConnection();
    }
}
