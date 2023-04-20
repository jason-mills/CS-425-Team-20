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
    }
}
