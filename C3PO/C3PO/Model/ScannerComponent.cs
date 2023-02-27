using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.Model
{
    internal class ScannerComponent : IComponent
    {
        public Task<int> StartOpAsync()
        {
            throw new NotImplementedException();
        }

        public Task<int> StopOpAsync()
        {
            throw new NotImplementedException();
        }
    }
}
