using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Management;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.Model
{
    public class USBDevices
    {
        private ManagementObjectCollection _usbList;
        /*
         *  Constructors
         */
        public USBDevices()
        {
            var hubList = new ManagementObjectSearcher("SELECT * FROM Win32_USBHub");
            _usbList = hubList.Get();
        }

        public List<string> GetNames()
        {
            List<string> usbNames = new List<string>();

            foreach(var usb in _usbList)
            {
                usbNames.Add((string)usb.GetPropertyValue("Name"));
            }

            return usbNames;
        }
    }
}
