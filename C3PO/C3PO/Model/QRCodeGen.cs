/* QRCodeGen.cs:
 *  Model class responsible for generating QR codes for URL links.
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
using IronBarCode;

namespace C3PO.Model
{
    public class QRCodeGen
    {
        private SettingsParser settings;

        public QRCodeGen(SettingsParser settings)
        {
            this.settings = settings;
        }

        public void GenQR(string url)
        {
            GeneratedBarcode qrCode = IronBarCode.QRCodeWriter.CreateQrCode(url);
            qrCode.SaveAsPng(settings.dir + "\\" + settings.outPrefix + "_qr.png");

            Process p = new Process()
            {
                StartInfo =
                {
                    FileName = settings.dir + "\\" + settings.outPrefix + "_qr.png",
                    UseShellExecute = true,
                    CreateNoWindow = false
                }
            };

            try
            {
                p.Start();
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }
        }
    }
}
