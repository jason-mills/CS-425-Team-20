using C3PO.Model;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.ViewModel.Commands
{
    internal class CommandUploadToCloud : CommandBase
    {
        // Attributes
        private SettingsParser settings;
        private string saveType;

        // Constructors
        public CommandUploadToCloud(SettingsParser settings, string saveType)
        {
            this.settings = settings;
            this.saveType = saveType;
        }

        public async Task SaveFileAsync(string fPath, bool reset = false, bool single = false)
        {
            var uploader = new CloudUploader();

            await uploader.UploadAsync(fPath, reset);

            if (single && settings.genQr)
            {
                string url = uploader.GetFileLink();
                QRCodeGen qr = new QRCodeGen(settings);
                qr.GenQR(url);
            }
        }

        public void SaveAllFiles(string fPath)
        {
            Task t1 = Task.Run(() =>
            {
                var scanner = new ScannerComponent(settings);
                var files = scanner.GetScansFromDir();
                SaveFileAsync(fPath + "\\final.xyz", true).Wait();
                foreach (var file in files)
                {
                    SaveFileAsync($"{fPath}\\{file}").Wait();
                }
            });
        }

        // Methods
        public override void Execute(object? parameter)
        {
            //string fPath = System.IO.Directory.GetCurrentDirectory() + "\\output";
            string fPath = settings.dir;
            if(saveType.Equals("final"))
            {
                Task t1 = Task.Run(() =>
                    _ = SaveFileAsync(fPath + "\\final.xyz", false, true)
                );
            }
            else if(saveType.Equals("all"))
            {
                SaveAllFiles(fPath);
            }
        }
    }
}
