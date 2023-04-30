using C3PO.Model;
using System;
using System.Collections.Generic;
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

        public async Task SaveFileAsync(string fPath, bool reset = false)
        {
            var uploader = new CloudUploader();

            //Task t1 = new Task(() => uploader.UploadAsync(fPath, reset).Wait());

            //t1.Start();
            await uploader.UploadAsync(fPath, reset);
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
                _ = SaveFileAsync(fPath + "\\final.xyz");
            }
            else if(saveType.Equals("all"))
            {
                SaveAllFiles(fPath);
            }
        }
    }
}
