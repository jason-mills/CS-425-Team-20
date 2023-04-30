using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Google.Apis.Auth.OAuth2;
using Google.Apis.Auth.OAuth2.Requests;
using gd = Google.Apis.Drive.v3;
using Google.Apis.Drive.v3.Data;
using Google.Apis.Services;
using Google.Apis.Util.Store;
using Google.Apis.Upload;

namespace C3PO.Model
{
    public class CloudUploader
    {
        string[] scopes;
        string clientId;
        string clientSecret;
        UserCredential? cred;
        gd.DriveService service;

        // Constructor
        public CloudUploader()
        {
            scopes = new string[] { gd.DriveService.Scope.DriveFile };
            clientId = "124258515310-msjeupun26bvk08h76mlagi8loekcaeq.apps.googleusercontent.com";
            clientSecret = "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";
            service = new gd.DriveService();
        }

        // Methods
        public async Task UploadAsync(string fPath, bool reset = false)
        {
            cred = await GoogleWebAuthorizationBroker.AuthorizeAsync(
                new ClientSecrets { ClientId = clientId, ClientSecret = clientSecret },
                scopes,
                Environment.UserName,
                CancellationToken.None,
                new FileDataStore("MyAppsToken"));

            if (reset == true && cred != null)
            {
                Reset();
            }

            CreateDriveService();
            _ = UploadFileAsync(fPath);
        }
        private async Task<UserCredential> GetCredentialsAsync()
        {
            cred = await GoogleWebAuthorizationBroker.AuthorizeAsync(
                new ClientSecrets { ClientId = clientId, ClientSecret = clientSecret},
                scopes,
                Environment.UserName,
                CancellationToken.None,
                new FileDataStore("MyAppsToken"));

            return cred;
        }

        private void CreateDriveService()
        {
            service = new gd.DriveService(new BaseClientService.Initializer()
            {
                HttpClientInitializer = cred
            });
        }

        private async Task<string> UploadFileAsync(string fPath)
        {
            // Create file metadata
            var fileMetadata = new gd.Data.File()
            {
                Name = fPath.Substring(fPath.LastIndexOf('\\') + 1)
            };

            // Stream file to drive
            string fileId = "";
            await using (var stream = new FileStream(fPath, FileMode.Open, FileAccess.Read))
            {
                var request = service.Files.Create(fileMetadata, stream, "text/plain");
                request.Fields = "*";
                var results = await request.UploadAsync(CancellationToken.None);

                if(results.Status == Google.Apis.Upload.UploadStatus.Failed)
                {
                    Console.WriteLine($"Error: Failed to upload file {fPath}");
                }

                if (request.ResponseBody != null)
                {
                    fileId = request.ResponseBody.Id;
                }
            }

            return fileId;
        }

        public bool Reset()
        {
            if (this.cred != null) {
                DateTime issuedUtc = this.cred.Token.IssuedUtc;
                DateTime limitUtc = issuedUtc.AddMinutes(1);
                if (limitUtc.CompareTo(DateTime.UtcNow) < 0)
                {
                    GoogleWebAuthorizationBroker.ReauthorizeAsync(cred, CancellationToken.None).Wait();
                }
            }

            return true;
        }
    }
}
