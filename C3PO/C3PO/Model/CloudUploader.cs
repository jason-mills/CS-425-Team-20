/* CloudUploader.cs:
 *  Model class responsible for handling interations for uploading files
 *  to the Google Drive cloud.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 *      CS426, 2023
 */

using C3PO.Model;
using Google.Apis.Drive.v3.Data;
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
using Google.Apis.Services;
using Google.Apis.Util.Store;
using Google.Apis.Upload;
using Google.Apis.Util;
using Google.Apis.Drive.v3;

namespace C3PO.Model
{
    public class CloudUploader
    {
        string[] scopes;
        string clientId;
        string clientSecret;
        UserCredential? cred;
        gd.DriveService service;
        string uploadedURL;
        static bool isRunning;

        // Constructor
        public CloudUploader()
        {
            scopes = new string[] { gd.DriveService.Scope.DriveFile };
            clientId = "124258515310-msjeupun26bvk08h76mlagi8loekcaeq.apps.googleusercontent.com";
            clientSecret = "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";
            service = new gd.DriveService();
            uploadedURL = "";
            isRunning = false;
        }

        // Methods
        public async Task UploadAsync(string fPath, bool reset = false)
        {
            isRunning = true;
            cred = await GoogleWebAuthorizationBroker.AuthorizeAsync(
                new ClientSecrets { ClientId = clientId, ClientSecret = clientSecret },
                scopes,
                Environment.UserName,
                CancellationToken.None,
                new FileDataStore("MyAppsToken"));

            if (IsExpired() || reset == true && cred != null)
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
                Name = fPath.Substring(fPath.LastIndexOf('\\') + 1),
                WritersCanShare = true,
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

                    Permission newPermission = new Permission();
                    newPermission.Type = "anyone";
                    newPermission.Role = "reader";
                    newPermission.AllowFileDiscovery = true;

                    var p = await service.Permissions.Create(newPermission, fileId).ExecuteAsync();

                    gd.FilesResource.GetRequest r = service.Files.Get(fileId);

                    r.IncludeLabels = "webViewLink";
                    r.Fields= "*";

                    gd.Data.File uploadedFile = await r.ExecuteAsync();

                    uploadedURL = uploadedFile.WebContentLink;
                }
            }

            isRunning = false;
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

        public bool IsExpired()
        {
            if(cred == null)
            {
                return true;
            }

            DateTime issuedUtc = this.cred.Token.IssuedUtc;
            DateTime limitUtc = issuedUtc.AddMinutes(1);
            if (cred.Token.IsExpired(SystemClock.Default) || limitUtc.CompareTo(DateTime.UtcNow) < 0)
            {
                return true;
            }

            return false;
        }

        public string GetFileLink()
        {
            while (isRunning) ;
            return uploadedURL;
        }
    }
}
