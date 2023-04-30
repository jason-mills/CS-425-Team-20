using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace C3PO.Model
{
    public class MetadataParser
    {
        private JObject? json;

        public MetadataParser(string fPath)
        {
            LoadMetadata(fPath);
        }

        public void LoadMetadata(string fPath)
        {
            // Test if file exists
            if (!File.Exists(fPath))
            {
                return;
            }

            // Load json object
            using (StreamReader reader = File.OpenText(@fPath))
            {
                json = (JObject)JToken.ReadFrom(new JsonTextReader(reader));
            }
        }

        public string? Get(string key)
        {
            if(json is not null)
            {
                return json[key]?.ToString();
            }

            return null;
        }
    }
}
