using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Shapes;
using System.Xml.Linq;

namespace C3PO.Model
{
    public class SettingsParser
    {
        private readonly string settingsPath;
        public int turnRadius;
        public int scansPerAngle;
        public int icpIters;
        public int distanceThresh;
        public string algo;
        public string dir;
        public string inPrefix;
        public string outPrefix;

        public SettingsParser()
        {
            settingsPath = System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.xml";

            turnRadius = 0;
            scansPerAngle = 0;
            icpIters = 0;
            distanceThresh = 0;
            algo = "icp";
            dir = Directory.GetCurrentDirectory() + "\\output\\";
            inPrefix = "temp";
            outPrefix = "Final";
        }

        public void ImportSettingsFile(string? fpath = null)
        {
            if (fpath is null)
            {
                fpath = settingsPath;
            }

            ImportSettings(fpath);
        }

        public void ExportSettingsFile(string dest, string? source = null)
        {
            if(source is null)
            {
                source = settingsPath;
            }

            ExportSettings(source, dest);
        }

        private void ExportSettings(string source, string dest)
        {
            // Define root
            XElement root = new XElement("C3PO");

            // Update children
            XElement settings = XElement.Load(source);

            XElement? hardwareSettings = (from subsystem in settings?.Descendants("subsystem")
                                          where subsystem?.Attribute("category")?.Value.ToString() == "hardware"
                                          select subsystem)?.FirstOrDefault();

            string defRot = scansPerAngle.ToString();
            string defPar = turnRadius.ToString();

            (hardwareSettings?.Element("scansPerAngle") ?? new XElement(defRot)).Value = defRot;
            (hardwareSettings?.Element("turnRadius") ?? new XElement(defPar)).Value = defPar;

            // Add children to root
            root.Add(hardwareSettings);

            // Save
            root.Save(dest);
        }

        private void ImportSettings(string fpath)
        {
            try
            {
                XElement settings = XElement.Load(fpath);

                XElement? hardwareSettings = (from subsystem in settings?.Descendants("subsystem")
                                              where subsystem?.Attribute("category")?.Value.ToString() == "hardware"
                                              select subsystem)?.FirstOrDefault();

                string defRot = scansPerAngle.ToString();
                string defPar = turnRadius.ToString();

                var test = hardwareSettings?.Element("scansPerAngle");

                int rot = Int32.Parse(
                    (hardwareSettings?.Element("rotations") ?? new XElement(defRot))
                    .Value
                    .ToString());
                int par = Int32.Parse(
                    (hardwareSettings?.Element("turnRadius") ?? new XElement(defPar))
                    .Value
                    .ToString());

                scansPerAngle = rot;
                turnRadius = par;
            }
            catch
            {
                return;
            }
        }
    }
}
