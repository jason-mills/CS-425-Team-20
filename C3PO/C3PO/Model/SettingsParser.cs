using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Shapes;
using System.Xml.Linq;
using static System.Windows.Forms.DataFormats;

namespace C3PO.Model
{
    public class SettingsParser
    {
        private readonly string settingsPath;
        public int turnRadius;
        public int scansPerAngle;
        public int icpIters;
        public double voxelMult;
        public string algo;
        public string dir;
        public string baseDir;
        public string inPrefix;
        public string outPrefix;
        public string regOrder;
        public bool interMode;
        public string inputFormat;
        public string outputFormat;
        public bool isUserScan;

        public SettingsParser()
        {
            settingsPath = System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.xaml";

            turnRadius = 30;
            scansPerAngle = 1;
            icpIters = 20;
            voxelMult = 1;
            algo = "point-to-plane";
            baseDir = Directory.GetCurrentDirectory() + "\\output\\";
            dir = UpdateDir();
            inPrefix = "out";
            outPrefix = "final";
            regOrder = GenDefaultRegOrder();
            interMode = false;
            inputFormat = ".xyz";
            outputFormat = ".stl";
            isUserScan = true;
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

            //XElement? hardwareSettings = (from subsystem in settings?.Descendants("subsystem")
            //                              where subsystem?.Attribute("category")?.Value.ToString() == "hardware"
            //                              select subsystem)?.FirstOrDefault();
            XElement? general = LoadCategory(settings, "general");
            XElement? scannin = LoadCategory(settings, "scanning");
            XElement? reconst = LoadCategory(settings, "reconstruction");

            if(general != null)
            {
                SaveGeneralSettings(general);
            }

            if(scannin != null)
            {
                SaveScanningSettings(scannin);
            }

            if(reconst != null)
            {
                SaveScanningSettings(reconst);
            }

            root.Add(general);
            root.Add(scannin);
            root.Add(reconst);


            //string defRot = scansPerAngle.ToString();
            //string defPar = turnRadius.ToString();

            //(hardwareSettings?.Element("scansPerAngle") ?? new XElement(defRot)).Value = defRot;
            //(hardwareSettings?.Element("turnRadius") ?? new XElement(defPar)).Value = defPar;

            //// Add children to root
            //root.Add(hardwareSettings);

            // Save
            root.Save(dest);
        }

        private void ImportSettings(string fpath)
        {
            try
            {
                XElement settings = XElement.Load(fpath);

                XElement? category = LoadCategory(settings, "general");
                if(category is not null)
                {
                    LoadGeneralSettings(category);
                }

                category = LoadCategory(settings, "scanning");
                if(category is not null)
                {
                    LoadScanningSettings(category);
                }

                category = LoadCategory(settings, "reconstruction");
                if(category is not null)
                {
                    LoadReconstructionSettings(category);
                }
            }
            catch
            {
                return;
            }
        }

        private XElement? LoadCategory(XElement settings, string category)
        {
            try
            {
                XElement? categorySettings = (from subsystem in settings?.Descendants("subsystem")
                                              where subsystem?.Attribute("category")?.Value.ToString() == category
                                              select subsystem)?.FirstOrDefault();

                return categorySettings;
            }
            catch
            {
                return null;
            }
        }

        private void LoadGeneralSettings(XElement subsystem)
        {
            // Get settings file values
            string iPrefix = (subsystem.Element("inPrefix") ?? new XElement("")).Value.ToString();
            string oPrefix = (subsystem.Element("outPrefix") ?? new XElement("")).Value.ToString();
            string lDir = (subsystem.Element("loadDir") ?? new XElement("")).Value.ToString();
            string iFormat = (subsystem.Element("inFormat") ?? new XElement("")).Value.ToString();
            string oFormat = (subsystem.Element("outFormat") ?? new XElement("")).Value.ToString();

            // Set to proper values
            inPrefix = iPrefix == "" ? "out" : iPrefix;
            outPrefix = oPrefix == "" ? "final" : oPrefix;
            dir = lDir == "" ? ".\\output" : lDir;
            inputFormat = iFormat == "" ? ".xyz" : iFormat;
            outputFormat = oFormat == "" ? ".stl" : oFormat;
        }

        private void LoadScanningSettings(XElement subsystem)
        {
            // Get settings file values
            string tRad = (subsystem.Element("turnRadius") ?? new XElement("")).Value.ToString();
            string scansPAng = (subsystem.Element("scansPerAngle") ?? new XElement("")).Value.ToString();

            // Set to proper values
            turnRadius = tRad == "" ? 30 : int.Parse(tRad);
            scansPerAngle = scansPAng == "" ? 1 : int.Parse(scansPAng);
        }

        private void LoadReconstructionSettings(XElement subsystem)
        {
            // Get settings file values
            string icpIt = (subsystem.Element("icpIters") ?? new XElement("")).Value.ToString();
            string distThresh = (subsystem.Element("distThresh") ?? new XElement("")).Value.ToString();
            string alg = (subsystem.Element("algo") ?? new XElement("")).Value.ToString();
            string order = (subsystem.Element("regOrder") ?? new XElement("")).Value.ToString();
            string intMode = (subsystem.Element("interMode") ?? new XElement("")).Value.ToString();

            // Set to proper values
            icpIters = icpIt == "" ? 20 : int.Parse(icpIt);
            voxelMult = distThresh == "" ? 0.00001 : double.Parse(distThresh);
            algo = alg == "" ? "icp" : alg;
            regOrder = order == "" ? GenDefaultRegOrder() : order;
            interMode = intMode == "" ? false : bool.Parse(intMode);
        }

        private void SaveGeneralSettings(XElement subsystem)
        {
            subsystem.Element("inPrefix")?.SetValue(inPrefix);
            subsystem.Element("outPrefix")?.SetValue(outPrefix);
            subsystem.Element("dir")?.SetValue(dir);
            subsystem.Element("inFormat")?.SetValue(inputFormat);
            subsystem.Element("outFormat")?.SetValue(outputFormat);
        }

        private void SaveScanningSettings(XElement subsystem)
        {
            subsystem.Element("turnRadius")?.SetValue(turnRadius);
            subsystem.Element("scansPerAngle")?.SetValue(scansPerAngle);
        }

        private void SaveReconstructionSettings(XElement subsystem)
        {
            subsystem.Element("icpIters")?.SetValue(icpIters);
            subsystem.Element("algo")?.SetValue(algo);
            subsystem.Element("regOrder")?.SetValue(regOrder.ToString());
            subsystem.Element("interMode")?.SetValue(interMode);
        }

        private string GenDefaultRegOrder()
        {
            string order = "";

            for(int i = 0; i < 360 / turnRadius - 1; i++)
            {
                order += $"{i},";
            }
            order += (360 / turnRadius - 1).ToString();

            return order;
        }

        public string UpdateDir()
        {
            string date = DateTime.Now.ToString("dd-MM-yyyy_HH-mm-ss");
            dir = baseDir + "\\" + date;

            return baseDir;
        }

        public void ResetDir()
        {
            baseDir = Directory.GetCurrentDirectory() + "\\output\\";
            dir = UpdateDir();
        }
    }
}
