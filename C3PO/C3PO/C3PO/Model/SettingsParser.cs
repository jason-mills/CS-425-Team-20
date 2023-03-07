using C3PO.ViewModel;
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
    internal class SettingsParser
    {
        private readonly string settingsPath;
        private SettingsViewModel vm;

        public SettingsParser(SettingsViewModel vm)
        {
            settingsPath = System.AppDomain.CurrentDomain.BaseDirectory + "\\resources\\settings.xml";
            this.vm = vm;
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

            string defRot = vm.NoOfRotations.ToString();
            string defPar = vm.Partitions.ToString();

            (hardwareSettings?.Element("rotations") ?? new XElement(defRot)).Value = defRot;
            (hardwareSettings?.Element("turnRadius") ?? new XElement(defPar)).Value = defPar;

            // Add children to root
            root.Add(hardwareSettings);

            // Save
            root.Save(dest);
        }

        private void ImportSettings(string fpath)
        {
            XElement settings = XElement.Load(fpath);

            XElement? hardwareSettings = (from subsystem in settings?.Descendants("subsystem")
                                         where subsystem?.Attribute("category")?.Value.ToString() == "hardware"
                                         select subsystem)?.FirstOrDefault();

            string defRot = vm.NoOfRotations.ToString();
            string defPar = vm.Partitions.ToString();

            int rot = Int32.Parse(
                (hardwareSettings?.Element("rotations") ?? new XElement(defRot))
                .Value
                .ToString());
            int par = Int32.Parse(
                (hardwareSettings?.Element("turnRadius") ?? new XElement(defPar))
                .Value
                .ToString());

            vm.NoOfRotations = rot;
            vm.Partitions = par;
        }
    }
}
