using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace C3PO.View
{
    /// <summary>
    /// Interaction logic for SettingsWindow.xaml
    /// </summary>
    public partial class SettingsWindow : UserControl
    {
        public SettingsWindow()
        {
            InitializeComponent();
        }

        public void FindCategory_Click(object sender, RoutedEventArgs e)
        {
            if(sender == null)
            {
                return;
            }

            Button control = (Button)sender;

            if(control.Name == "JumpToGen")
            {
                GenLabel.BringIntoView();
            }
            else if(control.Name == "JumpToScan")
            {
                ScanLabel.BringIntoView();
            }
            else if(control.Name == "JumpToHw")
            {
                HwLabel.BringIntoView();
            }
            else if(control.Name == "JumpToReg")
            {
                RegLabel.BringIntoView();
            }
        }
    }
}
