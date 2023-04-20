using C3PO.Stores;
using C3PO.View;
using C3PO.ViewModel;
using System;
using System.Collections.Generic;
using System.Configuration;
/* App.xaml.cs:
 *  Behind-code for global application handling.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 02, 2023
 * 
 */

using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;

namespace C3PO
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        private readonly NavigationStore _navigateStore;

        public App()
        {
            _navigateStore = new NavigationStore();
        }

        protected override void OnStartup(StartupEventArgs e)
        {
            _navigateStore.CurrentViewModel = new StartWindowViewModel(_navigateStore);

            MainWindow = new MainWindow();
            MainWindow.DataContext = new MainViewViewModel(_navigateStore);
            MainWindow.Show();
            base.OnStartup(e);

        }
    }
}
