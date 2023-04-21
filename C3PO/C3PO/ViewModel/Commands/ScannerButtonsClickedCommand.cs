﻿/* ScannerButtonsClickedCommand.cs:
 *  Class for the command that modifies the viewmodel's
 *  displayed scanning buttons.
 * authors:
 *  Froilan Luna-Lopez
 *      University of Nevada, Reno
 * date: February 1, 2023
 */

using C3PO.Model;
using MessagePack.Formatters;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Linq;
using System.Management.Automation.Language;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Forms;

namespace C3PO.ViewModel.Commands
{
    internal class ScannerButtonsClickedCommand : CommandBase
    {
        private StartWindowViewModel vm;
        private CancellationTokenSource cts;
        private CancellationToken cancelToken;
        private SettingsParser settings;
        public ScannerButtonsClickedCommand(StartWindowViewModel vm, SettingsParser settings)
        {
            this.vm = vm;
            this.settings = settings;
            
            cts = new CancellationTokenSource();
            cancelToken = cts.Token;

            vm.ComponentLinker.CT = cancelToken;
        }

        private void ScanBtnClicked()
        {
            Task t1 = Task.Factory.StartNew(() =>
            {
                vm.StartScan(cancelToken);
            }, cancelToken);
        }

        private void CancelBtnClicked()
        {
            cts.Cancel();
        }

        private void LoadBtnClicked()
        {
            // Configure file dialogue
            FolderBrowserDialog fbd = new FolderBrowserDialog();

            // Get file inputs
            DialogResult result = fbd.ShowDialog();

            // Test for improper input
            if(result != DialogResult.OK)
            {
                return;
            }

            settings.dir = fbd.SelectedPath;

            Task t1 = Task.Factory.StartNew(() =>
            {
                vm.FinishScan();
            }, cancelToken);

        }

        public override void Execute(object? sender)
        {
            sender = sender as string;
            // Get new buttons
            ObservableCollection<string> newButtons = new ObservableCollection<string>();
            if (sender.Equals("Scan") || sender.Equals("Resume"))
            {
                //newButtons.Add("Pause");
                newButtons.Add("Cancel");
                ScanBtnClicked();
            }
            else if (sender.Equals("Pause"))
            {
                newButtons.Add("Resume");
                newButtons.Add("Cancel");
            }
            else if (sender.Equals("Cancel"))
            {
                CancelBtnClicked();
                newButtons.Add("Scan");
                newButtons.Add("Load Scans");
            }
            else if(sender.Equals("Load Scans"))
            {
                newButtons.Add("Cancel");
                LoadBtnClicked();
            }

            vm.ScanButtonsSet(newButtons);
        }
    }
}
