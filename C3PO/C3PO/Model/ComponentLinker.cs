using C3PO.Stores;
using C3PO.ViewModel;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Management.Automation.Language;
using System.Security.Policy;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;

namespace C3PO.Model
{
    public class ComponentLinker
    {
        private IComponent _reconstructComp;
        private IComponent _scanComp;
        private SettingsParser settings;

        private CancellationToken _ct;
        public CancellationToken CT
        {
            get
            {
                return _ct;
            }
            set
            {
                _ct = value;
                _reconstructComp = new ReconstructionComponent(_ct, settings);
                _scanComp = new ScannerComponent(_ct, settings);
            }
        }

        private static DateTime _startTime;
        private static DateTime _endTime;
        private static TimeSpan _timeSpanned;
        public TimeSpan TimeSpanned
        {
            get
            {
                return _timeSpanned;
            }
            private set
            {
                _timeSpanned = value;
            }
        }
        private bool passedRecon;
        private bool isRunning;

        /* Constructors */
        public ComponentLinker(SettingsParser settings)
        {
            _startTime = new DateTime();
            _endTime = new DateTime();
            _timeSpanned = TimeSpan.Zero;
            this.settings = settings;
            _ct = new CancellationToken();
            passedRecon = false;
            isRunning = false;
            _reconstructComp = new ReconstructionComponent(_ct, settings);
            _scanComp = new ScannerComponent(_ct, settings);
        }

        public bool StartScan()
        {
            _startTime = DateTime.Now;
            isRunning = true;

            return _scanComp.StartOp();
        }

        public bool StartReconstruction()
        {
            if(settings.isUserScan)
            {
                _startTime = DateTime.Now;
            }

            passedRecon = true;
            isRunning = false;

            return _reconstructComp.StartOp();
        }

        public bool Finish()
        {
            if (passedRecon)
            {
                _endTime = DateTime.Now;
                _timeSpanned = (_endTime - _startTime);
                passedRecon = false;
            }

            return true;
        }

        public bool CheckConnections()
        {
            bool scanConnection = isRunning || _scanComp.CheckConnection(); 

            return scanConnection && _reconstructComp.CheckConnection();
        }

    }
}
