using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace Icom.CIV
{
    public partial class Core : IDisposable
    {
        private void RXThreadLoop()
        {
            bool runningRX = true;
            do
            {
                try
                {
                    if (CommandWaiting != null && CommandQueued())
                        CommandWaiting(this, EventArgs.Empty);

                    RXTriggerEvent.WaitOne();
                }
                catch (ThreadAbortException /*ex*/)
                {
                    runningRX = false;
                    break;
                }
            } while (runningRX);
        }

        // This loop runs inside a dedicated background thread.
        private void TXThreadLoop()
        {
            bool runningTX = true;
            do
            {
                try
                {
                    // Only process command queue if there's been no CIV traffic for at least a second
                    if (serialTimer.ElapsedMilliseconds > Config.MinLineQuietTimeMS || !serialTimer.IsRunning)
                    {
                        commandLock.WaitOne();
                        // Transmit all in queue
                        foreach (byte[] commandData in cmdStackTX)
                        {
                            TransmitCommandToRadio(commandData);
                        }

                        // Clear queue
                        cmdStackTX.Clear();
                        commandLock.ReleaseMutex();
                        TXTriggerEvent.WaitOne();
                    }
                    else
                    {
                        // Sleep for 1/10th of a second
                        Thread.Sleep(Config.PollTimeTransmitMS);
                    }
                }
                catch (ThreadAbortException /*e*/)
                {
                    runningTX = false;
                    break;
                }
            } while (runningTX);
        }
    }
}
