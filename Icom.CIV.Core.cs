using System;
using System.Reflection;
using System.ComponentModel;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.IO.Ports;

namespace Icom.CIV
{
    public delegate void CIVCommandReadyEvent(object sender, EventArgs e);

    public partial class Core : IDisposable
    {
        // Add a list of radios for which functionality is handled here
        private enum EnabledRadios
        {
            IC_746 = Radio.IC_746,
            IC_7100 = Radio.IC_7100
        }

        private SerialPort sp;
        private System.Exception serialException;
        private List<byte> cmdBuffer;
        private List<byte[]> cmdStackRX;
        private List<byte[]> cmdStackTX;
        private System.Diagnostics.Stopwatch serialTimer;
        private Thread waitLoopTX;
        private Thread waitLoopRX;
        public ConfigData Config;
        private static Mutex commandLock;
        private byte preAmble;

        // Trigger events (used when data is ready to send, or one or more commands have been received)
        private AutoResetEvent RXTriggerEvent;
        private AutoResetEvent TXTriggerEvent;

        private bool waitingForResult;
        private bool waitingForRawResult;
        private byte[] resultData;
        private byte[] lastCommand;

        // Event for waiting command(s)
        public event CIVCommandReadyEvent CommandWaiting;

        public Core()
        {
            // Dynamic command buffer
            cmdBuffer = new List<byte>();
            cmdStackRX = new List<byte[]>();
            cmdStackTX = new List<byte[]>();

            // Timer used to protect against clashing
            serialTimer = new System.Diagnostics.Stopwatch();
            Config = new ConfigData();

            // Mutex used to protect against races
            commandLock = new Mutex();
            preAmble = 0;

            waitingForResult = false;

            RXTriggerEvent = new AutoResetEvent(false);
            TXTriggerEvent = new AutoResetEvent(false);
        }

        ~Core()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                CloseSerialPort();
                // free managed resources
                if (sp != null)
                {
                    sp.Dispose();
                    sp = null;
                }

                if (TXTriggerEvent != null)
                {
                    TXTriggerEvent.Dispose();
                    TXTriggerEvent = null;
                }

                if (RXTriggerEvent != null)
                {
                    RXTriggerEvent.Dispose();
                    RXTriggerEvent = null;
                }
            }
            // free native resources if there are any.
            //if (nativeResource != IntPtr.Zero)
            //{
            //    Marshal.FreeHGlobal(nativeResource);
            //    nativeResource = IntPtr.Zero;
            //}
        }


        // Return list of available radios
        public string[] GetRadioNames()
        {
            int itemCount = Enum.GetNames(typeof(EnabledRadios)).Length;
            string[] rnames = new string[itemCount];
            uint item = 0;

            foreach (Radio thisRadio in Enum.GetValues(typeof(EnabledRadios)))
                rnames[item++] = GetEnumDescription(thisRadio);

            return rnames;
        }

        // Add command to received command queue, notify with event that command(s) ready
        private void QueueCommand()
        {
            // Validate command
            bool validMessage = true;
            if (cmdBuffer.Count < 3)
                validMessage = false;
            else
            {
                byte[] header = cmdBuffer.GetRange(0, 2).ToArray();

                // Check for promiscuity level
                if (header[0] != Config.ControllerAddress && header[0] != 0x00 && Config.ControllerPromiscuousLevel != PromiscuityLevel.PROMISCUOUS_ALLMESSAGES && Config.ControllerPromiscuousLevel != PromiscuityLevel.PROMISCUOUS_ANYCONTROLLER)
                    validMessage = false;
                else if (header[1] != Config.RadioAddress && Config.ControllerPromiscuousLevel != PromiscuityLevel.PROMISCUOUS_ALLMESSAGES && Config.ControllerPromiscuousLevel != PromiscuityLevel.PROMISCUOUS_ANYRADIO)
                    validMessage = false;
            }

            // If message is invalid, there's nothing more to do
            if (!validMessage)
            {
                cmdBuffer.Clear();
                return;
            }

            // If there is no process waiting on a result, then queue command and trigger event handler
            if (!waitingForResult && !waitingForRawResult)
            {
                cmdStackRX.Add(cmdBuffer.ToArray());
                cmdBuffer.Clear();

                RXTriggerEvent.Set();
                return;
            }

            if (!waitingForRawResult)
            {
                // Remove header, unless there is a process waiting for raw data
                cmdBuffer.RemoveRange(0, 2);
            }

            if (lastCommand != null && lastCommand.Length > 0)
            {
                // Create a temporary copy off the buffer, and trim it to match the length of the last command
                List<byte> tempBuffer = cmdBuffer.ToList();
                if (lastCommand != null && tempBuffer.Count > lastCommand.Length)
                    while (tempBuffer.Count > lastCommand.Length)
                        tempBuffer.Remove(tempBuffer.First());

                if (lastCommand != null && BitConverter.ToString(tempBuffer.ToArray()).Equals(BitConverter.ToString(lastCommand)))
                {
                    // If this was an echo of our sent command, then clear buffer and keep looking
                    cmdBuffer.Clear();
                    return;
                }
            }

            // Clear waiting flag and pass result back to waiting process
            waitingForResult = false;
            waitingForRawResult = false;
            resultData = cmdBuffer.ToArray();
            cmdBuffer.Clear();
            return;

        }

        // Add a byte to the current command buffer
        private void AddByteToBuffer(byte thisByte)
        {
            cmdBuffer.Add(thisByte);
        }

        // Clear buffer (only used if jammer received)
        private void ClearBuffer()
        {
            cmdBuffer.Clear();
        }

        public void setRadioID(Radio radioid)
        {
            Config.RadioID = radioid;
            if (Config.RadioAddress == 0x00)
                Config.RadioAddress = (byte)radioid;
        }

        public Radio getRadioID()
        {
            return Config.RadioID;
        }

        public string getRadioName()
        {
            return GetEnumDescription(Config.RadioID);
        }

        public void setRadioAddress(byte radioaddress)
        {
            Config.RadioAddress = radioaddress;
        }

        public byte getRadioAddress()
        {
            return Config.RadioAddress;
        }

        public void setControllerAddress(byte controlleraddress)
        {
            Config.ControllerAddress = controlleraddress;
        }

        public byte getControllerAddress()
        {
            return Config.ControllerAddress;
        }

        public RadioInfo GetRadioInfo()
        {
            RadioInfo thisRI;
            thisRI.baudRate = Config.SerialBaudRate;
            thisRI.CommPort = Config.SerialPort;
            thisRI.RadioAddress = Config.RadioAddress;
            thisRI.RadioID = Config.RadioID;
            thisRI.RadioName = GetEnumDescription(Config.RadioID);
            return thisRI;
        }

        // Return true if there are any received commands waiting
        public bool CommandQueued()
        {
            return (cmdStackRX.Count > 0);
        }

        // Read a single received command, remove it from the stack
        public byte[] ReadCommand()
        {
            // Read one command from stack
            byte[] thisCommand = cmdStackRX.First();
            cmdStackRX.Remove(cmdStackRX.First());
            return thisCommand;
        }

        // Queue a command to be sent to radio
        public void TransmitCommand(byte[] commandBuffer, bool immediate = false)
        {
            if (immediate)
                TransmitCommandToRadio(commandBuffer);
            else
            {
                commandLock.WaitOne();
                cmdStackTX.Add(commandBuffer);
                commandLock.ReleaseMutex();
                TXTriggerEvent.Set();
            }
        }

        // Instantly send command to radio
        private void TransmitCommandToRadio(byte[] commandBuffer, byte overridedestination = 0xFF, byte overridesender = 0xFF, bool raw = false)
        {
            List<byte> localBuffer = new List<byte>();
            localBuffer.Add((byte)SpecialByte.SPECIAL_PREAMBLE);
            localBuffer.Add((byte)SpecialByte.SPECIAL_PREAMBLE);
            if (!raw)
            {
                byte sender = (overridesender == 0xFF) ? Config.ControllerAddress : overridesender;
                byte destination = (overridedestination == 0xFF) ? Config.RadioAddress : overridedestination;

                localBuffer.Add(destination);
                localBuffer.Add(sender);
            }
            localBuffer.AddRange(commandBuffer);
            localBuffer.Add((byte)SpecialByte.SPECIAL_COMMANDEND);
            sp.Write(localBuffer.ToArray(), 0, localBuffer.ToArray().Length);
            // Remove pre-amble and command end code
            localBuffer.RemoveRange(0, 2);
            localBuffer.Remove(localBuffer.Last());
            lastCommand = localBuffer.ToArray();
        }

        public RadioInfo AutoDetectRadio(bool autoconnect = false)
        {
            if (sp != null && sp.IsOpen)
            {
                // If we have an open serial port, send the command to identify transceiver ID to broadcast address
                byte[] command = { (byte)CommandBytes.COMMAND_TRANCEIVER_ID_READ, 0x00 };
                TransmitCommandToRadio(command, 0x00);
                byte[] result = WaitForResponse(250 + ((300 / (int)Config.SerialBaudRate) * 1000), true);
                RadioInfo radioInfo = new RadioInfo();

                // Search for empty array, empty result or NG result (bad command)
                if (result.Length < 5 || result[0] == 0x00 || result[2] == (byte)SpecialByte.SPECIAL_COMMANDNG)
                {
                    radioInfo.RadioID = Radio.NULL_RADIO;
                    return radioInfo;
                }

                // Set radio data
                radioInfo.RadioAddress = result[1];
                radioInfo.RadioID = (Radio)result[4];
                radioInfo.CommPort = Config.SerialPort;
                radioInfo.baudRate = Config.SerialBaudRate;
                radioInfo.RadioName = GetEnumDescription(radioInfo.RadioID);
                return radioInfo;
            }
            else
            {
                // We need a radio ID to connect
                if (Config.RadioID == Radio.NULL_RADIO)
                    setRadioID(Radio.PC_CONTROL);

                // Loop through each port, then each CIV Baud rate
                foreach (string thisPort in GetSerialPorts())
                {
                    foreach (BaudRates thisBaud in Enum.GetValues(typeof(BaudRates)))
                    {
                        Config.ControllerPromiscuousLevel = PromiscuityLevel.PROMISCUOUS_ALLMESSAGES;
                        if (OpenSerialPort(thisPort, (int)thisBaud))
                        {
                            RadioInfo riTemp = AutoDetectRadio();
                            if (riTemp.RadioID != Radio.NULL_RADIO)
                            {
                                if (!autoconnect)
                                    CloseSerialPort();
                                else
                                {
                                    CloseSerialPort();
                                    Config.ControllerPromiscuousLevel = PromiscuityLevel.PROMISCUOUS_NONE;
                                    setRadioID(riTemp.RadioID);
                                    setRadioAddress(riTemp.RadioAddress);
                                    setControllerAddress((byte)Radio.PC_CONTROL);
                                    OpenSerialPort(riTemp.CommPort, riTemp.baudRate);
                                }
                                return riTemp;
                            }
                            CloseSerialPort();
                        }
                    }
                }
                RadioInfo riTemp2 = new RadioInfo();
                riTemp2.RadioID = Radio.NULL_RADIO;
                return riTemp2;
            }
        }

        public byte[] WaitForResponse(int maxWaitMS = 0, bool rawresult = false)
        {
            if (rawresult)
                waitingForRawResult = true;
            else
                waitingForResult = true;

            System.Diagnostics.Stopwatch thisTimer = new System.Diagnostics.Stopwatch();
            if (maxWaitMS > 0)
                thisTimer.Start();
            // Wait for a result
            while (resultData == null || resultData.Length == 0)
            {
                Thread.Sleep(50);
                if (maxWaitMS > 0 && thisTimer.ElapsedMilliseconds > maxWaitMS)
                {
                    byte[] thisTempResult = { 0x00 };
                    return thisTempResult;
                }
                
            }
            byte[] thisResult = resultData;
            resultData = null;
            return thisResult;
        }
    }
}
