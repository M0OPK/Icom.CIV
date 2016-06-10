using System;
using System.ComponentModel;
using System.Reflection;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Icom.CIV
{
    using System.IO.Ports;

    public delegate void CIVCommandReadyEvent(object sender, EventArgs e);

    public partial class Core
    {
        public enum SpecialByte
        {
            SPECIAL_COMMANDNG = 0xFA,
            SPECIAL_COMMANDOK = 0xFB,
            SPECIAL_JAMMER = 0xFC,
            SPECIAL_COMMANDEND = 0xFD,
            SPECIAL_PREAMBLE = 0xFE
        }

        public enum CommandBytes
        {
            COMMAND_FREQUENCY_SET                   = 0x00,
            COMMAND_MODE_SET                        = 0x01,
            COMMAND_FREQUENCY_BOUNDARY_READ         = 0x02,
            COMMAND_FREQUENCY_READ                  = 0x03,
            COMMAND_MODE_READ                       = 0x04,
            COMMAND_FREQUENCY_WRITE                 = 0x05,
            COMMAND_MODE_WRITE                      = 0x06,
            COMMAND_VFO_SET                         = 0x07,
            COMMAND_MEMORY_CHANNEL_SET              = 0x08,
            COMMAND_MEMORY_CHANNEL_WRITE            = 0x09,
            COMMAND_MEMORY_TRANSFER_VFO             = 0x0A,
            COMMAND_MEMORY_CHANNEL_CLEAR            = 0x0B,
            COMMAND_OFFSET_FREQUENCY_READ           = 0x0C,
            COMMAND_OFFSET_FREQUENCY_WRITE          = 0x0D,
            COMMAND_SCAN                            = 0x0E,
            COMMAND_SPLIT_DUPLEX                    = 0x0F,
            COMMAND_TUNING_STEP                     = 0x10,
            COMMAND_ATTENUATOR                      = 0x11,
            COMMAND_ANTENNA_CONTROL                 = 0x12,
            COMMAND_SPEECH_SYNTH                    = 0x13,
            COMMAND_AFRF_GAIN_SET                   = 0x14,
            COMMAND_AFRF_GAIN_READ                  = 0x15,
            COMMAND_PANEL_CONTROLS_SET              = 0x16,
            COMMAND_CW_MESSAGE_SEND                 = 0x17,
            COMMAND_POWER_ON_OFF                    = 0x18,
            COMMAND_TRANCEIVER_ID_READ              = 0x19,
            COMMAND_MEMORY_IF_READ_SET_MISC_PANEL   = 0x1A,
            COMMAND_REPEATER_TONE_SET               = 0x1B,
            COMMAND_TXRX                            = 0x1C,
            COMMAND_READ_SET_BAND_EDGES             = 0x1E,
            COMMAND_DSTAR_CALLSIGN_SET              = 0x1F,
            COMMAND_DSTAR_OTHER_SETTINGS            = 0x20
        }

        public enum RadioMode
        {
            MODE_LSB    = 0x00,
            MODE_USB    = 0x01,
            MODE_AM     = 0x02,
            MODE_CW     = 0x03,
            MODE_RTTY   = 0x04,
            MODE_FM     = 0x05,
            MODE_WFM    = 0x06,
            MODE_CW_R   = 0x07,
            MODE_RTTY_R = 0x08,
            MODE_DV     = 0x17
        }

        public enum PromiscuityLevel
        {
            PROMISCUOUS_NONE,
            PROMISCUOUS_ANYRADIO,
            PROMISCUOUS_ANYCONTROLLER,
            PROMISCUOUS_ALLMESSAGES
        }

        private enum BaudRates
        {
            BAUD_300        = 300,
            BAUD_1200       = 1200,
            BAUD_4800       = 4800,
            BAUD_9600       = 9600,
            BAUD_19200      = 19200
        }

        public struct RadioInfo
        {
            public Radio RadioID;
            public byte RadioAddress;
            public string CommPort;
            public int baudRate;
            public string RadioName;
        }

        // Add the default CIV address of all known radios here, plus name as text
        public enum Radio
        {
            [Description("Null Radio")]
            NULL_RADIO = 0x00,
            [Description("IC-271")]
            IC_271 = 0x20,
            [Description("IC-275")]
            IC_275 = 0x10,
            [Description("IC-375")]
            IC_375 = 0x12,
            [Description("IC-471")]
            IC_471 = 0x22,
            [Description("IC-475")]
            IC_475 = 0x14,
            [Description("IC-575")]
            IC_575 = 0x16,
            [Description("IC-7000")]
            IC_7000 = 0x70,
            [Description("IC-703")]
            IC_703 = 0x68,
            [Description("IC-706")]
            IC_706 = 0x48,
            [Description("IC-706 MkII")]
            IC_706MkII = 0x4E,
            [Description("IC-706 MkII G")]
            IC_706MkIIG = 0x58,
            [Description("IC-707")]
            IC_707 = 0x3e,
            [Description("IC-7100")]
            IC_7100 = 0x88,
            [Description("IC-718")]
            IC_718 = 0x5E,
            [Description("IC-7200")]
            IC_7200 = 0x76,
            [Description("IC-725")]
            IC_725 = 0x28,
            [Description("IC-726")]
            IC_726 = 0x30,
            [Description("IC-728")]
            IC_728 = 0x38,
            [Description("IC-729")]
            IC_729 = 0x3A,
            [Description("IC-735")]
            IC_735 = 0x04,
            [Description("IC-736")]
            IC_736 = 0x40,
            [Description("IC-737")]
            IC_737 = 0x3C,
            [Description("IC-738")]
            IC_738 = 0x44,
            [Description("IC-7400 (IC-746Pro)")]
            IC_7400 = 0x66,
            [Description("IC-746")]
            IC_746 = 0x56,
            [Description("IC-751 A")]
            IC_751A = 0x1C,
            [Description("IC-756")]
            IC_756 = 0x50,
            [Description("IC-756 Pro")]
            IC_756Pro = 0x5C,
            [Description("IC-756 Pro II")]
            IC_756ProII = 0x64,
            [Description("IC-756 Pro III")]
            IC_756ProIII = 0x6e,
            [Description("IC-761")]
            IC_761 = 0x1E,
            [Description("IC-765")]
            IC_765 = 0x2C,
            [Description("IC-775")]
            IC_775 = 0x46,
            [Description("IC-7700")]
            IC_7700 = 0x74,
            [Description("IC-78")]
            IC_78 = 0x62,
            [Description("IC-7800")]
            IC_7800 = 0x6A,
            [Description("IC-781")]
            IC_781 = 0x26,
            [Description("IC-820")]
            IC_820 = 0x42,
            [Description("IC-821")]
            IC_821 = 0x4C,
            [Description("IC-910")]
            IC_910 = 0x60,
            [Description("IC-970")]
            IC_970 = 0x2E,
            [Description("IC-1271")]
            IC_1271 = 0x24,
            [Description("IC-1275")]
            IC_1275 = 0x18,
            [Description("IC-R10")]
            IC_R10 = 0x52,
            [Description("IC-R20")]
            IC_R20 = 0x6C,
            [Description("IC-R71")]
            IC_R71 = 0x1A,
            [Description("IC-R72")]
            IC_R72 = 0x32,
            [Description("IC-R75")]
            IC_R75 = 0x5A,
            [Description("IC-R7000")]
            IC_R7000 = 0x08,
            [Description("IC-R7100")]
            IC_R7100 = 0x34,
            [Description("IC-R8500")]
            IC_R8500 = 0x4A,
            [Description("IC-R9000")]
            IC_R9000 = 0x2A,
            [Description("IC-R9500")]
            IC_R9500 = 0x72,
            [Description("IC-RX7")]
            IC_RX7 = 0x78,
            [Description("Controller/PC")]
            PC_CONTROL = 0xE0
        }

        // Add a list of radios for which functionality is handled here
        private enum EnabledRadios
        {
            IC_746 = Radio.IC_746,
            IC_7100 = Radio.IC_7100
        }

        // Sub-class containing configuration data
        public class ConfigData
        {
            // Serial port config
            public string SerialPort;
            public int SerialBaudRate;
            public int SerialDataBits;
            public StopBits SerialStopBits;
            public Parity SerialParity;

            // Delay timers
            public int PollTimeTransmitMS;
            public int MinLineQuietTimeMS;

            // CIV settings
            internal Radio RadioID;
            internal byte RadioAddress;
            internal byte ControllerAddress;
            public PromiscuityLevel ControllerPromiscuousLevel;
            
            public ConfigData()
            {
                // Default configuration
                SerialPort = "";
                SerialBaudRate = 9600;
                SerialDataBits = 8;
                SerialStopBits = StopBits.One;
                SerialParity = Parity.None;

                PollTimeTransmitMS = 50;
                MinLineQuietTimeMS = 100;

                RadioID = Radio.NULL_RADIO;
                RadioAddress = 0x00;
                ControllerAddress = (byte)Radio.PC_CONTROL;
                ControllerPromiscuousLevel = PromiscuityLevel.PROMISCUOUS_NONE;
            }
        }

        private SerialPort sp;
        private System.Exception serialException;
        private List<byte> cmdBuffer;
        private List<byte[]> cmdStackRX;
        private List<byte[]> cmdStackTX;
        private System.Diagnostics.Stopwatch serialTimer;
        private Thread waitLoop;
        public ConfigData Config;
        private static Mutex commandLock;
        private byte preAmble;

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
        }

        // Helper to extract description attribute
        private static string GetEnumDescription(Enum value)
        {
            FieldInfo fi = value.GetType().GetField(value.ToString());

            DescriptionAttribute[] attributes =
                (DescriptionAttribute[])fi.GetCustomAttributes(
                typeof(DescriptionAttribute),
                false);

            if (attributes != null &&
                attributes.Length > 0)
                return attributes[0].Description;
            else
                return value.ToString();
        }

        // Convert an unsigned integer into 5 bytes of 
        private byte[] IntToBCD5(uint numericvalue, int bytesize=5)
        {
            byte[] bcd = new byte[bytesize];
            for (int byteNo = 0; byteNo < bytesize; ++byteNo)
                bcd[byteNo] = 0;
            for (int digit = 0; digit < bytesize * 2; ++digit)
            {
                uint hexpart = numericvalue % 10;
                bcd[digit / 2] |= (byte)(hexpart << ((digit % 2) * 4));
                numericvalue /= 10;
            }
            return bcd;
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

        // Handler for received serial data
        private void SerialDataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            serialTimer.Reset();
            serialTimer.Start();
            SerialPort rs = (SerialPort)sender;
            byte[] buffer = new byte[rs.BytesToRead];
            rs.Read(buffer, 0, rs.BytesToRead);

            // Process buffer, byte by byte
            foreach (byte byteIn in buffer)
            {
                if (byteIn == (byte)SpecialByte.SPECIAL_COMMANDEND)
                {
                    // Handle end of command, queue command
                    QueueCommand();
                    preAmble = 0;
                }
                else if (preAmble == 2)
                {
                    // Add to buffer if preamblex2 already received (ignore extra premable)
                    if (byteIn != (byte)SpecialByte.SPECIAL_PREAMBLE)
                        AddByteToBuffer(byteIn);
                }
                else if (byteIn == (byte)SpecialByte.SPECIAL_PREAMBLE)
                {
                    // Pre-amble read
                    preAmble++;
                }
            }
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

                if (CommandWaiting != null)
                    CommandWaiting(this, EventArgs.Empty);
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

        // Returns a list of serial ports available
        public string[] GetSerialPorts()
        {
            return SerialPort.GetPortNames();
        }

        // Open serial port, and initialize for reception/sending of commands
        public bool OpenSerialPort(string portName, int baudRate = -1)
        {
            IntToBCD5(433375000, 4);
            // Check CIV config is valid
            if (Config.ControllerAddress == 0x00 || Config.RadioAddress == 0x00 || Config.RadioID == Radio.NULL_RADIO)
            {
                serialException = new System.Exception("CIV Configuration invalid");
                return false;
            }

            sp = new SerialPort(portName);
            try
            {
                // Set specified data into config options
                Config.SerialPort = portName;
                if (baudRate != -1)
                    Config.SerialBaudRate = baudRate;

                // Set Serial port settings from Config
                sp.BaudRate = Config.SerialBaudRate;
                sp.DataBits = Config.SerialDataBits;
                sp.StopBits = Config.SerialStopBits;
                sp.Parity = Config.SerialParity;

                // Setup data received handler for serial port
                sp.DataReceived += new SerialDataReceivedEventHandler(SerialDataReceivedHandler);

                // Open serial port
                sp.Open();
            }
            catch (System.Exception ex)
            {
                serialException = ex;
                return false;
            }

            if (sp.IsOpen)
            {
                // Create new thread for TX polling
                waitLoop = new Thread(new ThreadStart(ActionLoop));
                waitLoop.IsBackground = true;
                waitLoop.Start();
                preAmble = 0;
            }
            return sp.IsOpen;
        }

        public void CloseSerialPort()
        {
            if (!sp.IsOpen)
                return;

            waitLoop.Abort();
            waitLoop = null;

            sp.DataReceived -= SerialDataReceivedHandler;

            sp.Close();
            sp = null;
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

        // Return information about last received serial port exception
        public System.Exception GetSerialException()
        {
            return serialException;
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
                byte sender = Config.ControllerAddress;
                byte destination = Config.RadioAddress;
                if (overridesender != 0xFF)
                    sender = overridesender;
                if (overridedestination != 0xFF)
                    destination = overridedestination;

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

        public void TuneFrequency(double frequency)
        {
            uint freqInt;
            byte[] commandHeader = { (byte)CommandBytes.COMMAND_FREQUENCY_SET };
            byte[] command;
            if (Config.RadioID != Radio.IC_735)
            {
                freqInt = (uint)(frequency * 1000000);
                command = commandHeader.Concat(IntToBCD5(freqInt)).ToArray();
            }
            else
            {
                freqInt = (uint)(frequency * 10000);
                command = commandHeader.Concat(IntToBCD5(freqInt, 4)).ToArray();
            }
            TransmitCommand(command);
        }

        public void SetMode(RadioMode mode, byte filterLevel = 0)
        {
            byte[] commandHeader = { (byte)CommandBytes.COMMAND_MODE_SET };
            byte[] command;
            if (Config.RadioID == Radio.IC_R7000 && (mode == RadioMode.MODE_USB || mode == RadioMode.MODE_LSB))
            {
                // Special handling for ICOM 7000R SSB mode
                byte[] ICR7000SSB = { 0x05, 0x00 };
                command = commandHeader.Concat(ICR7000SSB).ToArray();
            }
            else
            {
                List<byte> modeSet = new List<byte>();
                modeSet.Add((byte)mode);
                if (filterLevel != 0)
                    modeSet.Add(filterLevel);
                command = commandHeader.Concat(modeSet).ToArray();
            }
            TransmitCommand(command);
        }

        // Overload for tune, to include mode
        public void TuneFrequency(double frequency, RadioMode mode, byte filterLevel = 0)
        {
            TuneFrequency(frequency);
            SetMode(mode, filterLevel);
        }

        // This loop runs inside a dedicated background thread.
        private void ActionLoop()
        {
            bool running = true;
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
                    }

                    // Sleep for 1/10th of a second
                    Thread.Sleep(Config.PollTimeTransmitMS);
                }
                catch(ThreadAbortException /*e*/)
                {
                    running = false;
                    Thread.ResetAbort();
                }
            } while (running);
        }
    }
}
