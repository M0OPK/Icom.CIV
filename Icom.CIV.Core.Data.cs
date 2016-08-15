using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;

namespace Icom.CIV
{
    public partial class Core : IDisposable
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
            [ExpectResponse(false)]
            COMMAND_FREQUENCY_SET = 0x00,
            [ExpectResponse(false)]
            COMMAND_MODE_SET = 0x01,
            [ExpectResponse(true)]
            COMMAND_FREQUENCY_BOUNDARY_READ = 0x02,
            [ExpectResponse(true)]
            COMMAND_FREQUENCY_READ = 0x03,
            [ExpectResponse(true)]
            COMMAND_MODE_READ = 0x04,
            [ExpectResponse(true)]
            COMMAND_FREQUENCY_WRITE = 0x05,
            [ExpectResponse(true)]
            COMMAND_MODE_WRITE = 0x06,
            [ExpectResponse(true)]
            COMMAND_VFO_SET = 0x07,
            [ExpectResponse(true)]
            COMMAND_MEMORY_CHANNEL_SET = 0x08,
            [ExpectResponse(true)]
            COMMAND_MEMORY_CHANNEL_WRITE = 0x09,
            [ExpectResponse(true)]
            COMMAND_MEMORY_TRANSFER_VFO = 0x0A,
            [ExpectResponse(true)]
            COMMAND_MEMORY_CHANNEL_CLEAR = 0x0B,
            [ExpectResponse(true)]
            COMMAND_OFFSET_FREQUENCY_READ = 0x0C,
            [ExpectResponse(true)]
            COMMAND_OFFSET_FREQUENCY_WRITE = 0x0D,
            [ExpectResponse(true)]
            COMMAND_SCAN = 0x0E,
            [ExpectResponse(true)]
            COMMAND_SPLIT_DUPLEX = 0x0F,
            [ExpectResponse(true)]
            COMMAND_TUNING_STEP = 0x10,
            [ExpectResponse(true)]
            COMMAND_ATTENUATOR = 0x11,
            [ExpectResponse(true)]
            COMMAND_ANTENNA_CONTROL = 0x12,
            [ExpectResponse(true)]
            COMMAND_SPEECH_SYNTH = 0x13,
            [ExpectResponse(true)]
            COMMAND_AFRF_GAIN_SET = 0x14,
            [ExpectResponse(true)]
            COMMAND_AFRF_GAIN_READ = 0x15,
            [ExpectResponse(true)]
            COMMAND_PANEL_CONTROLS_SET = 0x16,
            [ExpectResponse(true)]
            COMMAND_CW_MESSAGE_SEND = 0x17,
            [ExpectResponse(true)]
            COMMAND_POWER_ON_OFF = 0x18,
            [ExpectResponse(true)]
            COMMAND_TRANCEIVER_ID_READ = 0x19,
            [ExpectResponse(true)]
            COMMAND_MEMORY_IF_READ_SET_MISC_PANEL = 0x1A,
            [ExpectResponse(true)]
            COMMAND_REPEATER_TONE_SET = 0x1B,
            [ExpectResponse(true)]
            COMMAND_TXRX = 0x1C,
            [ExpectResponse(true)]
            COMMAND_READ_SET_BAND_EDGES = 0x1E,
            [ExpectResponse(true)]
            COMMAND_DSTAR_CALLSIGN_SET = 0x1F,
            [ExpectResponse(true)]
            COMMAND_DSTAR_OTHER_SETTINGS = 0x20
        }

        public enum RadioMode
        {
            MODE_LSB = 0x00,
            MODE_USB = 0x01,
            MODE_AM = 0x02,
            MODE_CW = 0x03,
            MODE_RTTY = 0x04,
            MODE_FM = 0x05,
            MODE_WFM = 0x06,
            MODE_CW_R = 0x07,
            MODE_RTTY_R = 0x08,
            MODE_DV = 0x17
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
            BAUD_300 = 300,
            BAUD_1200 = 1200,
            BAUD_4800 = 4800,
            BAUD_9600 = 9600,
            BAUD_19200 = 19200
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
    }

    public static class EnumExtensions
    {
        public static TAttribute GetAttribute<TAttribute>(this Enum value)
            where TAttribute : Attribute
        {
            var type = value.GetType();
            var name = Enum.GetName(type, value);
            return type.GetField(name)
                .GetCustomAttributes(false)
                .OfType<TAttribute>()
                .SingleOrDefault();
        }
    }

    public class ExpectResponseAttribute : Attribute
    {
        internal ExpectResponseAttribute(bool expectResponse)
        {
            ExpectResponse = expectResponse;
        }
        public bool ExpectResponse { get; private set; }
    }

    public static class CommandBytesExtensions
    {
        public static bool GetExpectsResponse(this Core.CommandBytes b)
        {
            return b.GetAttribute<ExpectResponseAttribute>().ExpectResponse;
        }
    }
}
