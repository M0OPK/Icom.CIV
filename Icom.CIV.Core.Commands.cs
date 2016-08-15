using System;
using System.Collections.Generic;
using System.Linq;

namespace Icom.CIV
{
    public partial class Core : IDisposable
    {
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
    }
}
