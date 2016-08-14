using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Threading;

namespace Icom.CIV
{
    public partial class Core : IDisposable
    {
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
                else if(byteIn == (byte)SpecialByte.SPECIAL_JAMMER)
                {
                    // When jammed, clear buffer. Sending radio should send command again
                    ClearBuffer();
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
                waitLoopTX = new Thread(new ThreadStart(TXThreadLoop));
                waitLoopTX.IsBackground = true;
                waitLoopTX.Start();
                waitLoopRX = new Thread(new ThreadStart(RXThreadLoop));
                waitLoopRX.IsBackground = true;
                waitLoopRX.Start();
                preAmble = 0;
            }
            return sp.IsOpen;
        }

        public void CloseSerialPort()
        {
            if (!sp.IsOpen)
                return;

            waitLoopTX.Abort();
            waitLoopTX.Join();
            waitLoopTX = null;
            waitLoopRX.Abort();
            waitLoopRX.Join();
            waitLoopRX = null;

            sp.DataReceived -= SerialDataReceivedHandler;

            sp.Close();
            sp = null;
        }

        // Return information about last received serial port exception
        public System.Exception GetSerialException()
        {
            return serialException;
        }
    }
}
