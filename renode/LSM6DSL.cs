//
// An LSM6DSL inertial measurement unit, modelled well enough for the PPA-100
// firmware to bring it up and read it.
//
// Renode's portable build ships no model for this part, and an I2C bus with
// nothing on it is indistinguishable from a broken one: every transfer runs
// out its 2 ms timeout, task_sensors never finishes inside its period, and a
// fixed priority scheduler quite correctly starves everything below it. So the
// emulator needs a device that answers.
//
// Only what the driver actually touches is implemented: the identity
// register, the three control registers, STATUS_REG, and the twelve output
// bytes. Everything else reads back as zero.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LSM6DSL : II2CPeripheral
    {
        public LSM6DSL()
        {
            Reset();
        }

        public void Reset()
        {
            registers = new byte[0x80];
            registers[RegWhoAmI] = WhoAmIValue;
            selectedRegister = 0;
            haveRegister = false;
            sample = 0;
            pollsSinceConversion = 0;
        }

        // The controller sends the register pointer, then either stops (a read
        // is about to follow) or keeps writing data bytes.
        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }

            var offset = 0;
            if(!haveRegister)
            {
                selectedRegister = (byte)(data[0] & 0x7F);
                haveRegister = true;
                offset = 1;
            }

            for(var i = offset; i < data.Length; i++)
            {
                registers[selectedRegister] = data[i];
                this.Log(LogLevel.Debug, "CTRL write 0x{0:X2} = 0x{1:X2}", selectedRegister, data[i]);
                selectedRegister = (byte)((selectedRegister + 1) & 0x7F);
            }
        }

        public byte[] Read(int count = 0)
        {
            if(count <= 0)
            {
                count = 1;
            }

            // The part converts at 416 Hz while the estimator polls at 1 kHz,
            // so most polls have nothing new. STATUS_REG is how it says so,
            // and the driver counts the difference. Modelling that honestly
            // keeps the firmware's stale_reads() counter meaningful: a device
            // that is always ready would let the filter treat one measurement
            // as three independent ones.
            if(selectedRegister == RegStatus)
            {
                pollsSinceConversion++;
                var ready = pollsSinceConversion >= PollsPerConversion;
                var result = new byte[count];
                result[0] = ready ? (byte)(StatusAccelReady | StatusGyroReady) : (byte)0x00;
                return result;
            }

            if(selectedRegister == RegOutXLG)
            {
                pollsSinceConversion = 0;
                sample++;
                return BuildMeasurement(count);
            }

            var plain = new byte[count];
            for(var i = 0; i < count; i++)
            {
                plain[i] = registers[(selectedRegister + i) & 0x7F];
            }
            return plain;
        }

        public void FinishTransmission()
        {
            haveRegister = false;
        }

        // Twelve bytes: gyro XYZ then accel XYZ, 16 bit two's complement,
        // little endian, in the order the driver burst reads them.
        //
        // The arm this stands in for moves slowly, so the signal is a gentle
        // sinusoid about rest rather than noise: enough for the estimator to
        // have something to fuse and for a reader to see the plumbing is live,
        // small enough that it is physically plausible for a settled arm.
        private byte[] BuildMeasurement(int count)
        {
            var t = sample * ConversionPeriodSeconds;
            var gyroZdps = 4.0 * Math.Sin(2.0 * Math.PI * 0.25 * t);
            var accelXg = 0.02 * Math.Cos(2.0 * Math.PI * 0.25 * t);
            var accelYg = 0.02 * Math.Sin(2.0 * Math.PI * 0.25 * t);

            var full = new byte[12];
            WriteInt16(full, 0, 0);                                   // gyro X
            WriteInt16(full, 2, 0);                                   // gyro Y
            WriteInt16(full, 4, (short)(gyroZdps / GyroDpsPerLsb));   // gyro Z
            WriteInt16(full, 6, (short)(accelXg / AccelGPerLsb));     // accel X
            WriteInt16(full, 8, (short)(accelYg / AccelGPerLsb));     // accel Y
            WriteInt16(full, 10, (short)(1.0 / AccelGPerLsb));        // accel Z, gravity

            var result = new byte[count];
            for(var i = 0; i < count && i < full.Length; i++)
            {
                result[i] = full[i];
            }
            return result;
        }

        private static void WriteInt16(byte[] buffer, int at, short value)
        {
            buffer[at] = (byte)(value & 0xFF);
            buffer[at + 1] = (byte)((value >> 8) & 0xFF);
        }

        private byte[] registers;
        private byte selectedRegister;
        private bool haveRegister;
        private long sample;
        private int pollsSinceConversion;

        private const byte RegWhoAmI = 0x0F;
        private const byte WhoAmIValue = 0x6A;
        private const byte RegStatus = 0x1E;
        private const byte RegOutXLG = 0x22;
        private const byte StatusAccelReady = 0x01;
        private const byte StatusGyroReady = 0x02;

        // 416 Hz conversion against a 1 kHz poll.
        private const int PollsPerConversion = 3;
        private const double ConversionPeriodSeconds = 1.0 / 416.0;
        private const double GyroDpsPerLsb = 0.070;
        private const double AccelGPerLsb = 0.000122;
    }
}
