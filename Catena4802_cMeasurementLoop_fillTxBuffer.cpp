/*

Module: Catena4802_cMeasurementLoop_fillBuffer.cpp

Function:
        Class for transmitting accumulated measurements.

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   May 2023

*/

#include <Catena_TxBuffer.h>

#include "Catena4802_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciCatena4802;

/*

Name:   McciCatena4802::cMeasurementLoop::fillTxBuffer()

Function:
        Prepare a messages in a TxBuffer with data from current measurements.

Definition:
        void McciCatena4802::cMeasurementLoop::fillTxBuffer(
                cMeasurementLoop::TxBuffer_t& b
                );

Description:
        A format 0x30 message is prepared from the data in the cMeasurementLoop
        object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);

    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags & Flags::Vbat) !=  Flags(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send Vdd if we can measure it.

    // Vbus is sent as 5000 * v
    if ((this->m_data.flags & Flags::Vcc) !=  Flags(0))
        {
        float Vbus = mData.Vbus;
        gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));
        b.putV(Vbus);
        }

    // send boot count
    if ((this->m_data.flags & Flags::Boot) !=  Flags(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    if ((this->m_data.flags & Flags::TH) != Flags(0))
        {
        if (this->m_fSht3x)
            {
            gCatena.SafePrintf(
                "SHT3x:   T: %d RH: %d\n",
                (int) mData.env.Temperature,
                (int) mData.env.Humidity
                );
            b.putT(mData.env.Temperature);
            // no method for 2-byte RH, directly encode it.
            b.put2uf((mData.env.Humidity / 100.0f) * 65535.0f);
            }
        }

    if ((this->m_data.flags & Flags::RS485) != Flags(0))
        {
        if (this->m_data.serial.nData > 0)
            {
            gCatena.SafePrintf("Channel %u Data:", this->m_data.serial.nChannel);
            for (uint8_t i = 0; i < this->m_data.serial.nData; i++)
                    {
                    gCatena.SafePrintf(" %02x", this->m_data.serial.readData[i]);
                    b.put(this->m_data.serial.readData[i]);
                    }
            b.put(this->m_data.serial.nChannel); // send channel number
            }
        else
            gCatena.SafePrintf("No data received from RS485 device\n");
        }

    gLed.Set(McciCatena::LedPattern::Off);
    }
