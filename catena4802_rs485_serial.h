/*

Name:   catena4802_rs485_serial.h

Function:
        Global linkage for catena4802_rs485_serial.ino

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   May 2023

*/

#ifndef _catena4802_rs485_serial_h_
# define _catena4802_rs485_serial_h_

#pragma once

#include <Catena.h>
#include <Catena_Led.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_Timer.h>
#include <SPI.h>
#include "Catena4802_cMeasurementLoop.h"

//  The global clock object

extern  McciCatena::Catena                      gCatena;
extern  McciCatena::cTimer                      ledTimer;
extern  McciCatena::Catena::LoRaWAN             gLoRaWAN;
extern  McciCatena::StatusLed                   gLed;

extern  SPIClass                                gSPI2;
extern  McciCatena4802::cMeasurementLoop        gMeasurementLoop;

//  The flash
extern  McciCatena::Catena_Mx25v8035f           gFlash;

#endif // !defined(_catena4802_rs485_serial_h_)