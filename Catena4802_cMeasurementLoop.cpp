/*

Module: Catena4802_cMeasurementLoop.cpp

Function:
        Class for transmitting accumulated measurements.

Copyright:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   May 2023

*/

#include "Catena4802_cMeasurementLoop.h"

#include <arduino_lmic.h>
#include <catena4802_rs485_serial.h>

using namespace McciCatena4802;
using namespace McciCatena;

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

#define UART2_BAUD_RATE     9600

#define UART2_RX_BUFFER_SIZE 64
#define UART2_TX_BUFFER_SIZE 64

#define UART2_RX_WAIT_TIME   3000   // ms

#define ASCII_STX     0x02    // Start of Text
#define ASCII_ACK     0x06    // Acknowledgment
#define ASCII_NAK     0x15    // Negative Acknowledgement
#define ASCII_DOLLOR  0x24    // $
#define ASCII_LF      0x0A    // Line Feed
#define ASCII_CR      0x0D    // Carriage Return
#define ASCII_SP      0x20    // Space

#define UART_READY      0
#define UART_RECEIVE    1
#define UART_READ       2
#define UART_WRITE      3
#define UART_TRANSMIT   4

byte com2TrxFlag = 0;         // unsigned char 0 ~ 255
byte com2RxHead = 0;
byte com2RxTail = 0;
byte com2TxHead = 0;
byte com2TxTail = 0;

volatile int8_t com2RxData[UART2_RX_BUFFER_SIZE] = {0};
volatile int8_t com2TxData[UART2_TX_BUFFER_SIZE] = {0};

unsigned long lastTimeCom2Rx = 0;

unsigned long u32wait;
constexpr uint8_t channel = 4;

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        }

    Wire.begin();

    if (this->m_Sht.begin())
        {
        this->m_fSht3x = true;
        gCatena.SafePrintf("SHT3x found: Env sensor\n");
        }
    else
        {
        this->m_fSht3x = false;
        gCatena.SafePrintf("No SHT3x found: check wiring\n");
        }

    pinMode(kRs485Rx, OUTPUT);
    pinMode(kRs485Tx, OUTPUT);
    Serial2.begin(9600);    // baud-rate at 9600
    this->m_fRs485 = true;
    delay(500);
    kRs485TxEn();
    Serial2.println("Hello VWI World!!! - Serial #2");    // Some leading characters are broken
    delay(500);
    kRs485RxEn();

    u32wait = millis() + UART2_RX_WAIT_TIME;
    com2TrxFlag = UART_WRITE;

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

/******************************************************************************
|
|               RS485 Receive and Transmit functions
|
******************************************************************************/

bool cMeasurementLoop::com2RxDataLoad()
    {
    bool fResult = false;

    if ((com2TrxFlag == UART_READY) && Serial2.available())
        {
        volatile char udr2 = Serial2.read();
        if (udr2 == 0x30)
            {
            com2RxTail = 0;
            com2TrxFlag = UART_RECEIVE;
            }
        fResult = true;
        }

    if ((com2TrxFlag == UART_RECEIVE) && Serial2.available())
        {
        volatile char udr2 = Serial2.read();

        if ((udr2 == ASCII_CR) && (com2TrxFlag == UART_RECEIVE))
            {
            com2RxData[com2RxTail] = 0;
            }
        else if ((udr2 == ASCII_LF) && (com2TrxFlag == UART_RECEIVE))
            {
            com2TrxFlag = UART_READ;
            }
        else
            {
            if (com2TrxFlag == UART_RECEIVE)                // STX로 시작한 0x0D 이전 데이터를 저장함
                {
                com2RxData[com2RxTail++] = udr2;
                if (com2RxTail > UART2_RX_BUFFER_SIZE)
                    com2RxTail = UART2_RX_BUFFER_SIZE;
                }
            }
        fResult = true;
        }

    return fResult;
    }

void cMeasurementLoop::com2TxDataLoad(uint8_t nChannel)
    {
    com2TxData[0] = '$';
    com2TxData[1] = 'S';
    com2TxData[2] = 'T';
    com2TxData[3] = 'N';
    com2TxData[4] = '_';
    com2TxData[5] = 'V';
    com2TxData[6] = 'W';
    com2TxData[7] = 'I';
    com2TxData[8] = ',';
    com2TxData[9] = '0' + nChannel;
    com2TxData[10] = ASCII_CR;
    com2TxData[11] = ASCII_LF;

    com2TrxFlag = UART_TRANSMIT;
    }

void cMeasurementLoop::com2TxProcess()
    {
    Serial2.write(com2TxData[0]);
    Serial2.write(com2TxData[1]);
    Serial2.write(com2TxData[2]);
    Serial2.write(com2TxData[3]);
    Serial2.write(com2TxData[4]);
    Serial2.write(com2TxData[5]);
    Serial2.write(com2TxData[6]);
    Serial2.write(com2TxData[7]);
    Serial2.write(com2TxData[8]);
    Serial2.write(com2TxData[9]);     // nCh
    Serial2.write(com2TxData[10]);    // 0x0D
    Serial2.write(com2TxData[11]);    // 0x0A

    com2TrxFlag = UART_READY;
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // set the LEDs to flash accordingly.
            gLed.Set(McciCatena::LedPattern::Sleeping);
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            this->sleep();
        break;

    // get some data. This is only called while booting up.
    case State::stWarmup:
        if (fEntry)
            {
            //start the timer
            this->setTimer(5 * 1000);
            }
        if (this->timedOut())
            newState = State::stMeasure;
        break;

    // fill in the measurement
    case State::stMeasure:
        if (fEntry)
            {
            this->updateSynchronousMeasurements();
            this->setTimer(1000);
            newState = State::stTransmit;
            }
        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;
            this->fillTxBuffer(b, this->m_data);

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            this->resetMeasurements();
            this->startTransmission(b);

            while (true)
                {
                std::uint32_t lmicCheckTime;
                os_runloop_once();
                lmicCheckTime = this->m_UplinkTimer.getRemaining();

                // if we can sleep, break out of this loop
                // NOTE: if that the TX is not ready, LMIC is still waiting for interrupt
                if (! os_queryTimeCriticalJobs(ms2osticks(lmicCheckTime)) && LMIC_queryTxReady())
                    {
                    break;
                    }

                gCatena.poll();
                yield();
                }
            }
        if (this->txComplete())
            {
            newState = State::stSleeping;

            // calculate the new sleep interval.
            this->updateTxCycleTime();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = Flags(0);
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    uint8_t writeData[12];
    uint32_t timeOut = 3000;
    uint32_t startTime;

    this->m_data.Vbat = gCatena.ReadVbat();
    this->m_data.flags |= Flags::Vbat;

    this->m_data.Vbus = gCatena.ReadVbus();
    this->m_data.flags |= Flags::Vcc;

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        this->m_data.flags |= Flags::Boot;
        }

    if (this->m_fSht3x)
        {
        cSHT3x::Measurements m;
        this->m_Sht.getTemperatureHumidity(m);
        this->m_data.env.Temperature = m.Temperature;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= Flags::TH;
        }

    this->m_data.serial.nChannel = 1;
    if (this->m_fRs485)
        {
        while (this->m_data.serial.nChannel <= channel)
            {
            if (com2TrxFlag == UART_WRITE)
                {
                // Serial.println(com2TrxFlag);
                com2TxDataLoad(this->m_data.serial.nChannel);
                }

            if (com2TrxFlag == UART_TRANSMIT)
                {
                // Serial.println(com2TrxFlag);
                kRs485TxEn();
                com2TxProcess();
                u32wait = millis() + 1000;         // wait a moment for tx data transmit
                }

            while (com2TrxFlag == UART_READY)
                {
                // Serial.println(com2TrxFlag);
                if (millis() > u32wait)
                    {
                    kRs485RxEn();
                    com2TrxFlag = UART_RECEIVE;
                    u32wait = millis() + UART2_RX_WAIT_TIME;      // wait a moment for slave answer
                    }
                }

            while (com2TrxFlag == UART_RECEIVE)
                {
                // Serial.println(com2TrxFlag);
                if (millis() > u32wait)
                    {
                    com2TrxFlag = UART_READ;
                    }
                }

            startTime = millis();

            while(!(this->m_data.serial.nData = Serial2.available()))
                {
                if ((millis() - startTime) > timeOut)
                    break;
                }

            if (this->m_data.serial.nData > 0)
                {
                // b.put(this->m_data.serial.nChannel); // send channel number
                // gCatena.SafePrintf("Channel %u Data:", this->m_data.serial.nChannel);
                for (uint8_t i = 0; i < this->m_data.serial.nData; i++)
                    {
                    this->m_data.serial.readData[i] = Serial2.read();
                    // b.put(this->m_data.serial.readData[i]);
                    // gCatena.SafePrintf(" %02x", this->m_data.serial.readData[i]);
                    }
                // gCatena.SafePrintf("\n");
                this->m_data.flags |= Flags::RS485;
                }

            this->m_data.serial.nChannel = this->m_data.serial.nChannel + 1;
            com2TrxFlag = UART_WRITE;
            }
        }

    // enable boost regulator if no USB power and VBat is less than 3.1V
    // if (!m_fUsbPower && (this->m_data.Vbat < 3.10f))
    //     {
    //     boostPowerOn();
    //     delay(50);
    //     }
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    gLed.Set(McciCatena::LedPattern::Sending);

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    constexpr unsigned kUplinkPort = 4;
    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed, kUplinkPort))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

    if (this->m_fTimerActive)
        {
        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
            {
            this->m_fTimerActive = false;
            this->m_fTimerEvent = true;
            fEvent = true;
            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();

    this->m_data.Vbus = gCatena.ReadVbus();
    setVbus(this->m_data.Vbus);
    }

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
        {
        // values greater than one are decremented and ultimately reset to default.
        this->m_txCycleCount = txCycleCount - 1;
        }
    else if (txCycleCount == 1)
        {
        // it's now one (otherwise we couldn't be here.)
        gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

        this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
        }
    else
        {
        // it's zero. Leave it alone.
        }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
        this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
        this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
            static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (! this->kEnableDeepSleep)
        {
        return false;
        }

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
#ifdef USBCON
    else if (Serial.dtr())
        {
        fDeepSleep = false;
        }
#endif
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                            " (USB will disconnect while asleep)"
#endif
                            ": ",
                            deepSleepDelay
                            );

        // sleep and print
        gLed.Set(McciCatena::LedPattern::TwoShort);

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //             static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

void cMeasurementLoop::deepSleepPrepare(void)
    {
    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    vout1PowerOff(); // internal peripherals Power off, specific to 4802.
    vout2PowerOff(); // external peripherals Power off, specific to 4802.
    //boostPowerOff();
    }

//
// call this after waking up from a long (> 15 minute) sleep to correct for LMIC sleep defect
// This should be done after updating micros() and updating LMIC's idea of time based on
// the sleep time.
//
void fixLmicTimeCalculationAfterWakeup(void)
    {
    ostime_t const now = os_getTime();
    // just tell the LMIC that we're available *now*.
    LMIC.globalDutyAvail = now;
    // no need to randomize
    // for EU-like, we need to reset all the channel avail times to "now"
#if CFG_LMIC_EU_like
    for (unsigned i = 0; i < MAX_BANDS; ++i)
        {
        LMIC.bands[i].avail = now;
        }
#endif
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    vout1PowerOn();  // internal peripherals Power on, specific to 4802.
    vout2PowerOn();  // external peripherals Power on, specific to 4802.
    delay(10);

    // if (!m_fUsbPower && (this->m_data.Vbat < 3.10f))
    //     {
    //     boostPowerOn();
    //     delay(20);
    //     }

    Serial.begin();
    Wire.begin();
    SPI.begin();

    fixLmicTimeCalculationAfterWakeup();
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }
