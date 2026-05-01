/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "WiFiUdpDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#if HAL_NUM_CAN_IFACES > 0
#include "CANIface.h"
#endif
#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif

static ESP32::UARTDriver cons(0);
#ifdef HAL_ESP32_WIFI
#if HAL_ESP32_WIFI == 1
static ESP32::WiFiDriver serial1Driver; //tcp, client should connect to 192.168.4.1 port 5760
#elif HAL_ESP32_WIFI == 2
static ESP32::WiFiUdpDriver serial1Driver; //udp
#else
static Empty::UARTDriver serial1Driver;
#endif
#else
static Empty::UARTDriver serial1Driver;
#endif
static ESP32::UARTDriver serial2Driver(2);
static ESP32::UARTDriver serial3Driver(1);
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;

#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif

static ESP32::I2CDeviceManager i2cDeviceManager;
#if defined(HAL_ESP32_SPI_BUSES)
static ESP32::SPIDeviceManager spiDeviceManager;
#else
static Empty::SPIDeviceManager spiDeviceManager;
#endif
#if AP_HAL_ANALOGIN_ENABLED
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ESP32::Storage storageDriver;
#endif
static ESP32::GPIO gpioDriver;
#if AP_SIM_ENABLED
static Empty::RCOutput rcoutDriver;
#else
static ESP32::RCOutput rcoutDriver;
#endif
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

#if HAL_NUM_CAN_IFACES > 0
static ESP32::CANIface canDriver;
static AP_HAL::CANIface* canIfaces[HAL_NUM_CAN_IFACES] = { &canDriver };
#endif

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

extern const AP_HAL::HAL& hal;

#if HAL_NUM_CAN_IFACES > 0 && HAL_ESP32_CAN_SELFTEST
static void esp32_can_selftest(void)
{
    AP_HAL::CANIface *iface = hal.can[0];
    if (iface == nullptr) {
        hal.console->printf("ESP32 CAN selftest: no CAN iface\r\n");
        return;
    }

    if (!iface->init(500000)) {
        hal.console->printf("ESP32 CAN selftest: init failed\r\n");
        return;
    }

    AP_HAL::CANFrame tx_frame {};
    tx_frame.id = 0x123;
    tx_frame.dlc = 1;
    tx_frame.canfd = false;
    tx_frame.data[0] = 0xA5;

    const uint64_t deadline = AP_HAL::micros64() + 1000000ULL;
    const int16_t send_ret = iface->send(tx_frame, deadline, AP_HAL::CANIface::Loopback);
    if (send_ret < 0) {
        hal.console->printf("ESP32 CAN selftest: send failed\r\n");
        return;
    }

    for (uint8_t i = 0; i < 50; i++) {
        AP_HAL::CANFrame rx_frame {};
        uint64_t rx_timestamp_us = 0;
        AP_HAL::CANIface::CanIOFlags rx_flags = 0;
        if (iface->receive(rx_frame, rx_timestamp_us, rx_flags) > 0) {
            hal.console->printf("ESP32 CAN selftest: PASS id=0x%03lx dlc=%u data0=0x%02x\r\n",
                                (unsigned long)(rx_frame.id & AP_HAL::CANFrame::MaskStdID),
                                (unsigned)rx_frame.dlc,
                                (unsigned)rx_frame.data[0]);
            return;
        }
        hal.scheduler->delay(10);
    }

    hal.console->printf("ESP32 CAN selftest: FAIL timeout busoff=%u err=%lu\r\n",
                        (unsigned)iface->is_busoff(),
                        (unsigned long)iface->getErrorCount());
}
#endif

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &cons, //Console/mavlink
        &serial1Driver, //Telem 1
        &serial2Driver, //Telem 2
        &serial3Driver, //GPS 1
        &serial4Driver, //GPS 2
        &serial5Driver, //Extra 1
        &serial6Driver, //Extra 2
        &serial7Driver, //Extra 3
        &serial8Driver, //Extra 4
        &serial9Driver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
#if HAL_NUM_CAN_IFACES > 0
        canIfaces
#else
        nullptr
#endif
    )
{}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
#if AP_SIM_ENABLED
    AP::sitl()->init();
#endif  // AP_SIM_ENABLED

    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    hal.scheduler->init();
#if HAL_NUM_CAN_IFACES > 0 && HAL_ESP32_CAN_SELFTEST
    esp32_can_selftest();
#endif
}

void AP_HAL::init()
{
}
