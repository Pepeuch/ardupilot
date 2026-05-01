/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_WheelEncoder.h"
#include "WheelEncoder_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include "driver/pulse_cnt.h"

class AP_WheelEncoder_ESP32_PCNT : public AP_WheelEncoder_Backend
{
public:
    AP_WheelEncoder_ESP32_PCNT(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state);
    ~AP_WheelEncoder_ESP32_PCNT() override;

    void update(void) override;

private:
    bool init(void);
    void deinit(void);
    static bool get_default_pins(uint8_t instance, int &pin_a, int &pin_b);

    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channel = nullptr;
    int32_t _distance_count = 0;
    uint32_t _total_count = 0;
    uint32_t _error_count = 0;
    bool _initialised = false;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32
