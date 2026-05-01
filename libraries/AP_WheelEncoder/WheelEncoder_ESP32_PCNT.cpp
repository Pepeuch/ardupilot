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

#include "WheelEncoder_ESP32_PCNT.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

namespace {

#ifndef HAL_ESP32_WENC_LEFT_A
#define HAL_ESP32_WENC_LEFT_A 2
#endif

#ifndef HAL_ESP32_WENC_LEFT_B
#define HAL_ESP32_WENC_LEFT_B 6
#endif

#ifndef HAL_ESP32_WENC_RIGHT_A
#define HAL_ESP32_WENC_RIGHT_A 7
#endif

#ifndef HAL_ESP32_WENC_RIGHT_B
#define HAL_ESP32_WENC_RIGHT_B 8
#endif

}

AP_WheelEncoder_ESP32_PCNT::AP_WheelEncoder_ESP32_PCNT(AP_WheelEncoder &frontend,
                                                       uint8_t instance,
                                                       AP_WheelEncoder::WheelEncoder_State &state) :
    AP_WheelEncoder_Backend(frontend, instance, state)
{
    _initialised = init();
}

AP_WheelEncoder_ESP32_PCNT::~AP_WheelEncoder_ESP32_PCNT()
{
    deinit();
}

bool AP_WheelEncoder_ESP32_PCNT::get_default_pins(uint8_t instance, int &pin_a, int &pin_b)
{
    switch (instance) {
    case 0:
        pin_a = HAL_ESP32_WENC_LEFT_A;
        pin_b = HAL_ESP32_WENC_LEFT_B;
        return true;
    case 1:
        pin_a = HAL_ESP32_WENC_RIGHT_A;
        pin_b = HAL_ESP32_WENC_RIGHT_B;
        return true;
    default:
        return false;
    }
}

bool AP_WheelEncoder_ESP32_PCNT::init(void)
{
    int pin_a = -1;
    int pin_b = -1;
    if (!get_default_pins(_state.instance, pin_a, pin_b)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: invalid ESP32 instance %u", _state.instance);
        return false;
    }

    const pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };

    esp_err_t err = pcnt_new_unit(&unit_config, &_unit);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: pcnt_new_unit failed %d", (int)err);
        deinit();
        return false;
    }

    const pcnt_chan_config_t channel_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };

    err = pcnt_new_channel(_unit, &channel_config, &_channel);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: pcnt_new_channel failed %d", (int)err);
        deinit();
        return false;
    }

    err = pcnt_channel_set_edge_action(_channel,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: set edge action failed %d", (int)err);
        deinit();
        return false;
    }

    err = pcnt_channel_set_level_action(_channel,
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: set level action failed %d", (int)err);
        deinit();
        return false;
    }

    err = pcnt_unit_enable(_unit);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: pcnt_unit_enable failed %d", (int)err);
        deinit();
        return false;
    }

    err = pcnt_unit_clear_count(_unit);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: pcnt_unit_clear_count failed %d", (int)err);
        deinit();
        return false;
    }

    err = pcnt_unit_start(_unit);
    if (err != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: pcnt_unit_start failed %d", (int)err);
        deinit();
        return false;
    }

    return true;
}

void AP_WheelEncoder_ESP32_PCNT::deinit(void)
{
    if (_unit != nullptr) {
        pcnt_unit_stop(_unit);
        pcnt_unit_disable(_unit);
    }

    if (_channel != nullptr) {
        pcnt_del_channel(_channel);
        _channel = nullptr;
    }

    if (_unit != nullptr) {
        pcnt_del_unit(_unit);
        _unit = nullptr;
    }
}

void AP_WheelEncoder_ESP32_PCNT::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!_initialised) {
        copy_state_to_frontend(_distance_count, _total_count, _error_count, now_ms);
        return;
    }

    int count = 0;
    esp_err_t err = pcnt_unit_get_count(_unit, &count);
    if (err != ESP_OK) {
        _error_count++;
        copy_state_to_frontend(_distance_count, _total_count, _error_count, now_ms);
        return;
    }

    if (count != 0) {
        _distance_count += count;
        _total_count += (count >= 0) ? (uint32_t)count : (uint32_t)(-count);

        err = pcnt_unit_clear_count(_unit);
        if (err != ESP_OK) {
            _error_count++;
        }
    }

    copy_state_to_frontend(_distance_count, _total_count, _error_count, now_ms);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32
