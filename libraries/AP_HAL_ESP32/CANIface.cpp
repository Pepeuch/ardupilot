/*
 * Prototype ESP32 TWAI CAN backend for ArduPilot
 *
 * This is a first-cut backend intended for bring-up:
 * - Classic CAN only
 * - No CAN FD
 * - Single TWAI controller only
 * - Worker thread drains RX/TX via TWAI alerts
 *
 * TODO later:
 * - hook real pins from hwdef-generated macros
 * - integrate with HAL_ESP32_Class.cpp
 * - enable HAL_NUM_CAN_IFACES / board macros
 * - add loopback test path and richer diagnostics
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_Common/ExpandingString.h>
#include <AP_CANManager/AP_CANManager.h>

#include <cstring>

#include "CANIface.h"

#if HAL_NUM_CAN_IFACES

extern const AP_HAL::HAL& hal;

using namespace ESP32;

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "ESP32CANIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

uint8_t CANIface::next_interface;

CANIface::CANIface(uint8_t index) :
    _self_index(index)
{
}

CANIface::CANIface() :
    CANIface(next_interface++)
{
}

CANIface::~CANIface()
{
    if (_driver_installed) {
        (void)twai_stop();
        (void)twai_driver_uninstall();
        _driver_installed = false;
    }
    _initialized = false;
}

bool CANIface::_get_timing_config(uint32_t bitrate, twai_timing_config_t &timing_cfg) const
{
    switch (bitrate) {
    case 250000:
        timing_cfg = TWAI_TIMING_CONFIG_250KBITS();
        return true;
    case 500000:
        timing_cfg = TWAI_TIMING_CONFIG_500KBITS();
        return true;
    case 1000000:
        timing_cfg = TWAI_TIMING_CONFIG_1MBITS();
        return true;
    default:
        return false;
    }
}

bool CANIface::_to_twai_message(const AP_HAL::CANFrame &frame, twai_message_t &msg,
                                CanIOFlags flags) const
{
    if (frame.isCanFDFrame()) {
        return false;
    }
    if (frame.isErrorFrame()) {
        return false;
    }
    if (frame.dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen) {
        return false;
    }

    memset(&msg, 0, sizeof(msg));

    msg.extd = frame.isExtended();
    msg.rtr = frame.isRemoteTransmissionRequest();
    msg.ss = (flags & AbortOnError) != 0;
    msg.self = (flags & Loopback) != 0;
    msg.identifier = msg.extd ? (frame.id & AP_HAL::CANFrame::MaskExtID)
                              : (frame.id & AP_HAL::CANFrame::MaskStdID);
    msg.data_length_code = frame.dlc;
    memcpy(msg.data, frame.data, frame.dlc);

    return true;
}

bool CANIface::_from_twai_message(const twai_message_t &msg, AP_HAL::CANFrame &frame) const
{
    if (msg.data_length_code > AP_HAL::CANFrame::NonFDCANMaxDataLen) {
        return false;
    }
    if (msg.dlc_non_comp) {
        return false;
    }

    frame = AP_HAL::CANFrame();
    frame.id = msg.extd ? (msg.identifier & AP_HAL::CANFrame::MaskExtID)
                        : (msg.identifier & AP_HAL::CANFrame::MaskStdID);

    if (msg.extd) {
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }
    if (msg.rtr) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    frame.dlc = msg.data_length_code;
    frame.canfd = false;
    memcpy(frame.data, msg.data, frame.dlc);

    return true;
}

bool CANIface::init(const uint32_t bitrate)
{
    if (_initialized) {
        return true;
    }

    if (_self_index != 0) {
        // Prototype limitation: current ESP32-S3 target uses a single TWAI controller.
        Debug("init: only controller index 0 is supported in prototype");
        return false;
    }

    if (HAL_ESP32_CAN_TX_PIN == TWAI_IO_UNUSED || HAL_ESP32_CAN_RX_PIN == TWAI_IO_UNUSED) {
        // TODO: wire these from hwdef-generated board macros
        Debug("init: CAN pins are not configured yet");
        return false;
    }

    twai_timing_config_t timing_cfg {};
    if (!_get_timing_config(bitrate, timing_cfg)) {
        Debug("init: unsupported bitrate %lu", (unsigned long)bitrate);
        return false;
    }

    twai_filter_config_t filter_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_general_config_t general_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)HAL_ESP32_CAN_TX_PIN,
                                    (gpio_num_t)HAL_ESP32_CAN_RX_PIN,
                                    TWAI_MODE_NORMAL);

    general_cfg.tx_queue_len = HAL_ESP32_TWAI_TX_QUEUE_LEN;
    general_cfg.rx_queue_len = HAL_ESP32_TWAI_RX_QUEUE_LEN;
    general_cfg.alerts_enabled =
        TWAI_ALERT_RX_DATA |
        TWAI_ALERT_TX_SUCCESS |
        TWAI_ALERT_TX_FAILED |
        TWAI_ALERT_TX_IDLE |
        TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_RX_FIFO_OVERRUN |
        TWAI_ALERT_BUS_OFF |
        TWAI_ALERT_BUS_RECOVERED |
        TWAI_ALERT_RECOVERY_IN_PROGRESS |
        TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_ARB_LOST |
        TWAI_ALERT_ERR_PASS |
        TWAI_ALERT_ABOVE_ERR_WARN |
        TWAI_ALERT_BELOW_ERR_WARN |
        TWAI_ALERT_ERR_ACTIVE;
    general_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;

    if (twai_driver_install(&general_cfg, &timing_cfg, &filter_cfg) != ESP_OK) {
        Debug("init: twai_driver_install failed");
        return false;
    }

    if (twai_start() != ESP_OK) {
        Debug("init: twai_start failed");
        (void)twai_driver_uninstall();
        return false;
    }

    _driver_installed = true;
    bitrate_ = bitrate;
    _bus_off = false;
    _initialized = true;

    hal.util->snprintf(_worker_name, sizeof(_worker_name), "can_%u", (unsigned)_self_index);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&CANIface::_worker, void),
                                      _worker_name,
                                      HAL_ESP32_CAN_WORKER_STACK,
                                      AP_HAL::Scheduler::PRIORITY_CAN,
                                      0)) {
        Debug("init: failed to create CAN worker");
        (void)twai_stop();
        (void)twai_driver_uninstall();
        _driver_installed = false;
        _initialized = false;
        return false;
    }

    Debug("init: TWAI backend started on controller %u", (unsigned)_self_index);
    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (!_initialized || !_driver_installed) {
        return -1;
    }
    if (frame.isCanFDFrame() || frame.isErrorFrame() || frame.dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen) {
        return -1;
    }

    {
        WITH_SEMAPHORE(sem);

        CanTxItem item {};
        item.frame = frame;
        item.deadline = tx_deadline;
        item.index = _tx_frame_counter++;
        item.loopback = (flags & Loopback) != 0;
        item.abort_on_error = (flags & AbortOnError) != 0;
        item.setup = true;

        stats.tx_requests++;

        if (!_tx_queue.push(item)) {
            stats.tx_overflow++;
            return 0;
        }
    }

    _drain_tx_queue();
    _signal_event();

    return AP_HAL::CANIface::send(frame, tx_deadline, flags);
}

int16_t CANIface::receive(AP_HAL::CANFrame &out_frame, uint64_t &out_timestamp_us,
                          CanIOFlags &out_flags)
{
    if (!_initialized || !_driver_installed) {
        return -1;
    }

    bool need_drain = false;
    {
        WITH_SEMAPHORE(sem);
        need_drain = _rx_queue.is_empty();
    }

    if (need_drain) {
        _drain_rx_queue();
    }

    CanRxItem item {};
    {
        WITH_SEMAPHORE(sem);
        if (!_rx_queue.pop(item)) {
            return 0;
        }
    }

    out_frame = item.frame;
    out_timestamp_us = item.timestamp_us;
    out_flags = item.flags;

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

void CANIface::_check_available(bool &read, bool &write,
                                const AP_HAL::CANFrame *pending_tx) const
{
    WITH_SEMAPHORE(sem);

    read = !_rx_queue.is_empty();
    write = (pending_tx != nullptr) && (_tx_queue.space() > 0);
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame * const pending_tx,
                      uint64_t blocking_deadline)
{
    if (!_initialized || !_driver_installed) {
        return false;
    }

    const bool want_read = read;
    const bool want_write = write;

    _drain_rx_queue();
    _drain_tx_queue();
    _check_available(read, write, pending_tx);

    if ((want_read && read) || (want_write && write)) {
        return true;
    }

    const uint64_t now = AP_HAL::micros64();
    if (sem_handle != nullptr && now < blocking_deadline) {
        (void)sem_handle->wait(blocking_deadline - now);
    }

    _drain_rx_queue();
    _drain_tx_queue();
    _check_available(read, write, pending_tx);

    return true;
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    sem_handle = handle;
    return true;
}

uint32_t CANIface::getErrorCount() const
{
    return stats.tx_rejected +
           stats.tx_overflow +
           stats.tx_timedout +
           stats.tx_abort +
           stats.rx_overflow +
           stats.rx_errors +
           stats.num_busoff_err;
}

void CANIface::_signal_event(void)
{
    if (sem_handle != nullptr) {
        sem_handle->signal();
    }
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    return _rx_queue.push(rx_item);
}

void CANIface::_drain_rx_queue(void)
{
    if (!_driver_installed) {
        return;
    }

    while (true) {
        twai_message_t msg {};
        const esp_err_t ret = twai_receive(&msg, 0);
        if (ret != ESP_OK) {
            break;
        }

        AP_HAL::CANFrame frame {};
        if (!_from_twai_message(msg, frame)) {
            WITH_SEMAPHORE(sem);
            stats.rx_errors++;
            continue;
        }

        CanRxItem item {};
        item.frame = frame;
        item.timestamp_us = AP_HAL::micros64();
        item.flags = 0;

        WITH_SEMAPHORE(sem);
        if (!add_to_rx_queue(item)) {
            stats.rx_overflow++;
        } else {
            stats.rx_received++;
        }
    }
}

void CANIface::_drain_tx_queue(void)
{
    if (!_driver_installed) {
        return;
    }

    while (true) {
        CanTxItem item {};
        bool have_item = false;

        {
            WITH_SEMAPHORE(sem);
            CanTxItem *item_ptr = _tx_queue[0];
            if (item_ptr == nullptr) {
                break;
            }

            const uint64_t now_us = AP_HAL::micros64();
            if (item_ptr->deadline < now_us) {
                stats.tx_timedout++;
                (void)_tx_queue.pop();
                continue;
            }

            item = *item_ptr;
            have_item = true;
        }

        if (!have_item) {
            break;
        }

        twai_message_t msg {};
        if (!_to_twai_message(item.frame, msg,
                              (item.loopback ? Loopback : 0) |
                              (item.abort_on_error ? AbortOnError : 0))) {
            WITH_SEMAPHORE(sem);
            stats.tx_rejected++;
            (void)_tx_queue.pop();
            continue;
        }

        const esp_err_t ret = twai_transmit(&msg, 0);
        if (ret == ESP_OK) {
            WITH_SEMAPHORE(sem);
            (void)_tx_queue.pop();
            continue;
        }

        if (ret == ESP_ERR_TIMEOUT || ret == ESP_FAIL) {
            // TWAI internal TX queue/controller busy. Try again after next alert wakeup.
            break;
        }

        WITH_SEMAPHORE(sem);
        stats.tx_rejected++;
        (void)_tx_queue.pop();
    }
}

void CANIface::_handle_alerts(uint32_t alerts)
{
    _last_alerts = alerts;

    WITH_SEMAPHORE(sem);

    if ((alerts & TWAI_ALERT_TX_SUCCESS) != 0U) {
        stats.tx_success++;
        stats.last_transmit_us = AP_HAL::micros64();
    }
    if ((alerts & TWAI_ALERT_RX_QUEUE_FULL) != 0U) {
        stats.rx_overflow++;
    }
    if ((alerts & TWAI_ALERT_RX_FIFO_OVERRUN) != 0U) {
        stats.rx_overflow++;
        stats.rx_errors++;
    }
    if ((alerts & TWAI_ALERT_TX_FAILED) != 0U) {
        stats.tx_rejected++;
    }
    if ((alerts & TWAI_ALERT_ARB_LOST) != 0U) {
        stats.rx_errors++;
    }
    if ((alerts & TWAI_ALERT_BUS_ERROR) != 0U) {
        stats.rx_errors++;
    }
    if ((alerts & TWAI_ALERT_BUS_OFF) != 0U) {
        _bus_off = true;
        stats.num_busoff_err++;
        (void)twai_initiate_recovery();
    }
    if ((alerts & TWAI_ALERT_BUS_RECOVERED) != 0U) {
        _bus_off = false;
    }
}

void CANIface::_worker(void)
{
    while (_initialized) {
        if (!_driver_installed) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        uint32_t alerts = 0;
        const esp_err_t ret = twai_read_alerts(&alerts, pdMS_TO_TICKS(HAL_ESP32_CAN_ALERT_WAIT_MS));
        if (ret == ESP_OK) {
            _handle_alerts(alerts);
        }

        // Always try to drain both sides after alert wait. This helps smooth micro-jitter.
        _drain_rx_queue();
        _drain_tx_queue();
        _signal_event();
    }
}

void CANIface::flush_tx()
{
    if (!_initialized || !_driver_installed) {
        return;
    }

    const uint64_t deadline = AP_HAL::micros64() + 1000000ULL;
    while (AP_HAL::micros64() < deadline) {
        _drain_tx_queue();
        hal.scheduler->delay_microseconds(1000);
    }
}

void CANIface::clear_rx()
{
    WITH_SEMAPHORE(sem);
    _rx_queue.clear();
    if (_driver_installed) {
        (void)twai_clear_receive_queue();
    }
}

#if !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    twai_status_info_t status {};
    memset(&status, 0, sizeof(status));
    if (_driver_installed) {
        (void)twai_get_status_info(&status);
    }

    str.printf("initialized:    %u\n"
               "driver:         %u\n"
               "bus_off:        %u\n"
               "bitrate:        %lu\n"
               "tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_overflow:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "busoff_err:     %lu\n"
               "last_tx_us:     %llu\n"
               "twai_state:     %u\n"
               "twai_msgs_to_tx:%lu\n"
               "twai_msgs_to_rx:%lu\n"
               "twai_tx_err:    %lu\n"
               "twai_rx_err:    %lu\n"
               "twai_tx_failed: %lu\n"
               "twai_rx_missed: %lu\n"
               "twai_rx_overrun:%lu\n"
               "twai_arb_lost:  %lu\n"
               "twai_bus_error: %lu\n"
               "last_alerts:    0x%08lx\n",
               (unsigned)_initialized,
               (unsigned)_driver_installed,
               (unsigned)_bus_off,
               (unsigned long)bitrate_,
               (unsigned long)stats.tx_requests,
               (unsigned long)stats.tx_rejected,
               (unsigned long)stats.tx_overflow,
               (unsigned long)stats.tx_success,
               (unsigned long)stats.tx_timedout,
               (unsigned long)stats.tx_abort,
               (unsigned long)stats.rx_received,
               (unsigned long)stats.rx_overflow,
               (unsigned long)stats.rx_errors,
               (unsigned long)stats.num_busoff_err,
               (unsigned long long)stats.last_transmit_us,
               (unsigned)status.state,
               (unsigned long)status.msgs_to_tx,
               (unsigned long)status.msgs_to_rx,
               (unsigned long)status.tx_error_counter,
               (unsigned long)status.rx_error_counter,
               (unsigned long)status.tx_failed_count,
               (unsigned long)status.rx_missed_count,
               (unsigned long)status.rx_overrun_count,
               (unsigned long)status.arb_lost_count,
               (unsigned long)status.bus_error_count,
               (unsigned long)_last_alerts);
}
#endif

#endif // HAL_NUM_CAN_IFACES
