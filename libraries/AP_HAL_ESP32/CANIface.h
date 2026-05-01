/*
 * Prototype ESP32 TWAI CAN backend for ArduPilot
 *
 * Classic CAN only (8-byte payload max)
 * No CAN FD support in this prototype
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "driver/twai.h"

#if HAL_NUM_CAN_IFACES

#ifndef HAL_ESP32_CAN_TX_QUEUE_SIZE
#define HAL_ESP32_CAN_TX_QUEUE_SIZE 64
#endif

#ifndef HAL_ESP32_CAN_RX_QUEUE_SIZE
#define HAL_ESP32_CAN_RX_QUEUE_SIZE 128
#endif

#ifndef HAL_ESP32_TWAI_TX_QUEUE_LEN
#define HAL_ESP32_TWAI_TX_QUEUE_LEN 8
#endif

#ifndef HAL_ESP32_TWAI_RX_QUEUE_LEN
#define HAL_ESP32_TWAI_RX_QUEUE_LEN 32
#endif

#ifndef HAL_ESP32_CAN_WORKER_STACK
#define HAL_ESP32_CAN_WORKER_STACK 2048
#endif

#ifndef HAL_ESP32_CAN_ALERT_WAIT_MS
#define HAL_ESP32_CAN_ALERT_WAIT_MS 10
#endif

#ifndef HAL_ESP32_CAN_TX_PIN
#define HAL_ESP32_CAN_TX_PIN TWAI_IO_UNUSED
#endif

#ifndef HAL_ESP32_CAN_RX_PIN
#define HAL_ESP32_CAN_RX_PIN TWAI_IO_UNUSED
#endif

namespace ESP32
{

class CANIface : public AP_HAL::CANIface
{
public:
    explicit CANIface(uint8_t index);
    CANIface();
    ~CANIface() override;

    static uint8_t next_interface;

    bool init(const uint32_t bitrate) override;
    bool init(const uint32_t bitrate, const uint32_t fdbitrate) override
    {
        return init(bitrate);
    }

    int16_t send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    int16_t receive(AP_HAL::CANFrame &out_frame, uint64_t &out_timestamp_us,
                    CanIOFlags &out_flags) override;

    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame * const pending_tx,
                uint64_t blocking_deadline) override;

    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

    uint32_t getErrorCount() const override;

    bool is_initialized() const override
    {
        return _initialized;
    }

    bool is_busoff() const override
    {
        return _bus_off;
    }

    void flush_tx() override;
    void clear_rx() override;

#if !defined(HAL_BOOTLOADER_BUILD)
    void get_stats(ExpandingString &str) override;
    const bus_stats_t *get_statistics(void) const override
    {
        return &stats;
    }
#endif

protected:
    bool add_to_rx_queue(const CanRxItem &rx_item) override;
    int8_t get_iface_num() const override
    {
        return _self_index;
    }

private:
    bool _get_timing_config(uint32_t bitrate, twai_timing_config_t &timing_cfg) const;
    bool _to_twai_message(const AP_HAL::CANFrame &frame, twai_message_t &msg,
                          CanIOFlags flags) const;
    bool _from_twai_message(const twai_message_t &msg, AP_HAL::CANFrame &frame) const;

    void _check_available(bool &read, bool &write,
                          const AP_HAL::CANFrame *pending_tx) const;
    void _signal_event(void);
    void _drain_rx_queue(void);
    void _drain_tx_queue(void);
    void _handle_alerts(uint32_t alerts);
    void _worker(void);

    uint8_t _self_index;
    bool _initialized = false;
    bool _driver_installed = false;
    bool _bus_off = false;
    uint32_t _tx_frame_counter = 0;
    uint32_t _last_alerts = 0;

    AP_HAL::BinarySemaphore *sem_handle = nullptr;

    ObjectArray<CanTxItem> _tx_queue{HAL_ESP32_CAN_TX_QUEUE_SIZE};
    ObjectArray<CanRxItem> _rx_queue{HAL_ESP32_CAN_RX_QUEUE_SIZE};

    HAL_Semaphore sem;

    AP_HAL::CANIface::bus_stats_t stats {};
    char _worker_name[16] {};
};

} // namespace ESP32

#endif // HAL_NUM_CAN_IFACES
