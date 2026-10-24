/*
 * Raw DroneCAN motor telemetry. GPLv3 or later.
 */
#include "AP_DroneCAN_config.h"

#if AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
#include "AP_DroneCAN_MotorTelemetry.h"
#include <AP_Math/AP_Math.h>

AP_DroneCAN_MotorTelemetry::Sequence AP_DroneCAN_MotorTelemetry::classify_sequence(uint8_t previous, uint8_t incoming)
{
    const uint8_t distance = uint8_t(incoming - previous);
    if (distance == 0) {
        return Sequence::DUPLICATE;
    }
    if (distance == 128) {
        return Sequence::AMBIGUOUS;
    }
    return distance < 128 ? Sequence::FORWARD : Sequence::OLD;
}

void AP_DroneCAN_MotorTelemetry::break_continuity(Sample &sample, Continuity reason)
{
    sample.counters.continuity_breaks++;
    sample.last_break = reason;
    sample.continuity = reason;
    sample.baseline_ready = false;
}

bool AP_DroneCAN_MotorTelemetry::accept_sequence(Sample &previous, uint8_t incoming, bool reset)
{
    if (reset || !previous.received || classify_sequence(previous.sample_seq, incoming) == Sequence::FORWARD) {
        return true;
    }
    // Retain the raw baseline and its timestamp, even after timeout.
    previous.counters.sequence_rejected++;
    break_continuity(previous, Continuity::SEQUENCE);
    return false;
}

bool AP_DroneCAN_MotorTelemetry::fresh(const Sample &sample, uint64_t now_us)
{
    return sample.received && now_us >= sample.timestamp_us && now_us - sample.timestamp_us < TIMEOUT_US;
}

void AP_DroneCAN_MotorTelemetry::update(uint64_t now_us)
{
    _updated_us = now_us;
    for (auto &entry : _entries) {
        for (Sample *sample : {
                 static_cast<Sample *>(&entry.stream.position), static_cast<Sample *>(&entry.stream.speed)
             }) {
            if (sample->fresh && !fresh(*sample, now_us)) {
                sample->fresh = false;
                break_continuity(*sample, Continuity::TIMEOUT);
            }
        }
    }
}

AP_DroneCAN_MotorTelemetry::Continuity AP_DroneCAN_MotorTelemetry::continuity(
    const Sample &previous, uint8_t seq, bool usable, uint64_t now_us, bool reset)
{
    if (!usable) {
        return Continuity::INVALID;
    }
    if (!previous.received || reset) {
        return Continuity::FIRST;
    }
    if (!fresh(previous, now_us)) {
        return Continuity::TIMEOUT;
    }
    if (uint8_t(seq - previous.sample_seq) != 1) {
        return Continuity::SEQUENCE;
    }
    if (!previous.baseline_ready) {
        return Continuity::FIRST;
    }
    return Continuity::CONTIGUOUS;
}

AP_DroneCAN_MotorTelemetry::Entry *AP_DroneCAN_MotorTelemetry::find_or_allocate(
    uint8_t source_node_id, uint8_t esc_index, bool position, uint64_t now_us)
{
    if (source_node_id == 0 || source_node_id > 127 || esc_index > 31) {
        _statistics.invalid_identity++;
        return nullptr;
    }
    Entry *available = nullptr;
    for (auto &entry : _entries) {
        const auto &stream = entry.stream;
        if (stream.source_node_id == source_node_id && stream.esc_index == esc_index) {
            return &entry;
        }
        if (available == nullptr && !fresh(stream.position, now_us) && !fresh(stream.speed, now_us)) {
            available = &entry;
        }
    }
    if (available == nullptr) {
        if (position) {
            _statistics.position_saturation++;
        } else {
            _statistics.speed_saturation++;
        }
        return nullptr;
    }
    *available = {};
    available->stream.identity_generation = ++_statistics.allocations;
    available->stream.source_node_id = source_node_id;
    available->stream.esc_index = esc_index;
    return available;
}

bool AP_DroneCAN_MotorTelemetry::receive(uint8_t source_node_id,
        const uavcan_equipment_esc_MotorPosition &msg, uint64_t now_us)
{
    update(now_us);
    Entry *entry = find_or_allocate(source_node_id, msg.esc_index, true, now_us);
    if (entry == nullptr) {
        return false;
    }
    auto &p = entry->stream.position;
    const bool ticks = msg.position.union_tag == UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_TICKS;
    const bool revolutions = msg.position.union_tag == UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    const bool usable = msg.valid && ((ticks && msg.position.ticks.ticks_per_revolution != 0) ||
                                      (revolutions && isfinite(msg.position.revolutions)));
    if (!usable) {
        p.counters.invalid++;
    }
    const bool new_origin = p.received && p.origin_id != msg.origin_id;
    const bool reset = entry->position_sequence_reset || new_origin;
    if (!accept_sequence(p, msg.sample_seq, reset)) {
        return false;
    }
    Continuity state = continuity(p, msg.sample_seq, usable, now_us, reset);
    // One break event per admitted transfer, even if several causes coincide.
    // Expiry and an observed node restart are separate maintenance events.
    Continuity reason = state;
    if (usable && new_origin) {
        state = reason = Continuity::ORIGIN;
    } else if (usable && p.received && p.usable) {
        if (p.value.union_tag != msg.position.union_tag) {
            state = reason = Continuity::REPRESENTATION;
        } else if (ticks && p.value.ticks.ticks_per_revolution != msg.position.ticks.ticks_per_revolution) {
            state = reason = Continuity::RESOLUTION;
        }
    }
    if (reason == Continuity::INVALID || reason == Continuity::ORIGIN || reason == Continuity::REPRESENTATION ||
        reason == Continuity::RESOLUTION || reason == Continuity::SEQUENCE) {
        break_continuity(p, reason);
    }
    p.counters.accepted++;
    p.timestamp_us = now_us;
    p.sample_seq = msg.sample_seq;
    p.received = true;
    p.valid = msg.valid;
    p.usable = usable;
    p.fresh = true;
    p.baseline_ready = usable && state != Continuity::SEQUENCE;
    p.continuity = state;
    p.origin_id = msg.origin_id;
    p.value = msg.position;
    entry->position_sequence_reset = false;
    return true;
}

bool AP_DroneCAN_MotorTelemetry::receive(uint8_t source_node_id,
        const com_mowglinext_esc_MotorSpeed &msg, uint64_t now_us)
{
    update(now_us);
    Entry *entry = find_or_allocate(source_node_id, msg.esc_index, false, now_us);
    if (entry == nullptr) {
        return false;
    }
    auto &s = entry->stream.speed;
    const bool usable = msg.valid && isfinite(msg.rpm);
    if (!usable) {
        s.counters.invalid++;
    }
    if (!accept_sequence(s, msg.sample_seq, entry->speed_sequence_reset)) {
        return false;
    }
    const Continuity state = continuity(s, msg.sample_seq, usable, now_us, entry->speed_sequence_reset);
    if (state == Continuity::INVALID || state == Continuity::SEQUENCE) {
        break_continuity(s, state);
    }
    s.counters.accepted++;
    s.timestamp_us = now_us;
    s.sample_seq = msg.sample_seq;
    s.received = true;
    s.valid = msg.valid;
    s.usable = usable;
    s.fresh = true;
    s.baseline_ready = usable && state != Continuity::SEQUENCE;
    s.continuity = state;
    s.rpm = msg.rpm;
    entry->speed_sequence_reset = false;
    return true;
}

void AP_DroneCAN_MotorTelemetry::receive(uint8_t source_node_id,
        const uavcan_protocol_NodeStatus &msg, uint64_t now_us)
{
    update(now_us);
    bool restarted = false;
    for (const auto &entry : _entries) {
        if (entry.stream.source_node_id == source_node_id && entry.node_status_received &&
            msg.uptime_sec < entry.node_uptime_sec) {
            restarted = true;
            break;
        }
    }
    for (auto &entry : _entries) {
        if (source_node_id == 0 || entry.stream.source_node_id != source_node_id) {
            continue;
        }
        entry.node_status_received = true;
        entry.node_uptime_sec = msg.uptime_sec;
        if (!restarted) {
            continue;
        }
        if (entry.stream.position.received) {
            break_continuity(entry.stream.position, Continuity::NODE_RESTART);
            entry.position_sequence_reset = true;
        }
        if (entry.stream.speed.received) {
            break_continuity(entry.stream.speed, Continuity::NODE_RESTART);
            entry.speed_sequence_reset = true;
        }
    }
}

void AP_DroneCAN_MotorTelemetry::copy_stream(const Stream &stream, uint64_t now_us, Stream &out)
{
    out = stream;
    for (Sample *sample : {
             static_cast<Sample *>(&out.position), static_cast<Sample *>(&out.speed)
         }) {
        sample->fresh = fresh(*sample, now_us);
        if (sample->received && !sample->fresh) {
            sample->baseline_ready = false;
            sample->continuity = Continuity::TIMEOUT;
        }
    }
}

bool AP_DroneCAN_MotorTelemetry::get(uint8_t source_node_id, uint8_t esc_index,
                                     uint64_t now_us, Stream &out) const
{
    if (source_node_id == 0) {
        return false;
    }
    for (const auto &entry : _entries) {
        if (entry.stream.source_node_id == source_node_id && entry.stream.esc_index == esc_index) {
            copy_stream(entry.stream, now_us, out);
            return true;
        }
    }
    return false;
}

void AP_DroneCAN_MotorTelemetry::get_snapshot(uint64_t now_us, Snapshot &out) const
{
    out.timestamp_us = now_us;
    out.updated_us = _updated_us;
    out.statistics = _statistics;
    out.occupied_mask = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) {
        copy_stream(_entries[i].stream, now_us, out.streams[i]);
        if (_entries[i].stream.source_node_id != 0) {
            out.occupied_mask |= 1U << i;
        }
    }
}
#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
