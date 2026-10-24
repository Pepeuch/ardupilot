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

bool AP_DroneCAN_MotorTelemetry::accept_sequence(Sample &previous, uint8_t incoming)
{
    if (!previous.received || classify_sequence(previous.sample_seq, incoming) == Sequence::FORWARD) {
        return true;
    }
    // Keep the last accepted payload/sequence/time even after timeout. Without
    // a publication-rate bound these classifications are only presumptive:
    // reject the ambiguous half-range and do not let replays refresh data.
    previous.continuity = Continuity::SEQUENCE;
    return false;
}

bool AP_DroneCAN_MotorTelemetry::fresh(const Sample &sample, uint64_t now_us)
{
    return sample.received && now_us >= sample.timestamp_us && now_us - sample.timestamp_us < TIMEOUT_US;
}

AP_DroneCAN_MotorTelemetry::Continuity AP_DroneCAN_MotorTelemetry::continuity(
    const Sample &previous, uint8_t seq, bool usable, uint64_t now_us)
{
    if (!usable) {
        return Continuity::INVALID;
    }
    if (!previous.received) {
        return Continuity::FIRST;
    }
    if (!fresh(previous, now_us)) {
        return Continuity::TIMEOUT;
    }
    if (!previous.usable) {
        return Continuity::FIRST;
    }
    if (uint8_t(seq - previous.sample_seq) != 1) {
        return Continuity::SEQUENCE;
    }
    if (previous.continuity == Continuity::SEQUENCE) {
        return Continuity::FIRST;
    }
    return Continuity::CONTIGUOUS;
}

AP_DroneCAN_MotorTelemetry::Stream *AP_DroneCAN_MotorTelemetry::find_or_allocate(
    uint8_t source_node_id, uint8_t esc_index, uint64_t now_us)
{
    if (source_node_id == 0 || source_node_id > 127 || esc_index > 31) {
        return nullptr;
    }
    Stream *available = nullptr;
    for (auto &stream : _streams) {
        if (stream.source_node_id == source_node_id && stream.esc_index == esc_index) {
            return &stream;
        }
        if (available == nullptr && !fresh(stream.position, now_us) && !fresh(stream.speed, now_us)) {
            available = &stream;
        }
    }
    if (available != nullptr) {
        *available = {};
        available->source_node_id = source_node_id;
        available->esc_index = esc_index;
    }
    return available;
}

bool AP_DroneCAN_MotorTelemetry::receive(uint8_t source_node_id,
        const uavcan_equipment_esc_MotorPosition &msg, uint64_t now_us)
{
    Stream *stream = find_or_allocate(source_node_id, msg.esc_index, now_us);
    if (stream == nullptr) {
        return false;
    }
    auto &p = stream->position;
    const bool new_origin = p.received && p.origin_id != msg.origin_id;
    if (!new_origin && !accept_sequence(p, msg.sample_seq)) {
        return false;
    }
    const bool ticks = msg.position.union_tag == UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_TICKS;
    const bool revolutions = msg.position.union_tag == UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    const bool usable = msg.valid && ((ticks && msg.position.ticks.ticks_per_revolution != 0) ||
                                      (revolutions && isfinite(msg.position.revolutions)));
    Continuity state = continuity(p, msg.sample_seq, usable, now_us);
    if (usable && new_origin) {
        state = Continuity::ORIGIN;
    } else if (state == Continuity::CONTIGUOUS) {
        if (p.value.union_tag != msg.position.union_tag) {
            state = Continuity::REPRESENTATION;
        } else if (ticks && p.value.ticks.ticks_per_revolution != msg.position.ticks.ticks_per_revolution) {
            state = Continuity::RESOLUTION;
        }
    }
    // Only admitted transfers replace the raw baseline. After a sequence break
    // the next admitted sample establishes a baseline, without continuity.
    p.timestamp_us = now_us;
    p.sample_seq = msg.sample_seq;
    p.received = true;
    p.valid = msg.valid;
    p.usable = usable;
    p.fresh = true;
    p.continuity = state;
    p.origin_id = msg.origin_id;
    p.value = msg.position;
    return true;
}

bool AP_DroneCAN_MotorTelemetry::receive(uint8_t source_node_id,
        const com_mowglinext_esc_MotorSpeed &msg, uint64_t now_us)
{
    Stream *stream = find_or_allocate(source_node_id, msg.esc_index, now_us);
    if (stream == nullptr) {
        return false;
    }
    auto &s = stream->speed;
    if (!accept_sequence(s, msg.sample_seq)) {
        return false;
    }
    const bool usable = msg.valid && isfinite(msg.rpm);
    const Continuity state = continuity(s, msg.sample_seq, usable, now_us);
    s.timestamp_us = now_us;
    s.sample_seq = msg.sample_seq;
    s.received = true;
    s.valid = msg.valid;
    s.usable = usable;
    s.fresh = true;
    s.continuity = state;
    s.rpm = msg.rpm;
    return true;
}

bool AP_DroneCAN_MotorTelemetry::get(uint8_t source_node_id, uint8_t esc_index,
                                     uint64_t now_us, Stream &out) const
{
    if (source_node_id == 0) {
        return false;
    }
    for (const auto &stream : _streams) {
        if (stream.source_node_id != source_node_id || stream.esc_index != esc_index) {
            continue;
        }
        out = stream;
        out.position.fresh = fresh(out.position, now_us);
        out.speed.fresh = fresh(out.speed, now_us);
        if (out.position.received && !out.position.fresh) {
            out.position.continuity = Continuity::TIMEOUT;
        }
        if (out.speed.received && !out.speed.fresh) {
            out.speed.continuity = Continuity::TIMEOUT;
        }
        return true;
    }
    return false;
}
#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
