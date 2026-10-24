/*
 * Experimental raw motor observation codec. GPLv3 or later.
 */
#include "AP_DroneCAN_config.h"

#if AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
#include "AP_DroneCAN_MotorTelemetryCodec.h"
#include <limits>
#include <string.h>

using Codec = AP_DroneCAN_MotorTelemetryCodec;

constexpr uint16_t Codec::EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE;
constexpr uint8_t Codec::VERSION;
constexpr uint8_t Codec::POSITION_LENGTH;
constexpr uint8_t Codec::SPEED_LENGTH;
constexpr uint8_t Codec::MAX_PAYLOAD_LENGTH;

static_assert(Codec::EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE > 32767, "Use a local experimental payload type only");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Motor wire V1 requires IEEE-754 binary32");
static_assert(Codec::POSITION_LENGTH <= Codec::MAX_PAYLOAD_LENGTH && Codec::SPEED_LENGTH <= Codec::MAX_PAYLOAD_LENGTH,
              "Motor wire V1 exceeds the TUNNEL payload");

namespace
{

void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = uint8_t(value);
    p[1] = uint8_t(value >> 8);
}

void put_u32(uint8_t *p, uint32_t value)
{
    for (uint8_t i = 0; i < 4; i++) {
        p[i] = uint8_t(value >> (8 * i));
    }
}

void put_u64(uint8_t *p, uint64_t value)
{
    for (uint8_t i = 0; i < 8; i++) {
        p[i] = uint8_t(value >> (8 * i));
    }
}

uint16_t get_u16(const uint8_t *p)
{
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

uint32_t get_u32(const uint8_t *p)
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < 4; i++) {
        value |= uint32_t(p[i]) << (8 * i);
    }
    return value;
}

uint64_t get_u64(const uint8_t *p)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; i++) {
        value |= uint64_t(p[i]) << (8 * i);
    }
    return value;
}

uint32_t float_bits(const float &value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void get_float(const uint8_t *p, float &value)
{
    const uint32_t bits = get_u32(p);
    memcpy(&value, &bits, sizeof(value));
}

bool finite(const float &value)
{
    return (float_bits(value) & 0x7f800000U) != 0x7f800000U;
}

int32_t get_i32(const uint8_t *p)
{
    const uint32_t bits = get_u32(p);
    // Avoid implementation-defined out-of-range unsigned-to-signed casts.
    return bits <= INT32_MAX ? int32_t(bits) : -1 - int32_t(UINT32_MAX - bits);
}

} // namespace

bool Codec::encode_continuity(Store::Continuity value, uint8_t &wire)
{
    // Explicit mapping: the store enum's memory representation is not a protocol.
    switch (value) {
    case Store::Continuity::FIRST:          wire = 0; return true;
    case Store::Continuity::CONTIGUOUS:     wire = 1; return true;
    case Store::Continuity::SEQUENCE:       wire = 2; return true;
    case Store::Continuity::TIMEOUT:        wire = 3; return true;
    case Store::Continuity::INVALID:        wire = 4; return true;
    case Store::Continuity::ORIGIN:         wire = 5; return true;
    case Store::Continuity::REPRESENTATION: wire = 6; return true;
    case Store::Continuity::RESOLUTION:     wire = 7; return true;
    case Store::Continuity::NODE_RESTART:   wire = 8; return true;
    }
    return false;
}

bool Codec::decode_continuity(uint8_t wire, Store::Continuity &value)
{
    switch (wire) {
    case 0: value = Store::Continuity::FIRST;          return true;
    case 1: value = Store::Continuity::CONTIGUOUS;     return true;
    case 2: value = Store::Continuity::SEQUENCE;       return true;
    case 3: value = Store::Continuity::TIMEOUT;        return true;
    case 4: value = Store::Continuity::INVALID;       return true;
    case 5: value = Store::Continuity::ORIGIN;        return true;
    case 6: value = Store::Continuity::REPRESENTATION; return true;
    case 7: value = Store::Continuity::RESOLUTION;    return true;
    case 8: value = Store::Continuity::NODE_RESTART;  return true;
    }
    return false;
}

Codec::Result Codec::validate(const Record &r)
{
    if (r.type != Type::POSITION && r.type != Type::SPEED) {
        return Result::UNKNOWN_TYPE;
    }
    uint8_t ignored;
    if (r.source_node_id == 0 || r.source_node_id > 127 || r.esc_index > 31 ||
        !encode_continuity(r.sample.continuity, ignored) || !encode_continuity(r.sample.last_break, ignored) ||
        (!r.context.session_tag_present && r.context.session_id != 0) ||
        (!r.context.delivery_counters_valid && (r.context.export_seq != 0 || r.context.superseded_count != 0)) ||
        (!r.sample.received && (r.sample.valid || r.sample.usable || r.sample.fresh || r.sample.baseline_ready)) ||
        (r.sample.usable && !r.sample.valid) ||
        (r.sample.baseline_ready && (!r.sample.usable || !r.sample.fresh)) ||
        r.updated_us > r.snapshot_us || r.sample.timestamp_us > r.snapshot_us) {
        return Result::INVALID_FIELD;
    }
    if (r.type == Type::SPEED) {
        return r.sample.usable && !finite(r.rpm) ? Result::INVALID_FIELD : Result::OK;
    }
    switch (r.position.representation) {
    case Representation::TICKS:
        return r.sample.usable && r.position.ticks_per_revolution == 0 ? Result::INVALID_FIELD : Result::OK;
    case Representation::REVOLUTIONS:
        return r.sample.usable && !finite(r.position.revolutions) ? Result::INVALID_FIELD : Result::OK;
    }
    return Result::UNKNOWN_REPRESENTATION;
}

Codec::Result Codec::encode(const Record &r, uint8_t *p, size_t capacity, uint8_t &length)
{
    if (p == nullptr) {
        return Result::INVALID_ARGUMENT;
    }
    const Result result = validate(r);
    if (result != Result::OK) {
        return result;
    }
    const uint8_t size = r.type == Type::POSITION ? POSITION_LENGTH : SPEED_LENGTH;
    if (capacity < size) {
        return Result::BAD_LENGTH;
    }
    p[0] = 0x4d;
    p[1] = 0x54;
    p[2] = VERSION;
    p[3] = r.type == Type::POSITION ? 1 : 2;
    p[4] = size;
    p[5] = 0; // reserved
    p[6] = r.context.driver_index;
    p[7] = r.source_node_id;
    p[8] = r.esc_index;
    p[9] = uint8_t(r.sample.received) | (uint8_t(r.sample.valid) << 1) | (uint8_t(r.sample.usable) << 2) |
           (uint8_t(r.sample.fresh) << 3) | (uint8_t(r.sample.baseline_ready) << 4);
    p[10] = r.sample.sample_seq;
    encode_continuity(r.sample.continuity, p[11]);
    encode_continuity(r.sample.last_break, p[12]);
    p[13] = uint8_t(r.context.session_tag_present) | (uint8_t(r.context.delivery_counters_valid) << 1);
    put_u16(p + 14, 0); // reserved
    put_u64(p + 16, r.context.session_id);
    put_u64(p + 24, r.snapshot_us);
    put_u64(p + 32, r.updated_us);
    put_u64(p + 40, r.sample.timestamp_us);
    put_u32(p + 48, r.identity_generation);
    put_u32(p + 52, r.sample.counters.accepted);
    put_u32(p + 56, r.sample.counters.invalid);
    put_u32(p + 60, r.sample.counters.sequence_rejected);
    put_u32(p + 64, r.sample.counters.continuity_breaks);
    put_u32(p + 68, r.statistics.allocations);
    put_u32(p + 72, r.statistics.position_saturation);
    put_u32(p + 76, r.statistics.speed_saturation);
    put_u32(p + 80, r.statistics.invalid_identity);
    put_u32(p + 84, r.context.export_seq);
    put_u32(p + 88, r.context.superseded_count);
    if (r.type == Type::POSITION) {
        put_u32(p + 92, r.position.origin_id);
        p[96] = r.position.representation == Representation::TICKS ? 0 : 1;
        p[97] = 0;
        put_u16(p + 98, r.position.representation == Representation::TICKS ? r.position.ticks_per_revolution : 0);
        put_u32(p + 100, r.position.representation == Representation::TICKS ?
                uint32_t(r.position.ticks) : float_bits(r.position.revolutions));
    } else {
        put_u32(p + 92, float_bits(r.rpm));
    }
    length = size;
    return Result::OK;
}

Codec::Result Codec::decode(const uint8_t *p, size_t length, Record &record)
{
    if (p == nullptr) {
        return Result::INVALID_ARGUMENT;
    }
    if (length < 5 || length > MAX_PAYLOAD_LENGTH) {
        return Result::BAD_LENGTH;
    }
    if (p[0] != 0x4d || p[1] != 0x54) {
        return Result::BAD_MAGIC;
    }
    if (p[2] != VERSION) {
        return Result::UNKNOWN_VERSION;
    }
    if (p[3] != 1 && p[3] != 2) {
        return Result::UNKNOWN_TYPE;
    }
    const uint8_t size = p[3] == 1 ? POSITION_LENGTH : SPEED_LENGTH;
    if (length != size || p[4] != size) {
        return Result::BAD_LENGTH;
    }
    if (p[5] != 0 || p[14] != 0 || p[15] != 0 || (p[9] & ~0x1fU) != 0 || (p[13] & ~0x03U) != 0) {
        return Result::INVALID_FIELD;
    }
    Record r {};
    r.type = p[3] == 1 ? Type::POSITION : Type::SPEED;
    r.context.driver_index = p[6];
    r.source_node_id = p[7];
    r.esc_index = p[8];
    r.sample.received = (p[9] & 1U) != 0;
    r.sample.valid = (p[9] & 2U) != 0;
    r.sample.usable = (p[9] & 4U) != 0;
    r.sample.fresh = (p[9] & 8U) != 0;
    r.sample.baseline_ready = (p[9] & 16U) != 0;
    r.sample.sample_seq = p[10];
    if (!decode_continuity(p[11], r.sample.continuity) || !decode_continuity(p[12], r.sample.last_break)) {
        return Result::INVALID_FIELD;
    }
    r.context.session_tag_present = (p[13] & 1U) != 0;
    r.context.delivery_counters_valid = (p[13] & 2U) != 0;
    r.context.session_id = get_u64(p + 16);
    r.snapshot_us = get_u64(p + 24);
    r.updated_us = get_u64(p + 32);
    r.sample.timestamp_us = get_u64(p + 40);
    r.identity_generation = get_u32(p + 48);
    r.sample.counters.accepted = get_u32(p + 52);
    r.sample.counters.invalid = get_u32(p + 56);
    r.sample.counters.sequence_rejected = get_u32(p + 60);
    r.sample.counters.continuity_breaks = get_u32(p + 64);
    r.statistics.allocations = get_u32(p + 68);
    r.statistics.position_saturation = get_u32(p + 72);
    r.statistics.speed_saturation = get_u32(p + 76);
    r.statistics.invalid_identity = get_u32(p + 80);
    r.context.export_seq = get_u32(p + 84);
    r.context.superseded_count = get_u32(p + 88);
    if (r.type == Type::POSITION) {
        r.position.origin_id = get_u32(p + 92);
        if (p[96] > 1) {
            return Result::UNKNOWN_REPRESENTATION;
        }
        if (p[97] != 0 || (p[96] == 1 && get_u16(p + 98) != 0)) {
            return Result::INVALID_FIELD;
        }
        r.position.representation = p[96] == 0 ? Representation::TICKS : Representation::REVOLUTIONS;
        if (r.position.representation == Representation::TICKS) {
            r.position.ticks_per_revolution = get_u16(p + 98);
            r.position.ticks = get_i32(p + 100);
        } else {
            get_float(p + 100, r.position.revolutions);
        }
    } else {
        get_float(p + 92, r.rpm);
    }
    const Result result = validate(r);
    if (result == Result::OK) {
        record = r;
    }
    return result;
}

Codec::Result Codec::from_snapshot(const Store::Snapshot &snapshot, uint8_t slot, Type type,
                                   const Context &context, Record &record)
{
    if (slot >= Store::CAPACITY || (snapshot.occupied_mask & (1U << slot)) == 0) {
        return Result::INVALID_ARGUMENT;
    }
    if (type != Type::POSITION && type != Type::SPEED) {
        return Result::UNKNOWN_TYPE;
    }
    const auto &stream = snapshot.streams[slot];
    Record r {};
    r.type = type;
    r.context = context;
    r.snapshot_us = snapshot.timestamp_us;
    r.updated_us = snapshot.updated_us;
    r.identity_generation = stream.identity_generation;
    r.source_node_id = stream.source_node_id;
    r.esc_index = stream.esc_index;
    r.statistics = snapshot.statistics;
    if (type == Type::POSITION) {
        r.sample = stream.position;
        r.position.origin_id = stream.position.origin_id;
        switch (stream.position.value.union_tag) {
        case UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_TICKS:
            r.position.representation = Representation::TICKS;
            r.position.ticks = stream.position.value.ticks.ticks;
            r.position.ticks_per_revolution = stream.position.value.ticks.ticks_per_revolution;
            break;
        case UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS:
            r.position.representation = Representation::REVOLUTIONS;
            memcpy(&r.position.revolutions, &stream.position.value.revolutions, sizeof(float));
            break;
        default:
            return Result::UNKNOWN_REPRESENTATION;
        }
    } else {
        r.sample = stream.speed;
        memcpy(&r.rpm, &stream.speed.rpm, sizeof(float));
    }
    const Result result = validate(r);
    if (result == Result::OK) {
        record = r;
    }
    return result;
}
#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
