/*
 * Experimental raw motor observation codec. GPLv3 or later.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "AP_DroneCAN_MotorTelemetry.h"

// No MAVLink dependency or transmission. See MotorTelemetryWireV1.md for the
// complete byte contract, session limitations and snapshot replacement policy.
class AP_DroneCAN_MotorTelemetryCodec
{
public:
    using Store = AP_DroneCAN_MotorTelemetry;

    // Private experiment only: not registered, reserved or globally unique.
    static constexpr uint16_t EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE = 32769;
    static constexpr uint8_t VERSION = 1;
    static constexpr uint8_t POSITION_LENGTH = 104;
    static constexpr uint8_t SPEED_LENGTH = 96;
    static constexpr uint8_t MAX_PAYLOAD_LENGTH = 128;

    enum class Type : uint8_t { POSITION = 1, SPEED = 2 };
    enum class Representation : uint8_t { TICKS = 0, REVOLUTIONS = 1 };
    enum class Result : uint8_t {
        OK,
        INVALID_ARGUMENT,
        BAD_MAGIC,
        UNKNOWN_VERSION,
        UNKNOWN_TYPE,
        BAD_LENGTH,
        UNKNOWN_REPRESENTATION,
        INVALID_FIELD,
    };

    struct Context {
        uint64_t session_id; // caller-owned correlation tag, not proof of uniqueness
        uint32_t export_seq;
        uint32_t superseded_count;
        uint8_t driver_index;
        bool session_tag_present;
        bool delivery_counters_valid;
    };

    struct Position {
        uint32_t origin_id;
        Representation representation;
        int32_t ticks;
        uint16_t ticks_per_revolution;
        float revolutions;
    };

    // A logical value object, NOT a packed wire struct. Inactive fields are
    // ignored on encode and zeroed on decode. A missing sample can be reported.
    struct Record {
        Type type;
        Context context;
        uint64_t snapshot_us;
        uint64_t updated_us;
        uint32_t identity_generation;
        uint8_t source_node_id;
        uint8_t esc_index;
        Store::Sample sample;
        Store::Statistics statistics;
        Position position;
        float rpm;
    };

    // Failure leaves output arguments unchanged. Encode requires capacity >=
    // the selected fixed record length; decode requires the exact wire length.
    static Result encode(const Record &record, uint8_t *payload, size_t capacity, uint8_t &length);
    static Result decode(const uint8_t *payload, size_t length, Record &record);
    static Result from_snapshot(const Store::Snapshot &snapshot, uint8_t slot, Type type,
                                const Context &context, Record &record);

private:
    static Result validate(const Record &record);
    static bool encode_continuity(Store::Continuity value, uint8_t &wire);
    static bool decode_continuity(uint8_t wire, Store::Continuity &value);
};
