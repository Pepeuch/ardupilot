/*
 * Raw DroneCAN motor telemetry. GPLv3 or later.
 */
#pragma once

#include <stdint.h>
#include <uavcan.equipment.esc.MotorPosition.h>
#include <com.mowglinext.esc.MotorSpeed.h>

// One store per AP_DroneCAN instance: driver identity is implicit. The caller
// serializes all access. No ESC offset, physical bus identity or odometry here.
class AP_DroneCAN_MotorTelemetry
{
public:
    static constexpr uint8_t CAPACITY = 8;
    static constexpr uint64_t TIMEOUT_US = 500000;

    enum class Continuity : uint8_t {
        FIRST,
        CONTIGUOUS,
        SEQUENCE,
        TIMEOUT,
        INVALID,
        ORIGIN,
        REPRESENTATION,
        RESOLUTION,
    };

    struct Sample {
        uint64_t timestamp_us;
        uint8_t sample_seq;
        bool received;
        bool valid;       // publisher's valid bit
        bool usable;      // valid and numerically well formed
        bool fresh;       // recomputed by get(), never a promise of sensor health
        // Only temporal/sequence/metadata continuity. Physical plausibility and
        // a bound on publication rate are NOT known; no delta is certified.
        Continuity continuity;
    };

    struct Position : Sample {
        uint32_t origin_id;
        uavcan_equipment_esc_MotorPositionValue value;
    };

    struct Speed : Sample {
        float rpm;
    };

    struct Stream {
        uint8_t source_node_id;
        uint8_t esc_index; // raw 0..31, never esc_index + esc_offset
        Position position;
        Speed speed;
    };

    // false: malformed identity, full store, duplicate or non-forward sequence.
    // Rejected sequences preserve the raw sample and its timestamp, but break
    // continuity. Active identities are never evicted. Fully expired entries
    // may be reused, with BOTH samples cleared.
    bool receive(uint8_t source_node_id, const uavcan_equipment_esc_MotorPosition &msg, uint64_t now_us);
    bool receive(uint8_t source_node_id, const com_mowglinext_esc_MotorSpeed &msg, uint64_t now_us);
    bool get(uint8_t source_node_id, uint8_t esc_index, uint64_t now_us, Stream &out) const;

private:
    Stream _streams[CAPACITY] {};

    enum class Sequence : uint8_t { FORWARD, DUPLICATE, AMBIGUOUS, OLD };
    static Sequence classify_sequence(uint8_t previous, uint8_t incoming);
    static bool accept_sequence(Sample &previous, uint8_t incoming);
    static bool fresh(const Sample &sample, uint64_t now_us);
    static Continuity continuity(const Sample &previous, uint8_t seq, bool usable, uint64_t now_us);
    Stream *find_or_allocate(uint8_t source_node_id, uint8_t esc_index, uint64_t now_us);
};
