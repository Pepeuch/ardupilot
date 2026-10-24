/*
 * Raw DroneCAN motor telemetry. GPLv3 or later.
 */
#pragma once

#include <stdint.h>
#include <uavcan.equipment.esc.MotorPosition.h>
#include <com.mowglinext.esc.MotorSpeed.h>
#include <uavcan.protocol.NodeStatus.h>

// One store per AP_DroneCAN instance: driver identity is implicit. The caller
// serializes all access, including maintenance. No physical conversions here.
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
        NODE_RESTART,
    };

    // All counters wrap modulo 2^32. No reset within an allocated identity.
    // Their differences require the same driver session and identity generation;
    // equality cannot rule out a complete counter cycle.
    struct Counters {
        uint32_t accepted;          // admitted transfers, including invalid ones
        uint32_t invalid;           // invalid payloads, including sequence rejects
        uint32_t sequence_rejected; // duplicate, half-range ambiguous, presumed old
        uint32_t continuity_breaks; // also the continuity generation
    };

    struct Sample {
        uint64_t timestamp_us;
        Counters counters;
        uint8_t sample_seq;
        bool received;
        bool valid;       // publisher's valid bit
        bool usable;      // valid and numerically well formed
        bool fresh;       // recomputed by getters without creating events
        // Candidate baseline in this generation, never a physical guarantee.
        // The retained raw measurement can predate an observed node restart.
        bool baseline_ready;
        // Only temporal/sequence/metadata continuity. Physical plausibility and
        // a bound on publication rate are NOT known; no delta is certified.
        Continuity continuity;
        Continuity last_break; // retained even after FIRST / CONTIGUOUS

        uint32_t continuity_generation() const
        {
            return counters.continuity_breaks;
        }
    };

    struct Position : Sample {
        uint32_t origin_id;
        uavcan_equipment_esc_MotorPositionValue value;
    };

    struct Speed : Sample {
        float rpm;
    };

    struct Stream {
        uint32_t identity_generation; // new driver-wide allocation serial
        uint8_t source_node_id;
        uint8_t esc_index; // raw 0..31, never esc_index + esc_offset
        Position position;
        Speed speed;
    };

    struct Statistics {
        uint32_t allocations; // also the driver-wide identity generation
        uint32_t position_saturation;
        uint32_t speed_saturation;
        uint32_t invalid_identity; // malformed motor identities, not NodeStatus
    };

    struct Snapshot {
        uint64_t timestamp_us;
        uint64_t updated_us; // most recent maintenance, not a sample timestamp
        Statistics statistics;
        uint8_t occupied_mask;
        // Assigned slots, including stale entries until reuse. Use occupied_mask
        // to enumerate; freshness is independent for each sample type.
        Stream streams[CAPACITY];
    };

    // false: malformed identity, full store, duplicate or non-forward sequence.
    // Rejections preserve the raw sample and timestamp. Invalid payloads are
    // counted only when an identity has a slot; saturation is counted per driver.
    bool receive(uint8_t source_node_id, const uavcan_equipment_esc_MotorPosition &msg, uint64_t now_us);
    bool receive(uint8_t source_node_id, const com_mowglinext_esc_MotorSpeed &msg, uint64_t now_us);
    void receive(uint8_t source_node_id, const uavcan_protocol_NodeStatus &msg, uint64_t now_us);

    // Call independently of readers, and on reception, to record each expiry
    // once. A getter may observe stale data before the next maintenance tick;
    // it never changes counters or generations.
    void update(uint64_t now_us);
    bool get(uint8_t source_node_id, uint8_t esc_index, uint64_t now_us, Stream &out) const;
    void get_snapshot(uint64_t now_us, Snapshot &out) const;

private:
    friend class AP_DroneCAN_MotorTelemetry_Test;

    struct Entry {
        Stream stream;
        uint32_t node_uptime_sec;
        bool node_status_received;
        bool position_sequence_reset;
        bool speed_sequence_reset;
    };

    Entry _entries[CAPACITY] {};
    Statistics _statistics {};
    uint64_t _updated_us {};

    enum class Sequence : uint8_t { FORWARD, DUPLICATE, AMBIGUOUS, OLD };
    static Sequence classify_sequence(uint8_t previous, uint8_t incoming);
    static bool accept_sequence(Sample &previous, uint8_t incoming, bool reset);
    static bool fresh(const Sample &sample, uint64_t now_us);
    static void break_continuity(Sample &sample, Continuity reason);
    static void copy_stream(const Stream &stream, uint64_t now_us, Stream &out);
    static Continuity continuity(const Sample &previous, uint8_t seq, bool usable, uint64_t now_us, bool reset);
    Entry *find_or_allocate(uint8_t source_node_id, uint8_t esc_index, bool position, uint64_t now_us);
};
