#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_DroneCAN/AP_DroneCAN_config.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
#include <AP_DroneCAN/AP_DroneCAN_MotorTelemetryCodec.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <string.h>

using Codec = AP_DroneCAN_MotorTelemetryCodec;
using Store = AP_DroneCAN_MotorTelemetry;
using Result = Codec::Result;
using Type = Codec::Type;
using Rep = Codec::Representation;
using C = Store::Continuity;

static_assert(Codec::MAX_PAYLOAD_LENGTH == MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN, "TUNNEL capacity changed");
static_assert(MAVLINK_MSG_ID_TUNNEL == 385, "Unexpected TUNNEL ID");

static uint32_t bits(const float &value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

class MotorTelemetryCodec : public ::testing::Test
{
protected:
    Codec::Record r {};
    uint8_t payload[128] {};
    uint8_t length {};

    void SetUp() override
    {
        r.type = Type::POSITION;
        r.context = {0x0102030405060708ULL, 0x31323334U, 0x41424344U, 2, true, true};
        r.snapshot_us = 0x1000000000000300ULL;
        r.updated_us = 0x1000000000000200ULL;
        r.sample.timestamp_us = 0x1000000000000100ULL;
        r.identity_generation = UINT32_MAX;
        r.source_node_id = 42;
        r.esc_index = 31;
        r.sample.counters = {UINT32_MAX, 0x01020304, 0x10203040, 0x55667788};
        r.sample.sample_seq = 255;
        r.sample.received = r.sample.valid = r.sample.usable = r.sample.fresh = r.sample.baseline_ready = true;
        r.sample.continuity = C::CONTIGUOUS;
        r.sample.last_break = C::INVALID;
        r.statistics = {UINT32_MAX, 0x0a0b0c0d, 0x11121314, 0x21222324};
        r.position = {0x89abcdef, Rep::TICKS, INT32_MIN, UINT16_MAX, 0.0f};
        r.rpm = -0.125f;
    }

    static void expect_equal(const Codec::Record &a, const Codec::Record &b)
    {
        EXPECT_EQ(a.type, b.type);
        EXPECT_EQ(a.context.session_id, b.context.session_id);
        EXPECT_EQ(a.context.session_tag_present, b.context.session_tag_present);
        EXPECT_EQ(a.context.delivery_counters_valid, b.context.delivery_counters_valid);
        EXPECT_EQ(a.context.driver_index, b.context.driver_index);
        EXPECT_EQ(a.context.export_seq, b.context.export_seq);
        EXPECT_EQ(a.context.superseded_count, b.context.superseded_count);
        EXPECT_EQ(a.snapshot_us, b.snapshot_us);
        EXPECT_EQ(a.updated_us, b.updated_us);
        EXPECT_EQ(a.identity_generation, b.identity_generation);
        EXPECT_EQ(a.source_node_id, b.source_node_id);
        EXPECT_EQ(a.esc_index, b.esc_index);
        EXPECT_EQ(a.sample.timestamp_us, b.sample.timestamp_us);
        EXPECT_EQ(a.sample.sample_seq, b.sample.sample_seq);
        EXPECT_EQ(a.sample.received, b.sample.received);
        EXPECT_EQ(a.sample.valid, b.sample.valid);
        EXPECT_EQ(a.sample.usable, b.sample.usable);
        EXPECT_EQ(a.sample.fresh, b.sample.fresh);
        EXPECT_EQ(a.sample.baseline_ready, b.sample.baseline_ready);
        EXPECT_EQ(a.sample.continuity, b.sample.continuity);
        EXPECT_EQ(a.sample.last_break, b.sample.last_break);
        EXPECT_EQ(a.sample.counters.accepted, b.sample.counters.accepted);
        EXPECT_EQ(a.sample.counters.invalid, b.sample.counters.invalid);
        EXPECT_EQ(a.sample.counters.sequence_rejected, b.sample.counters.sequence_rejected);
        EXPECT_EQ(a.sample.continuity_generation(), b.sample.continuity_generation());
        EXPECT_EQ(a.statistics.allocations, b.statistics.allocations);
        EXPECT_EQ(a.statistics.position_saturation, b.statistics.position_saturation);
        EXPECT_EQ(a.statistics.speed_saturation, b.statistics.speed_saturation);
        EXPECT_EQ(a.statistics.invalid_identity, b.statistics.invalid_identity);
        if (a.type == Type::POSITION) {
            EXPECT_EQ(a.position.origin_id, b.position.origin_id);
            EXPECT_EQ(a.position.representation, b.position.representation);
            if (a.position.representation == Rep::TICKS) {
                EXPECT_EQ(a.position.ticks, b.position.ticks);
                EXPECT_EQ(a.position.ticks_per_revolution, b.position.ticks_per_revolution);
            } else {
                EXPECT_EQ(bits(a.position.revolutions), bits(b.position.revolutions));
            }
        } else {
            EXPECT_EQ(bits(a.rpm), bits(b.rpm));
        }
    }

    void round_trip()
    {
        ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
        Codec::Record decoded {};
        ASSERT_EQ(Result::OK, Codec::decode(payload, length, decoded));
        expect_equal(r, decoded);
        uint8_t again[128] {};
        uint8_t again_length = 0;
        ASSERT_EQ(Result::OK, Codec::encode(decoded, again, sizeof(again), again_length));
        EXPECT_EQ(length, again_length);
        EXPECT_EQ(0, memcmp(payload, again, length));
    }
};

// Independent Python struct.pack vectors; see motor_telemetry_v1_reference.py.
static const uint8_t TICKS_VECTOR[] = {
    0x4d, 0x54, 0x01, 0x01, 0x68, 0x00, 0x02, 0x2a, 0x1f, 0x1f, 0xff, 0x01,
    0x04, 0x03, 0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x03, 0x02, 0x01,
    0x40, 0x30, 0x20, 0x10, 0x88, 0x77, 0x66, 0x55, 0xff, 0xff, 0xff, 0xff,
    0x0d, 0x0c, 0x0b, 0x0a, 0x14, 0x13, 0x12, 0x11, 0x24, 0x23, 0x22, 0x21,
    0x34, 0x33, 0x32, 0x31, 0x44, 0x43, 0x42, 0x41, 0xef, 0xcd, 0xab, 0x89,
    0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x80,
};

static const uint8_t REVOLUTIONS_VECTOR[] = {
    0x4d, 0x54, 0x01, 0x01, 0x68, 0x00, 0x02, 0x2a, 0x1f, 0x1f, 0xff, 0x01,
    0x04, 0x03, 0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x03, 0x02, 0x01,
    0x40, 0x30, 0x20, 0x10, 0x88, 0x77, 0x66, 0x55, 0xff, 0xff, 0xff, 0xff,
    0x0d, 0x0c, 0x0b, 0x0a, 0x14, 0x13, 0x12, 0x11, 0x24, 0x23, 0x22, 0x21,
    0x34, 0x33, 0x32, 0x31, 0x44, 0x43, 0x42, 0x41, 0xef, 0xcd, 0xab, 0x89,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0xbf,
};

static const uint8_t SPEED_VECTOR[] = {
    0x4d, 0x54, 0x01, 0x02, 0x60, 0x00, 0x02, 0x2a, 0x1f, 0x1f, 0xff, 0x01,
    0x04, 0x03, 0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x03, 0x02, 0x01,
    0x40, 0x30, 0x20, 0x10, 0x88, 0x77, 0x66, 0x55, 0xff, 0xff, 0xff, 0xff,
    0x0d, 0x0c, 0x0b, 0x0a, 0x14, 0x13, 0x12, 0x11, 0x24, 0x23, 0x22, 0x21,
    0x34, 0x33, 0x32, 0x31, 0x44, 0x43, 0x42, 0x41, 0x00, 0x00, 0x00, 0xbe,
};

TEST_F(MotorTelemetryCodec, IndependentTicksVector)
{
    round_trip();
    ASSERT_EQ(sizeof(TICKS_VECTOR), length);
    EXPECT_EQ(0, memcmp(TICKS_VECTOR, payload, sizeof(TICKS_VECTOR)));
}

TEST_F(MotorTelemetryCodec, IndependentRevolutionsVector)
{
    r.position.representation = Rep::REVOLUTIONS;
    r.position.revolutions = -1.25f;
    round_trip();
    ASSERT_EQ(sizeof(REVOLUTIONS_VECTOR), length);
    EXPECT_EQ(0, memcmp(REVOLUTIONS_VECTOR, payload, sizeof(REVOLUTIONS_VECTOR)));
}

TEST_F(MotorTelemetryCodec, IndependentSpeedVector)
{
    r.type = Type::SPEED;
    round_trip();
    ASSERT_EQ(sizeof(SPEED_VECTOR), length);
    EXPECT_EQ(0, memcmp(SPEED_VECTOR, payload, sizeof(SPEED_VECTOR)));
}

TEST_F(MotorTelemetryCodec, SignedTickBoundaries)
{
    for (int32_t ticks : {
             INT32_MIN, INT32_MAX, -1, 0, 1
         }) {
        r.position.ticks = ticks;
        r.position.origin_id = UINT32_MAX;
        round_trip();
    }
}

TEST_F(MotorTelemetryCodec, SignedFloatBitsIncludingZeros)
{
    for (float value : {
             -1000.125f, -0.0625f, -0.0f, 0.0f, 0.03125f, 1000.125f
             }) {
        r.type = Type::SPEED;
        r.rpm = value;
        round_trip();
        r.type = Type::POSITION;
        r.position.representation = Rep::REVOLUTIONS;
        r.position.revolutions = value;
        round_trip();
    }
}

TEST_F(MotorTelemetryCodec, FiniteFloatBitBoundaries)
{
    for (uint32_t value : {
             0x00000001U, 0x007fffffU, 0x7f7fffffU, 0x80000001U, 0xff7fffffU
         }) {
        r.type = Type::SPEED;
        memcpy(&r.rpm, &value, sizeof(value));
        round_trip();
        r.type = Type::POSITION;
        r.position.representation = Rep::REVOLUTIONS;
        memcpy(&r.position.revolutions, &value, sizeof(value));
        round_trip();
    }
}

TEST_F(MotorTelemetryCodec, IdentityBoundaries)
{
    for (uint8_t node : {
             1, 127
         }) {
        for (uint8_t esc : {
                 0, 31
             }) {
            r.source_node_id = node;
            r.esc_index = esc;
            r.context.driver_index = node == 1 ? 0 : 255;
            round_trip();
        }
    }
}

TEST_F(MotorTelemetryCodec, InvalidNumericalPayloadsRemainRaw)
{
    r.sample.usable = r.sample.baseline_ready = false;
    for (uint32_t value : {
             0x7fc12345U, 0x7f800000U, 0xff800000U
         }) {
        r.type = Type::SPEED;
        memcpy(&r.rpm, &value, sizeof(value));
        round_trip();
        r.sample.usable = true;
        EXPECT_EQ(Result::INVALID_FIELD, Codec::encode(r, payload, sizeof(payload), length));
        r.sample.usable = false;
        r.type = Type::POSITION;
        r.position.representation = Rep::REVOLUTIONS;
        memcpy(&r.position.revolutions, &value, sizeof(value));
        round_trip();
        r.sample.usable = true;
        EXPECT_EQ(Result::INVALID_FIELD, Codec::encode(r, payload, sizeof(payload), length));
        r.sample.usable = false;
    }
    r.position.representation = Rep::TICKS;
    r.position.ticks_per_revolution = 0;
    round_trip();
    r.sample.usable = true;
    EXPECT_EQ(Result::INVALID_FIELD, Codec::encode(r, payload, sizeof(payload), length));
}

TEST_F(MotorTelemetryCodec, CounterWrapAndZeroGenerationsAreValues)
{
    r.sample.counters = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    r.statistics = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    r.context.export_seq = r.context.superseded_count = UINT32_MAX;
    round_trip();
    r.sample.counters.accepted++;
    r.sample.counters.invalid++;
    r.sample.counters.sequence_rejected++;
    r.sample.counters.continuity_breaks++;
    r.statistics.allocations++;
    r.statistics.position_saturation++;
    r.statistics.speed_saturation++;
    r.statistics.invalid_identity++;
    r.identity_generation++;
    r.context.export_seq++;
    r.context.superseded_count++;
    round_trip();
    EXPECT_EQ(0U, r.identity_generation);
    EXPECT_EQ(0U, r.sample.continuity_generation());
}

TEST_F(MotorTelemetryCodec, SessionAndDeliveryInformationCanBeUnknown)
{
    r.context.session_id = UINT64_MAX;
    round_trip();
    r.context.session_id = 0; // A present zero tag is distinct from an absent tag.
    round_trip();
    r.context.session_tag_present = false;
    r.context.delivery_counters_valid = false;
    EXPECT_EQ(Result::INVALID_FIELD, Codec::encode(r, payload, sizeof(payload), length));
    r.context.export_seq = r.context.superseded_count = 0;
    round_trip();
    EXPECT_EQ(0U, payload[13]);
    r.context.session_id = 1;
    EXPECT_EQ(Result::INVALID_FIELD, Codec::encode(r, payload, sizeof(payload), length));
}

TEST_F(MotorTelemetryCodec, ContinuityCodesAreExplicit)
{
    const C values[] = {C::FIRST, C::CONTIGUOUS, C::SEQUENCE, C::TIMEOUT, C::INVALID,
                        C::ORIGIN, C::REPRESENTATION, C::RESOLUTION, C::NODE_RESTART
                       };
    for (uint8_t i = 0; i < 9; i++) {
        r.sample.continuity = r.sample.last_break = values[i];
        round_trip();
        EXPECT_EQ(i, payload[11]);
        EXPECT_EQ(i, payload[12]);
    }
}

TEST_F(MotorTelemetryCodec, ExactLengthsAndTruncation)
{
    for (Type type : {
             Type::POSITION, Type::SPEED
         }) {
        r.type = type;
        ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
        for (size_t n = 0; n < length; n++) {
            Codec::Record sentinel = r;
            EXPECT_EQ(Result::BAD_LENGTH, Codec::decode(payload, n, sentinel));
            expect_equal(r, sentinel);
        }
        for (size_t n : {
                 size_t(length + 1), size_t(128), size_t(129)
             }) {
            Codec::Record sentinel = r;
            EXPECT_EQ(Result::BAD_LENGTH, Codec::decode(payload, n, sentinel));
            expect_equal(r, sentinel);
        }
        payload[4]--;
        EXPECT_EQ(Result::BAD_LENGTH, Codec::decode(payload, length, r));
    }
}

TEST_F(MotorTelemetryCodec, EncodeFailureAndCapacityDoNotWriteOutputs)
{
    for (Type type : {
             Type::POSITION, Type::SPEED
         }) {
        r.type = type;
        const uint8_t required = type == Type::POSITION ? 104 : 96;
        for (size_t capacity = 0; capacity < required; capacity++) {
            memset(payload, 0x5a, sizeof(payload));
            length = 0xa5;
            EXPECT_EQ(Result::BAD_LENGTH, Codec::encode(r, payload, capacity, length));
            EXPECT_EQ(0xa5, length);
            for (uint8_t byte : payload) {
                EXPECT_EQ(0x5a, byte);
            }
        }
    }
    length = 0xa5;
    EXPECT_EQ(Result::INVALID_ARGUMENT, Codec::encode(r, nullptr, 128, length));
    EXPECT_EQ(0xa5, length);
    Codec::Record sentinel = r;
    EXPECT_EQ(Result::INVALID_ARGUMENT, Codec::decode(nullptr, 96, sentinel));
    expect_equal(r, sentinel);
}

TEST_F(MotorTelemetryCodec, UnknownHeadersAndFieldsAreRejected)
{
    struct Mutation {
        uint8_t offset;
        uint8_t value;
        Result result;
    };
    const Mutation changes[] = {
        {0, 0, Result::BAD_MAGIC}, {1, 0, Result::BAD_MAGIC},
        {2, 0, Result::UNKNOWN_VERSION}, {2, 2, Result::UNKNOWN_VERSION}, {2, 255, Result::UNKNOWN_VERSION},
        {3, 0, Result::UNKNOWN_TYPE}, {3, 3, Result::UNKNOWN_TYPE},
        {4, 96, Result::BAD_LENGTH}, {5, 1, Result::INVALID_FIELD},
        {7, 0, Result::INVALID_FIELD}, {7, 128, Result::INVALID_FIELD}, {8, 32, Result::INVALID_FIELD},
        {9, 0xff, Result::INVALID_FIELD}, {9, 0x1e, Result::INVALID_FIELD},
        {9, 0x1d, Result::INVALID_FIELD}, {9, 0x17, Result::INVALID_FIELD},
        {11, 9, Result::INVALID_FIELD}, {12, 255, Result::INVALID_FIELD},
        {13, 0xff, Result::INVALID_FIELD}, {14, 1, Result::INVALID_FIELD}, {15, 1, Result::INVALID_FIELD},
        {39, 0x11, Result::INVALID_FIELD}, {47, 0x11, Result::INVALID_FIELD},
        {96, 2, Result::UNKNOWN_REPRESENTATION}, {97, 1, Result::INVALID_FIELD},
    };
    for (const auto &change : changes) {
        ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
        payload[change.offset] = change.value;
        Codec::Record sentinel = r;
        EXPECT_EQ(change.result, Codec::decode(payload, length, sentinel)) << int(change.offset);
        expect_equal(r, sentinel);
    }
    r.position.representation = Rep::REVOLUTIONS;
    ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
    payload[98] = 1; // Float records must carry canonical zero resolution.
    EXPECT_EQ(Result::INVALID_FIELD, Codec::decode(payload, length, r));
}

TEST_F(MotorTelemetryCodec, MalformedUsableNumericalValuesAreRejected)
{
    for (Type type : {
             Type::POSITION, Type::SPEED
         }) {
        r.type = type;
        r.position.representation = Rep::REVOLUTIONS;
        for (uint8_t exponent : {
                 0x7f, 0xff
             }) {
            ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
            const uint8_t offset = type == Type::POSITION ? 100 : 92;
            payload[offset] = payload[offset + 1] = 0;
            payload[offset + 2] = 0x80;
            payload[offset + 3] = exponent;
            Codec::Record sentinel = r;
            EXPECT_EQ(Result::INVALID_FIELD, Codec::decode(payload, length, sentinel));
            expect_equal(r, sentinel);
        }
    }
    r.type = Type::POSITION;
    r.position.representation = Rep::TICKS;
    ASSERT_EQ(Result::OK, Codec::encode(r, payload, sizeof(payload), length));
    payload[98] = payload[99] = 0;
    EXPECT_EQ(Result::INVALID_FIELD, Codec::decode(payload, length, r));
}

TEST_F(MotorTelemetryCodec, InvalidLogicalRecordsAreRejectedBeforeWriting)
{
    const Codec::Record valid = r;
    for (uint8_t condition = 0; condition < 8; condition++) {
        r = valid;
        Result expected = Result::INVALID_FIELD;
        switch (condition) {
        case 0: r.type = static_cast<Type>(3); expected = Result::UNKNOWN_TYPE; break;
        case 1: r.position.representation = static_cast<Rep>(2); expected = Result::UNKNOWN_REPRESENTATION; break;
        case 2: r.sample.continuity = static_cast<C>(255); break;
        case 3: r.sample.last_break = static_cast<C>(255); break;
        case 4: r.source_node_id = 0; break;
        case 5: r.esc_index = 32; break;
        case 6: r.updated_us = r.snapshot_us + 1; break;
        case 7: r.sample.timestamp_us = r.snapshot_us + 1; break;
        }
        memset(payload, 0x5a, sizeof(payload));
        length = 0xa5;
        EXPECT_EQ(expected, Codec::encode(r, payload, sizeof(payload), length));
        EXPECT_EQ(0xa5, length);
        for (uint8_t byte : payload) {
            EXPECT_EQ(0x5a, byte);
        }
    }
}

TEST_F(MotorTelemetryCodec, UnalignedBuffersAndCanaries)
{
    uint8_t unaligned[106];
    memset(unaligned, 0x5a, sizeof(unaligned));
    ASSERT_EQ(Result::OK, Codec::encode(r, unaligned + 1, 104, length));
    EXPECT_EQ(0x5a, unaligned[0]);
    EXPECT_EQ(0x5a, unaligned[105]);
    Codec::Record decoded {};
    ASSERT_EQ(Result::OK, Codec::decode(unaligned + 1, length, decoded));
    expect_equal(r, decoded);
}

TEST_F(MotorTelemetryCodec, SnapshotProjectionPreservesHiddenBreakAndMissingType)
{
    Store store;
    uavcan_equipment_esc_MotorPosition p {};
    p.esc_index = 31;
    p.valid = true;
    p.origin_id = UINT32_MAX;
    p.position.ticks.ticks = INT32_MIN;
    p.position.ticks.ticks_per_revolution = UINT16_MAX;
    ASSERT_TRUE(store.receive(42, p, 100));
    p.valid = false;
    p.sample_seq++;
    ASSERT_TRUE(store.receive(42, p, 200));
    p.valid = true;
    p.sample_seq++;
    ASSERT_TRUE(store.receive(42, p, 300));
    p.sample_seq++;
    ASSERT_TRUE(store.receive(42, p, 400));
    Store::Snapshot snapshot {};
    store.get_snapshot(450, snapshot);
    const Codec::Context context {};
    ASSERT_EQ(Result::OK, Codec::from_snapshot(snapshot, 0, Type::POSITION, context, r));
    EXPECT_EQ(C::CONTIGUOUS, r.sample.continuity);
    EXPECT_EQ(C::INVALID, r.sample.last_break);
    EXPECT_EQ(1U, r.sample.continuity_generation());
    EXPECT_EQ(4U, r.sample.counters.accepted);
    round_trip();
    ASSERT_EQ(Result::OK, Codec::from_snapshot(snapshot, 0, Type::SPEED, context, r));
    EXPECT_FALSE(r.sample.received);
    EXPECT_FALSE(r.sample.valid);
    EXPECT_FALSE(r.sample.usable);
    EXPECT_EQ(0U, r.sample.counters.accepted);
    round_trip();
    Codec::Record sentinel = r;
    EXPECT_EQ(Result::INVALID_ARGUMENT, Codec::from_snapshot(snapshot, 1, Type::POSITION, context, sentinel));
    EXPECT_EQ(Result::INVALID_ARGUMENT, Codec::from_snapshot(snapshot, 8, Type::POSITION, context, sentinel));
    EXPECT_EQ(Result::UNKNOWN_TYPE, Codec::from_snapshot(snapshot, 0, static_cast<Type>(99), context, sentinel));
    expect_equal(r, sentinel);
}

TEST_F(MotorTelemetryCodec, DriverStatisticsAndSnapshotReplacementRemainExplicit)
{
    Store store;
    com_mowglinext_esc_MotorSpeed s {};
    s.valid = true;
    s.rpm = -0.03125f;
    for (uint8_t node = 1; node <= Store::CAPACITY; node++) {
        ASSERT_TRUE(store.receive(node, s, 100));
    }
    uavcan_equipment_esc_MotorPosition p {};
    EXPECT_FALSE(store.receive(99, p, 200));
    EXPECT_FALSE(store.receive(99, s, 200));
    EXPECT_FALSE(store.receive(0, s, 200));
    for (uint8_t seq = 1; seq <= 3; seq++) {
        s.sample_seq = seq;
        ASSERT_TRUE(store.receive(1, s, 200 + seq));
    }
    Store::Snapshot snapshot {};
    store.get_snapshot(300, snapshot);
    // Caller previously exported accepted=1; now 4. Two intermediate samples
    // were superseded. The codec neither calculates nor mutates this policy.
    Codec::Context context {0, 13, 9, 2, false, true};
    ASSERT_EQ(Result::OK, Codec::from_snapshot(snapshot, 0, Type::SPEED, context, r));
    EXPECT_EQ(4U, r.sample.counters.accepted);
    EXPECT_EQ(1U, r.statistics.position_saturation);
    EXPECT_EQ(1U, r.statistics.speed_saturation);
    EXPECT_EQ(1U, r.statistics.invalid_identity);
    EXPECT_EQ(8U, r.statistics.allocations);
    round_trip();
    Codec::Record same {};
    ASSERT_EQ(Result::OK, Codec::from_snapshot(snapshot, 0, Type::SPEED, context, same));
    expect_equal(r, same);
    Store::Snapshot after {};
    store.get_snapshot(300, after);
    EXPECT_EQ(snapshot.streams[0].speed.counters.accepted, after.streams[0].speed.counters.accepted);
    EXPECT_EQ(snapshot.streams[0].speed.continuity_generation(), after.streams[0].speed.continuity_generation());
}

TEST_F(MotorTelemetryCodec, TunnelContainerRoundTripAndZeroTruncation)
{
    EXPECT_TRUE(Codec::EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE > 32767);
    for (float rpm : {
             -0.125f, 0.0f, -0.0f
             }) {
        r.type = Type::SPEED;
        r.rpm = rpm;
        mavlink_tunnel_t packet {};
        packet.target_system = 1;
        packet.target_component = 191;
        packet.payload_type = Codec::EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE;
        ASSERT_EQ(Result::OK, Codec::encode(r, packet.payload, sizeof(packet.payload), packet.payload_length));
        mavlink_message_t message {};
        mavlink_status_t status {};
        mavlink_msg_tunnel_encode_status(1, 1, &status, &message, &packet); // Local state only, no send.
        EXPECT_EQ(385U, message.msgid);
        EXPECT_LE(message.len, 5U + packet.payload_length);
        mavlink_tunnel_t restored {};
        mavlink_msg_tunnel_decode(&message, &restored); // Restores MAVLink2-trimmed zeros.
        EXPECT_TRUE(restored.payload_type == Codec::EXPERIMENTAL_TUNNEL_PAYLOAD_TYPE);
        EXPECT_EQ(packet.payload_length, restored.payload_length);
        EXPECT_EQ(0, memcmp(packet.payload, restored.payload, sizeof(packet.payload)));
        Codec::Record decoded {};
        ASSERT_EQ(Result::OK, Codec::decode(restored.payload, restored.payload_length, decoded));
        expect_equal(r, decoded);
        EXPECT_EQ(Result::BAD_LENGTH, Codec::decode(restored.payload, sizeof(restored.payload), decoded));
    }
}
#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED

AP_GTEST_MAIN()
