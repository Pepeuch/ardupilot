#include <AP_gtest.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <limits>
#include <atomic>
#include <thread>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
using Store = AP_DroneCAN_MotorTelemetry;
using C = Store::Continuity;

// Seed counters near wrap without adding a runtime mutation API.
class AP_DroneCAN_MotorTelemetry_Test
{
public:
    static Store::Counters &position_counters(Store &store)
    {
        return store._entries[0].stream.position.counters;
    }
    static Store::Counters &speed_counters(Store &store)
    {
        return store._entries[0].stream.speed.counters;
    }
    static Store::Statistics &statistics(Store &store)
    {
        return store._statistics;
    }
};

class MotorTelemetry : public ::testing::Test
{
protected:
    Store store;
    Store::Stream out {};
    uavcan_equipment_esc_MotorPosition p {};
    com_mowglinext_esc_MotorSpeed s {};

    void SetUp() override
    {
        p.esc_index = 31;
        p.valid = true;
        p.origin_id = 123;
        p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_TICKS;
        p.position.ticks.ticks_per_revolution = 4096;
        s.esc_index = 31;
        s.valid = true;
    }

    void position(uint64_t time_us)
    {
        ASSERT_TRUE(store.receive(42, p, time_us));
        ASSERT_TRUE(store.get(42, 31, time_us, out));
    }

    void speed(uint64_t time_us)
    {
        ASSERT_TRUE(store.receive(42, s, time_us));
        ASSERT_TRUE(store.get(42, 31, time_us, out));
    }
};

TEST_F(MotorTelemetry, RawTicksAndWrap)
{
    p.position.ticks.ticks = 123456;
    p.sample_seq = 255;
    position(0);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(123456, out.position.value.ticks.ticks);
    EXPECT_EQ(4096, out.position.value.ticks.ticks_per_revolution);
    EXPECT_EQ(123U, out.position.origin_id);
    p.position.ticks.ticks = -123456;
    p.sample_seq = 0;
    position(1000);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(-123456, out.position.value.ticks.ticks);
    EXPECT_EQ(1000U, out.position.timestamp_us);
    EXPECT_TRUE(out.position.usable);
    // The store deliberately does not certify this physically unbounded jump.
}

TEST_F(MotorTelemetry, PositionMetadata)
{
    position(1);
    p.sample_seq++;
    p.origin_id++;
    position(2);
    EXPECT_EQ(C::ORIGIN, out.position.continuity);
    p.sample_seq++;
    p.position.ticks.ticks_per_revolution++;
    position(3);
    EXPECT_EQ(C::RESOLUTION, out.position.continuity);
    p.sample_seq++;
    p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    p.position.revolutions = -1.25f;
    position(4);
    EXPECT_EQ(C::REPRESENTATION, out.position.continuity);
    EXPECT_FLOAT_EQ(-1.25f, out.position.value.revolutions);
    p.sample_seq++;
    p.position.revolutions = 2.75f;
    position(5);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_FLOAT_EQ(2.75f, out.position.value.revolutions);
}

TEST_F(MotorTelemetry, InvalidPositionDropsBaseline)
{
    position(1);
    p.sample_seq++;
    p.valid = false;
    position(2);
    EXPECT_FALSE(out.position.valid);
    EXPECT_FALSE(out.position.usable);
    EXPECT_EQ(C::INVALID, out.position.continuity);
    p.sample_seq++;
    p.valid = true;
    position(3);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    p.sample_seq++;
    p.position.ticks.ticks_per_revolution = 0;
    position(4);
    EXPECT_FALSE(out.position.usable);
    p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    for (float value : {
             std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()
         }) {
        p.sample_seq++;
        p.position.revolutions = value;
        position(5);
        EXPECT_TRUE(out.position.valid);
        EXPECT_FALSE(out.position.usable);
        EXPECT_EQ(C::INVALID, out.position.continuity);
    }
}

TEST_F(MotorTelemetry, SignedFractionalSpeedAndSequenceWrap)
{
    s.sample_seq = 255;
    speed(0);
    EXPECT_FLOAT_EQ(0.0f, out.speed.rpm);
    EXPECT_TRUE(out.speed.usable);
    s.sample_seq = 0;
    s.rpm = -0.125f;
    speed(1);
    EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
    EXPECT_FLOAT_EQ(-0.125f, out.speed.rpm);
    s.sample_seq++;
    s.rpm = 12.375f;
    speed(2);
    EXPECT_FLOAT_EQ(12.375f, out.speed.rpm);
    EXPECT_EQ(2U, out.speed.timestamp_us);
}

TEST_F(MotorTelemetry, InvalidSpeed)
{
    speed(1);
    for (float value : {
             std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()
         }) {
        s.sample_seq++;
        s.rpm = value;
        speed(2);
        EXPECT_TRUE(out.speed.valid);
        EXPECT_FALSE(out.speed.usable);
        EXPECT_EQ(C::INVALID, out.speed.continuity);
    }
    s.sample_seq++;
    s.rpm = 0;
    speed(3);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    s.valid = false;
    s.sample_seq++;
    speed(4);
    EXPECT_FALSE(out.speed.valid);
    EXPECT_FALSE(out.speed.usable);
    s.valid = true;
    s.sample_seq++;
    speed(5);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
}

TEST_F(MotorTelemetry, OldThenDuplicateCannotManufactureContinuity)
{
    p.sample_seq = s.sample_seq = 10;
    p.position.ticks.ticks = 12345;
    s.rpm = -1.25f;
    position(100);
    speed(100);
    for (uint8_t seq : {
             9, 10
         }) {
        p.sample_seq = s.sample_seq = seq;
        p.position.ticks.ticks = -999;
        s.rpm = 99.0f;
        EXPECT_FALSE(store.receive(42, p, 200));
        EXPECT_FALSE(store.receive(42, s, 200));
        ASSERT_TRUE(store.get(42, 31, 200, out));
        EXPECT_EQ(10, out.position.sample_seq);
        EXPECT_EQ(10, out.speed.sample_seq);
        EXPECT_EQ(100U, out.position.timestamp_us);
        EXPECT_EQ(100U, out.speed.timestamp_us);
        EXPECT_EQ(12345, out.position.value.ticks.ticks);
        EXPECT_FLOAT_EQ(-1.25f, out.speed.rpm);
        EXPECT_EQ(C::SEQUENCE, out.position.continuity);
        EXPECT_EQ(C::SEQUENCE, out.speed.continuity);
    }
    p.sample_seq = s.sample_seq = 11;
    position(300);
    speed(300);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    p.sample_seq = s.sample_seq = 12;
    position(400);
    speed(400);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
}

TEST_F(MotorTelemetry, DuplicateThenNextSampleRebaselines)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    p.valid = s.valid = false;
    p.position.ticks.ticks_per_revolution = 1;
    EXPECT_FALSE(store.receive(42, p, 200));
    EXPECT_FALSE(store.receive(42, s, 200));
    ASSERT_TRUE(store.get(42, 31, 200, out));
    EXPECT_TRUE(out.position.valid);
    EXPECT_TRUE(out.speed.valid);
    EXPECT_EQ(4096, out.position.value.ticks.ticks_per_revolution);
    EXPECT_EQ(100U, out.position.timestamp_us);
    EXPECT_EQ(100U, out.speed.timestamp_us);
    p.valid = s.valid = true;
    p.position.ticks.ticks_per_revolution = 4096;
    p.sample_seq = s.sample_seq = 11;
    position(300);
    speed(300);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
}

TEST_F(MotorTelemetry, ReplaysDoNotExtendFreshnessEvenAfterTimeout)
{
    p.sample_seq = s.sample_seq = 10;
    position(0);
    speed(0);
    EXPECT_FALSE(store.receive(42, p, Store::TIMEOUT_US - 1));
    EXPECT_FALSE(store.receive(42, s, Store::TIMEOUT_US - 1));
    ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US - 1, out));
    EXPECT_TRUE(out.position.fresh);
    EXPECT_TRUE(out.speed.fresh);
    for (uint8_t seq : {
             10, 9, 138
         }) {
        p.sample_seq = s.sample_seq = seq;
        EXPECT_FALSE(store.receive(42, p, Store::TIMEOUT_US));
        EXPECT_FALSE(store.receive(42, s, Store::TIMEOUT_US));
        ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US, out));
        EXPECT_EQ(0U, out.position.timestamp_us);
        EXPECT_EQ(0U, out.speed.timestamp_us);
        EXPECT_EQ(10, out.position.sample_seq);
        EXPECT_EQ(10, out.speed.sample_seq);
        EXPECT_FALSE(out.position.fresh);
        EXPECT_FALSE(out.speed.fresh);
        EXPECT_EQ(C::TIMEOUT, out.position.continuity);
        EXPECT_EQ(C::TIMEOUT, out.speed.continuity);
    }
    p.sample_seq = s.sample_seq = 11;
    position(Store::TIMEOUT_US + 1);
    speed(Store::TIMEOUT_US + 1);
    EXPECT_EQ(C::TIMEOUT, out.position.continuity);
    EXPECT_EQ(C::TIMEOUT, out.speed.continuity);
}

TEST_F(MotorTelemetry, ForwardGapsRequireNewBaseline)
{
    for (uint8_t distance : {
             2, 127
         }) {
        store = Store{};
        p.sample_seq = s.sample_seq = 10;
        position(100);
        speed(100);
        p.sample_seq = s.sample_seq = uint8_t(10 + distance);
        position(200);
        speed(200);
        EXPECT_EQ(C::SEQUENCE, out.position.continuity);
        EXPECT_EQ(C::SEQUENCE, out.speed.continuity);
        EXPECT_EQ(p.sample_seq, out.position.sample_seq);
        EXPECT_EQ(s.sample_seq, out.speed.sample_seq);
        p.sample_seq++;
        s.sample_seq++;
        position(300);
        speed(300);
        EXPECT_EQ(C::FIRST, out.position.continuity);
        EXPECT_EQ(C::FIRST, out.speed.continuity);
        p.sample_seq++;
        s.sample_seq++;
        position(400);
        speed(400);
        EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
        EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
    }
}

TEST_F(MotorTelemetry, HalfRangeAndOlderSequencesAreRejected)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    // 128 is undecidable; 129..255 are presumed old under half-range ordering.
    for (uint8_t distance : {
             128, 129, 255
         }) {
        p.sample_seq = s.sample_seq = uint8_t(10 + distance);
        EXPECT_FALSE(store.receive(42, p, 200));
        EXPECT_FALSE(store.receive(42, s, 200));
        ASSERT_TRUE(store.get(42, 31, 200, out));
        EXPECT_EQ(10, out.position.sample_seq);
        EXPECT_EQ(10, out.speed.sample_seq);
        EXPECT_EQ(100U, out.position.timestamp_us);
        EXPECT_EQ(100U, out.speed.timestamp_us);
        EXPECT_EQ(C::SEQUENCE, out.position.continuity);
        EXPECT_EQ(C::SEQUENCE, out.speed.continuity);
    }
}

TEST_F(MotorTelemetry, FullSequenceCycleCannotBeDisambiguated)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    // A full cycle aliases a duplicate and is conservatively rejected.
    p.sample_seq = s.sample_seq = uint8_t(10 + 256);
    EXPECT_FALSE(store.receive(42, p, 200));
    EXPECT_FALSE(store.receive(42, s, 200));
    ASSERT_TRUE(store.get(42, 31, 200, out));
    EXPECT_EQ(100U, out.position.timestamp_us);
    EXPECT_EQ(100U, out.speed.timestamp_us);
    // 257 aliases +1: only the timeout can break this temporal ambiguity.
    p.sample_seq = s.sample_seq = uint8_t(10 + 257);
    position(Store::TIMEOUT_US + 100);
    speed(Store::TIMEOUT_US + 100);
    EXPECT_EQ(C::TIMEOUT, out.position.continuity);
    EXPECT_EQ(C::TIMEOUT, out.speed.continuity);
    // Within the window, +257 is indistinguishable from +1. CONTIGUOUS is
    // therefore only a sequence/time indication, never physical proof.
    Store alias;
    p.sample_seq = s.sample_seq = 10;
    ASSERT_TRUE(alias.receive(42, p, 100));
    ASSERT_TRUE(alias.receive(42, s, 100));
    p.sample_seq = s.sample_seq = uint8_t(10 + 257);
    ASSERT_TRUE(alias.receive(42, p, 200));
    ASSERT_TRUE(alias.receive(42, s, 200));
    ASSERT_TRUE(alias.get(42, 31, 200, out));
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
}

TEST_F(MotorTelemetry, NewPositionOriginAllowsSequenceRestart)
{
    p.sample_seq = 10;
    position(100);
    p.origin_id++;
    p.sample_seq = 9;
    position(200);
    EXPECT_EQ(9, out.position.sample_seq);
    EXPECT_EQ(C::ORIGIN, out.position.continuity);
    p.sample_seq++;
    position(300);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    p.origin_id++;
    position(400); // Same sequence with a new origin is not a duplicate.
    EXPECT_EQ(C::ORIGIN, out.position.continuity);
}

TEST_F(MotorTelemetry, RawInt32BoundariesArePreserved)
{
    p.position.ticks.ticks = INT32_MAX;
    position(100);
    EXPECT_EQ(INT32_MAX, out.position.value.ticks.ticks);
    EXPECT_EQ(4096, out.position.value.ticks.ticks_per_revolution);
    p.sample_seq++;
    p.position.ticks.ticks = INT32_MIN;
    position(200);
    EXPECT_EQ(INT32_MIN, out.position.value.ticks.ticks);
    EXPECT_EQ(4096, out.position.value.ticks.ticks_per_revolution);
    p.sample_seq++;
    p.position.ticks.ticks = INT32_MAX;
    position(300);
    EXPECT_EQ(INT32_MAX, out.position.value.ticks.ticks);
    // These are raw values, with no subtraction, unwrapping or physical delta.
}

TEST_F(MotorTelemetry, IndependentTimeouts)
{
    position(0);
    speed(100);
    ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US - 1, out));
    EXPECT_TRUE(out.position.fresh);
    ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US, out));
    EXPECT_FALSE(out.position.fresh);
    EXPECT_EQ(C::TIMEOUT, out.position.continuity);
    EXPECT_TRUE(out.speed.fresh);
    p.sample_seq++;
    position(Store::TIMEOUT_US);
    EXPECT_EQ(C::TIMEOUT, out.position.continuity);
    s.sample_seq++;
    speed(Store::TIMEOUT_US + 100);
    EXPECT_EQ(C::TIMEOUT, out.speed.continuity);
    p.sample_seq++;
    position(Store::TIMEOUT_US + 101);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
}

TEST_F(MotorTelemetry, IdentitiesAndSaturation)
{
    for (uint8_t node = 1; node <= Store::CAPACITY; node++) {
        p.position.ticks.ticks = node;
        ASSERT_TRUE(store.receive(node, p, 0));
    }
    EXPECT_FALSE(store.receive(100, s, 1));
    ASSERT_TRUE(store.get(1, 31, 1, out));
    EXPECT_EQ(1, out.position.value.ticks.ticks);
    // Updating an existing identity still works at capacity.
    ASSERT_TRUE(store.receive(1, s, Store::TIMEOUT_US - 1));
    ASSERT_TRUE(store.receive(100, s, Store::TIMEOUT_US));
    ASSERT_TRUE(store.get(100, 31, Store::TIMEOUT_US, out));
    EXPECT_FALSE(out.position.received);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    EXPECT_TRUE(store.get(1, 31, Store::TIMEOUT_US, out));
    EXPECT_FALSE(store.get(2, 31, Store::TIMEOUT_US, out));
    p.esc_index = 0;
    ASSERT_TRUE(store.receive(1, p, Store::TIMEOUT_US));
    ASSERT_TRUE(store.get(1, 0, Store::TIMEOUT_US, out));
    EXPECT_FALSE(out.speed.received);
    EXPECT_FALSE(store.receive(0, p, 1));
    EXPECT_FALSE(store.receive(128, p, 1));
    p.esc_index = 32;
    EXPECT_FALSE(store.receive(1, p, 1));
}

TEST_F(MotorTelemetry, InvalidBreakRemainsObservableAfterContiguousSamples)
{
    position(100);
    p.valid = false;
    p.sample_seq++;
    position(200);
    EXPECT_EQ(1U, out.position.continuity_generation());
    p.valid = true;
    p.sample_seq++;
    position(300);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    p.sample_seq++;
    position(400);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(1U, out.position.continuity_generation());
    EXPECT_EQ(C::INVALID, out.position.last_break);
    EXPECT_EQ(4U, out.position.counters.accepted);
    EXPECT_EQ(1U, out.position.counters.invalid);
    EXPECT_EQ(0U, out.position.counters.sequence_rejected);
    EXPECT_EQ(0U, out.speed.counters.accepted);
    EXPECT_EQ(0U, out.speed.continuity_generation());
}

TEST_F(MotorTelemetry, SpeedInvalidityAndGapCountOneTransferBreak)
{
    speed(100);
    s.valid = false;
    s.sample_seq = 2; // Invalid and skipped sequence in the same transfer.
    speed(200);
    EXPECT_EQ(1U, out.speed.continuity_generation());
    s.valid = true;
    s.sample_seq++;
    speed(300);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    s.sample_seq++;
    speed(400);
    EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
    EXPECT_EQ(1U, out.speed.continuity_generation());
    EXPECT_EQ(1U, out.speed.counters.invalid);
    EXPECT_EQ(4U, out.speed.counters.accepted);
    EXPECT_EQ(C::INVALID, out.speed.last_break);
    EXPECT_EQ(0U, out.position.counters.accepted);
    EXPECT_EQ(0U, out.position.continuity_generation());
}

TEST_F(MotorTelemetry, RejectedInvalidPayloadHasIndependentCounters)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    p.sample_seq = 9;
    p.valid = false;
    EXPECT_FALSE(store.receive(42, p, 200));
    s.rpm = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(store.receive(42, s, 200));
    ASSERT_TRUE(store.get(42, 31, 200, out));
    EXPECT_EQ(1U, out.position.counters.accepted);
    EXPECT_EQ(1U, out.position.counters.invalid);
    EXPECT_EQ(1U, out.position.counters.sequence_rejected);
    EXPECT_EQ(1U, out.position.continuity_generation());
    EXPECT_EQ(1U, out.speed.counters.accepted);
    EXPECT_EQ(1U, out.speed.counters.invalid);
    EXPECT_EQ(1U, out.speed.counters.sequence_rejected);
    EXPECT_EQ(1U, out.speed.continuity_generation());
    EXPECT_EQ(100U, out.position.timestamp_us);
    EXPECT_EQ(100U, out.speed.timestamp_us);
    EXPECT_TRUE(out.position.valid);
    EXPECT_TRUE(out.speed.valid);
    s.sample_seq++;
    s.rpm = -0.125f;
    speed(300);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    s.sample_seq++;
    speed(400);
    EXPECT_EQ(C::CONTIGUOUS, out.speed.continuity);
    EXPECT_EQ(1U, out.speed.continuity_generation());
    EXPECT_EQ(1U, out.position.counters.accepted);
    p.sample_seq = 11;
    p.valid = true;
    position(500);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(2U, out.position.counters.accepted);
    EXPECT_EQ(3U, out.speed.counters.accepted);
}

TEST_F(MotorTelemetry, RepeatedRejectionsAreEventsWithoutRefreshingSamples)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    for (uint8_t seq : {
             9, 10, 138
         }) {
        p.sample_seq = seq;
        EXPECT_FALSE(store.receive(42, p, 200));
    }
    ASSERT_TRUE(store.get(42, 31, 200, out));
    EXPECT_EQ(1U, out.position.counters.accepted);
    EXPECT_EQ(3U, out.position.counters.sequence_rejected);
    EXPECT_EQ(3U, out.position.continuity_generation());
    EXPECT_EQ(0U, out.speed.continuity_generation());
    EXPECT_EQ(100U, out.position.timestamp_us);
    p.sample_seq = 11;
    position(300);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    p.sample_seq++;
    position(400);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(3U, out.position.continuity_generation());
}

TEST_F(MotorTelemetry, TimeoutEventsComeFromMaintenanceNotReaders)
{
    position(0);
    speed(100);
    Store::Snapshot snapshot {};
    for (uint8_t i = 0; i < 5; i++) {
        store.get_snapshot(Store::TIMEOUT_US, snapshot);
        EXPECT_FALSE(snapshot.streams[0].position.fresh);
        EXPECT_TRUE(snapshot.streams[0].speed.fresh);
        EXPECT_EQ(0U, snapshot.streams[0].position.continuity_generation());
        EXPECT_EQ(100U, snapshot.updated_us);
    }
    store.update(Store::TIMEOUT_US);
    for (uint8_t i = 0; i < 5; i++) {
        store.update(Store::TIMEOUT_US + 1);
        store.get_snapshot(Store::TIMEOUT_US + 1, snapshot);
        ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US + 1, out));
        EXPECT_EQ(1U, out.position.continuity_generation());
        EXPECT_EQ(C::TIMEOUT, out.position.last_break);
        EXPECT_EQ(0U, out.speed.continuity_generation());
        EXPECT_EQ(1U, snapshot.streams[0].position.continuity_generation());
    }
    store.update(Store::TIMEOUT_US + 100);
    ASSERT_TRUE(store.get(42, 31, Store::TIMEOUT_US + 100, out));
    EXPECT_EQ(1U, out.speed.continuity_generation());
    p.sample_seq++;
    position(Store::TIMEOUT_US + 101);
    EXPECT_EQ(C::TIMEOUT, out.position.continuity);
    p.sample_seq++;
    position(Store::TIMEOUT_US + 102);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(1U, out.position.continuity_generation());
}

TEST_F(MotorTelemetry, MetadataBreaksHavePersistentGenerations)
{
    position(100);
    p.origin_id++;
    p.sample_seq++;
    position(200);
    EXPECT_EQ(1U, out.position.continuity_generation());
    EXPECT_EQ(C::ORIGIN, out.position.last_break);
    p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    p.position.revolutions = -0.25f;
    p.sample_seq++;
    position(300);
    EXPECT_EQ(2U, out.position.continuity_generation());
    EXPECT_EQ(C::REPRESENTATION, out.position.last_break);
    p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_TICKS;
    p.position.ticks.ticks = INT32_MIN;
    p.position.ticks.ticks_per_revolution = 100;
    p.sample_seq++;
    position(400);
    EXPECT_EQ(3U, out.position.continuity_generation());
    p.position.ticks.ticks_per_revolution++;
    p.sample_seq++;
    position(500);
    EXPECT_EQ(4U, out.position.continuity_generation());
    EXPECT_EQ(C::RESOLUTION, out.position.last_break);
    p.sample_seq++;
    position(600);
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(4U, out.position.continuity_generation());
    EXPECT_EQ(INT32_MIN, out.position.value.ticks.ticks);
}

TEST_F(MotorTelemetry, SaturationAndIdentityReassignmentAreVisible)
{
    Store::Snapshot snapshot {};
    store.get_snapshot(0, snapshot);
    EXPECT_EQ(0U, snapshot.occupied_mask);
    for (uint8_t node = 1; node <= Store::CAPACITY; node++) {
        ASSERT_TRUE(store.receive(node, p, 100));
    }
    EXPECT_FALSE(store.receive(99, p, 200));
    EXPECT_FALSE(store.receive(99, s, 200));
    EXPECT_FALSE(store.receive(0, p, 200));
    store.get_snapshot(200, snapshot);
    EXPECT_EQ(255U, snapshot.occupied_mask);
    EXPECT_EQ(1U, snapshot.statistics.position_saturation);
    EXPECT_EQ(1U, snapshot.statistics.speed_saturation);
    EXPECT_EQ(1U, snapshot.statistics.invalid_identity);
    EXPECT_EQ(8U, snapshot.statistics.allocations);
    const uint32_t original_generation = snapshot.streams[0].identity_generation;
    ASSERT_TRUE(store.receive(99, s, Store::TIMEOUT_US + 100));
    store.get_snapshot(Store::TIMEOUT_US + 100, snapshot);
    EXPECT_EQ(9U, snapshot.statistics.allocations);
    EXPECT_NE(original_generation, snapshot.streams[0].identity_generation);
    EXPECT_EQ(99U, snapshot.streams[0].source_node_id);
    EXPECT_EQ(0U, snapshot.streams[0].position.counters.accepted);
    EXPECT_EQ(0U, snapshot.streams[0].position.continuity_generation());
    EXPECT_FALSE(snapshot.streams[0].position.received);
    EXPECT_EQ(1U, snapshot.streams[0].speed.counters.accepted);
    EXPECT_EQ(C::FIRST, snapshot.streams[0].speed.continuity);
    EXPECT_EQ(1U, snapshot.streams[1].position.continuity_generation());
    // Returning identity can move to another slot: generation is driver-wide.
    ASSERT_TRUE(store.receive(1, s, Store::TIMEOUT_US + 101));
    ASSERT_TRUE(store.get(1, 31, Store::TIMEOUT_US + 101, out));
    EXPECT_NE(original_generation, out.identity_generation);
    EXPECT_EQ(10U, out.identity_generation);
    EXPECT_EQ(0U, out.position.counters.accepted);
    EXPECT_EQ(1U, out.speed.counters.accepted);
    store.get_snapshot(Store::TIMEOUT_US + 101, snapshot);
    EXPECT_EQ(1U, snapshot.statistics.position_saturation);
    EXPECT_EQ(1U, snapshot.statistics.speed_saturation);
}

TEST_F(MotorTelemetry, NodeRestartInvalidatesOnlyMatchingNodeStreams)
{
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    p.esc_index = s.esc_index = 0;
    ASSERT_TRUE(store.receive(42, p, 100));
    ASSERT_TRUE(store.receive(42, s, 100));
    ASSERT_TRUE(store.receive(43, p, 100));
    uavcan_protocol_NodeStatus status {};
    status.uptime_sec = 100;
    store.receive(42, status, 150);
    status.uptime_sec = 101;
    store.receive(42, status, 160);
    status.uptime_sec = 1;
    store.receive(42, status, 200);
    for (uint8_t esc : {
             0, 31
         }) {
        ASSERT_TRUE(store.get(42, esc, 200, out));
        EXPECT_EQ(1U, out.position.continuity_generation());
        EXPECT_EQ(1U, out.speed.continuity_generation());
        EXPECT_EQ(C::NODE_RESTART, out.position.last_break);
        EXPECT_EQ(C::NODE_RESTART, out.speed.continuity);
        EXPECT_FALSE(out.position.baseline_ready);
        EXPECT_FALSE(out.speed.baseline_ready);
        EXPECT_EQ(100U, out.position.timestamp_us);
        EXPECT_EQ(100U, out.speed.timestamp_us);
        EXPECT_EQ(10U, out.position.sample_seq);
        EXPECT_EQ(1U, out.position.counters.accepted);
    }
    ASSERT_TRUE(store.get(43, 0, 200, out));
    EXPECT_EQ(0U, out.position.continuity_generation());
    p.sample_seq = s.sample_seq = 0;
    ASSERT_TRUE(store.receive(42, p, 300));
    ASSERT_TRUE(store.receive(42, s, 300));
    ASSERT_TRUE(store.get(42, 0, 300, out));
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    EXPECT_EQ(1U, out.position.continuity_generation());
    p.sample_seq = s.sample_seq = 1;
    ASSERT_TRUE(store.receive(42, p, 400));
    ASSERT_TRUE(store.receive(42, s, 400));
    store.receive(42, status, 500); // Equal uptime is not another restart.
    ASSERT_TRUE(store.get(42, 0, 500, out));
    EXPECT_EQ(C::CONTIGUOUS, out.position.continuity);
    EXPECT_EQ(1U, out.position.continuity_generation());
    EXPECT_EQ(C::NODE_RESTART, out.position.last_break);
}

TEST_F(MotorTelemetry, UnknownNodeStatusDoesNotAllocateAndResetSurvivesExpiry)
{
    uavcan_protocol_NodeStatus status {};
    Store::Snapshot snapshot {};
    store.receive(0, status, 0);
    store.receive(42, status, 0);
    store.get_snapshot(0, snapshot);
    EXPECT_EQ(0U, snapshot.occupied_mask);
    EXPECT_EQ(0U, snapshot.statistics.allocations);
    p.sample_seq = s.sample_seq = 10;
    position(100);
    speed(100);
    status.uptime_sec = 100;
    store.receive(42, status, 200);
    status.uptime_sec = 0;
    store.receive(42, status, 300);
    store.update(Store::TIMEOUT_US + 100);
    p.sample_seq = s.sample_seq = 0;
    position(Store::TIMEOUT_US + 101);
    speed(Store::TIMEOUT_US + 101);
    EXPECT_EQ(C::FIRST, out.position.continuity);
    EXPECT_EQ(C::FIRST, out.speed.continuity);
    EXPECT_EQ(2U, out.position.continuity_generation());
    EXPECT_EQ(2U, out.speed.continuity_generation());
}

TEST_F(MotorTelemetry, CounterWrapIsDefinedAndIdentityResetIsExplicit)
{
    position(100);
    speed(100);
    auto &counters = AP_DroneCAN_MotorTelemetry_Test::position_counters(store);
    auto &speed_counters = AP_DroneCAN_MotorTelemetry_Test::speed_counters(store);
    counters = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    speed_counters = counters;
    p.valid = s.valid = false;
    EXPECT_FALSE(store.receive(42, p, 200));
    EXPECT_FALSE(store.receive(42, s, 200));
    ASSERT_TRUE(store.get(42, 31, 200, out));
    EXPECT_EQ(UINT32_MAX, out.position.counters.accepted);
    EXPECT_EQ(0U, out.position.counters.invalid);
    EXPECT_EQ(0U, out.position.counters.sequence_rejected);
    EXPECT_EQ(0U, out.position.continuity_generation());
    EXPECT_EQ(UINT32_MAX, out.speed.counters.accepted);
    EXPECT_EQ(0U, out.speed.counters.invalid);
    EXPECT_EQ(0U, out.speed.counters.sequence_rejected);
    EXPECT_EQ(0U, out.speed.continuity_generation());
    p.sample_seq++;
    s.sample_seq++;
    position(300);
    speed(300);
    EXPECT_EQ(0U, out.speed.counters.accepted);
    EXPECT_EQ(1U, out.speed.counters.invalid);
    EXPECT_EQ(1U, out.speed.continuity_generation());
    EXPECT_EQ(0U, out.position.counters.accepted);
    EXPECT_EQ(1U, out.position.counters.invalid);
    EXPECT_EQ(1U, out.position.continuity_generation());
    auto &statistics = AP_DroneCAN_MotorTelemetry_Test::statistics(store);
    statistics = {UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
    EXPECT_FALSE(store.receive(0, p, 400));
    for (uint8_t node = 1; node < Store::CAPACITY; node++) {
        ASSERT_TRUE(store.receive(node, p, 400));
    }
    EXPECT_FALSE(store.receive(99, p, 400));
    EXPECT_FALSE(store.receive(99, s, 400));
    Store::Snapshot snapshot {};
    store.get_snapshot(400, snapshot);
    EXPECT_EQ(0U, snapshot.statistics.invalid_identity);
    EXPECT_EQ(0U, snapshot.statistics.position_saturation);
    EXPECT_EQ(0U, snapshot.statistics.speed_saturation);
    EXPECT_EQ(0U, snapshot.streams[1].identity_generation); // Allocation wrap.
    s.valid = true;
    ASSERT_TRUE(store.receive(99, s, Store::TIMEOUT_US + 400));
    ASSERT_TRUE(store.get(99, 31, Store::TIMEOUT_US + 400, out));
    EXPECT_EQ(0U, out.position.counters.accepted);
    EXPECT_EQ(1U, out.speed.counters.accepted);
    EXPECT_EQ(0U, out.speed.counters.invalid);
    EXPECT_EQ(0U, out.speed.continuity_generation());
}

// Deliver encoded payloads through the actual AP_DroneCAN subscribers, decode
// wrappers and ObjCallbacks. Transport CRC/reassembly and hardware are outside
// this test; the complete transfer payload is supplied to HandlerList.
template <typename Message>
static void deliver(uint8_t driver_index, uint8_t source_node_id, Message &msg)
{
    uint8_t payload[Message::cxx_iface::MAX_SIZE] {};
    CanardRxTransfer transfer {};
    transfer.source_node_id = source_node_id;
    transfer.data_type_id = Message::cxx_iface::ID;
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.payload_head = payload;
#if CANARD_ENABLE_CANFD || CANARD_ENABLE_TAO_OPTION
    transfer.payload_len = Message::cxx_iface::encode(&msg, payload, true);
#if CANARD_ENABLE_TAO_OPTION
    transfer.tao = true;
#endif
#else
    transfer.payload_len = Message::cxx_iface::encode(&msg, payload);
#endif
    uint64_t signature = 0;
    ASSERT_TRUE(Canard::HandlerList::accept_message(driver_index, transfer.data_type_id,
                CanardTransferTypeBroadcast, signature));
    EXPECT_TRUE(Message::cxx_iface::SIGNATURE == signature);
    Canard::HandlerList::handle_message(driver_index, transfer);
}

TEST_F(MotorTelemetry, NativeSubscriptionsSeparateDriversAndSources)
{
    static AP_DroneCAN driver0(0);
    static AP_DroneCAN driver1(1);
    Store::Stream other {};
    p.position.ticks.ticks = -7654321;
    deliver(0, 42, p);
    ASSERT_TRUE(driver0.get_motor_telemetry(42, 31, out));
    EXPECT_EQ(-7654321, out.position.value.ticks.ticks);
    EXPECT_FALSE(driver1.get_motor_telemetry(42, 31, other));
    const uint64_t position_time = out.position.timestamp_us;
    p.position.ticks.ticks = 99;
    deliver(0, 42, p); // Duplicate through the real subscriber.
    ASSERT_TRUE(driver0.get_motor_telemetry(42, 31, out));
    EXPECT_EQ(-7654321, out.position.value.ticks.ticks);
    EXPECT_EQ(position_time, out.position.timestamp_us);
    EXPECT_EQ(C::SEQUENCE, out.position.continuity);
    s.rpm = -0.0625f;
    deliver(1, 42, s);
    ASSERT_TRUE(driver1.get_motor_telemetry(42, 31, other));
    EXPECT_FLOAT_EQ(-0.0625f, other.speed.rpm);
    EXPECT_FALSE(other.position.received);
    const uint64_t speed_time = other.speed.timestamp_us;
    s.sample_seq = 255; // Presumed old relative to the accepted sequence 0.
    s.rpm = 99.0f;
    deliver(1, 42, s);
    ASSERT_TRUE(driver1.get_motor_telemetry(42, 31, other));
    EXPECT_FLOAT_EQ(-0.0625f, other.speed.rpm);
    EXPECT_EQ(0, other.speed.sample_seq);
    EXPECT_EQ(speed_time, other.speed.timestamp_us);
    EXPECT_EQ(C::SEQUENCE, other.speed.continuity);
    p.position.union_tag = UAVCAN_EQUIPMENT_ESC_MOTORPOSITIONVALUE_REVOLUTIONS;
    p.position.revolutions = 1.125f;
    deliver(0, 43, p);
    ASSERT_TRUE(driver0.get_motor_telemetry(43, 31, other));
    EXPECT_FLOAT_EQ(1.125f, other.position.value.revolutions);
    ASSERT_TRUE(driver0.get_motor_telemetry(42, 31, out));
    EXPECT_EQ(-7654321, out.position.value.ticks.ticks);
}
static AP_DroneCAN &snapshot_test_driver()
{
    static AP_DroneCAN driver(2);
    return driver;
}

TEST_F(MotorTelemetry, NativeNodeStatusAndSnapshotWithoutLogger)
{
    auto &driver = snapshot_test_driver();
    p.sample_seq = s.sample_seq = 10;
    deliver(2, 127, p);
    deliver(2, 127, s);
    uavcan_protocol_NodeStatus status {};
    status.uptime_sec = 100;
    deliver(2, 127, status);
    status.uptime_sec = 0;
    deliver(2, 127, status);
    ASSERT_TRUE(driver.get_motor_telemetry(127, 31, out));
    EXPECT_EQ(C::NODE_RESTART, out.position.continuity);
    EXPECT_EQ(1U, out.position.continuity_generation());
    EXPECT_EQ(1U, out.speed.continuity_generation());
    Store::Snapshot snapshot {};
    driver.get_motor_telemetry_snapshot(snapshot);
    EXPECT_NE(0U, snapshot.occupied_mask);
    bool found = false;
    for (uint8_t i = 0; i < Store::CAPACITY; i++) {
        if ((snapshot.occupied_mask & (1U << i)) != 0 && snapshot.streams[i].source_node_id == 127) {
            EXPECT_EQ(out.identity_generation, snapshot.streams[i].identity_generation);
            EXPECT_EQ(out.position.timestamp_us, snapshot.streams[i].position.timestamp_us);
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(MotorTelemetry, NativeSnapshotsAreAtomicDuringReception)
{
    auto &driver = snapshot_test_driver();
    std::atomic<bool> done {false};
    std::thread receiver([&]() {
        auto msg = p;
        msg.esc_index = 0;
        for (int32_t i = 1; i <= 1000; i++) {
            msg.sample_seq = uint8_t(i);
            msg.position.ticks.ticks = i;
            deliver(2, 126, msg);
        }
        done.store(true);
    });
    Store::Snapshot snapshot {};
    do {
        driver.get_motor_telemetry_snapshot(snapshot);
        for (uint8_t i = 0; i < Store::CAPACITY; i++) {
            if ((snapshot.occupied_mask & (1U << i)) == 0 || snapshot.streams[i].source_node_id != 126) {
                continue;
            }
            const auto &sample = snapshot.streams[i].position;
            EXPECT_EQ(sample.counters.accepted, uint32_t(sample.value.ticks.ticks));
            EXPECT_EQ(uint8_t(sample.counters.accepted), sample.sample_seq);
        }
    } while (!done.load());
    receiver.join();
    ASSERT_TRUE(driver.get_motor_telemetry(126, 0, out));
    EXPECT_EQ(1000U, out.position.counters.accepted);
    EXPECT_EQ(1000, out.position.value.ticks.ticks);
}

#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED

AP_GTEST_MAIN()
