#include <AP_gtest.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <limits>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_DRONECAN_MOTOR_TELEMETRY_ENABLED
using Store = AP_DroneCAN_MotorTelemetry;
using C = Store::Continuity;

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
#endif // AP_DRONECAN_MOTOR_TELEMETRY_ENABLED

AP_GTEST_MAIN()
