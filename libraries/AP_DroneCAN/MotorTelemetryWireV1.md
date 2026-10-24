# Experimental motor observation wire format V1

This local experiment carries the native motor observations held by
`AP_DroneCAN_MotorTelemetry`. It is not odometry, a motor command, or a public
MAVLink allocation. This implementation and its tests are AI-assisted.

## Envelope and compatibility

- MAVLink message: `TUNNEL`, ID **385**, as defined by the pinned MAVLink XML.
- Local `payload_type`: **32769 / 0x8001**. It is **not registered**, globally
  unique or reserved. Do not distribute it as a public allocation. Both endpoints
  must explicitly agree to this experiment; a different local experiment may
  collide with this value. Public integration requires registration or dedicated
  messages through the appropriate upstream process.
- One autonomous record per TUNNEL payload: position **104 bytes**, speed
  **96 bytes**, both within the **128-byte** payload capacity. No fragmentation.
- Fill `payload_length` with exactly 104 or 96. Zero the unused part of the
  128-byte array before MAVLink packing. The codec writes only the record bytes.
- A receiver must first validate the MAVLink frame (including CRC), destination
  and payload_type, then pass exactly `payload_length` accessible bytes to decode.
  Do not pass the array capacity (128) as the record length.
- MAVLink 2 may truncate trailing zero bytes. The generated TUNNEL decoder
  restores the zero-filled array before the inner codec is called. Legal MAVLink
  zero truncation is different from a truncated inner payload.
- Message 385 requires MAVLink 2. The pinned library also has wider system-ID
  header extensions: standard MAVLink interoperability requires compatible
  endpoints or system IDs <=255 without those extensions.
- No send, subscription, scheduler or session-ID generator exists in this lot.
  The codec and its logical Record can be used independently of TUNNEL. A future
  dedicated MAVLink message can map the same logical fields without adopting
  this private payload_type or copying the Record's memory layout.

## Scalar encoding

Offsets below start at byte zero of TUNNEL.payload. All multi-byte scalars use
**little-endian** encoding, without padding or implicit alignment. Signed ticks
use the exact 32-bit two's-complement representation. Floats are **IEEE-754
binary32**, preserving signed zero, fractions and subnormals. There are no
integer-RPM, gearing, wheel-radius, distance or integration conversions.

Only the active position representation is encoded. Unknown representations are
rejected, including for an unusable sample. Non-finite raw float bits and zero
resolution can be retained when USABLE is false. They must never be consumed as
usable measurements. VALID is the publisher's bit; USABLE is the store's separate
numerical validation result. VALID=false does not make the raw value meaningful.

The C++ Record is a logical object, not a packed wire struct. Encode each field;
never transmit sizeof(Record), padding, a native enum, or a whole C++ object.
The implementation uses memcpy only for individual float bit representations.

## Common prefix: 92 bytes

| Offset | Bytes | Type | Field / meaning |
| ---: | ---: | --- | --- |
| 0 | 2 | bytes | Magic: `4d 54` (ASCII MT) |
| 2 | 1 | uint8 | Version: 1 |
| 3 | 1 | uint8 | Record type: 1 position, 2 speed |
| 4 | 1 | uint8 | Total inner record length: 104 or 96 |
| 5 | 1 | uint8 | Reserved, must be zero |
| 6 | 1 | uint8 | Driver index, copied from caller context, not a physical bus |
| 7 | 1 | uint8 | DroneCAN source_node_id, 1..127 |
| 8 | 1 | uint8 | Raw esc_index, 0..31; no ESC offset |
| 9 | 1 | bitmask | Sample flags below |
| 10 | 1 | uint8 | Source sample_seq, independent for each type |
| 11 | 1 | uint8 | Current continuity code below |
| 12 | 1 | uint8 | Retained last_break code below |
| 13 | 1 | bitmask | Context flags below |
| 14 | 2 | uint16 | Reserved, must be zero |
| 16 | 8 | uint64 | Caller-owned export session correlation tag |
| 24 | 8 | uint64 | Snapshot timestamp, microseconds since controller boot |
| 32 | 8 | uint64 | Last store maintenance timestamp, same clock |
| 40 | 8 | uint64 | Retained sample reception timestamp, same clock |
| 48 | 4 | uint32 | Identity generation of the allocated store entry |
| 52 | 4 | uint32 | Accepted transfers for this identity/type |
| 56 | 4 | uint32 | Invalid payloads for this identity/type |
| 60 | 4 | uint32 | Sequence rejections for this identity/type |
| 64 | 4 | uint32 | Continuity breaks, also the continuity generation |
| 68 | 4 | uint32 | Driver-wide allocation counter |
| 72 | 4 | uint32 | Driver-wide position saturation rejections |
| 76 | 4 | uint32 | Driver-wide speed saturation rejections |
| 80 | 4 | uint32 | Driver-wide malformed motor identity counter |
| 84 | 4 | uint32 | Caller-owned export sequence, when known |
| 88 | 4 | uint32 | Caller-owned cumulative superseded-sample count, when known |

Sample flags at offset 9:

| Bit | Meaning |
| ---: | --- |
| 0 | RECEIVED: at least one sample of this type has been admitted |
| 1 | VALID: retained publisher valid bit |
| 2 | USABLE: retained numerical validity result |
| 3 | FRESH: fresh at snapshot capture time |
| 4 | BASELINE_READY: store candidate baseline in this continuity generation |
| 5..7 | Reserved, must be zero |

Context flags at offset 13:

| Bit | Meaning |
| ---: | --- |
| 0 | SESSION_TAG_PRESENT: tag supplied; **not** proof of uniqueness |
| 1 | DELIVERY_COUNTERS_VALID: export_seq and superseded_count supplied |
| 2..7 | Reserved, must be zero |

With SESSION_TAG_PRESENT clear, the session field must be zero. With
DELIVERY_COUNTERS_VALID clear, both delivery counters must be zero. A present
zero tag and known zero delivery counters are valid, distinguished by flags.

Continuity and last_break have an explicit mapping, independent of C++ enum ABI:

| Code | Meaning |
| ---: | --- |
| 0 | FIRST |
| 1 | CONTIGUOUS: only the store's sequence/time/metadata criteria passed |
| 2 | SEQUENCE |
| 3 | TIMEOUT |
| 4 | INVALID |
| 5 | ORIGIN |
| 6 | REPRESENTATION |
| 7 | RESOLUTION |
| 8 | NODE_RESTART |

CONTIGUOUS never proves physical continuity or continuity between two exports.
last_break can remain INVALID, for example, while continuity is CONTIGUOUS.
A change of continuity_breaks reveals an intervening break despite recovery.
A missing sample can be reported for an assigned identity: RECEIVED, VALID,
USABLE, FRESH and BASELINE_READY are then all false. Its raw value is not data.

## Position suffix: 12 bytes

| Offset | Bytes | Type | Field |
| ---: | ---: | --- | --- |
| 92 | 4 | uint32 | origin_id, unchanged |
| 96 | 1 | uint8 | Representation: 0 native ticks, 1 mechanical revolutions |
| 97 | 1 | uint8 | Reserved, must be zero |
| 98 | 2 | uint16 | ticks_per_revolution for ticks; canonical zero for revolutions |
| 100 | 4 | int32 / float32 | Native signed ticks / mechanical revolutions |

No tick subtraction or unwrapping is performed. INT32_MIN and INT32_MAX are
ordinary raw values. Tick resolution zero is permitted only when USABLE=false.
Float revolutions must be finite when USABLE=true. The logical Record's inactive
position fields are ignored on encode and zeroed on decode.

## Speed suffix: 4 bytes

| Offset | Bytes | Type | Field |
| ---: | ---: | --- | --- |
| 92 | 4 | float32 | Signed mechanical shaft RPM |

Positive zero, negative zero and negative/fractional RPM are retained exactly.
Non-finite values require USABLE=false. Position fields of the logical Record
are inactive and zeroed on decode.

## Sessions, generations and counter lifetime

The codec does not generate or certify a session identity. A caller-provided tag
is a correlation hint scoped to the controller/export context. A random nonce
can collide; an unsupported or failed generator must not be represented as a
guaranteed unique boot ID. Without a reliable tag, the receiver needs an explicit
new-context boundary outside this codec and must rebaseline on reconnect/reset.
Even a present tag never proves physical continuity or authenticates a motor.

The future exporter must renew its context/tag (or declare session unknown and
require a new-context boundary) on controller boot, store recreation or deliberate
counter resets. Resetting delivery counters also requires a new export context.
A new subscription/link starts a fresh consumer baseline. No negotiation or
lifetime management is implemented here.

Store counters are copied with A2.1 semantics: accepted includes admitted invalid
samples; invalid includes sequence-rejected invalid payloads; sequence_rejected
and continuity_breaks are independent counters. Statistics are driver-wide and
are repeated in every record, not attributed to its particular motor. No record
is fabricated for an unassigned entry just to carry statistics.

All counters and generations are uint32 modulo 2^32. Zero is not an absence
sentinel. Equality cannot rule out a complete cycle. Compare counters only within
a known context and the same (driver, node, raw ESC, identity generation, type).
The driver-wide allocation serial changes on entry reassignment; sample counters
then restart. An observed node restart changes continuity generations without
resetting accepted counts or the raw sample timestamp. Controller reboot is a
separate session boundary. Comparisons across unknown contexts are not justified.

## Snapshot replacement and delivery counters

The policy is **latest accepted snapshot**, independently for position and speed.
No queue, averaging, interpolation, acquisition reconstruction or replay is added.
The adapter is a pure projection of one assigned entry in an atomic A2.1 snapshot.
It does not call update(), change counters, recompute freshness or remember readers.
The caller must supply the actual driver index corresponding to that snapshot.

For a future exporter choosing to supply delivery counters:

- Scope them to one recipient/export context, driver, native identity, identity
  generation and record type. Never share a destructive read cursor among links.
- export_seq counts records handed successfully to the local outbound transport,
  modulo 2^32. Preparing or failing to enqueue a packet does not advance it.
  A replay of the same record retains its sequence. This is not a delivery ACK.
- superseded_count counts accepted samples replaced before export, cumulatively
  modulo 2^32. Invalid admitted samples count as accepted samples here too.
- Given a known previous exported accepted count A and a new count B in the same
  context, let d=(B-A) modulo 2^32, only if the interval/rate bounds make it
  unambiguous. Add d-1 for d>0, and zero for d=0. For example 1→4 skips samples
  2 and 3: add two, not three. Rejects and status-only changes do not count as
  superseded accepted samples. Do not claim a count for unknown/full-cycle gaps.
- On the first export or an unknown delivery baseline, do not invent a history.
  Clear DELIVERY_COUNTERS_VALID and its fields until the external context can
  define those counters. The codec validates the flag/zero rules, not this policy.

No exporter exists in A2.2a. The two fields are supplied unchanged by Context.
A gap in export_seq indicates absent/reordered records under the agreed ordering;
it does not reveal their values. Counter differences and last_break expose
coalescing and intervening breaks, but cannot reconstruct individual samples or
resolve sample_seq aliases modulo 256. A future consumer must not treat two
snapshots as physically contiguous merely because the last raw state says so.

## Time, freshness and validation

All three timestamps are controller monotonic microseconds, never UNIX time.
Do not infer a clock type from magnitude. The sample timestamp is local receipt,
not ESC acquisition. snapshot_us is capture time, not actual MAVLink send time.
The maintenance timestamp can precede capture: a read never creates an expiry
event. FRESH is copied at capture time and says nothing about network delay or
freshness at arrival. Repeated status exports retain the sample timestamp.

The codec rejects unknown version/type/representation/continuity codes, wrong
magic, nonzero reserved bits/bytes, inconsistent inner/actual lengths and all
truncated records. V1 requires exact sizes; it does not accept trailing bytes as
extensions. A new layout requires a new version with a documented contract.
It also rejects impossible basic flags (VALID/USABLE/FRESH/BASELINE_READY without
RECEIVED; USABLE without VALID; BASELINE_READY without USABLE and FRESH), timestamps
later than snapshot_us, noncanonical float-mode resolution, and non-finite usable
values. It does not certify sensor health or validate physical plausibility.

Malformed input leaves the decoded Record unchanged. Encode validates before
writing; failure leaves output bytes and output length unchanged. Null pointers
are INVALID_ARGUMENT. Too-short/too-long accessible data is BAD_LENGTH; after a
minimum five-byte header, validation checks magic, version, kind and exact length
before accessing the remainder. All reserved fields are explicitly zeroed on
encode. Buffers and logical inputs must not overlap; callers must initialize
logical records and provide at least the capacity they declare.

## Reference vectors

The independent Python struct.pack reference is
`tests/motor_telemetry_v1_reference.py`. Its common layout is `<2s12BH4Q11I`
(92 bytes), with suffix `<IBBHi`, `<IBBHf` or `<f`. The C++ tests contain literal
vectors produced by that reference, not by the production encoder, and compare
every byte. Run it to obtain JSON payloads for ticks, revolutions, negative RPM
and both zero signs. JSON common uint64 values use hex strings to avoid loss in
consumers limited to 53-bit integer precision.

The following synthetic vectors exercise byte order and integer width; their
large timestamps are test values, not realistic controller uptimes. They use
session tag 0x0102030405060708, driver 2, node 42, ESC 31, source sequence 255,
CONTIGUOUS with retained last_break INVALID, and nonzero continuity generation.

### Ticks INT32_MIN, resolution 65535, origin 0x89abcdef

104 bytes:

```text
0000: 4d 54 01 01 68 00 02 2a 1f 1f ff 01 04 03 00 00
0010: 08 07 06 05 04 03 02 01 00 03 00 00 00 00 00 10
0020: 00 02 00 00 00 00 00 10 00 01 00 00 00 00 00 10
0030: ff ff ff ff ff ff ff ff 04 03 02 01 40 30 20 10
0040: 88 77 66 55 ff ff ff ff 0d 0c 0b 0a 14 13 12 11
0050: 24 23 22 21 34 33 32 31 44 43 42 41 ef cd ab 89
0060: 00 00 ff ff 00 00 00 80
```

### Mechanical revolutions -1.25, origin 0x89abcdef

104 bytes:

```text
0000: 4d 54 01 01 68 00 02 2a 1f 1f ff 01 04 03 00 00
0010: 08 07 06 05 04 03 02 01 00 03 00 00 00 00 00 10
0020: 00 02 00 00 00 00 00 10 00 01 00 00 00 00 00 10
0030: ff ff ff ff ff ff ff ff 04 03 02 01 40 30 20 10
0040: 88 77 66 55 ff ff ff ff 0d 0c 0b 0a 14 13 12 11
0050: 24 23 22 21 34 33 32 31 44 43 42 41 ef cd ab 89
0060: 01 00 00 00 00 00 a0 bf
```

### Mechanical RPM -0.125

96 bytes:

```text
0000: 4d 54 01 02 60 00 02 2a 1f 1f ff 01 04 03 00 00
0010: 08 07 06 05 04 03 02 01 00 03 00 00 00 00 00 10
0020: 00 02 00 00 00 00 00 10 00 01 00 00 00 00 00 10
0030: ff ff ff ff ff ff ff ff 04 03 02 01 40 30 20 10
0040: 88 77 66 55 ff ff ff ff 0d 0c 0b 0a 14 13 12 11
0050: 24 23 22 21 34 33 32 31 44 43 42 41 00 00 00 be
```

### Ticks INT32_MAX, resolution 65535, origin 0x89abcdef

104 bytes:

```text
0000: 4d 54 01 01 68 00 02 2a 1f 1f ff 01 04 03 00 00
0010: 08 07 06 05 04 03 02 01 00 03 00 00 00 00 00 10
0020: 00 02 00 00 00 00 00 10 00 01 00 00 00 00 00 10
0030: ff ff ff ff ff ff ff ff 04 03 02 01 40 30 20 10
0040: 88 77 66 55 ff ff ff ff 0d 0c 0b 0a 14 13 12 11
0050: 24 23 22 21 34 33 32 31 44 43 42 41 ef cd ab 89
0060: 00 00 ff ff ff ff ff 7f
```
