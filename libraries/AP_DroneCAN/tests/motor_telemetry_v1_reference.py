#!/usr/bin/env python3
"""Independent struct.pack vectors for MotorTelemetryWireV1.md, not flight code."""
# AP_FLAKE8_CLEAN

import json
import struct


HEADER = struct.Struct("<2s12BH4Q11I")
assert HEADER.size == 92


def vector(kind, representation=0, value=None):
    length = 104 if kind == 1 else 96
    payload = HEADER.pack(
        b"MT", 1, kind, length, 0, 2, 42, 31, 31, 255, 1, 4, 3, 0,
        0x0102030405060708,
        0x1000000000000300, 0x1000000000000200, 0x1000000000000100,
        0xFFFFFFFF, 0xFFFFFFFF, 0x01020304, 0x10203040, 0x55667788,
        0xFFFFFFFF, 0x0A0B0C0D, 0x11121314, 0x21222324, 0x31323334, 0x41424344,
    )
    if kind == 1:
        if representation == 0:
            ticks = -2147483648 if value is None else value
            payload += struct.pack("<IBBHi", 0x89ABCDEF, 0, 0, 65535, ticks)
        else:
            payload += struct.pack("<IBBHf", 0x89ABCDEF, 1, 0, 0, value)
    else:
        payload += struct.pack("<f", value)
    assert len(payload) == length <= 128
    return payload


def main():
    examples = {
        "position_ticks": vector(1),
        "position_ticks_max": vector(1, value=2147483647),
        "position_revolutions": vector(1, 1, -1.25),
        "speed_negative_fraction": vector(2, value=-0.125),
        "speed_positive_zero": vector(2, value=0.0),
        "speed_negative_zero": vector(2, value=-0.0),
    }
    print(json.dumps({
        "payload_type": 32769,
        "allocation": "unregistered local experiment only",
        "session": "caller-supplied correlation tag; uniqueness not guaranteed",
        "common_fields": {
            "driver_index": 2, "source_node_id": 42, "esc_index": 31,
            "sample_flags": 31, "sample_seq": 255, "continuity": 1, "last_break": 4, "context_flags": 3,
            "session_id": "0x0102030405060708",
            "snapshot_us": "0x1000000000000300", "updated_us": "0x1000000000000200",
            "sample_us": "0x1000000000000100", "identity_generation": 0xFFFFFFFF,
            "accepted": 0xFFFFFFFF, "invalid": 0x01020304, "sequence_rejected": 0x10203040,
            "continuity_breaks": 0x55667788, "allocations": 0xFFFFFFFF,
            "position_saturation": 0x0A0B0C0D, "speed_saturation": 0x11121314,
            "invalid_identity": 0x21222324, "export_seq": 0x31323334, "superseded_count": 0x41424344,
        },
        "examples": {name: {"payload_length": len(data), "hex": data.hex(" ")}
                     for name, data in examples.items()},
    }, indent=2))


if __name__ == "__main__":
    main()
