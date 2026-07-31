"""Tests for the LiTime BMS frame parser.

Only parse_response is covered: it reads hand-written binary at hardcoded struct offsets, so a
wrong offset yields a plausible-looking number instead of an error. The rest of the node is
arithmetic and BLE plumbing whose failures are obvious as soon as it runs.
"""

import struct

from duatic_monitoring.battery_monitor_node import MIN_FRAME_LEN, parse_response

# Long enough that every optional field (soh, cycles) is present and the trailing checksum byte
# does not overlap one of them.
FRAME_LEN = 104


def build_frame(
    voltage_mv=26400,
    cell_mv=(3300, 3300, 3300, 3300),
    current_ma=5000,
    temps=(21, 22, 0, 0, 0),
    remaining_ah_x100=8500,
    capacity_ah_x100=10000,
    battery_state=0,
    soc=87,
    soh=98,
    cycles=42,
    declared_len=FRAME_LEN,
):
    """Build a BMS status frame with a valid checksum."""
    frame = bytearray(FRAME_LEN)
    # Byte 2 declares the payload length; the 4 framing bytes are not counted in it.
    frame[2] = declared_len - 4

    struct.pack_into("<H", frame, 12, voltage_mv)
    for i, mv in enumerate(cell_mv):
        struct.pack_into("<H", frame, 16 + i * 2, mv)
    struct.pack_into("<i", frame, 48, current_ma)
    for i, t in enumerate(temps):
        struct.pack_into("<h", frame, 52 + i * 2, t)
    struct.pack_into("<H", frame, 62, remaining_ah_x100)
    struct.pack_into("<I", frame, 64, capacity_ah_x100)
    struct.pack_into("<H", frame, 88, battery_state)
    struct.pack_into("<H", frame, 90, soc)
    struct.pack_into("<I", frame, 92, soh)
    struct.pack_into("<I", frame, 96, cycles)

    frame[FRAME_LEN - 1] = sum(frame[: FRAME_LEN - 1]) & 0xFF
    return bytes(frame)


def test_a_valid_frame_parses_into_physical_units():
    parsed = parse_response(build_frame())

    assert parsed is not None
    assert parsed["voltage"] == 26.4
    assert parsed["current"] == 5.0
    assert parsed["soc"] == 87
    assert parsed["remaining_ah"] == 85.0
    assert parsed["capacity_ah"] == 100.0
    assert parsed["cycles"] == 42
    assert parsed["soh"] == 98
    # Unpopulated cells and temperature sensors read as 0 and are dropped, except the first.
    assert parsed["cell_voltages"] == [3.3, 3.3, 3.3, 3.3]
    assert parsed["temps"] == [21.0, 22.0]


def test_a_bad_checksum_is_rejected():
    frame = bytearray(build_frame())
    frame[FRAME_LEN - 1] ^= 0xFF

    assert parse_response(bytes(frame)) is None


def test_a_too_short_frame_is_rejected():
    # Fewer bytes than the mandatory fields need.
    assert parse_response(build_frame()[: MIN_FRAME_LEN - 1]) is None

    # Enough bytes, but the frame itself declares a length the mandatory fields do not fit in.
    assert parse_response(build_frame(declared_len=64)) is None

    # Declares more than actually arrived.
    assert parse_response(build_frame()[:-1]) is None


def test_a_trailing_partial_frame_does_not_bleed_into_the_result():
    frame = build_frame(voltage_mv=26400, soc=87)
    partial = build_frame(voltage_mv=13200, soc=12)[:40]

    assert parse_response(frame + partial) == parse_response(frame)
