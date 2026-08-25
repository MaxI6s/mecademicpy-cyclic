"""White-box tests of the wire encoding used by the simulated robot.

The helpers under test are private to :mod:`mock_robot.server`, but they are
what makes the mock a faithful EtherNet/IP adapter, so they are worth pinning
down on their own -- and they document the frame layout for anyone porting the
project.
"""

import socket
import struct

import pytest

from mock_robot.server import (
    _build_cpf,
    _parse_connection_path,
    _parse_cpf,
    _parse_socket_info_port,
    _strip_request_path,
    build_cyclic_frame,
    parse_cyclic_frame,
    strip_run_idle_header,
)

#: Item type of a connected data item.
_CONNECTED_DATA = 0x00B1
#: Item type of a sequenced address item.
_SEQUENCED_ADDRESS = 0x8002


def build_scanner_frame(connection_id: int, sequence: int, payload: bytes) -> bytes:
    """Build the frame a scanner produces, the way ``ethernetip`` does.

    Args:
        connection_id: Originator-to-target connection identifier.
        sequence: 32-bit sequence number.
        payload: Sequence count, run/idle header and assembly image.

    Returns:
        The datagram a scanner would send.
    """
    return (
        struct.pack("<HHH", 2, _SEQUENCED_ADDRESS, 8)
        + struct.pack("<II", connection_id, sequence)
        + struct.pack("<HH", _CONNECTED_DATA, len(payload))
        + payload
    )


def test_cyclic_frame_round_trip() -> None:
    """A frame this server produces can be parsed back by the same rules."""
    assembly = bytes(range(64))
    frame = build_cyclic_frame(0x12345678, 9, assembly)
    connection_id, payload = parse_cyclic_frame(frame)
    assert connection_id == 0x12345678
    assert payload == assembly


def test_parse_scanner_frame() -> None:
    """A frame produced by a scanner is parsed down to its assembly image."""
    assembly = bytes([0xAA]) * 64
    payload = struct.pack("<H", 3) + b"\x01\x00\x00\x00" + assembly
    connection_id, parsed = parse_cyclic_frame(build_scanner_frame(0xDEADBEEF, 3, payload))
    assert connection_id == 0xDEADBEEF
    assert strip_run_idle_header(parsed, 64) == assembly


def testparse_cyclic_frame_rejects_garbage() -> None:
    """A datagram that is not a cyclic frame is rejected, not misread."""
    with pytest.raises(ValueError):
        parse_cyclic_frame(b"\x00")
    with pytest.raises(ValueError):
        parse_cyclic_frame(_build_cpf({0x0000: b""}))


def teststrip_run_idle_header_handles_both_layouts() -> None:
    """The run/idle header is optional and told apart by the payload size."""
    assembly = bytes(8)
    assert strip_run_idle_header(assembly, 8) == assembly
    assert strip_run_idle_header(b"\x01\x00\x00\x00" + assembly, 8) == assembly
    assert strip_run_idle_header(bytes(5), 8) is None


def test_cpf_round_trip() -> None:
    """Items survive a build/parse cycle."""
    items = {0x0000: b"", 0x00B2: b"\x54\x02payload"}
    assert _parse_cpf(_build_cpf(items)) == items


def test_parse_cpf_rejects_truncated_data() -> None:
    """A structure announcing more items than it carries is rejected."""
    with pytest.raises(ValueError):
        _parse_cpf(struct.pack("<HHH", 2, 0x00B2, 8))


def test_strip_request_path_removes_service_and_path() -> None:
    """The CIP service and its request path are removed from a message."""
    request = bytes([0x54, 0x02, 0x20, 0x06, 0x24, 0x01]) + b"body"
    assert _strip_request_path(request) == b"body"


def test_strip_request_path_rejects_truncated_message() -> None:
    """A message whose path is cut short is rejected."""
    assert _strip_request_path(bytes([0x54, 0x08, 0x20])) is None


def test_parse_connection_path_finds_the_connection_points() -> None:
    """Both connection points are extracted, output first then input."""
    path = (
        struct.pack(">H", 0x3404)
        + struct.pack("<HHH", 0, 0, 0)
        + bytes([0x00, 0x00])  # Electronic key: vendor, device type, product, revision.
        + bytes([0x20, 0x04])  # Class 4 (Assembly).
        + bytes([0x24, 0x01])  # Configuration instance 1.
        + bytes([0x2C, 150])  # Output assembly.
        + bytes([0x2C, 100])  # Input assembly.
    )
    assert _parse_connection_path(path) == [150, 100]


def test_parse_connection_path_handles_16_bit_instances() -> None:
    """Connection points above 255 use the 16 bit segment form."""
    path = bytes([0x20, 0x04]) + bytes([0x2D]) + struct.pack("<H", 400)
    assert _parse_connection_path(path) == [400]


def test_parse_socket_info_port() -> None:
    """The UDP port the scanner listens on is read from the socket item."""
    item = struct.pack(">HHI", socket.AF_INET, 51234, 0) + bytes(8)
    assert _parse_socket_info_port(item) == 51234
    assert _parse_socket_info_port(None) is None
    assert _parse_socket_info_port(b"\x00") is None
