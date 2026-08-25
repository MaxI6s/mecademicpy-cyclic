"""Tests of the dependency-free originator used by the diagnostic tool.

It is hand-written CIP, so it is worth pinning down: its encoding is checked
against the adapter side of the same exchange, :mod:`mock_robot.server`.
"""

import socket
import struct

import pytest

from mecademic_fieldbus.io_map import IoMap
from tools.eip_originator import (
    ITEM_SOCKADDR_TARGET_TO_ORIGINATOR,
    EipOriginator,
    SocketAddress,
    build_output_frame,
    encode_connection_path,
    is_multicast,
    parse_input_frame,
)

#: Item type of the originator-to-target socket address.
ITEM_SOCKADDR_ORIGINATOR_TO_TARGET = 0x8000


# ----------------------------------------------------------------------
# Encoding helpers
# ----------------------------------------------------------------------
def test_socket_address_round_trip() -> None:
    """A socket address item survives an encode/decode cycle."""
    address = SocketAddress("192.168.0.100", 2222)
    decoded = SocketAddress.parse(address.encode())
    assert decoded is not None
    assert decoded.address == "192.168.0.100"
    assert decoded.port == 2222
    assert repr(decoded) == "192.168.0.100:2222"


def test_socket_address_is_big_endian() -> None:
    """The item is big endian, unlike the rest of CIP."""
    encoded = SocketAddress("1.2.3.4", 0x08AE).encode()
    family, port = struct.unpack_from(">HH", encoded, 0)
    assert family == socket.AF_INET
    assert port == 0x08AE
    assert encoded[4:8] == bytes([1, 2, 3, 4])


def test_socket_address_rejects_a_truncated_item() -> None:
    """A short item is reported as unusable rather than misread."""
    assert SocketAddress.parse(b"\x00\x02") is None


def test_multicast_range() -> None:
    """The multicast range is recognised, and nothing else is."""
    assert is_multicast("239.192.1.5") is True
    assert is_multicast("224.0.0.1") is True
    assert is_multicast("192.168.0.100") is False
    assert is_multicast("240.0.0.1") is False
    assert is_multicast("not an address") is False


def test_output_frame_round_trip(io_map: IoMap) -> None:
    """A produced frame parses back to the assembly it carried."""
    from mock_robot.server import parse_cyclic_frame, strip_run_idle_header

    assembly = bytes(range(io_map.output_assembly_size))
    frame = build_output_frame(0x12345678, 7, assembly)
    connection_id, payload = parse_cyclic_frame(frame)
    assert connection_id == 0x12345678
    assert strip_run_idle_header(payload, io_map.output_assembly_size) == assembly


def test_input_frame_parsing(io_map: IoMap) -> None:
    """A target frame, which carries no run/idle header, parses back."""
    from mock_robot.server import build_cyclic_frame

    assembly = bytes(io_map.input_assembly_size)
    parsed = parse_input_frame(build_cyclic_frame(0xABCD, 1, assembly), io_map.input_assembly_size)
    assert parsed == (0xABCD, assembly)


def test_input_frame_parsing_rejects_garbage(io_map: IoMap) -> None:
    """Anything that is not a usable cyclic frame is reported as such."""
    assert parse_input_frame(b"\x00", io_map.input_assembly_size) is None


def test_encode_connection_path(io_map: IoMap) -> None:
    """The path is encoded exactly as the vendor file spells it."""
    assert encode_connection_path("20 04 2C 96 2C 64") == bytes(
        [0x20, 0x04, 0x2C, 0x96, 0x2C, 0x64]
    )
    assert encode_connection_path(io_map.connection.connection_path) == bytes(
        [0x20, 0x04, 0x2C, 0x96, 0x2C, 0x64]
    )


def test_encode_connection_path_rejects_an_odd_length() -> None:
    """A path must be a whole number of 16 bit words."""
    with pytest.raises(ValueError):
        encode_connection_path("20 04 2C")


# ----------------------------------------------------------------------
# Reply parsing
# ----------------------------------------------------------------------
def build_reply_items(status: int = 0, socket_items: dict = None) -> dict:
    """Build the reply items of a Forward Open exchange.

    Args:
        status: CIP general status to report.
        socket_items: Extra socket address items, keyed by item type.

    Returns:
        The items a target would return.
    """
    body = struct.pack("<IIHHIIIBB", 0x11111111, 0x22222222, 1, 2, 3, 8000, 9000, 0, 0)
    items = {0x00B2: bytes([0xD4, 0x00, status, 0x00]) + body}
    items.update(socket_items or {})
    return items


def test_reply_parsing_extracts_the_connection_ids() -> None:
    """The identifiers and intervals are read from the reply."""
    originator = EipOriginator("127.0.0.1")
    reply = originator._parse_forward_open_reply(build_reply_items())
    assert reply.accepted is True
    assert reply.ot_connection_id == 0x11111111
    assert reply.to_connection_id == 0x22222222
    assert reply.ot_api == 8000
    assert reply.to_api == 9000


def test_reply_parsing_surfaces_a_multicast_destination() -> None:
    """The T->O socket address item is kept, which is the whole point.

    The third-party stack parses this item and discards it, which is why a
    multicast robot looks silent rather than misconfigured.
    """
    originator = EipOriginator("127.0.0.1")
    items = build_reply_items(
        socket_items={
            ITEM_SOCKADDR_TARGET_TO_ORIGINATOR: SocketAddress("239.192.1.5", 2222).encode()
        }
    )
    reply = originator._parse_forward_open_reply(items)
    announced = reply.target_to_originator_address
    assert announced is not None
    assert announced.address == "239.192.1.5"
    assert is_multicast(announced.address) is True


def test_reply_parsing_reports_a_refusal() -> None:
    """A refused Forward Open carries its extended status."""
    originator = EipOriginator("127.0.0.1")
    items = {0x00B2: bytes([0xD4, 0x00, 0x01, 0x01]) + struct.pack("<H", 0x0117)}
    reply = originator._parse_forward_open_reply(items)
    assert reply.accepted is False
    assert reply.extended_status == 0x0117


def test_reply_parsing_rejects_a_missing_response() -> None:
    """A reply without the CIP item is refused rather than guessed at."""
    originator = EipOriginator("127.0.0.1")
    with pytest.raises(ValueError):
        originator._parse_forward_open_reply({0x0000: b""})
