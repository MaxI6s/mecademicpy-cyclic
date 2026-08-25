"""Tests of the transport layer that need no network.

The EtherNet/IP transport itself is exercised end to end in
``test_integration_mock_robot.py``; what is checked here is the contract of the
abstraction and the parts of the implementation that can be tested in
isolation.
"""

import pytest

from mecademic_fieldbus.exceptions import FieldbusConnectionError, FieldbusProtocolError
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.transports import AVAILABLE_TRANSPORTS, get_transport_class

from .fake_transport import FakeTransport

ethernetip_transport = pytest.importorskip(
    "mecademic_fieldbus.transports.ethernetip",
    reason="the 'ethernetip' package is not installed",
)


def test_get_transport_class_returns_the_ethernetip_transport() -> None:
    """The registry hands out the transport class without instantiating it."""
    assert get_transport_class("ethernetip") is ethernetip_transport.EtherNetIpTransport
    assert "ethernetip" in AVAILABLE_TRANSPORTS


def test_get_transport_class_rejects_an_unknown_name() -> None:
    """Asking for a transport that does not exist fails loudly."""
    with pytest.raises(FieldbusConnectionError):
        get_transport_class("profinet")


def test_transport_reports_a_missing_protocol_stack(monkeypatch: pytest.MonkeyPatch) -> None:
    """Without the third-party stack, the failure names the package to install."""
    monkeypatch.setattr(ethernetip_transport, "_ethernetip", None)
    with pytest.raises(FieldbusConnectionError) as error:
        ethernetip_transport.EtherNetIpTransport(
            input_instance=100,
            output_instance=150,
            config_instance=1,
            input_size=64,
            output_size=64,
        )
    assert "ethernetip" in str(error.value)


def test_bit_packing_round_trip() -> None:
    """The process image of the stack packs and unpacks without loss."""
    data = bytes([0x00, 0xFF, 0xA5, 0x01, 0x80])
    bits = [0] * (len(data) * 8)
    ethernetip_transport._bytes_into_bits(data, bits)
    assert ethernetip_transport._bits_to_bytes(bits) == data


def test_bit_packing_updates_the_list_in_place() -> None:
    """The stack keeps a reference to the list, so it must never be replaced."""
    bits = [0] * 8
    original = bits
    ethernetip_transport._bytes_into_bits(b"\xff", bits)
    assert bits is original
    assert all(bits)


def test_fake_transport_enforces_the_assembly_size(io_map: IoMap) -> None:
    """A transport refuses an output image of the wrong size."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    transport.connect("in-memory")
    with pytest.raises(FieldbusProtocolError):
        transport.write_output_assembly(b"too short")


def test_transport_context_manager_disconnects(io_map: IoMap) -> None:
    """The base class disconnects when leaving a ``with`` block."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    transport.connect("in-memory")
    with transport:
        assert transport.is_connected is True
    assert transport.is_connected is False


def test_transport_repr_reports_the_connection_state(io_map: IoMap) -> None:
    """The representation is useful in a debugger."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    assert "connected=False" in repr(transport)


def test_scanner_listens_on_the_standard_udp_port_by_default(io_map: IoMap) -> None:
    """A target that ignores the T->O socket address item still reaches us.

    Many firmwares produce to the standard port whatever the Forward Open
    advertises, so binding an ephemeral port by default would silently break
    the receive path against real hardware.
    """
    transport = ethernetip_transport.EtherNetIpTransport.from_io_map(io_map)
    assert ethernetip_transport.DEFAULT_ORIGINATOR_UDP_PORT == 2222
    assert transport.originator_udp_port == 2222


def test_transport_takes_its_geometry_from_the_io_map(io_map: IoMap) -> None:
    """``from_io_map`` wires the instances, sizes and RPI from the spec."""
    transport = ethernetip_transport.EtherNetIpTransport.from_io_map(io_map)
    assert transport._input_size == io_map.input_assembly_size
    assert transport._output_size == io_map.output_assembly_size
    assert transport._input_instance == io_map.input_assembly_instance
    assert transport._output_instance == io_map.output_assembly_instance
    # The Meca500 connection path carries no configuration assembly.
    assert transport._config_instance is None
    assert transport._rpi_ms == io_map.connection.rpi_microseconds_default // 1000
