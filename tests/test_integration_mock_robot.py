"""End-to-end tests against the simulated robot, over real sockets.

These tests exercise the production stack in full: the real
:class:`~mecademic_fieldbus.transports.ethernetip.EtherNetIpTransport` opens a
Class 1 connection to :class:`~mock_robot.server.MockRobotServer`, which drives
a :class:`~mock_robot.simulator.RobotSimulator`.

The Forward Open is the one built from the real vendor file: connection path
``20 04 2C 96 2C 64`` with no configuration assembly, 60 bytes out, 252 bytes
in, and a 32 bit run/idle header on the way out.

They are marked ``integration`` and skipped when the third-party EtherNet/IP
stack is missing or when the standard ports are already taken::

    pytest -m integration          # run only these
    pytest -m "not integration"    # skip them
"""

import socket
import time
from typing import Callable

import pytest

from mecademic_fieldbus.exceptions import FieldbusConnectionError
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot import FieldbusRobot
from mock_robot.server import DEFAULT_TCP_PORT, DEFAULT_UDP_PORT, MockRobotServer
from mock_robot.simulator import RobotSimulator, SimulatorState

pytestmark = pytest.mark.integration

ethernetip = pytest.importorskip("ethernetip", reason="the 'ethernetip' package is not installed")

from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport  # noqa: E402

#: Host every test binds to and connects to.
HOST = "127.0.0.1"

#: Requested packet interval, at the minimum the robot declares.
RPI_MS = 10

#: The scanner and the simulated robot share this machine, and the robot holds
#: the standard UDP 2222, so the scanner has to listen on an ephemeral port and
#: advertise it in the Forward Open.  That only works against a target that
#: honours the T->O socket address item -- the mock does, a real robot may not,
#: which is why the transport defaults to 2222 instead.
SCANNER_UDP_PORT = 0


def port_is_free(kind: int, port: int) -> bool:
    """Check whether a port can be bound on the loopback interface.

    Args:
        kind: ``socket.SOCK_STREAM`` or ``socket.SOCK_DGRAM``.
        port: Port number to probe.

    Returns:
        ``True`` when the port is available.
    """
    probe = socket.socket(socket.AF_INET, kind)
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        probe.bind((HOST, port))
    except OSError:
        return False
    finally:
        probe.close()
    return True


def require_standard_ports() -> None:
    """Skip the current test when the EtherNet/IP ports are already in use.

    The stack under test connects to TCP 44818 and produces to UDP 2222; these
    ports are not negotiable, so the tests cannot fall back to another one.
    """
    if not port_is_free(socket.SOCK_STREAM, DEFAULT_TCP_PORT):
        pytest.skip("TCP port {} is already in use".format(DEFAULT_TCP_PORT))
    if not port_is_free(socket.SOCK_DGRAM, DEFAULT_UDP_PORT):
        pytest.skip("UDP port {} is already in use".format(DEFAULT_UDP_PORT))


def wait_until(predicate: Callable[[], bool], description: str, timeout_s: float = 5.0) -> None:
    """Poll a predicate until it holds, or fail the test.

    Args:
        predicate: Condition to wait for.
        description: What is being waited on, used in the failure message.
        timeout_s: Time allowed.

    Raises:
        AssertionError: If the predicate never holds.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail("timed out waiting for {}".format(description))


@pytest.fixture
def mock_server(demo_io_map: IoMap) -> MockRobotServer:
    """Run a simulated robot on the loopback interface for one test."""
    require_standard_ports()
    simulator = RobotSimulator(demo_io_map, activation_time_s=0.1, homing_time_s=0.2)
    server = MockRobotServer(simulator, host=HOST)
    server.start()
    try:
        yield server
    finally:
        server.stop()


@pytest.fixture
def connected_robot(mock_server: MockRobotServer, demo_io_map: IoMap) -> FieldbusRobot:
    """Return a facade connected to the simulated robot over EtherNet/IP."""
    transport = EtherNetIpTransport.from_io_map(
        demo_io_map, rpi_ms=RPI_MS, originator_udp_port=SCANNER_UDP_PORT
    )
    robot = FieldbusRobot(transport, demo_io_map, default_timeout_s=10.0)
    robot.Connect(HOST)
    try:
        yield robot
    finally:
        robot.Disconnect()


def test_forward_open_succeeds(
    connected_robot: FieldbusRobot, mock_server: MockRobotServer
) -> None:
    """A Class 1 connection is established and cyclic data flows both ways."""
    assert connected_robot.is_connected is True
    assert mock_server.connection_count == 1
    status = connected_robot.GetStatusRobot()
    assert status.activated is False
    assert status.error_status is False
    assert connected_robot.GetMotionStatus().fifo_space > 0


def test_connect_fails_when_nothing_listens(demo_io_map: IoMap) -> None:
    """Connecting to a closed port raises a connection error, not a crash."""
    require_standard_ports()
    transport = EtherNetIpTransport.from_io_map(demo_io_map, originator_udp_port=SCANNER_UDP_PORT)
    with pytest.raises(FieldbusConnectionError):
        transport.connect(HOST)
    assert transport.is_connected is False


def test_forward_open_is_rejected_on_a_mismatched_assembly(
    mock_server: MockRobotServer, demo_io_map: IoMap
) -> None:
    """A strict adapter refuses a scanner asking for unknown assembly instances."""
    transport = EtherNetIpTransport.from_io_map(
        demo_io_map,
        input_instance=demo_io_map.input_assembly_instance + 1,
        output_instance=demo_io_map.output_assembly_instance + 1,
        rpi_ms=RPI_MS,
        originator_udp_port=SCANNER_UDP_PORT,
    )
    with pytest.raises(FieldbusConnectionError) as error:
        transport.connect(HOST)
    assert "Forward Open" in str(error.value)


def test_forward_open_is_rejected_on_a_mismatched_size(
    mock_server: MockRobotServer, demo_io_map: IoMap
) -> None:
    """A scanner built on the wrong layout version is caught by the sizes.

    This is what replaces an explicit revision field: the robot validates the
    negotiated connection sizes, so an I/O map that does not match its firmware
    cannot open a connection at all.
    """
    transport = EtherNetIpTransport.from_io_map(
        demo_io_map,
        input_size=demo_io_map.input_assembly_size - 4,
        rpi_ms=RPI_MS,
        originator_udp_port=SCANNER_UDP_PORT,
    )
    with pytest.raises(FieldbusConnectionError):
        transport.connect(HOST)


def test_full_session_over_the_wire(
    connected_robot: FieldbusRobot, mock_server: MockRobotServer
) -> None:
    """Activate, home, move and deactivate, over real sockets."""
    connected_robot.ActivateRobot()
    assert connected_robot.GetStatusRobot().activated is True

    connected_robot.Home()
    assert connected_robot.GetStatusRobot().is_ready is True

    move_id = connected_robot.MoveJoints(0.0, 0.0, 0.0, 0.0, 30.0, 0.0)
    connected_robot.WaitIdle(timeout_s=10.0)
    assert connected_robot.GetMotionStatus().move_id == move_id
    assert connected_robot.GetRobotPosition().joints[4] == pytest.approx(30.0, abs=0.01)

    connected_robot.DeactivateRobot()
    assert connected_robot.GetStatusRobot().activated is False
    wait_until(
        lambda: mock_server.simulator.state is SimulatorState.DEACTIVATED,
        "the simulated robot to power down",
    )


def test_pause_and_resume_over_the_wire(
    connected_robot: FieldbusRobot, mock_server: MockRobotServer
) -> None:
    """The rising-edge resume bit works through a real cyclic connection."""
    connected_robot.ActivateRobot()
    connected_robot.Home()
    connected_robot.PauseMotion()
    wait_until(lambda: connected_robot.GetMotionStatus().paused, "the robot to report a pause")

    connected_robot.ResumeMotion()
    wait_until(lambda: not connected_robot.GetMotionStatus().paused, "the robot to resume motion")


def test_error_and_reset_over_the_wire(
    connected_robot: FieldbusRobot, mock_server: MockRobotServer
) -> None:
    """An error raised by the robot is seen and cleared through the fieldbus."""
    mock_server.simulator.inject_error()
    wait_until(lambda: connected_robot.GetStatusRobot().error_status, "the error status")
    assert connected_robot.GetStatusRobot().error_code != 0

    connected_robot.ResetError()
    assert connected_robot.GetStatusRobot().error_status is False


def test_disconnect_closes_the_connection(
    connected_robot: FieldbusRobot, mock_server: MockRobotServer
) -> None:
    """A Forward Close releases the connection on the adapter side."""
    assert mock_server.connection_count == 1
    connected_robot.Disconnect()
    wait_until(lambda: mock_server.connection_count == 0, "the connection to be released")
    assert connected_robot.is_connected is False


def test_hand_written_originator_opens_a_connection(mock_server: MockRobotServer) -> None:
    """The diagnostic originator speaks the same CIP as the production stack.

    It is hand-written, so it is exercised against the adapter side of the same
    exchange: session, Forward Open on the real connection path, cyclic
    production and consumption, then Forward Close.
    """
    import socket as socket_module
    import threading

    from tools.eip_originator import (
        EipOriginator,
        build_output_frame,
        encode_connection_path,
        parse_input_frame,
    )

    io_map = mock_server.simulator.io_map
    listener = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_DGRAM)
    listener.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    listener.bind(("0.0.0.0", 0))
    listener.settimeout(2.0)

    originator = EipOriginator(HOST)
    path = encode_connection_path(io_map.connection.connection_path)
    stop = threading.Event()
    producer = None
    try:
        originator.open_session()
        reply = originator.forward_open(
            path,
            io_map.output_assembly_size,
            io_map.input_assembly_size,
            rpi_microseconds=RPI_MS * 1000,
            originator_udp_port=int(listener.getsockname()[1]),
        )
        assert reply.accepted is True
        assert reply.to_connection_id != 0
        # The mock reports where it receives, like a real adapter does.
        assert reply.socket_addresses

        def produce() -> None:
            sender = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_DGRAM)
            sequence = 0
            while not stop.is_set():
                sender.sendto(
                    build_output_frame(
                        reply.ot_connection_id, sequence, io_map.empty_output_assembly()
                    ),
                    (HOST, DEFAULT_UDP_PORT),
                )
                sequence += 1
                time.sleep(RPI_MS / 1000.0)
            sender.close()

        producer = threading.Thread(target=produce, daemon=True)
        producer.start()

        datagram, source = listener.recvfrom(4096)
        parsed = parse_input_frame(datagram, io_map.input_assembly_size)
        assert parsed is not None
        connection_id, assembly = parsed
        assert source[0] == HOST
        assert connection_id == reply.to_connection_id
        assert io_map.decode_motion_status(assembly).fifo_space > 0

        assert originator.forward_close(path) is True
    finally:
        stop.set()
        if producer is not None:
            producer.join(timeout=1.0)
        originator.close()
        listener.close()
