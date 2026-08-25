"""Minimal EtherNet/IP adapter exposing a :class:`~mock_robot.simulator.RobotSimulator`.

This server plays the role the robot plays on a real installation: it accepts a
session, answers a Forward Open, then consumes the cyclic output assembly and
produces the cyclic input assembly.  It is enough to exercise the whole
production stack -- :class:`~mecademic_fieldbus.transports.ethernetip.EtherNetIpTransport`,
the I/O map and :class:`~mecademic_fieldbus.robot.FieldbusRobot` -- without any
hardware.

It is implemented on raw sockets, with no third-party dependency, for two
reasons: ``python-ethernetip`` only implements the *scanner* side, and an
explicit implementation doubles as readable documentation of the wire format
for a future port to another language.

Scope: exactly what a Class 1 originator needs.  Discovery services
(ListIdentity, ListServices) and explicit messaging are answered with "invalid
command".

The Forward Open is validated against the assembly specification -- connection
points and negotiated sizes must match -- so the mock rejects a mis-configured
scanner the way the real robot would.

TODO: model connection timeouts (the robot should drop the connection when the
scanner stops producing) and the corresponding watchdog bit of the input
assembly.
"""

import logging
import socket
import struct
import threading
import time
from types import TracebackType
from typing import Dict, List, Optional, Tuple, Type

from .simulator import RobotSimulator

__all__ = ["MockRobotServer", "DEFAULT_TCP_PORT", "DEFAULT_UDP_PORT"]

logger = logging.getLogger(__name__)

#: Standard EtherNet/IP encapsulation port (explicit messaging, TCP).
DEFAULT_TCP_PORT = 44818

#: Standard EtherNet/IP implicit I/O port (cyclic data, UDP).
DEFAULT_UDP_PORT = 2222

# --- Encapsulation layer ---------------------------------------------------
_ENCAP_HEADER = struct.Struct("<HHII8sI")
_ENCAP_HEADER_SIZE = _ENCAP_HEADER.size

_CMD_NOP = 0x0000
_CMD_REGISTER_SESSION = 0x0065
_CMD_UNREGISTER_SESSION = 0x0066
_CMD_SEND_RR_DATA = 0x006F

_ENCAP_STATUS_SUCCESS = 0x0000
_ENCAP_STATUS_INVALID_COMMAND = 0x0001
_ENCAP_STATUS_INVALID_SESSION = 0x0064

# --- Common Packet Format item types ---------------------------------------
_ITEM_NULL_ADDRESS = 0x0000
_ITEM_CONNECTED_DATA = 0x00B1
_ITEM_UNCONNECTED_DATA = 0x00B2
_ITEM_SOCKADDR_TARGET_TO_ORIGINATOR = 0x8001
_ITEM_SEQUENCED_ADDRESS = 0x8002

# --- CIP layer -------------------------------------------------------------
_SERVICE_FORWARD_OPEN = 0x54
_SERVICE_FORWARD_CLOSE = 0x4E
_SERVICE_RESPONSE_FLAG = 0x80

_CIP_STATUS_SUCCESS = 0x00
_CIP_STATUS_CONNECTION_FAILURE = 0x01

#: Extended status returned when the requested assembly instances are unknown.
_EXT_STATUS_INVALID_CONNECTION_POINT = 0x0117
#: Extended status returned when the negotiated connection sizes do not match.
_EXT_STATUS_INVALID_CONNECTION_SIZE = 0x0109

#: Size of the run/idle header prepended by the originator to its cyclic data.
_RUN_IDLE_HEADER_SIZE = 4
#: Size of the 16-bit sequence count carried by a connected data item.
_SEQUENCE_COUNT_SIZE = 2

#: Forward Open request, once the CIP service and request path are stripped.
_FORWARD_OPEN = struct.Struct("<BBIIHHIB3sIHIHBB")
#: Forward Close request, once the CIP service and request path are stripped.
_FORWARD_CLOSE = struct.Struct("<BBHHIBB")
#: Forward Open reply body, after the three CIP status bytes.
_FORWARD_OPEN_REPLY = struct.Struct("<IIHHIIIBB")
#: Forward Close reply body, after the three CIP status bytes.
_FORWARD_CLOSE_REPLY = struct.Struct("<HHIBB")

#: How often the accept and receive loops check whether the server is stopping.
_LOOP_TIMEOUT_S = 0.2


class _IoConnection:
    """One open Class 1 connection towards a scanner.

    Attributes:
        ot_connection_id: Connection ID stamped by the scanner on its cyclic
            frames (originator to target).
        to_connection_id: Connection ID this server stamps on its own cyclic
            frames (target to originator).
        connection_serial: Serial number chosen by the scanner.
        vendor_id: Vendor ID of the scanner.
        originator_serial: Serial number of the scanner.
        peer_address: UDP endpoint the cyclic data is produced to.
        interval_s: Production period, derived from the requested RPI.
        sequence: Sequence number of the next produced frame.
    """

    def __init__(
        self,
        ot_connection_id: int,
        to_connection_id: int,
        connection_serial: int,
        vendor_id: int,
        originator_serial: int,
        peer_address: Tuple[str, int],
        interval_s: float,
    ) -> None:
        self.ot_connection_id = ot_connection_id
        self.to_connection_id = to_connection_id
        self.connection_serial = connection_serial
        self.vendor_id = vendor_id
        self.originator_serial = originator_serial
        self.peer_address = peer_address
        self.interval_s = interval_s
        self.sequence = 0
        self.next_production = 0.0


class MockRobotServer:
    """Expose a :class:`~mock_robot.simulator.RobotSimulator` on the network.

    Args:
        simulator: Simulator to expose.  Its I/O map defines the assembly
            instances and sizes the server accepts.
        host: Interface to bind to.  Defaults to the loopback interface, which
            is what integration tests want.
        tcp_port: Encapsulation port.  Scanners connect to the standard 44818;
            pass ``0`` only if the scanner can be told about the change.
        udp_port: Implicit I/O port.  The standard 2222 is where scanners send
            their cyclic data; it is not negotiable for most stacks.
        strict: Reject a Forward Open whose assembly instances or connection
            sizes do not match the I/O map.  Turning it off is useful to see
            what a foreign scanner actually asks for.

    Example:
        >>> from mecademic_fieldbus import get_io_map
        >>> from mock_robot.simulator import RobotSimulator
        >>> simulator = RobotSimulator(get_io_map())
        >>> with MockRobotServer(simulator) as server:  # doctest: +SKIP
        ...     server.serve_forever()
    """

    def __init__(
        self,
        simulator: RobotSimulator,
        host: str = "127.0.0.1",
        tcp_port: int = DEFAULT_TCP_PORT,
        udp_port: int = DEFAULT_UDP_PORT,
        strict: bool = True,
    ) -> None:
        self._simulator = simulator
        self._io_map = simulator.io_map
        self._host = host
        self._requested_tcp_port = tcp_port
        self._requested_udp_port = udp_port
        self._strict = strict

        self._tcp_socket: Optional[socket.socket] = None
        self._udp_socket: Optional[socket.socket] = None
        self._threads: List[threading.Thread] = []
        self._client_sockets: List[socket.socket] = []
        self._connections: Dict[int, _IoConnection] = {}
        self._sessions: set = set()
        self._next_session = 1
        self._next_connection_id = 0x20000000
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._running = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    @property
    def is_running(self) -> bool:
        """Whether the server sockets are open and the threads are alive."""
        return self._running

    @property
    def tcp_port(self) -> int:
        """Encapsulation port actually bound, useful when ``0`` was requested."""
        if self._tcp_socket is None:
            return self._requested_tcp_port
        return int(self._tcp_socket.getsockname()[1])

    @property
    def udp_port(self) -> int:
        """Implicit I/O port actually bound, useful when ``0`` was requested."""
        if self._udp_socket is None:
            return self._requested_udp_port
        return int(self._udp_socket.getsockname()[1])

    @property
    def simulator(self) -> RobotSimulator:
        """The simulator exposed by this server."""
        return self._simulator

    @property
    def connection_count(self) -> int:
        """Number of Class 1 connections currently open."""
        with self._lock:
            return len(self._connections)

    def start(self) -> None:
        """Bind the sockets and start the background threads.

        Raises:
            OSError: If a port is already in use.
        """
        if self._running:
            return
        self._stop_event.clear()
        self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._tcp_socket.bind((self._host, self._requested_tcp_port))
            self._tcp_socket.listen(4)
            self._tcp_socket.settimeout(_LOOP_TIMEOUT_S)
            self._udp_socket.bind((self._host, self._requested_udp_port))
            self._udp_socket.settimeout(_LOOP_TIMEOUT_S)
        except OSError:
            self._close_sockets()
            raise
        self._running = True
        self._start_thread(self._accept_loop, "mock-robot-tcp")
        self._start_thread(self._receive_loop, "mock-robot-udp-in")
        self._start_thread(self._produce_loop, "mock-robot-udp-out")
        logger.info(
            "mock robot listening on %s:%d (TCP) and %s:%d (UDP)",
            self._host,
            self.tcp_port,
            self._host,
            self.udp_port,
        )

    def stop(self) -> None:
        """Stop the background threads and close every socket.

        Safe to call on an already stopped server.
        """
        if not self._running:
            self._close_sockets()
            return
        self._running = False
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        with self._lock:
            self._connections.clear()
            self._sessions.clear()
        self._close_sockets()
        logger.info("mock robot stopped")

    def serve_forever(self, poll_interval_s: float = 0.5) -> None:
        """Block until the server is stopped or the process is interrupted.

        Args:
            poll_interval_s: Delay between two checks of the stop flag.
        """
        if not self._running:
            self.start()
        try:
            while not self._stop_event.wait(poll_interval_s):
                pass
        except KeyboardInterrupt:  # pragma: no cover - interactive use only
            pass
        finally:
            self.stop()

    def __enter__(self) -> "MockRobotServer":
        """Start the server and return it, for use as a context manager."""
        self.start()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[TracebackType],
    ) -> None:
        """Stop the server when leaving a ``with`` block.

        Args:
            exc_type: Type of the exception that caused the exit, if any.
            exc_value: Exception that caused the exit, if any.
            traceback: Traceback of the exception, if any.
        """
        self.stop()

    def __repr__(self) -> str:
        """Return a short representation including the bound ports."""
        return "<MockRobotServer {}:{} running={} connections={}>".format(
            self._host, self.tcp_port, self._running, self.connection_count
        )

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------
    def _start_thread(self, target, name: str) -> None:
        """Start one daemon background thread.

        Args:
            target: Callable run by the thread.
            name: Thread name, for debugging.
        """
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _accept_loop(self) -> None:
        """Accept encapsulation (TCP) connections until the server stops."""
        while not self._stop_event.is_set():
            try:
                client, address = self._tcp_socket.accept()  # type: ignore[union-attr]
            except socket.timeout:
                continue
            except OSError:
                break
            logger.debug("session opened from %s:%d", address[0], address[1])
            self._client_sockets.append(client)
            self._start_thread(
                lambda sock=client, addr=address: self._session_loop(sock, addr),
                "mock-robot-session",
            )

    def _session_loop(self, client: socket.socket, address: Tuple[str, int]) -> None:
        """Serve one encapsulation session until the peer closes it.

        Args:
            client: Connected TCP socket.
            address: Peer address, used as the default cyclic data destination.
        """
        client.settimeout(_LOOP_TIMEOUT_S)
        buffer = b""
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer += chunk
                while len(buffer) >= _ENCAP_HEADER_SIZE:
                    command, length, session, _status, context, options = _ENCAP_HEADER.unpack_from(
                        buffer
                    )
                    if len(buffer) < _ENCAP_HEADER_SIZE + length:
                        break
                    payload = buffer[_ENCAP_HEADER_SIZE : _ENCAP_HEADER_SIZE + length]
                    buffer = buffer[_ENCAP_HEADER_SIZE + length :]
                    reply = self._handle_encapsulation(
                        command, session, context, options, payload, address[0]
                    )
                    if reply is not None:
                        client.sendall(reply)
        finally:
            try:
                client.close()
            except OSError:
                pass
            if client in self._client_sockets:
                self._client_sockets.remove(client)
            logger.debug("session closed from %s:%d", address[0], address[1])

    def _receive_loop(self) -> None:
        """Consume the cyclic output assemblies produced by the scanner."""
        while not self._stop_event.is_set():
            try:
                datagram, _address = self._udp_socket.recvfrom(2048)  # type: ignore[union-attr]
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                connection_id, payload = _parse_cyclic_frame(datagram)
            except ValueError as exc:
                logger.debug("ignoring malformed cyclic frame: %s", exc)
                continue
            with self._lock:
                known = any(
                    connection.ot_connection_id == connection_id
                    for connection in self._connections.values()
                )
            if not known:
                logger.debug("ignoring cyclic frame for unknown connection 0x%08X", connection_id)
                continue
            assembly = _strip_run_idle_header(payload, self._io_map.output_assembly_size)
            if assembly is None:
                logger.warning(
                    "ignoring cyclic frame of %d bytes, expected %d",
                    len(payload),
                    self._io_map.output_assembly_size,
                )
                continue
            self._simulator.apply_output_assembly(assembly)

    def _produce_loop(self) -> None:
        """Produce the cyclic input assembly towards every open connection."""
        while not self._stop_event.is_set():
            now = time.monotonic()
            with self._lock:
                due = [
                    connection
                    for connection in self._connections.values()
                    if now >= connection.next_production
                ]
                period = min(
                    [connection.interval_s for connection in self._connections.values()]
                    or [_LOOP_TIMEOUT_S]
                )
            if due:
                assembly = self._simulator.build_input_assembly()
                for connection in due:
                    connection.next_production = now + connection.interval_s
                    self._send_cyclic_frame(connection, assembly)
            else:
                self._simulator.update()
            time.sleep(min(period, _LOOP_TIMEOUT_S) / 2.0)

    def _send_cyclic_frame(self, connection: _IoConnection, assembly: bytes) -> None:
        """Send one input assembly image to a scanner.

        Args:
            connection: Connection to produce to.
            assembly: Raw input assembly image.
        """
        frame = _build_cyclic_frame(connection.to_connection_id, connection.sequence, assembly)
        connection.sequence = (connection.sequence + 1) & 0xFFFFFFFF
        try:
            self._udp_socket.sendto(frame, connection.peer_address)  # type: ignore[union-attr]
        except OSError as exc:
            logger.debug("cannot produce towards %s: %s", connection.peer_address, exc)

    # ------------------------------------------------------------------
    # Encapsulation layer
    # ------------------------------------------------------------------
    def _handle_encapsulation(
        self,
        command: int,
        session: int,
        context: bytes,
        options: int,
        payload: bytes,
        peer_ip: str,
    ) -> Optional[bytes]:
        """Handle one encapsulation message.

        Args:
            command: Encapsulation command code.
            session: Session handle sent by the peer.
            context: Sender context, echoed back untouched.
            options: Option flags, echoed back untouched.
            payload: Command specific data.
            peer_ip: IP address of the peer.

        Returns:
            The encoded reply, or ``None`` when the command needs no answer.
        """
        if command == _CMD_REGISTER_SESSION:
            with self._lock:
                handle = self._next_session
                self._next_session += 1
                self._sessions.add(handle)
            logger.debug("session 0x%08X registered", handle)
            return _encapsulate(command, handle, context, options, payload, _ENCAP_STATUS_SUCCESS)

        if command == _CMD_UNREGISTER_SESSION:
            with self._lock:
                self._sessions.discard(session)
            return None

        if command == _CMD_NOP:
            return None

        if command == _CMD_SEND_RR_DATA:
            with self._lock:
                known_session = session in self._sessions
            if not known_session:
                return _encapsulate(
                    command, session, context, options, b"", _ENCAP_STATUS_INVALID_SESSION
                )
            reply_payload = self._handle_send_rr_data(payload, peer_ip)
            if reply_payload is None:
                return _encapsulate(
                    command, session, context, options, b"", _ENCAP_STATUS_INVALID_COMMAND
                )
            return _encapsulate(
                command, session, context, options, reply_payload, _ENCAP_STATUS_SUCCESS
            )

        logger.debug("unsupported encapsulation command 0x%04X", command)
        return _encapsulate(command, session, context, options, b"", _ENCAP_STATUS_INVALID_COMMAND)

    def _handle_send_rr_data(self, payload: bytes, peer_ip: str) -> Optional[bytes]:
        """Handle the CIP request carried by a SendRRData message.

        Args:
            payload: Interface handle, timeout and CPF of the request.
            peer_ip: IP address of the peer.

        Returns:
            The encoded reply payload, or ``None`` when the request is not
            supported.
        """
        if len(payload) < 6:
            return None
        interface_handle, timeout = struct.unpack_from("<IH", payload, 0)
        try:
            items = _parse_cpf(payload[6:])
        except ValueError as exc:
            logger.debug("malformed CPF: %s", exc)
            return None

        request = items.get(_ITEM_UNCONNECTED_DATA)
        if request is None or not request:
            return None
        service = request[0]
        body = _strip_request_path(request)
        if body is None:
            return None

        if service == _SERVICE_FORWARD_OPEN:
            reply = self._handle_forward_open(
                body, items.get(_ITEM_SOCKADDR_TARGET_TO_ORIGINATOR), peer_ip
            )
        elif service == _SERVICE_FORWARD_CLOSE:
            reply = self._handle_forward_close(body)
        else:
            logger.debug("unsupported CIP service 0x%02X", service)
            return None

        return struct.pack("<IH", interface_handle, timeout) + _build_cpf(
            {_ITEM_NULL_ADDRESS: b"", _ITEM_UNCONNECTED_DATA: reply}
        )

    def _handle_forward_open(
        self, body: bytes, socket_info: Optional[bytes], peer_ip: str
    ) -> bytes:
        """Open a Class 1 connection on behalf of a scanner.

        Args:
            body: Forward Open request, request path already stripped.
            socket_info: Target-to-originator socket address item, when the
                scanner supplied one.
            peer_ip: IP address of the scanner.

        Returns:
            The encoded CIP reply, successful or not.
        """
        if len(body) < _FORWARD_OPEN.size:
            return _cip_error(_SERVICE_FORWARD_OPEN, _EXT_STATUS_INVALID_CONNECTION_POINT)
        (
            _prio_tick,
            _timeout_ticks,
            ot_connection_id,
            _to_connection_id,
            connection_serial,
            vendor_id,
            originator_serial,
            _multiplier,
            _reserved,
            ot_rpi_us,
            ot_parameters,
            to_rpi_us,
            to_parameters,
            _type_trigger,
            path_words,
        ) = _FORWARD_OPEN.unpack_from(body, 0)
        path = body[_FORWARD_OPEN.size : _FORWARD_OPEN.size + path_words * 2]
        connection_points = _parse_connection_path(path)

        rejection = self._validate_forward_open(connection_points, ot_parameters, to_parameters)
        if rejection is not None:
            logger.warning("rejecting Forward Open: extended status 0x%04X", rejection)
            return _cip_error(_SERVICE_FORWARD_OPEN, rejection)

        udp_port = _parse_socket_info_port(socket_info) or DEFAULT_UDP_PORT
        with self._lock:
            self._next_connection_id += 1
            to_connection_id = self._next_connection_id
            connection = _IoConnection(
                ot_connection_id=ot_connection_id,
                to_connection_id=to_connection_id,
                connection_serial=connection_serial,
                vendor_id=vendor_id,
                originator_serial=originator_serial,
                peer_address=(peer_ip, udp_port),
                interval_s=max(to_rpi_us / 1e6, 0.001),
            )
            self._connections[to_connection_id] = connection
        logger.info(
            "Forward Open accepted: O->T 0x%08X, T->O 0x%08X, producing to %s:%d every %.1f ms",
            ot_connection_id,
            to_connection_id,
            peer_ip,
            udp_port,
            connection.interval_s * 1000.0,
        )
        reply = _FORWARD_OPEN_REPLY.pack(
            ot_connection_id,
            to_connection_id,
            connection_serial,
            vendor_id,
            originator_serial,
            ot_rpi_us,
            to_rpi_us,
            0,
            0,
        )
        return _cip_success(_SERVICE_FORWARD_OPEN, reply)

    def _handle_forward_close(self, body: bytes) -> bytes:
        """Close the Class 1 connection matching a Forward Close request.

        Args:
            body: Forward Close request, request path already stripped.

        Returns:
            The encoded CIP reply.
        """
        if len(body) < _FORWARD_CLOSE.size:
            return _cip_error(_SERVICE_FORWARD_CLOSE, _EXT_STATUS_INVALID_CONNECTION_POINT)
        (
            _prio_tick,
            _timeout_ticks,
            connection_serial,
            vendor_id,
            originator_serial,
            _path_words,
            _reserved,
        ) = _FORWARD_CLOSE.unpack_from(body, 0)
        with self._lock:
            for key, connection in list(self._connections.items()):
                if (
                    connection.connection_serial == connection_serial
                    and connection.vendor_id == vendor_id
                ):
                    del self._connections[key]
                    logger.info("Forward Close accepted for T->O 0x%08X", key)
                    break
        reply = _FORWARD_CLOSE_REPLY.pack(connection_serial, vendor_id, originator_serial, 0, 0)
        return _cip_success(_SERVICE_FORWARD_CLOSE, reply)

    def _validate_forward_open(
        self,
        connection_points: List[int],
        ot_parameters: int,
        to_parameters: int,
    ) -> Optional[int]:
        """Check a Forward Open request against the assembly specification.

        Args:
            connection_points: Connection point instances found in the path, in
                order: output assembly then input assembly.
            ot_parameters: Originator-to-target network connection parameters.
            to_parameters: Target-to-originator network connection parameters.

        Returns:
            ``None`` when the request is acceptable, otherwise the extended
            status to answer with.
        """
        if not self._strict:
            return None
        expected_points = [
            self._io_map.output_assembly_instance,
            self._io_map.input_assembly_instance,
        ]
        if connection_points != expected_points:
            logger.warning(
                "connection points %s do not match the I/O map %s",
                connection_points,
                expected_points,
            )
            return _EXT_STATUS_INVALID_CONNECTION_POINT
        profile = self._io_map.connection
        expected_ot = self._io_map.output_assembly_size + _SEQUENCE_COUNT_SIZE
        if profile.output_run_idle_header:
            expected_ot += _RUN_IDLE_HEADER_SIZE
        expected_to = self._io_map.input_assembly_size + _SEQUENCE_COUNT_SIZE
        if profile.input_run_idle_header:
            expected_to += _RUN_IDLE_HEADER_SIZE
        actual_ot = ot_parameters & 0x1FF
        actual_to = to_parameters & 0x1FF
        if actual_ot != expected_ot or actual_to != expected_to:
            logger.warning(
                "connection sizes O->T=%d T->O=%d do not match the I/O map (%d / %d)",
                actual_ot,
                actual_to,
                expected_ot,
                expected_to,
            )
            return _EXT_STATUS_INVALID_CONNECTION_SIZE
        return None

    def _close_sockets(self) -> None:
        """Close every socket owned by the server, ignoring errors."""
        for client in list(self._client_sockets):
            try:
                client.close()
            except OSError:
                pass
        self._client_sockets = []
        for sock in (self._tcp_socket, self._udp_socket):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._tcp_socket = None
        self._udp_socket = None


# ----------------------------------------------------------------------
# Encapsulation and CIP encoding helpers
# ----------------------------------------------------------------------
def _encapsulate(
    command: int, session: int, context: bytes, options: int, payload: bytes, status: int
) -> bytes:
    """Wrap a payload in an EtherNet/IP encapsulation header.

    Args:
        command: Encapsulation command code, echoed from the request.
        session: Session handle.
        context: Sender context, echoed from the request.
        options: Option flags, echoed from the request.
        payload: Command specific data.
        status: Encapsulation status code.

    Returns:
        The complete encapsulation message.
    """
    header = _ENCAP_HEADER.pack(command, len(payload), session, status, context, options)
    return header + payload


def _parse_cpf(data: bytes) -> Dict[int, bytes]:
    """Parse a Common Packet Format structure.

    Only one item of each type is expected in the exchanges this server
    supports, so items are returned keyed by type identifier.

    Args:
        data: Item count followed by the items themselves.

    Returns:
        A mapping of item type identifier to item data.

    Raises:
        ValueError: If the structure is truncated.
    """
    if len(data) < 2:
        raise ValueError("truncated item count")
    (item_count,) = struct.unpack_from("<H", data, 0)
    offset = 2
    items: Dict[int, bytes] = {}
    for _ in range(item_count):
        if len(data) < offset + 4:
            raise ValueError("truncated item header")
        type_id, length = struct.unpack_from("<HH", data, offset)
        offset += 4
        if len(data) < offset + length:
            raise ValueError("truncated item data")
        items[type_id] = data[offset : offset + length]
        offset += length
    return items


def _build_cpf(items: Dict[int, bytes]) -> bytes:
    """Build a Common Packet Format structure.

    Args:
        items: Mapping of item type identifier to item data, in the order the
            items must appear.

    Returns:
        The encoded structure.
    """
    payload = struct.pack("<H", len(items))
    for type_id, data in items.items():
        payload += struct.pack("<HH", type_id, len(data)) + data
    return payload


def _strip_request_path(request: bytes) -> Optional[bytes]:
    """Remove the CIP service byte and request path from a message.

    Args:
        request: CIP request, starting with its service code.

    Returns:
        The service specific body, or ``None`` if the message is truncated.
    """
    if len(request) < 2:
        return None
    path_words = request[1]
    start = 2 + path_words * 2
    if len(request) < start:
        return None
    return request[start:]


def _parse_connection_path(path: bytes) -> List[int]:
    """Extract the connection point instances of a Forward Open path.

    Only the segment types a Class 1 Forward Open needs are decoded: the
    electronic key, and logical class/instance/connection point segments in
    their packed 8 and 16 bit forms.

    Args:
        path: Connection path of the Forward Open request.

    Returns:
        The connection point instances, in the order they appear -- which is
        the output assembly then the input assembly.

    TODO: support padded EPATH encodings if a scanner that uses them shows up.
    """
    connection_points: List[int] = []
    offset = 0
    while offset < len(path):
        segment = path[offset]
        if segment == 0x34:  # Electronic key segment: format byte then key data.
            if offset + 1 >= len(path):
                break
            offset += 2 + path[offset + 1] * 2
            continue
        if segment in (0x20, 0x24, 0x2C, 0x30):  # 8 bit logical segments.
            if offset + 1 >= len(path):
                break
            if segment == 0x2C:
                connection_points.append(path[offset + 1])
            offset += 2
            continue
        if segment in (0x21, 0x25, 0x2D, 0x31):  # 16 bit logical segments.
            if offset + 3 > len(path):
                break
            (value,) = struct.unpack_from("<H", path, offset + 1)
            if segment == 0x2D:
                connection_points.append(value)
            offset += 3
            continue
        logger.debug("unsupported EPATH segment 0x%02X, stopping the walk", segment)
        break
    return connection_points


def _parse_socket_info_port(data: Optional[bytes]) -> Optional[int]:
    """Extract the UDP port from a socket address information item.

    Args:
        data: Item data, or ``None`` when the scanner sent none.

    Returns:
        The UDP port the scanner listens on, or ``None`` when unavailable.
    """
    if not data or len(data) < 4:
        return None
    _family, port = struct.unpack_from(">HH", data, 0)
    return int(port) or None


def _cip_success(service: int, data: bytes) -> bytes:
    """Build a successful CIP response.

    Args:
        service: Service code of the request.
        data: Service specific response data.

    Returns:
        The encoded response.
    """
    return bytes([service | _SERVICE_RESPONSE_FLAG, 0x00, _CIP_STATUS_SUCCESS, 0x00]) + data


def _cip_error(service: int, extended_status: int) -> bytes:
    """Build a CIP connection failure response.

    Args:
        service: Service code of the request.
        extended_status: Extended status word explaining the refusal.

    Returns:
        The encoded response.
    """
    return bytes(
        [service | _SERVICE_RESPONSE_FLAG, 0x00, _CIP_STATUS_CONNECTION_FAILURE, 0x01]
    ) + struct.pack("<H", extended_status)


# ----------------------------------------------------------------------
# Cyclic (Class 1) frame helpers
# ----------------------------------------------------------------------
def _build_cyclic_frame(connection_id: int, sequence: int, assembly: bytes) -> bytes:
    """Build one target-to-originator cyclic frame.

    Args:
        connection_id: Connection identifier assigned to the scanner.
        sequence: 32-bit sequence number of this frame.
        assembly: Raw input assembly image.

    Returns:
        The datagram to send.
    """
    payload = struct.pack("<H", sequence & 0xFFFF) + assembly
    return (
        struct.pack("<HHH", 2, _ITEM_SEQUENCED_ADDRESS, 8)
        + struct.pack("<II", connection_id, sequence)
        + struct.pack("<HH", _ITEM_CONNECTED_DATA, len(payload))
        + payload
    )


def _parse_cyclic_frame(datagram: bytes) -> Tuple[int, bytes]:
    """Parse one originator-to-target cyclic frame.

    Args:
        datagram: Datagram received on the implicit I/O port.

    Returns:
        A tuple ``(connection_id, payload)`` where the payload is the connected
        data item without its sequence count.

    Raises:
        ValueError: If the datagram is not a usable cyclic frame.
    """
    items = _parse_cpf(datagram)
    address = items.get(_ITEM_SEQUENCED_ADDRESS)
    data = items.get(_ITEM_CONNECTED_DATA)
    if address is None or len(address) < 8:
        raise ValueError("missing sequenced address item")
    if data is None or len(data) < _SEQUENCE_COUNT_SIZE:
        raise ValueError("missing connected data item")
    connection_id, _sequence = struct.unpack_from("<II", address, 0)
    return connection_id, data[_SEQUENCE_COUNT_SIZE:]


def _strip_run_idle_header(payload: bytes, expected_size: int) -> Optional[bytes]:
    """Remove the optional run/idle header of an output assembly image.

    Whether the header is present depends on the connection type negotiated at
    Forward Open time, so both layouts are accepted and told apart by size.

    Args:
        payload: Connected data item without its sequence count.
        expected_size: Size of the output assembly.

    Returns:
        The raw output assembly image, or ``None`` if the size is unexpected.

    TODO: the run/idle word also carries the scanner run state; honour it once
    the robot behaviour in idle mode is specified.
    """
    if len(payload) == expected_size:
        return payload
    if len(payload) == expected_size + _RUN_IDLE_HEADER_SIZE:
        return payload[_RUN_IDLE_HEADER_SIZE:]
    return None
