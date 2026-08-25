"""A dependency-free EtherNet/IP originator, used for on-site diagnosis.

The production transport wraps a third-party stack, which is the right thing to
do -- but it makes a poor diagnostic instrument: it filters incoming frames on
criteria it does not report, and it drops the socket address information the
target returns in its Forward Open reply.  That reply is exactly what one needs
when a robot accepts a connection and then produces nothing: it says **where the
target will send**, including the group address when the connection is
multicast.

This module opens a session and a Class 1 connection with raw sockets and
``struct`` only, and hands back everything the target answered.  It is the
mirror image of :mod:`mock_robot.server`, which implements the adapter side of
the same exchange, and it is tested against it.

It is a diagnostic instrument, not a transport: it does not maintain the
connection, and it has no error recovery.
"""

import socket
import struct
from typing import Dict, List, Optional, Tuple

from mock_robot.server import _build_cpf, _parse_cpf, strip_run_idle_header

__all__ = [
    "EipOriginator",
    "ForwardOpenReply",
    "SocketAddress",
    "build_output_frame",
    "is_multicast",
    "DEFAULT_TCP_PORT",
    "DEFAULT_UDP_PORT",
]

#: Standard EtherNet/IP encapsulation port.
DEFAULT_TCP_PORT = 44818

#: Standard EtherNet/IP implicit I/O port.
DEFAULT_UDP_PORT = 2222

#: Run/idle header prepended by a scanner to the data it produces.
RUN_IDLE_HEADER = b"\x01\x00\x00\x00"

_ENCAP_HEADER = struct.Struct("<HHII8sI")
_CMD_REGISTER_SESSION = 0x0065
_CMD_UNREGISTER_SESSION = 0x0066
_CMD_SEND_RR_DATA = 0x006F

_ITEM_NULL_ADDRESS = 0x0000
_ITEM_UNCONNECTED_DATA = 0x00B2
#: Socket address of the originator-to-target direction.
ITEM_SOCKADDR_ORIGINATOR_TO_TARGET = 0x8000
#: Socket address of the target-to-originator direction: where the target sends.
ITEM_SOCKADDR_TARGET_TO_ORIGINATOR = 0x8001
_ITEM_SEQUENCED_ADDRESS = 0x8002
_ITEM_CONNECTED_DATA = 0x00B1

_SERVICE_FORWARD_OPEN = 0x54
_SERVICE_FORWARD_CLOSE = 0x4E

#: Connection Manager request path: class 0x06, instance 1.
_CONNECTION_MANAGER_PATH = bytes([0x02, 0x20, 0x06, 0x24, 0x01])

#: Forward Open request body, after the service and its request path.
_FORWARD_OPEN = struct.Struct("<BBIIHHIB3sIHIHBB")

#: Forward Open reply body, after the four CIP status bytes.
_FORWARD_OPEN_REPLY = struct.Struct("<IIHHIIIBB")

#: Forward Close request body, after the service and its request path.
_FORWARD_CLOSE = struct.Struct("<BBHHIBB")

# Network connection parameter bit positions.
_PARAM_FIXED_SIZE = 0 << 9
_PARAM_PRIORITY_SCHEDULED = 2 << 10
_PARAM_TYPE_MULTICAST = 1 << 13
_PARAM_TYPE_POINT_TO_POINT = 2 << 13

#: Transport class 1, client direction, cyclic trigger.
_TRANSPORT_CLASS_1_CYCLIC = 0x01


class SocketAddress:
    """One socket address information item of a Forward Open exchange.

    Attributes:
        address: IPv4 address, in dotted notation.
        port: UDP port.
    """

    def __init__(self, address: str, port: int) -> None:
        self.address = address
        self.port = port

    @classmethod
    def parse(cls, data: bytes) -> Optional["SocketAddress"]:
        """Decode a socket address information item.

        The item is big endian, unlike everything else in CIP.

        Args:
            data: Raw item data.

        Returns:
            The decoded address, or ``None`` if the item is truncated.
        """
        if len(data) < 8:
            return None
        _family, port, raw_address = struct.unpack_from(">HHI", data, 0)
        return cls(socket.inet_ntoa(struct.pack(">I", raw_address)), int(port))

    def encode(self) -> bytes:
        """Encode this address as a socket address information item.

        Returns:
            The 16 byte item data.
        """
        return struct.pack(
            ">HHI",
            socket.AF_INET,
            self.port,
            struct.unpack(">I", socket.inet_aton(self.address))[0],
        ) + bytes(8)

    def __repr__(self) -> str:
        """Return the address in ``host:port`` form."""
        return "{}:{}".format(self.address, self.port)


class ForwardOpenReply:
    """Everything a target answered to a Forward Open request.

    Attributes:
        ot_connection_id: Identifier to stamp on the frames this scanner
            produces.
        to_connection_id: Identifier the target stamps on the frames it
            produces.
        ot_api: Actual originator-to-target packet interval, in microseconds.
        to_api: Actual target-to-originator packet interval, in microseconds.
        socket_addresses: Socket address items of the reply, keyed by item type.
            The one under :data:`ITEM_SOCKADDR_TARGET_TO_ORIGINATOR` says where
            the target will send its data.
        general_status: CIP general status; ``0`` on success.
        extended_status: Extended status word when the request was refused.
    """

    def __init__(
        self,
        ot_connection_id: int = 0,
        to_connection_id: int = 0,
        ot_api: int = 0,
        to_api: int = 0,
        socket_addresses: Optional[Dict[int, SocketAddress]] = None,
        general_status: int = 0,
        extended_status: int = 0,
    ) -> None:
        self.ot_connection_id = ot_connection_id
        self.to_connection_id = to_connection_id
        self.ot_api = ot_api
        self.to_api = to_api
        self.socket_addresses = socket_addresses or {}
        self.general_status = general_status
        self.extended_status = extended_status

    @property
    def accepted(self) -> bool:
        """Whether the target accepted the connection."""
        return self.general_status == 0

    @property
    def target_to_originator_address(self) -> Optional[SocketAddress]:
        """Where the target says it will send its cyclic data, if it said."""
        return self.socket_addresses.get(ITEM_SOCKADDR_TARGET_TO_ORIGINATOR)


def is_multicast(address: str) -> bool:
    """Tell whether an IPv4 address belongs to the multicast range.

    Args:
        address: IPv4 address in dotted notation.

    Returns:
        ``True`` for anything in ``224.0.0.0/4``.
    """
    try:
        first = int(address.split(".")[0])
    except (ValueError, IndexError):
        return False
    return 224 <= first <= 239


def build_output_frame(connection_id: int, sequence: int, assembly: bytes) -> bytes:
    """Build one originator-to-target cyclic frame.

    Args:
        connection_id: Connection identifier the target expects on this
            direction.
        sequence: 32 bit sequence number of this frame.
        assembly: Raw output assembly image.

    Returns:
        The datagram to send.
    """
    payload = struct.pack("<H", sequence & 0xFFFF) + RUN_IDLE_HEADER + assembly
    return (
        struct.pack("<HHH", 2, _ITEM_SEQUENCED_ADDRESS, 8)
        + struct.pack("<II", connection_id, sequence)
        + struct.pack("<HH", _ITEM_CONNECTED_DATA, len(payload))
        + payload
    )


def parse_input_frame(datagram: bytes, assembly_size: int) -> Optional[Tuple[int, bytes]]:
    """Parse one target-to-originator cyclic frame.

    Args:
        datagram: Datagram received on an implicit I/O socket.
        assembly_size: Expected size of the input assembly.

    Returns:
        A tuple ``(connection id, assembly image)``, or ``None`` when the
        datagram is not a usable cyclic frame.
    """
    try:
        items = _parse_cpf(datagram)
    except ValueError:
        return None
    address = items.get(_ITEM_SEQUENCED_ADDRESS)
    data = items.get(_ITEM_CONNECTED_DATA)
    if address is None or len(address) < 8 or data is None or len(data) < 2:
        return None
    connection_id, _sequence = struct.unpack_from("<II", address, 0)
    assembly = strip_run_idle_header(data[2:], assembly_size)
    if assembly is None:
        return None
    return connection_id, assembly


class EipOriginator:
    """Open an EtherNet/IP session and a Class 1 connection, and report it all.

    Args:
        address: IPv4 address of the target.
        tcp_port: Encapsulation port.
        timeout_s: Time allowed for each request/response exchange.

    Example:
        >>> originator = EipOriginator("192.168.0.100")     # doctest: +SKIP
        >>> originator.open_session()                       # doctest: +SKIP
        >>> reply = originator.forward_open(...)            # doctest: +SKIP
        >>> print(reply.target_to_originator_address)       # doctest: +SKIP
    """

    def __init__(
        self, address: str, tcp_port: int = DEFAULT_TCP_PORT, timeout_s: float = 10.0
    ) -> None:
        self.address = address
        self.tcp_port = tcp_port
        self.timeout_s = timeout_s
        self.session = 0
        self.connection_serial = 0x4949
        self.vendor_id = 0x1234
        self.originator_serial = 0xBEEFF00D
        self._socket: Optional[socket.socket] = None

    def connect(self) -> None:
        """Open the TCP connection to the target.

        Raises:
            OSError: If the target cannot be reached.
        """
        self._socket = socket.create_connection((self.address, self.tcp_port), self.timeout_s)
        self._socket.settimeout(self.timeout_s)

    def open_session(self) -> int:
        """Register an encapsulation session.

        Returns:
            The session handle assigned by the target.

        Raises:
            OSError: If the exchange fails or the target refuses.
        """
        if self._socket is None:
            self.connect()
        payload = struct.pack("<HH", 1, 0)
        reply = self._exchange(_CMD_REGISTER_SESSION, payload, session=0)
        self.session = reply["session"]
        return self.session

    def forward_open(
        self,
        connection_path: bytes,
        output_size: int,
        input_size: int,
        rpi_microseconds: int,
        originator_udp_port: int = DEFAULT_UDP_PORT,
        multicast: bool = False,
        output_run_idle_header: bool = True,
    ) -> ForwardOpenReply:
        """Open a Class 1 connection and return everything the target answered.

        Args:
            connection_path: Connection path, already encoded.
            output_size: Size of the output assembly, in bytes.
            input_size: Size of the input assembly, in bytes.
            rpi_microseconds: Requested packet interval, in microseconds.
            originator_udp_port: Port advertised as the one this scanner
                listens on for the data the target produces.
            multicast: Whether to request a multicast target-to-originator
                connection instead of point to point.
            output_run_idle_header: Whether the produced data carries a 32 bit
                run/idle header, which counts towards the negotiated size.

        Returns:
            The parsed reply, accepted or refused.

        Raises:
            OSError: If the exchange fails.
            ValueError: If the target answers something unusable.
        """
        self.connection_serial = (self.connection_serial + 1) & 0xFFFF
        ot_connection_id = 0x20000000 | (self.connection_serial & 0xFFFF)

        ot_size = output_size + 2 + (4 if output_run_idle_header else 0)
        to_size = input_size + 2
        ot_parameters = ot_size | _PARAM_FIXED_SIZE | _PARAM_PRIORITY_SCHEDULED
        ot_parameters |= _PARAM_TYPE_POINT_TO_POINT
        to_parameters = to_size | _PARAM_FIXED_SIZE | _PARAM_PRIORITY_SCHEDULED
        to_parameters |= _PARAM_TYPE_MULTICAST if multicast else _PARAM_TYPE_POINT_TO_POINT

        body = _FORWARD_OPEN.pack(
            0x0A,  # priority and tick time
            0xF5,  # timeout ticks
            ot_connection_id,
            0,  # the target assigns the target-to-originator identifier
            self.connection_serial,
            self.vendor_id,
            self.originator_serial,
            4,  # connection timeout multiplier
            bytes(3),
            rpi_microseconds,
            ot_parameters,
            rpi_microseconds,
            to_parameters,
            _TRANSPORT_CLASS_1_CYCLIC,
            len(connection_path) // 2,
        )
        request = bytes([_SERVICE_FORWARD_OPEN]) + _CONNECTION_MANAGER_PATH
        request += body + connection_path

        items = {_ITEM_NULL_ADDRESS: b"", _ITEM_UNCONNECTED_DATA: request}
        if originator_udp_port:
            items[ITEM_SOCKADDR_TARGET_TO_ORIGINATOR] = SocketAddress(
                "0.0.0.0", originator_udp_port
            ).encode()

        reply_items = self._send_rr_data(items)
        return self._parse_forward_open_reply(reply_items)

    def forward_close(self, connection_path: bytes) -> bool:
        """Close the Class 1 connection.

        Args:
            connection_path: The same path used to open it.

        Returns:
            ``True`` when the target accepted the request.
        """
        body = _FORWARD_CLOSE.pack(
            0x0A,
            0xF5,
            self.connection_serial,
            self.vendor_id,
            self.originator_serial,
            len(connection_path) // 2,
            0,
        )
        request = bytes([_SERVICE_FORWARD_CLOSE]) + _CONNECTION_MANAGER_PATH
        request += body + connection_path
        try:
            items = self._send_rr_data({_ITEM_NULL_ADDRESS: b"", _ITEM_UNCONNECTED_DATA: request})
        except (OSError, ValueError):
            return False
        reply = items.get(_ITEM_UNCONNECTED_DATA, b"")
        return len(reply) > 2 and reply[2] == 0

    def close(self) -> None:
        """Unregister the session and close the socket, ignoring errors."""
        if self._socket is None:
            return
        try:
            if self.session:
                self._socket.sendall(
                    _ENCAP_HEADER.pack(_CMD_UNREGISTER_SESSION, 0, self.session, 0, bytes(8), 0)
                )
        except OSError:
            pass
        try:
            self._socket.close()
        except OSError:
            pass
        self._socket = None
        self.session = 0

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _exchange(self, command: int, payload: bytes, session: Optional[int] = None) -> dict:
        """Send one encapsulation message and read its answer.

        Args:
            command: Encapsulation command code.
            payload: Command specific data.
            session: Session handle to send, defaulting to the current one.

        Returns:
            A dictionary with the reply ``session``, ``status`` and ``payload``.

        Raises:
            OSError: If the socket fails or the peer closes the connection.
            ValueError: If the target reports an encapsulation error.
        """
        if self._socket is None:
            raise OSError("not connected")
        handle = self.session if session is None else session
        header = _ENCAP_HEADER.pack(command, len(payload), handle, 0, bytes(8), 0)
        self._socket.sendall(header + payload)

        head = self._receive_exactly(_ENCAP_HEADER.size)
        reply_command, length, reply_session, status, _context, _options = _ENCAP_HEADER.unpack(
            head
        )
        data = self._receive_exactly(length) if length else b""
        if status != 0:
            raise ValueError(
                "the target rejected command 0x{:04X} with encapsulation status "
                "0x{:08X}".format(reply_command, status)
            )
        return {"session": reply_session, "status": status, "payload": data}

    def _send_rr_data(self, items: Dict[int, bytes]) -> Dict[int, bytes]:
        """Send an unconnected CIP request and return the reply items.

        Args:
            items: Common Packet Format items of the request.

        Returns:
            The Common Packet Format items of the reply.

        Raises:
            OSError: If the socket fails.
            ValueError: If the reply cannot be parsed.
        """
        payload = struct.pack("<IH", 0, 0) + _build_cpf(items)
        reply = self._exchange(_CMD_SEND_RR_DATA, payload)["payload"]
        if len(reply) < 6:
            raise ValueError("truncated SendRRData reply")
        return _parse_cpf(reply[6:])

    def _parse_forward_open_reply(self, items: Dict[int, bytes]) -> ForwardOpenReply:
        """Turn the reply items of a Forward Open into a result object.

        Args:
            items: Common Packet Format items of the reply.

        Returns:
            The parsed reply.

        Raises:
            ValueError: If the mandatory item is missing or truncated.
        """
        data = items.get(_ITEM_UNCONNECTED_DATA)
        if data is None or len(data) < 4:
            raise ValueError("the Forward Open reply carries no CIP response")
        general_status = data[2]
        additional_size = data[3]
        body = data[4:]

        addresses: Dict[int, SocketAddress] = {}
        for item_type in (
            ITEM_SOCKADDR_ORIGINATOR_TO_TARGET,
            ITEM_SOCKADDR_TARGET_TO_ORIGINATOR,
        ):
            item = items.get(item_type)
            if item is not None:
                parsed = SocketAddress.parse(item)
                if parsed is not None:
                    addresses[item_type] = parsed

        if general_status != 0:
            extended = 0
            if additional_size > 0 and len(body) >= 2:
                (extended,) = struct.unpack_from("<H", body, 0)
            return ForwardOpenReply(
                socket_addresses=addresses,
                general_status=general_status,
                extended_status=extended,
            )

        if len(body) < _FORWARD_OPEN_REPLY.size:
            raise ValueError("truncated Forward Open reply")
        (
            ot_connection_id,
            to_connection_id,
            _serial,
            _vendor,
            _originator_serial,
            ot_api,
            to_api,
            _reply_size,
            _reserved,
        ) = _FORWARD_OPEN_REPLY.unpack_from(body, 0)
        return ForwardOpenReply(
            ot_connection_id=ot_connection_id,
            to_connection_id=to_connection_id,
            ot_api=ot_api,
            to_api=to_api,
            socket_addresses=addresses,
        )

    def _receive_exactly(self, size: int) -> bytes:
        """Read exactly ``size`` bytes from the socket.

        Args:
            size: Number of bytes to read.

        Returns:
            The bytes read.

        Raises:
            OSError: If the peer closes the connection first.
        """
        if self._socket is None:
            raise OSError("not connected")
        chunks: List[bytes] = []
        remaining = size
        while remaining > 0:
            chunk = self._socket.recv(remaining)
            if not chunk:
                raise OSError("the target closed the connection")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)


def encode_connection_path(path: str) -> bytes:
    """Encode a connection path written as space separated hexadecimal bytes.

    Args:
        path: Path exactly as the vendor file spells it, for example
            ``"20 04 2C 96 2C 64"``.

    Returns:
        The encoded path.

    Raises:
        ValueError: If the path is not made of hexadecimal byte pairs, or does
            not fit a whole number of 16 bit words.
    """
    tokens = path.split()
    encoded = bytes(int(token, 16) for token in tokens)
    if len(encoded) % 2:
        raise ValueError("a connection path must be a whole number of words")
    return encoded
