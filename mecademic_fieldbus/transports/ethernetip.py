"""EtherNet/IP transport, implemented on top of the ``ethernetip`` package.

This module is the **only** place in the project that imports a third-party
EtherNet/IP stack.  Every call to it is confined here so that the library can
be moved to another stack -- or reimplemented from raw sockets, or ported to
another language -- without touching the I/O map or the facade.

The stack used is ``python-ethernetip`` (https://codeberg.org/paperwork/python-ethernetip,
published on PyPI as ``ethernetip``).  It was chosen because, unlike most
Python EtherNet/IP libraries, it supports Class 1 implicit I/O towards a
*generic* adapter -- ``registerAssembly`` plus ``sendFwdOpenReq`` plus a
``produce`` cycle -- and not only Rockwell tag messaging.

The Forward Open parameters are taken from the robot vendor file: the
connection path carries no configuration assembly, the scanner prepends a
32 bit run/idle header to the data it produces, and the robot produces
modeless data in return.  The stack used here matches that natively.

TODO: confirm against real hardware whether the robot honours the T->O socket
address item of the Forward Open request; if it does not, the scanner has to
listen on UDP 2222 and ``originator_udp_port`` must be set accordingly.
"""

import socket
import threading
import time
from typing import TYPE_CHECKING, Any, List, Optional

from ..exceptions import FieldbusConnectionError, FieldbusProtocolError
from .base import FieldbusTransport

if TYPE_CHECKING:  # pragma: no cover - typing only, no runtime dependency
    from ..io_map.base import IoMap

try:  # pragma: no cover - exercised through the import error path only
    import ethernetip as _ethernetip
except ImportError:  # pragma: no cover
    _ethernetip = None

__all__ = ["EtherNetIpTransport", "DEFAULT_RPI_MS"]

#: Default requested packet interval, in milliseconds, for both directions.
#: The Meca500 EDS declares 10 ms as both its minimum and its recommended
#: value, with 10 s as the maximum.
DEFAULT_RPI_MS = 10

#: Size of the dummy datagram used to wake the receiving thread of the stack up.
#: It matches the header of a cyclic frame so the stack can parse and discard it.
_WAKE_UP_DATAGRAM_SIZE = 20

#: Message shown when the optional protocol stack is missing.
_MISSING_STACK_MESSAGE = (
    "the 'ethernetip' package is required by EtherNetIpTransport; "
    "install it with: pip install 'mecademic-fieldbus[ethernetip]'"
)


class EtherNetIpTransport(FieldbusTransport):
    """Class 1 (cyclic, implicit) EtherNet/IP originator towards the robot.

    The assembly instance numbers and sizes are supplied by the caller rather
    than read from an I/O map, so that this layer stays independent from
    :mod:`mecademic_fieldbus.io_map`.  Wire them together at the application
    level::

        io_map = get_io_map()
        transport = EtherNetIpTransport.from_io_map(io_map)

    Args:
        input_instance: CIP instance of the assembly produced by the robot.
        output_instance: CIP instance of the assembly consumed by the robot.
        config_instance: CIP instance of the configuration assembly, or
            ``None`` when the robot connection path carries none.  The Meca500
            declares its path as ``20 04 2C 96 2C 64``, with no configuration
            segment, so ``None`` is what it expects.
        input_size: Size of the input assembly, in bytes.
        output_size: Size of the output assembly, in bytes.
        rpi_ms: Requested packet interval for both directions, in
            milliseconds.  The robot rejects anything below its declared
            minimum.
        originator_udp_port: UDP port this scanner listens on for the cyclic
            data produced by the robot.  ``0`` picks a free port and advertises
            it in the Forward Open request, which is what allows a scanner and
            a simulated robot to run on the same machine.  TODO: check whether
            the robot firmware honours the T->O socket address item; if it does
            not, force this to 2222.
        settle_time_s: Time to wait after the Forward Open for the first cyclic
            frames to arrive.  Defaults to a few RPI periods.  TODO: replace
            with a proper "first frame received" event once the stack exposes
            one.

    Raises:
        FieldbusConnectionError: If the ``ethernetip`` package is not installed.
    """

    def __init__(
        self,
        input_instance: int,
        output_instance: int,
        config_instance: Optional[int],
        input_size: int,
        output_size: int,
        rpi_ms: int = DEFAULT_RPI_MS,
        originator_udp_port: int = 0,
        settle_time_s: Optional[float] = None,
    ) -> None:
        if _ethernetip is None:
            raise FieldbusConnectionError(_MISSING_STACK_MESSAGE)
        self._input_instance = input_instance
        self._output_instance = output_instance
        self._config_instance = config_instance
        self._input_size = input_size
        self._output_size = output_size
        self._rpi_ms = rpi_ms
        self._originator_udp_port = originator_udp_port
        self._settle_time_s = (
            settle_time_s if settle_time_s is not None else max(3.0 * rpi_ms / 1000.0, 0.1)
        )
        self._address: str = ""
        self._enip: Any = None
        self._connection: Any = None
        self._input_bits: Optional[List[Any]] = None
        self._output_bits: Optional[List[Any]] = None
        self._lock = threading.Lock()

    @classmethod
    def from_io_map(cls, io_map: "IoMap", **kwargs: Any) -> "EtherNetIpTransport":
        """Build a transport sized and addressed by an I/O map.

        Only the geometry and connection properties of the map are read, never
        a field: the transport still knows nothing of the assembly contents,
        and the plain constructor remains available for callers that want the
        two layers strictly decoupled.

        Args:
            io_map: Map whose specification describes the robot assemblies.
            **kwargs: Overrides passed to the constructor, ``rpi_ms`` for
                instance.

        Returns:
            A transport ready to connect.

        Raises:
            FieldbusConnectionError: If the ``ethernetip`` package is missing.
        """
        options = {
            "input_instance": io_map.input_assembly_instance,
            "output_instance": io_map.output_assembly_instance,
            "config_instance": io_map.config_assembly_instance,
            "input_size": io_map.input_assembly_size,
            "output_size": io_map.output_assembly_size,
            "rpi_ms": max(io_map.connection.rpi_microseconds_default // 1000, 1),
        }
        options.update(kwargs)
        return cls(**options)

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------
    def connect(self, address: str, **kwargs: Any) -> None:
        """Register a session, open the Class 1 connection and start producing.

        Args:
            address: IPv4 address of the robot.
            **kwargs: Accepted for interface compatibility; none is used.

        Raises:
            FieldbusConnectionError: If the session, the Forward Open, or the
                underlying sockets fail.
        """
        if self.is_connected:
            return
        if kwargs:
            raise TypeError("unexpected keyword arguments: {}".format(", ".join(sorted(kwargs))))

        self._address = address
        self._enip = _ethernetip.EtherNetIP(address)
        try:
            self._connection = self._enip.explicit_conn(address)
        except OSError as exc:
            self._enip = None
            raise FieldbusConnectionError(
                "cannot reach the EtherNet/IP device at {}: {}".format(address, exc)
            )

        try:
            self._register_assemblies()
            if self._connection.registerSession() != 0:
                raise FieldbusConnectionError(
                    "the device at {} refused the EtherNet/IP session".format(address)
                )
            self._enip.startIO(udp_port=self._originator_udp_port)
            self._open_io_connection()
            self._connection.produce()
        except FieldbusConnectionError:
            self._teardown()
            raise
        except OSError as exc:
            self._teardown()
            raise FieldbusConnectionError(
                "cannot start the cyclic exchange with {}: {}".format(address, exc)
            )

        # TODO: the stack does not expose a "first cyclic frame received"
        # event, so we simply let a few periods elapse before handing the
        # connection over.
        time.sleep(self._settle_time_s)

    def disconnect(self) -> None:
        """Stop producing, close the Class 1 connection and unregister the session.

        Safe to call on an already disconnected transport.
        """
        if self._connection is None and self._enip is None:
            return
        connection = self._connection
        if connection is not None:
            try:
                connection.stopProduce()
                connection.sendFwdCloseReq(
                    self._input_instance, self._output_instance, self._config_instance
                )
                connection.unregisterSession()
            except OSError:
                # The peer may already be gone; closing is best effort.
                pass
        self._teardown()

    @property
    def is_connected(self) -> bool:
        """Whether the Class 1 connection is currently established."""
        return self._connection is not None and self._input_bits is not None

    # ------------------------------------------------------------------
    # Cyclic data
    # ------------------------------------------------------------------
    def read_input_assembly(self) -> bytes:
        """Return the latest input assembly image received from the robot.

        Returns:
            The raw bytes of the input assembly.  Before the first cyclic frame
            is received, this is an all-zero image.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        if not self.is_connected:
            raise FieldbusConnectionError("transport is not connected")
        with self._lock:
            return _bits_to_bytes(self._input_bits or [])

    def write_output_assembly(self, data: bytes) -> None:
        """Latch a new output assembly image to be produced cyclically.

        Args:
            data: Raw bytes of the output assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
            FieldbusProtocolError: If the buffer size does not match the
                negotiated output assembly size.
        """
        if not self.is_connected:
            raise FieldbusConnectionError("transport is not connected")
        if len(data) != self._output_size:
            raise FieldbusProtocolError(
                "output assembly must be {} bytes, got {}".format(self._output_size, len(data))
            )
        # The producing thread of the stack reads this list without locking, so
        # a frame straddling two writes is theoretically possible.
        # TODO: revisit if the firmware turns out to be sensitive to it.
        with self._lock:
            _bytes_into_bits(data, self._output_bits or [])

    # ------------------------------------------------------------------
    # Internals: every call to the third-party stack lives below
    # ------------------------------------------------------------------
    def _register_assemblies(self) -> None:
        """Allocate the input and output process images in the stack.

        Raises:
            FieldbusConnectionError: If the stack refuses to register them.
        """
        self._input_bits = self._enip.registerAssembly(
            _ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT,
            self._input_size,
            self._input_instance,
            self._connection,
        )
        self._output_bits = self._enip.registerAssembly(
            _ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT,
            self._output_size,
            self._output_instance,
            self._connection,
        )
        if self._input_bits is None or self._output_bits is None:
            raise FieldbusConnectionError(
                "cannot register assemblies {} (in) and {} (out)".format(
                    self._input_instance, self._output_instance
                )
            )

    def _open_io_connection(self) -> None:
        """Send the Forward Open request that opens the Class 1 connection.

        Raises:
            FieldbusConnectionError: If the device rejects the request or does
                not answer.
        """
        status = self._connection.sendFwdOpenReq(
            self._input_instance,
            self._output_instance,
            self._config_instance,
            torpi=self._rpi_ms,
            otrpi=self._rpi_ms,
            originator_udp_port=self._enip.originator_udp_port,
        )
        if status is None:
            raise FieldbusConnectionError(
                "no Forward Open answer from {}; is the robot in EtherNet/IP mode?".format(
                    self._address
                )
            )
        if status != 0:
            raise FieldbusConnectionError(
                "the device at {} rejected the Forward Open "
                "(extended status 0x{:04X})".format(self._address, status)
            )

    def _teardown(self) -> None:
        """Release the stack objects and forget the process images."""
        self._stop_io()
        self._enip = None
        self._connection = None
        self._input_bits = None
        self._output_bits = None

    def _stop_io(self) -> None:
        """Stop the receiving thread of the stack, quietly.

        ``stopIO`` closes the receiving socket while the thread of the stack is
        still blocked in ``select`` on it, which surfaces as an unhandled "bad
        file descriptor" traceback in that thread.  Clearing the run flag first
        and then waking the thread with a dummy datagram lets it notice the
        flag and return on its own.

        TODO: remove this workaround if the upstream stack learns to shut its
        listener down cleanly.
        """
        enip = self._enip
        if enip is None:
            return
        try:
            self._wake_up_listener(enip)
        except (AttributeError, OSError):
            # Unknown or already released internals: fall back to the plain
            # shutdown, noisy traceback included.
            pass
        try:
            enip.stopIO()
        except OSError:
            pass

    @staticmethod
    def _wake_up_listener(enip: Any) -> None:
        """Ask the receiving thread of the stack to return from its select call.

        Args:
            enip: Stack instance owning the receiving socket and thread.

        Raises:
            AttributeError: If the stack does not expose the expected internals.
            OSError: If the socket is already closed.
        """
        if enip.udpsock is None or enip.io_state != 1:
            return
        port = int(enip.udpsock.getsockname()[1])
        enip.io_state = 0
        waker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # A minimal, non-matching cyclic frame: long enough for the stack to
            # parse it, addressed to a connection it does not know.
            waker.sendto(bytes(_WAKE_UP_DATAGRAM_SIZE), ("127.0.0.1", port))
        finally:
            waker.close()
        thread = getattr(enip, "udpthread", None)
        if thread is not None:
            thread.join(timeout=1.0)


def _bits_to_bytes(bits: List[Any]) -> bytes:
    """Pack the bit list used by the stack into bytes.

    The stack represents a process image as one entry per bit, least
    significant bit first.  This is a representation detail of the transport,
    not a field layout: no meaning is attached to any position here.

    Args:
        bits: Process image as a list of truthy values.

    Returns:
        The packed bytes.
    """
    buffer = bytearray(len(bits) // 8)
    for index, bit in enumerate(bits):
        if bit:
            buffer[index // 8] |= 1 << (index % 8)
    return bytes(buffer)


def _bytes_into_bits(data: bytes, bits: List[Any]) -> None:
    """Unpack bytes into the stack bit list, in place.

    The list object is shared with the producing thread of the stack and must
    never be replaced, only updated.

    Args:
        data: Packed process image.
        bits: Process image as a list of truthy values, updated in place.
    """
    for index in range(len(bits)):
        bits[index] = bool(data[index // 8] & (1 << (index % 8)))
