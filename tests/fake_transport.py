"""In-memory transport used by the unit tests.

:class:`FakeTransport` implements the
:class:`~mecademic_fieldbus.transports.base.FieldbusTransport` contract without
touching the network, exposing the two assembly images as plain attributes so a
test can inject a byte fixture and assert on what was produced.

For a transport backed by realistic robot behaviour rather than fixtures, use
:class:`~mock_robot.loopback.LoopbackTransport`, which ships with the library
so that demonstrations can use it too.
"""

from typing import Any, List, Optional

from mecademic_fieldbus.exceptions import FieldbusConnectionError, FieldbusProtocolError
from mecademic_fieldbus.transports.base import FieldbusTransport

__all__ = ["FakeTransport"]


class FakeTransport(FieldbusTransport):
    """A transport backed by two in-memory buffers.

    Args:
        input_size: Size of the input assembly, in bytes.
        output_size: Size of the output assembly, in bytes.
        initial_input: Initial input assembly image.  Defaults to zeros.

    Attributes:
        written_assemblies: Every output image handed over, in order.
        connect_calls: Every address :meth:`connect` was called with.
    """

    def __init__(
        self,
        input_size: int,
        output_size: int,
        initial_input: Optional[bytes] = None,
    ) -> None:
        self.input_size = input_size
        self.output_size = output_size
        self._input = bytes(initial_input if initial_input is not None else bytes(input_size))
        self._output = bytes(output_size)
        self._connected = False
        self.written_assemblies: List[bytes] = []
        self.connect_calls: List[str] = []

    def connect(self, address: str, **kwargs: Any) -> None:
        """Mark the transport as connected.

        Args:
            address: Address recorded in :attr:`connect_calls`.
            **kwargs: Ignored.
        """
        self.connect_calls.append(address)
        self._connected = True

    def disconnect(self) -> None:
        """Mark the transport as disconnected."""
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """Whether the transport is currently marked as connected."""
        return self._connected

    def read_input_assembly(self) -> bytes:
        """Return the injected input assembly image.

        Returns:
            The current input image.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        if not self._connected:
            raise FieldbusConnectionError("transport is not connected")
        return self._input

    def write_output_assembly(self, data: bytes) -> None:
        """Record an output assembly image.

        Args:
            data: Raw output assembly image.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
            FieldbusProtocolError: If the image has the wrong size.
        """
        if not self._connected:
            raise FieldbusConnectionError("transport is not connected")
        if len(data) != self.output_size:
            raise FieldbusProtocolError(
                "output assembly must be {} bytes, got {}".format(self.output_size, len(data))
            )
        self._output = bytes(data)
        self.written_assemblies.append(self._output)

    # ------------------------------------------------------------------
    # Test helpers
    # ------------------------------------------------------------------
    def set_input_assembly(self, data: bytes) -> None:
        """Inject the input assembly image the robot is supposed to produce.

        Args:
            data: Raw input assembly image.
        """
        self._input = bytes(data)

    @property
    def last_output_assembly(self) -> bytes:
        """The most recent output assembly image handed over."""
        return self._output
