"""Protocol agnostic transport interface.

A transport moves *opaque bytes* between the scanner and the robot: it knows
about sessions, connections and timeouts, but nothing about the content of the
assemblies.  Interpreting those bytes is the job of
:mod:`mecademic_fieldbus.io_map`.

Adding a new fieldbus later (Profinet, or anything else) means adding one file
in this package with one class implementing :class:`FieldbusTransport` -- and
changing nothing else in the library.  EtherCAT is the known exception: it
normally requires a real-time master and will not fit this interface as-is.
"""

import abc
from types import TracebackType
from typing import Any, Optional, Type

__all__ = ["FieldbusTransport"]


class FieldbusTransport(abc.ABC):
    """Cyclic byte transport towards a fieldbus device.

    Implementations are expected to maintain the cyclic exchange in the
    background once :meth:`connect` returns, so that
    :meth:`read_input_assembly` and :meth:`write_output_assembly` are cheap,
    non-blocking accesses to the latest process image.
    """

    @abc.abstractmethod
    def connect(self, address: str, **kwargs: Any) -> None:
        """Open the connection and start the cyclic exchange.

        Args:
            address: Address of the device, typically an IPv4 address.
            **kwargs: Implementation specific options.

        Raises:
            FieldbusConnectionError: If the connection cannot be established.
        """

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Stop the cyclic exchange and close the connection.

        Implementations must make this safe to call on an already disconnected
        transport.
        """

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        """Whether the cyclic exchange is currently established."""

    @abc.abstractmethod
    def read_input_assembly(self) -> bytes:
        """Return the latest input assembly image received from the device.

        Returns:
            The raw bytes of the input assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """

    @abc.abstractmethod
    def write_output_assembly(self, data: bytes) -> None:
        """Set the output assembly image sent cyclically to the device.

        The image is *latched*: the transport keeps producing it until the next
        call.

        Args:
            data: Raw bytes of the output assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
            FieldbusProtocolError: If the buffer size does not match the
                assembly size negotiated with the device.
        """

    def __enter__(self) -> "FieldbusTransport":
        """Return the transport itself, for use as a context manager."""
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[TracebackType],
    ) -> None:
        """Disconnect the transport when leaving a ``with`` block.

        Args:
            exc_type: Type of the exception that caused the exit, if any.
            exc_value: Exception that caused the exit, if any.
            traceback: Traceback of the exception, if any.
        """
        self.disconnect()

    def __repr__(self) -> str:
        """Return a short representation including the connection state."""
        return "<{} connected={}>".format(type(self).__name__, self.is_connected)
