"""In-memory transports used by the unit tests.

These transports implement the
:class:`~mecademic_fieldbus.transports.base.FieldbusTransport` contract without
touching the network, which keeps the unit tests fast and deterministic:

* :class:`FakeTransport` exposes the two assembly images as plain attributes,
  so a test can inject a byte fixture and assert on what was produced.
* :class:`SimulatorTransport` wires a
  :class:`~mock_robot.simulator.RobotSimulator` directly to the facade, which
  exercises the full command and feedback loop with no protocol involved.
"""

from typing import Any, List, Optional

from mecademic_fieldbus.exceptions import FieldbusConnectionError, FieldbusProtocolError
from mecademic_fieldbus.transports.base import FieldbusTransport
from mock_robot.simulator import RobotSimulator

__all__ = ["FakeTransport", "SimulatorTransport"]


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


class SimulatorTransport(FieldbusTransport):
    """A transport that hands the assemblies straight to a simulator.

    No socket, no protocol: writing an output assembly applies it to the
    simulator, and reading the input assembly asks the simulator to produce
    one.  This is the fastest way to test the facade against realistic robot
    behaviour.

    Args:
        simulator: Simulator to drive.
    """

    def __init__(self, simulator: RobotSimulator) -> None:
        self._simulator = simulator
        self._connected = False

    def connect(self, address: str, **kwargs: Any) -> None:
        """Mark the transport as connected.

        Args:
            address: Ignored.
            **kwargs: Ignored.
        """
        self._connected = True

    def disconnect(self) -> None:
        """Mark the transport as disconnected."""
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """Whether the transport is currently marked as connected."""
        return self._connected

    def read_input_assembly(self) -> bytes:
        """Ask the simulator to produce its current input assembly.

        Returns:
            The raw input assembly image.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        if not self._connected:
            raise FieldbusConnectionError("transport is not connected")
        return self._simulator.build_input_assembly()

    def write_output_assembly(self, data: bytes) -> None:
        """Apply an output assembly image to the simulator.

        Args:
            data: Raw output assembly image.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        if not self._connected:
            raise FieldbusConnectionError("transport is not connected")
        self._simulator.apply_output_assembly(data)

    @property
    def simulator(self) -> RobotSimulator:
        """The simulator driven by this transport."""
        return self._simulator
