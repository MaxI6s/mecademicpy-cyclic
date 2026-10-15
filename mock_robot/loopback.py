"""A transport that wires the facade straight to the simulator.

No socket, no protocol, no Forward Open: writing an output assembly applies it
to a :class:`~mock_robot.simulator.RobotSimulator`, and reading the input
assembly asks that simulator to produce one.

That makes it the right backend for two jobs.  Tests get the facade exercised
against realistic robot behaviour with nothing to bind and nothing to race.
Demonstrations get a robot that cannot fail to come up in front of an audience
-- no port in use, no firewall, no cable.  When the *protocol* is the subject,
use :class:`~mock_robot.server.MockRobotServer` with the real
:class:`~mecademic_fieldbus.transports.ethernetip.EtherNetIpTransport` instead;
this transport deliberately skips all of it.
"""

from typing import Any

from mecademic_fieldbus.exceptions import FieldbusConnectionError
from mecademic_fieldbus.transports.base import FieldbusTransport

from .simulator import RobotSimulator

__all__ = ["LoopbackTransport"]


class LoopbackTransport(FieldbusTransport):
    """Exchange assemblies with a simulator in process, with no network.

    Args:
        simulator: Simulator to drive.

    Example:
        >>> from mecademic_fieldbus import FieldbusRobot, get_io_map
        >>> from mock_robot import LoopbackTransport, RobotSimulator
        >>> io_map = get_io_map()
        >>> simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
        >>> robot = FieldbusRobot(LoopbackTransport(simulator), io_map)
        >>> robot.Connect("in-process")
        >>> robot.GetStatusRobot().activated
        False
    """

    def __init__(self, simulator: RobotSimulator) -> None:
        self._simulator = simulator
        self._connected = False

    @property
    def simulator(self) -> RobotSimulator:
        """The simulator driven by this transport."""
        return self._simulator

    def connect(self, address: str, **kwargs: Any) -> None:
        """Mark the transport as connected.

        Args:
            address: Ignored; there is nothing to reach.
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

        Each call advances the simulated robot, which is what lets a
        step-by-step walkthrough move the state machine by polling it.

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
