"""Simulated Mecademic robot, for development and testing without hardware.

Two levels of mocking are available, for two different needs:

* :class:`~mock_robot.simulator.RobotSimulator` -- an in-memory state machine
  that consumes and produces real assembly images.  Combined with an in-memory
  transport it gives fast, deterministic unit tests with no network at all.
* :class:`~mock_robot.server.MockRobotServer` -- a minimal EtherNet/IP adapter
  that puts the simulator on the network, so the real
  :class:`~mecademic_fieldbus.transports.ethernetip.EtherNetIpTransport` can be
  exercised end to end.

This package is a development tool.  It shares the I/O map of the library, so
it stays in sync with the assembly specification by construction.

Run it standalone with::

    python -m mock_robot --help
"""

from .server import MockRobotServer
from .simulator import RobotSimulator, SimulatorErrorCode, SimulatorState

__all__ = ["MockRobotServer", "RobotSimulator", "SimulatorErrorCode", "SimulatorState"]
