"""Standalone fieldbus scanner library for Mecademic robots.

``mecademic_fieldbus`` drives a robot that is already configured as an
EtherNet/IP adapter (target), by acting as the scanner (originator) of the
cyclic IN/OUT assemblies.

The package is intentionally self-contained: it depends on no vendor library,
and its core (data structures, I/O map, facade) depends on nothing outside the
standard library.  Only the EtherNet/IP transport needs a third-party protocol
stack, and it is isolated in a single module.

Layers, from the application down to the wire:

1. :class:`~mecademic_fieldbus.robot.FieldbusRobot` -- the public facade.
2. :mod:`mecademic_fieldbus.io_map` -- named/typed fields to raw assembly bytes.
3. :mod:`mecademic_fieldbus.transports` -- protocol specific byte transport.

Enabling the fieldbus mode on the robot itself is out of scope: use the robot
web interface or your own tooling first, then drive it with this library.
"""

from .exceptions import (
    FieldbusConnectionError,
    FieldbusError,
    FieldbusIoMapError,
    FieldbusProtocolError,
    FieldbusSpecError,
    FieldbusStateError,
    FieldbusTimeoutError,
    FieldbusUnsupportedFeature,
    RobotErrorStatus,
)
from .io_map import IoMap, IoMapV1, get_io_map
from .robot import FieldbusRobot
from .robot_classes import (
    InverseKinematicsConfiguration,
    MotionCommand,
    MotionControl,
    MotionStatus,
    RobotControl,
    RobotPosition,
    RobotSafetyStatus,
    RobotStatus,
    SafetyStopFlags,
)
from .transports import FieldbusTransport

__version__ = "0.1.0"

__all__ = [
    "__version__",
    "FieldbusRobot",
    "FieldbusTransport",
    "IoMap",
    "IoMapV1",
    "get_io_map",
    "InverseKinematicsConfiguration",
    "MotionCommand",
    "MotionControl",
    "MotionStatus",
    "RobotControl",
    "RobotPosition",
    "RobotSafetyStatus",
    "RobotStatus",
    "SafetyStopFlags",
    "FieldbusError",
    "FieldbusConnectionError",
    "FieldbusTimeoutError",
    "FieldbusProtocolError",
    "FieldbusIoMapError",
    "FieldbusSpecError",
    "FieldbusStateError",
    "FieldbusUnsupportedFeature",
    "RobotErrorStatus",
]
