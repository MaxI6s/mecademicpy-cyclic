"""Abstract, versioned mapping between raw assembly bytes and logical fields.

An :class:`IoMap` is the only component of the library allowed to know where a
field lives inside an assembly.  Everything above it -- the facade, the
application, the robot simulator -- exchanges the typed structures of
:mod:`mecademic_fieldbus.robot_classes` and never sees an offset.

The interface is deliberately **symmetric**.  The ``decode_*``/``encode_*``
pairs exist for both directions so that the same map can be used by:

* the *scanner* side (:class:`~mecademic_fieldbus.robot.FieldbusRobot`), which
  decodes the input assembly and encodes the output assembly;
* the *adapter* side (:class:`~mock_robot.simulator.RobotSimulator`), which
  does exactly the opposite.

Concrete implementations are versioned: one class per assembly layout version,
each backed by a declarative specification under ``io_map/spec/``.

Features an assembly version does not carry are reported through
:class:`~mecademic_fieldbus.exceptions.FieldbusUnsupportedFeature` rather than
silently doing nothing, so an application always knows what it is getting.
"""

import abc
from typing import Dict, Optional

from ..exceptions import FieldbusUnsupportedFeature
from ..robot_classes import (
    MotionCommand,
    MotionControl,
    MotionStatus,
    RobotControl,
    RobotPosition,
    RobotSafetyStatus,
    RobotStatus,
)
from .spec_loader import ConnectionProfile

__all__ = ["IoMap"]


class IoMap(abc.ABC):
    """Translate raw assembly bytes to and from named, typed fields.

    Attributes:
        version: Version of the assembly layout implemented by this map.  It
            matches the ``assembly_version`` of the declarative specification
            it is built from.
    """

    version: str = ""

    # ------------------------------------------------------------------
    # Assembly geometry
    # ------------------------------------------------------------------
    @property
    @abc.abstractmethod
    def input_assembly_size(self) -> int:
        """Size of the input assembly (robot to scanner), in bytes."""

    @property
    @abc.abstractmethod
    def output_assembly_size(self) -> int:
        """Size of the output assembly (scanner to robot), in bytes."""

    @property
    @abc.abstractmethod
    def input_assembly_instance(self) -> int:
        """CIP instance number of the input assembly."""

    @property
    @abc.abstractmethod
    def output_assembly_instance(self) -> int:
        """CIP instance number of the output assembly."""

    @property
    @abc.abstractmethod
    def config_assembly_instance(self) -> Optional[int]:
        """CIP instance of the configuration assembly, or ``None``.

        ``None`` means the robot connection path carries no configuration
        assembly, and that a Forward Open must not add one.
        """

    @property
    @abc.abstractmethod
    def connection(self) -> ConnectionProfile:
        """Connection parameters the robot advertises for this layout."""

    @property
    @abc.abstractmethod
    def motion_command_ids(self) -> Dict[str, int]:
        """Known motion command identifiers, keyed by command name.

        Empty until they are filled into the specification: the vendor files
        do not publish them.
        """

    @abc.abstractmethod
    def empty_input_assembly(self) -> bytes:
        """Return an all-zero image of the input assembly."""

    @abc.abstractmethod
    def empty_output_assembly(self) -> bytes:
        """Return an all-zero image of the output assembly."""

    def motion_command_id(self, name: str) -> int:
        """Return the identifier of a named motion command.

        Args:
            name: Command name, as used in the ``motion_commands`` section of
                the specification.

        Returns:
            The identifier to write into the output assembly.

        Raises:
            FieldbusUnsupportedFeature: If the specification does not define
                that command.
        """
        try:
            return self.motion_command_ids[name]
        except KeyError:
            known = ", ".join(sorted(self.motion_command_ids)) or "none"
            raise FieldbusUnsupportedFeature(
                "motion command {!r} is not defined for assembly version {}; "
                "add it to the 'motion_commands.ids' section of the specification "
                "(known commands: {})".format(name, self.version, known)
            )

    # ------------------------------------------------------------------
    # Optional features
    # ------------------------------------------------------------------
    @property
    def digital_output_count(self) -> int:
        """Number of digital outputs carried by the assemblies.

        ``0``, the default, means this layout exposes none.
        """
        return 0

    def decode_output_state(self, raw_input: bytes) -> object:
        """Decode the digital output read-back from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The digital output states reported by the robot.

        Raises:
            FieldbusUnsupportedFeature: If this layout carries no digital
                outputs, which is the default.
        """
        raise self._no_digital_io()

    def encode_output_state(
        self, output_state: object, raw_output: Optional[bytes] = None
    ) -> bytes:
        """Write requested digital output states into an output assembly image.

        Args:
            output_state: Digital output states to request.
            raw_output: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated output assembly image.

        Raises:
            FieldbusUnsupportedFeature: If this layout carries no digital
                outputs, which is the default.
        """
        raise self._no_digital_io()

    def _no_digital_io(self) -> FieldbusUnsupportedFeature:
        """Build the error raised when digital I/O is asked of a layout without it.

        Returns:
            The exception to raise.
        """
        return FieldbusUnsupportedFeature(
            "assembly version {} carries no digital inputs or outputs; on this robot "
            "they are reachable through the dynamic data slots or the text API, "
            "not through the cyclic assemblies".format(self.version)
        )

    # ------------------------------------------------------------------
    # Input assembly: robot to scanner
    # ------------------------------------------------------------------
    @abc.abstractmethod
    def decode_status(self, raw_input: bytes) -> RobotStatus:
        """Decode the general robot status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def decode_motion_status(self, raw_input: bytes) -> MotionStatus:
        """Decode the motion queue status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded motion status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def decode_safety_status(self, raw_input: bytes) -> RobotSafetyStatus:
        """Decode the safety status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded safety status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def decode_position(self, raw_input: bytes) -> RobotPosition:
        """Decode the position feedback from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded position.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def encode_status(self, status: RobotStatus, raw_input: Optional[bytes] = None) -> bytes:
        """Write a robot status into an input assembly image.

        This is the adapter-side counterpart of :meth:`decode_status`; it is
        used by the robot simulator.

        Args:
            status: Status to encode.
            raw_input: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated input assembly image.
        """

    @abc.abstractmethod
    def encode_motion_status(
        self, motion_status: MotionStatus, raw_input: Optional[bytes] = None
    ) -> bytes:
        """Write a motion status into an input assembly image.

        Args:
            motion_status: Motion status to encode.
            raw_input: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated input assembly image.
        """

    @abc.abstractmethod
    def encode_safety_status(
        self, safety_status: RobotSafetyStatus, raw_input: Optional[bytes] = None
    ) -> bytes:
        """Write a safety status into an input assembly image.

        Args:
            safety_status: Safety status to encode.
            raw_input: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated input assembly image.
        """

    @abc.abstractmethod
    def encode_position(self, position: RobotPosition, raw_input: Optional[bytes] = None) -> bytes:
        """Write a position feedback into an input assembly image.

        Args:
            position: Position to encode.
            raw_input: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated input assembly image.

        Raises:
            FieldbusIoMapError: If a vector does not have the expected number
                of elements.
        """

    # ------------------------------------------------------------------
    # Output assembly: scanner to robot
    # ------------------------------------------------------------------
    @abc.abstractmethod
    def encode_robot_control(
        self, control: RobotControl, raw_output: Optional[bytes] = None
    ) -> bytes:
        """Write the robot control bits into an output assembly image.

        Args:
            control: Control bits to request.
            raw_output: Image to update.  Defaults to an all-zero image, which
                clears every other field of the assembly.

        Returns:
            The updated output assembly image.
        """

    @abc.abstractmethod
    def encode_motion_control(
        self, control: MotionControl, raw_output: Optional[bytes] = None
    ) -> bytes:
        """Write the motion control bits and move id into an output assembly image.

        Args:
            control: Motion control bits to request.
            raw_output: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated output assembly image.
        """

    @abc.abstractmethod
    def encode_motion_command(
        self, command: MotionCommand, raw_output: Optional[bytes] = None
    ) -> bytes:
        """Write a motion command into an output assembly image.

        Setting the command does not by itself request its execution: the
        robot only acts on a *change* of the move id, and only while the
        setpoint bit of :class:`~mecademic_fieldbus.robot_classes.MotionControl`
        is set.

        Args:
            command: Command to write.
            raw_output: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated output assembly image.

        Raises:
            FieldbusIoMapError: If the argument list does not fit.
        """

    @abc.abstractmethod
    def decode_robot_control(self, raw_output: bytes) -> RobotControl:
        """Decode the robot control bits from an output assembly image.

        This is the adapter-side counterpart of :meth:`encode_robot_control`;
        it is used by the robot simulator.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded control bits.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def decode_motion_control(self, raw_output: bytes) -> MotionControl:
        """Decode the motion control bits from an output assembly image.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded motion control bits.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    @abc.abstractmethod
    def decode_motion_command(self, raw_output: bytes) -> MotionCommand:
        """Decode a motion command from an output assembly image.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded command.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    @abc.abstractmethod
    def describe_layout(self) -> str:
        """Return a human readable description of the assembly layout.

        Intended for documentation and debugging: it is the only sanctioned way
        of showing offsets to a human, and it always reflects the declarative
        specification rather than a hand-written copy of it.

        Returns:
            A multi-line description of both assemblies.
        """

    def __repr__(self) -> str:
        """Return a short representation naming the implemented version."""
        return "<{} version={!r}>".format(type(self).__name__, self.version)
