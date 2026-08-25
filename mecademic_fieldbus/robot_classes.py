"""Plain data structures exchanged between the facade and the I/O map layer.

These classes are the *logical* view of the robot: named, typed fields with no
notion of bit or word position.  The translation to and from the raw assembly
bytes is the exclusive responsibility of :mod:`mecademic_fieldbus.io_map`.

The field names follow the vocabulary of the vendor assembly definition (see
``io_map/spec/assembly_v1.json``, generated from the official EDS), converted
to the usual Python conventions, so that a value can be traced back to the
programming manual without guesswork.  Fields the vendor marks as deprecated
are not exposed here; they remain in the specification.

Everything is implemented locally with the standard library only, so the
package stays installable on its own and stays easy to port to another
language later on.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence, Tuple

__all__ = [
    "JOINT_COUNT",
    "POSE_COUNT",
    "MOTION_ARGUMENT_COUNT",
    "InverseKinematicsConfiguration",
    "MotionCommand",
    "MotionControl",
    "MotionStatus",
    "RobotControl",
    "RobotPosition",
    "RobotSafetyStatus",
    "RobotStatus",
    "SafetyStopFlags",
]

#: Number of joints of the robot arm.
JOINT_COUNT = 6

#: Number of components of a Cartesian pose ``(x, y, z, alpha, beta, gamma)``.
POSE_COUNT = 6

#: Number of arguments carried by a motion command in the output assembly.
MOTION_ARGUMENT_COUNT = 6


@dataclass(frozen=True)
class RobotStatus:
    """General robot status, decoded from the input assembly.

    Attributes:
        busy: The robot is activating, deactivating or homing.
        activated: The motors are powered and the robot is ready to home.
        homed: Homing has completed; the robot accepts motion commands.
        simulation_mode: The robot reacts to commands but does not move.
        brakes_engaged: The holding brakes are engaged, which is only possible
            while the robot is deactivated.
        recovery_mode: The robot moves slowly, ignoring joint limits and
            without requiring homing.
        collision: The robot is in a collision event.
        out_of_work_zone: The robot is outside its work zone.
        monitoring_mode: The cyclic connection in use may only monitor the
            robot, not control it.
        error_code: Robot error code; any non-zero value means the robot is in
            error.  See the programming manual for the meaning of each code.
    """

    busy: bool = False
    activated: bool = False
    homed: bool = False
    simulation_mode: bool = False
    brakes_engaged: bool = False
    recovery_mode: bool = False
    collision: bool = False
    out_of_work_zone: bool = False
    monitoring_mode: bool = False
    error_code: int = 0

    @property
    def error_status(self) -> bool:
        """Whether the robot reports an error.

        The assembly carries no dedicated error bit: a non-zero error code is
        what signals the error state.
        """
        return self.error_code != 0

    @property
    def is_ready(self) -> bool:
        """Whether the robot is activated, homed, idle and free of errors."""
        return self.activated and self.homed and not self.error_status and not self.busy


@dataclass(frozen=True)
class MotionStatus:
    """Motion queue status, decoded from the input assembly.

    Attributes:
        paused: The robot is paused.
        end_of_block: The robot is not moving and has no command left to
            process.
        end_of_movement: The robot is not moving, though its queue may still
            hold commands.
        cleared: The command queue has been cleared.
        excessive_torque: The robot detects an excessive torque.
        reached_checkpoint_id: Checkpoint the robot reached most recently.
        discarded_checkpoint_id: Checkpoint the robot discarded most recently.
        move_id: Identifier of the latest motion command received and queued.
        fifo_space: Number of motion commands the robot can still queue.
        offline_program_id: Offline program currently running, ``0`` if none.
    """

    paused: bool = False
    end_of_block: bool = False
    end_of_movement: bool = False
    cleared: bool = False
    excessive_torque: bool = False
    reached_checkpoint_id: int = 0
    discarded_checkpoint_id: int = 0
    move_id: int = 0
    fifo_space: int = 0
    offline_program_id: int = 0

    @property
    def is_idle(self) -> bool:
        """Whether the robot has stopped moving with an empty queue."""
        return self.end_of_block


@dataclass(frozen=True)
class SafetyStopFlags:
    """The safety stop conditions the robot reports through one bit mask.

    The same shape is used for the currently active stops and for the ones that
    can be reset, which is how the vendor assembly models them.

    Attributes:
        estop: The power supply detected an emergency stop signal.
        pstop2: The power supply detected a software (category 2) stop signal.
        reboot: The robot just rebooted, disabling motor voltage.
        connection_dropped: The control connection was lost while the robot was
            not idle, or while the connection timer was active.
    """

    estop: bool = False
    pstop2: bool = False
    reboot: bool = False
    connection_dropped: bool = False

    @property
    def any_active(self) -> bool:
        """Whether any of the known conditions is set."""
        return self.estop or self.pstop2 or self.reboot or self.connection_dropped


@dataclass(frozen=True)
class RobotSafetyStatus:
    """Safety status, decoded from the input assembly.

    This is a convenience view for diagnostics only.  It is **not** a safety
    function and must never be used to implement one.

    Attributes:
        stops: Safety stop signals currently detected.
        resettable_stops: Safety stop signals that can now be cleared, using
            the power supply reset function or, for some of them, by resuming
            motion.
        reset_ready: The stop conditions can be reset from the power supply.
        motor_voltage_on: State of the motor voltage.
    """

    stops: SafetyStopFlags = field(default_factory=SafetyStopFlags)
    resettable_stops: SafetyStopFlags = field(default_factory=SafetyStopFlags)
    reset_ready: bool = False
    motor_voltage_on: bool = False

    @property
    def is_stopped(self) -> bool:
        """Whether any safety stop condition is currently asserted."""
        return self.stops.any_active


class InverseKinematicsConfiguration(IntEnum):
    """Sign of one inverse kinematics configuration parameter."""

    NEGATIVE = -1
    UNDEFINED = 0
    POSITIVE = 1

    @classmethod
    def from_raw(cls, value: int) -> "InverseKinematicsConfiguration":
        """Convert a raw assembly value into a configuration sign.

        Args:
            value: Raw integer decoded from the input assembly.

        Returns:
            The matching member, or :attr:`UNDEFINED` for anything unexpected.
        """
        try:
            return cls(value)
        except ValueError:
            return cls.UNDEFINED


@dataclass(frozen=True)
class RobotPosition:
    """Position feedback, decoded from the input assembly.

    Attributes:
        joints: Measured angle of each joint, in degrees.
        pose: Measured end effector pose ``(x, y, z, alpha, beta, gamma)``, in
            millimetres and degrees.
        shoulder: Shoulder inverse kinematics configuration.
        elbow: Elbow inverse kinematics configuration.
        wrist: Wrist inverse kinematics configuration.
        turn: Turn inverse kinematics configuration.
    """

    joints: Tuple[float, ...] = ()
    pose: Tuple[float, ...] = ()
    shoulder: InverseKinematicsConfiguration = InverseKinematicsConfiguration.UNDEFINED
    elbow: InverseKinematicsConfiguration = InverseKinematicsConfiguration.UNDEFINED
    wrist: InverseKinematicsConfiguration = InverseKinematicsConfiguration.UNDEFINED
    turn: InverseKinematicsConfiguration = InverseKinematicsConfiguration.UNDEFINED


@dataclass(frozen=True)
class RobotControl:
    """Robot control bits written to the output assembly.

    All of these are **level** triggered: the vendor documents each one as
    acting "as soon as, and as long as" it is set, so the scanner keeps
    producing them for as long as the request must hold.

    Attributes:
        deactivate: Deactivate the robot.  Takes precedence over
            :attr:`activate`.
        activate: Activate the robot, unless :attr:`deactivate` is also set.
        home: Find the home position once the robot is activated.
        reset_error: Clear the last robot error.
        enable_simulation: Enable simulation mode.  Only taken into account
            while the robot is deactivated.
        enable_recovery_mode: Enable recovery mode.
    """

    deactivate: bool = False
    activate: bool = False
    home: bool = False
    reset_error: bool = False
    enable_simulation: bool = False
    enable_recovery_mode: bool = False


@dataclass(frozen=True)
class MotionControl:
    """Motion control bits written to the output assembly.

    Attributes:
        move_id: Identifier of the command currently being sent.  The robot
            treats the output assembly as carrying a *new* command only when
            this value changes, which is the whole handshake.
        setpoint: Must be set while the assembly carries a command to execute;
            when it is clear the robot ignores the motion fields entirely.
        pause: Pause the robot immediately, without clearing the queue.
        clear_move: Erase the queued motion commands and pause the robot.
        resume_motion: Resume motion.  Unlike the others this one acts on its
            **rising edge** only.
        use_variables: Interpret the motion command arguments as identifiers of
            robot variables instead of values.
    """

    move_id: int = 0
    setpoint: bool = False
    pause: bool = False
    clear_move: bool = False
    resume_motion: bool = False
    use_variables: bool = False


@dataclass(frozen=True)
class MotionCommand:
    """A motion command written to the output assembly.

    The command identifiers are **not** published in the EDS, the GSDML or the
    ESI: the vendor files all defer them to the programming manual.  Fill them
    into the ``motion_commands`` section of the assembly specification to give
    them names.

    Attributes:
        command_id: Identifier of the command to execute.
        arguments: Up to :data:`MOTION_ARGUMENT_COUNT` arguments, whose meaning
            depends on the command.
    """

    command_id: int = 0
    arguments: Tuple[float, ...] = ()

    @classmethod
    def none(cls) -> "MotionCommand":
        """Return the neutral command, meaning "no motion requested"."""
        return cls.build(0, ())

    @classmethod
    def build(cls, command_id: int, arguments: Sequence[float]) -> "MotionCommand":
        """Build a command from any sequence of arguments.

        Args:
            command_id: Identifier of the command to execute.
            arguments: Argument values; fewer than
                :data:`MOTION_ARGUMENT_COUNT` are padded with zeros.

        Returns:
            The command.

        Raises:
            ValueError: If more arguments are given than the assembly carries.
        """
        values = [float(value) for value in arguments]
        if len(values) > MOTION_ARGUMENT_COUNT:
            raise ValueError(
                "a motion command carries at most {} arguments, got {}".format(
                    MOTION_ARGUMENT_COUNT, len(values)
                )
            )
        values.extend(0.0 for _ in range(MOTION_ARGUMENT_COUNT - len(values)))
        return cls(command_id=command_id, arguments=tuple(values))
