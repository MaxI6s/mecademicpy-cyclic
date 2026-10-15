"""In-memory model of a Meca500 behaving as a fieldbus adapter.

:class:`RobotSimulator` owns a small state machine and the same I/O map as the
production code, so it consumes real output assembly images and produces real
input assembly images.  It knows nothing about EtherNet/IP: putting it on the
network is the job of :mod:`mock_robot.server`.

The modelled behaviour follows what the vendor files document:

* every ``RobotControl`` bit is level triggered, and ``Deactivate`` wins over
  ``Activate``;
* ``MotionControl_ResumeMotion`` acts on its rising edge only;
* a motion command is taken into account when ``MotionControl_MoveId``
  *changes* while ``MotionControl_Setpoint`` is set;
* the robot signals an error through a non-zero ``RobotStatus_Error``.

What it cannot model faithfully is anything the vendor files leave to the
programming manual: the motion command identifiers, the error code table and
the real kinematics.  Those are marked with a TODO and replaced with clearly
synthetic stand-ins.
"""

import threading
import time
from enum import Enum, IntEnum
from typing import Callable, Dict, Optional, Sequence, Tuple

from mecademic_fieldbus.io_map.base import IoMap
from mecademic_fieldbus.robot_classes import (
    DYNAMIC_DATA_SLOTS,
    JOINT_COUNT,
    POSE_COUNT,
    DynamicData,
    InverseKinematicsConfiguration,
    MotionCommand,
    MotionControl,
    MotionStatus,
    RobotControl,
    RobotPosition,
    RobotSafetyStatus,
    RobotStatus,
    RobotTimestamp,
    SafetyStopFlags,
)

__all__ = [
    "RobotSimulator",
    "SimulatorState",
    "SimulatorErrorCode",
    "SUPPORTED_COMMANDS",
    "DEFAULT_FIFO_SPACE",
]

#: Commands this simulator actually executes.  Every identifier the
#: specification declares is *accepted and acknowledged*, like a real robot;
#: only these few have a modelled effect.  An identifier that is in no
#: specification at all is still reported as unknown.
SUPPORTED_COMMANDS = frozenset(
    {
        "MoveJoints",
        "MoveJointsRel",
        "MovePose",
        "MoveLin",
        "MoveLinRelTrf",
        "MoveLinRelWrf",
        "MoveJointsVel",
        "Delay",
        "SetCheckpoint",
        "SetJointVel",
    }
)

#: Lowest identifier that addresses a robot variable, in both the motion
#: command field and the dynamic data configuration.
VARIABLE_ID_FIRST = 10000

#: Highest such identifier.
VARIABLE_ID_LAST = 19999

#: Dynamic data types this simulator actually fills in.  Any other type the
#: specification declares is echoed back with zeroed values, which is honest:
#: the mock does not pretend to measure what it does not model.
MODELLED_DYNAMIC_TYPES = (
    "FirmwareVersion",
    "ActualJointPosition",
    "ActualEndEffectorPose",
    "MotionQueueVelocities",
    "TimeScaling",
)

#: Firmware version the simulator reports, so the slot shows something real.
SIMULATED_FIRMWARE = (11.0, 3.0, 3.0, 12637.0)

#: Motion queue capacity the simulator advertises, matching the maximum the
#: EDS declares for ``MotionStatus_FIFOSpace``.
DEFAULT_FIFO_SPACE = 13000


class SimulatorState(Enum):
    """States of the simulated robot.

    The nominal sequence is
    ``DEACTIVATED -> ACTIVATING -> ACTIVATED -> HOMING -> IDLE -> MOVING``,
    with :attr:`ERROR` reachable from any state and leaving only through a
    reset.  :attr:`DEACTIVATING` exists so that ``RobotStatus_Busy`` behaves
    like the real robot, which reports it while activating, deactivating and
    homing.
    """

    DEACTIVATED = "deactivated"
    ACTIVATING = "activating"
    DEACTIVATING = "deactivating"
    ACTIVATED = "activated"
    HOMING = "homing"
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"


class SimulatorErrorCode(IntEnum):
    """Error codes produced by the simulator.

    They sit just under the maximum the EDS declares for
    ``RobotStatus_Error`` so that they fit the field while staying far away
    from the range the real robot uses.

    TODO: replace with the real robot error codes; the programming manual is
    the only place that documents them.
    """

    NONE = 0
    UNKNOWN_MOTION_COMMAND = 32001
    MOVE_WITHOUT_HOMING = 32002
    MOVE_OUT_OF_RANGE = 32003
    INJECTED = 32099


class RobotSimulator:
    """A robot state machine driven by output assemblies.

    The simulator is thread-safe: :mod:`mock_robot.server` calls
    :meth:`apply_output_assembly` from its receiving thread and
    :meth:`build_input_assembly` from its producing thread.

    Args:
        io_map: I/O map used to decode commands and encode feedback.  This must
            be the very same version the scanner uses.
        activation_time_s: Simulated duration of activation and deactivation.
        homing_time_s: Simulated duration of the homing sequence.
        joint_speed_deg_s: Speed applied to every joint during a move.
        joint_limits_deg: Lower and upper limit applied to every joint.
            TODO: the real Meca500 limits differ per joint and come from the
            programming manual.
        home_position_deg: Joint angles reached at the end of homing.
        motion_command_ids: Command identifiers the simulator recognises.
            Defaults to those of the I/O map, restricted to
            :data:`SUPPORTED_COMMANDS`.
        clock: Monotonic time source, injectable so tests can drive time.

    Example:
        >>> from mecademic_fieldbus import get_io_map
        >>> simulator = RobotSimulator(get_io_map(), activation_time_s=0.0)
        >>> simulator.state
        <SimulatorState.DEACTIVATED: 'deactivated'>
    """

    def __init__(
        self,
        io_map: IoMap,
        activation_time_s: float = 0.5,
        homing_time_s: float = 1.0,
        joint_speed_deg_s: float = 90.0,
        joint_limits_deg: Tuple[float, float] = (-175.0, 175.0),
        home_position_deg: Optional[Sequence[float]] = None,
        motion_command_ids: Optional[Dict[str, int]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._io_map = io_map
        self._activation_time_s = activation_time_s
        self._homing_time_s = homing_time_s
        self._joint_speed_deg_s = joint_speed_deg_s
        self._joint_limits_deg = joint_limits_deg
        self._clock = clock
        self._lock = threading.RLock()

        self._command_ids = (
            dict(motion_command_ids)
            if motion_command_ids is not None
            else io_map.motion_command_ids
        )
        self._command_names = {value: name for name, value in self._command_ids.items()}

        self._home_position = (
            tuple(float(value) for value in home_position_deg)
            if home_position_deg is not None
            else tuple(0.0 for _ in range(JOINT_COUNT))
        )
        if len(self._home_position) != JOINT_COUNT:
            raise ValueError("home_position_deg must have {} elements".format(JOINT_COUNT))

        self._state = SimulatorState.DEACTIVATED
        self._error_code = SimulatorErrorCode.NONE
        self._homed = False
        self._paused = False
        self._cleared = False
        self._simulation_mode = False
        self._recovery_mode = False
        self._joints: Tuple[float, ...] = tuple(0.0 for _ in range(JOINT_COUNT))

        self._state_deadline = 0.0
        self._move_start_time = 0.0
        self._move_end_time = 0.0
        self._move_start_joints = self._joints
        self._move_target_joints = self._joints
        self._received_move_id = 0
        self._previous_move_id = 0
        self._previous_resume_motion = False
        self._last_advance: Optional[float] = None
        self._joint_velocity: Optional[Tuple[float, ...]] = None
        self._dynamic_data_cycles = 0
        self._dynamic_config: Tuple[int, ...] = tuple(0 for _ in range(DYNAMIC_DATA_SLOTS))
        self._variables: Dict[int, Tuple[float, ...]] = {}
        self._automatic_index = 0
        self._clock_offset_s = 0.0
        self._reached_checkpoint = 0
        self._velocity_percent = 100.0
        self._offline_program_id = 0

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------
    @property
    def io_map(self) -> IoMap:
        """I/O map used to decode commands and encode feedback."""
        return self._io_map

    @property
    def motion_command_ids(self) -> Dict[str, int]:
        """Motion command identifiers this simulator recognises."""
        return dict(self._command_ids)

    @property
    def state(self) -> SimulatorState:
        """Current state of the simulated robot."""
        with self._lock:
            return self._state

    @property
    def error_code(self) -> int:
        """Current error code, ``0`` when the robot is not in error."""
        with self._lock:
            return int(self._error_code)

    @property
    def joint_positions(self) -> Tuple[float, ...]:
        """Current joint angles, in degrees."""
        with self._lock:
            return self._joints

    # ------------------------------------------------------------------
    # Cyclic interface, as seen from the fieldbus
    # ------------------------------------------------------------------
    def synchronise_clock(self, host_time_s: int) -> None:
        """Adopt the host clock, the way ``HostTime`` does on a real robot.

        Args:
            host_time_s: Seconds since the UNIX epoch, ``0`` to leave the clock
                alone.
        """
        if host_time_s <= 0:
            return
        with self._lock:
            self._clock_offset_s = host_time_s - self._clock()

    def apply_output_assembly(self, raw_output: bytes, now: Optional[float] = None) -> None:
        """Consume one output assembly image and update the internal state.

        Args:
            raw_output: Raw output assembly image received from the scanner.
            now: Current time, defaulting to the injected clock.  Explicit
                values make tests deterministic.

        Raises:
            FieldbusIoMapError: If the image is shorter than the assembly.
        """
        robot_control = self._io_map.decode_robot_control(raw_output)
        motion_control = self._io_map.decode_motion_control(raw_output)
        command = self._io_map.decode_motion_command(raw_output)
        dynamic_config = self._io_map.decode_dynamic_config(raw_output)
        with self._lock:
            self._dynamic_config = dynamic_config
            timestamp = self._clock() if now is None else now
            self._advance(timestamp)
            self._apply_robot_control(robot_control, timestamp)
            self._apply_motion_control(motion_control)
            self._apply_motion_command(motion_control, command, timestamp)

    def build_input_assembly(self, now: Optional[float] = None) -> bytes:
        """Advance the state machine and produce the current input assembly.

        Args:
            now: Current time, defaulting to the injected clock.

        Returns:
            The raw input assembly image the robot would produce.
        """
        with self._lock:
            self._advance(self._clock() if now is None else now)
            raw = self._io_map.empty_input_assembly()
            raw = self._io_map.encode_status(self.get_status(), raw)
            raw = self._io_map.encode_motion_status(self.get_motion_status(), raw)
            raw = self._io_map.encode_safety_status(self.get_safety_status(), raw)
            raw = self._io_map.encode_position(self.get_position(), raw)
            raw = self._io_map.encode_timestamp(self.get_timestamp(), raw)
            for slot in range(DYNAMIC_DATA_SLOTS):
                raw = self._io_map.encode_dynamic_data(slot, self.get_dynamic_data(slot), raw)
            return raw

    def update(self, now: Optional[float] = None) -> None:
        """Advance the state machine without producing an assembly.

        Args:
            now: Current time, defaulting to the injected clock.
        """
        with self._lock:
            self._advance(self._clock() if now is None else now)

    # ------------------------------------------------------------------
    # Logical state, as the library sees it
    # ------------------------------------------------------------------
    def get_status(self) -> RobotStatus:
        """Return the robot status corresponding to the current state.

        Returns:
            The status that will be published in the input assembly.
        """
        with self._lock:
            activated = self._state in (
                SimulatorState.ACTIVATED,
                SimulatorState.HOMING,
                SimulatorState.IDLE,
                SimulatorState.MOVING,
            )
            busy = self._state in (
                SimulatorState.ACTIVATING,
                SimulatorState.DEACTIVATING,
                SimulatorState.HOMING,
            )
            return RobotStatus(
                busy=busy,
                activated=activated,
                homed=activated and self._homed,
                simulation_mode=self._simulation_mode,
                brakes_engaged=not activated,
                recovery_mode=self._recovery_mode,
                collision=False,
                out_of_work_zone=False,
                monitoring_mode=False,
                error_code=int(self._error_code),
            )

    def get_motion_status(self) -> MotionStatus:
        """Return the motion status corresponding to the current state.

        Returns:
            The motion status that will be published in the input assembly.
        """
        with self._lock:
            moving = self._state is SimulatorState.MOVING
            return MotionStatus(
                paused=self._paused,
                end_of_block=not moving,
                end_of_movement=not moving,
                cleared=self._cleared,
                excessive_torque=False,
                reached_checkpoint_id=self._reached_checkpoint,
                discarded_checkpoint_id=0,
                move_id=self._received_move_id,
                fifo_space=DEFAULT_FIFO_SPACE - (1 if moving else 0),
                offline_program_id=self._offline_program_id,
            )

    def get_safety_status(self) -> RobotSafetyStatus:
        """Return the safety status of the simulated robot.

        Returns:
            A safety status with no stop asserted.

        TODO: model the emergency and protective stops, and the reset sequence
        they require, once the mock needs to exercise them.
        """
        with self._lock:
            activated = self._state in (
                SimulatorState.ACTIVATED,
                SimulatorState.HOMING,
                SimulatorState.IDLE,
                SimulatorState.MOVING,
            )
            return RobotSafetyStatus(
                stops=SafetyStopFlags(),
                resettable_stops=SafetyStopFlags(),
                reset_ready=False,
                motor_voltage_on=activated,
            )

    def get_timestamp(self) -> RobotTimestamp:
        """Return the free-running counters, and advance the cycle count.

        A real robot publishes a clock and a dynamic data cycle count that
        change on every frame, which is what proves to a scanner that the
        connection is live.  The simulator does the same, so that a scanner
        cannot tell a stale process image from a fresh one here either.

        Returns:
            The counters to publish in the input assembly.
        """
        with self._lock:
            self._dynamic_data_cycles = (self._dynamic_data_cycles + 1) & 0x7FFFFFFF
            now = self._clock() + self._clock_offset_s
            return RobotTimestamp(
                seconds=int(now),
                microseconds=int((now - int(now)) * 1000000),
                dynamic_data_cycles=self._dynamic_data_cycles,
            )

    def get_dynamic_data(self, slot: int) -> DynamicData:
        """Return what this robot publishes in one dynamic data slot.

        The requested type is echoed back whatever happens, so a scanner can
        tell a stale slot from a fresh one.  Types the simulator does not model
        come back with zeroed values rather than invented ones.

        Args:
            slot: Slot index, from ``0``.

        Returns:
            The payload to publish.
        """
        with self._lock:
            requested = self._dynamic_config[slot]
            type_id = self._resolve_dynamic_type(requested)
            return DynamicData.build(type_id, self._dynamic_values(type_id))

    def get_variable(self, cyclic_id: int) -> Tuple[float, ...]:
        """Return the values of one robot variable.

        Args:
            cyclic_id: Identifier the variable was written under.

        Returns:
            Its values, or an empty tuple when it was never set.
        """
        with self._lock:
            return self._variables.get(cyclic_id, ())

    def _resolve_dynamic_type(self, requested: int) -> int:
        """Turn a requested type into the one actually published.

        Type ``0`` means "automatic": the real robot cycles through everything
        it can report, a different type each cycle, and so does this.

        Args:
            requested: Type asked for in the output assembly.

        Returns:
            The type to publish.
        """
        if requested != 0:
            return requested
        names = MODELLED_DYNAMIC_TYPES
        known = self._io_map.dynamic_data_types
        available = [known[name] for name in names if name in known]
        if not available:
            return 0
        self._automatic_index = (self._automatic_index + 1) % len(available)
        return available[self._automatic_index]

    def _dynamic_values(self, type_id: int) -> Tuple[float, ...]:
        """Return the values to publish for one dynamic data type.

        Args:
            type_id: Type being published.

        Returns:
            Up to six values, empty for a type this simulator does not model.
        """
        if VARIABLE_ID_FIRST <= type_id <= VARIABLE_ID_LAST:
            return self._variables.get(type_id, ())
        known = self._io_map.dynamic_data_types
        named = {value: name for name, value in known.items()}
        name = named.get(type_id)
        if name == "FirmwareVersion":
            return SIMULATED_FIRMWARE
        if name == "ActualJointPosition":
            return self._joints
        if name == "ActualEndEffectorPose":
            return self._fake_forward_kinematics(self._joints)
        if name == "MotionQueueVelocities":
            return (self._velocity_percent, 100.0, 1000.0, 300.0, 100.0, 100.0)
        if name == "TimeScaling":
            return (100.0,)
        return ()

    def get_position(self) -> RobotPosition:
        """Return the position feedback corresponding to the current state.

        Returns:
            The position that will be published in the input assembly.
        """
        with self._lock:
            return RobotPosition(
                joints=self._joints,
                pose=self._fake_forward_kinematics(self._joints),
                shoulder=InverseKinematicsConfiguration.POSITIVE,
                elbow=InverseKinematicsConfiguration.POSITIVE,
                wrist=InverseKinematicsConfiguration.POSITIVE,
                turn=InverseKinematicsConfiguration.UNDEFINED,
            )

    # ------------------------------------------------------------------
    # Test helpers
    # ------------------------------------------------------------------
    def inject_error(self, error_code: int = int(SimulatorErrorCode.INJECTED)) -> None:
        """Force the simulated robot into the error state.

        Args:
            error_code: Error code to report.
        """
        with self._lock:
            self._enter_error(error_code)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _advance(self, now: float) -> None:
        """Apply the time-based transitions of the state machine.

        Args:
            now: Current time.
        """
        elapsed = 0.0 if self._last_advance is None else max(now - self._last_advance, 0.0)
        self._last_advance = now
        if self._state is SimulatorState.ACTIVATING and now >= self._state_deadline:
            self._state = SimulatorState.ACTIVATED
        elif self._state is SimulatorState.DEACTIVATING and now >= self._state_deadline:
            self._state = SimulatorState.DEACTIVATED
            self._homed = False
        elif self._state is SimulatorState.HOMING and now >= self._state_deadline:
            self._state = SimulatorState.IDLE
            self._homed = True
            self._joints = self._home_position
        elif self._joint_velocity is not None:
            # A velocity-mode command is held: integrate it for as long as it
            # stays held, rather than running towards a target.
            if not self._paused:
                self._joints = self._clamp(
                    tuple(
                        angle + speed * elapsed
                        for angle, speed in zip(self._joints, self._joint_velocity)
                    )
                )
            self._state = (
                SimulatorState.MOVING
                if any(speed for speed in self._joint_velocity) and not self._paused
                else SimulatorState.IDLE
            )
        elif self._state is SimulatorState.MOVING:
            if self._paused:
                # Freeze the move by pushing both of its bounds forward.
                self._move_start_time += elapsed
                self._move_end_time += elapsed
            self._joints = self._interpolate(now)
            if now >= self._move_end_time:
                self._state = SimulatorState.IDLE
                self._joints = self._move_target_joints

    def _apply_robot_control(self, control: RobotControl, now: float) -> None:
        """Apply the robot control bits of an output assembly.

        Args:
            control: Control bits decoded from the output assembly.
            now: Current time.
        """
        if self._state is SimulatorState.ERROR:
            if control.reset_error:
                self._error_code = SimulatorErrorCode.NONE
                self._homed = False
                self._state = SimulatorState.DEACTIVATED
            return

        self._recovery_mode = control.enable_recovery_mode
        if self._state is SimulatorState.DEACTIVATED:
            # Simulation mode may only be toggled while the robot is off.
            self._simulation_mode = control.enable_simulation

        # Deactivation wins over activation, as the vendor documents.
        if control.deactivate:
            if self._state not in (SimulatorState.DEACTIVATED, SimulatorState.DEACTIVATING):
                self._state = SimulatorState.DEACTIVATING
                self._state_deadline = now + self._activation_time_s
            return

        if control.activate and self._state is SimulatorState.DEACTIVATED:
            self._state = SimulatorState.ACTIVATING
            self._state_deadline = now + self._activation_time_s
            return

        if control.home and not self._homed and self._state is SimulatorState.ACTIVATED:
            self._state = SimulatorState.HOMING
            self._state_deadline = now + self._homing_time_s

    def _apply_motion_control(self, control: MotionControl) -> None:
        """Apply the motion control bits of an output assembly.

        Args:
            control: Motion control bits decoded from the output assembly.
        """
        if control.clear_move:
            self._release_velocity()
            if self._state is SimulatorState.MOVING:
                self._state = SimulatorState.IDLE
            self._cleared = True
            self._paused = True
            # Emptying the queue abandons a running offline program with it.
            self._offline_program_id = 0
        elif control.pause:
            self._paused = True

        # ResumeMotion is the only bit that acts on a rising edge.
        if control.resume_motion and not self._previous_resume_motion:
            self._paused = False
            self._cleared = False
        self._previous_resume_motion = control.resume_motion

    def _apply_motion_command(
        self, control: MotionControl, command: MotionCommand, now: float
    ) -> None:
        """Start a new move when the scanner changes the move id.

        Args:
            control: Motion control bits decoded from the output assembly.
            command: Motion command decoded from the output assembly.
            now: Current time.
        """
        if not control.setpoint:
            self._previous_move_id = control.move_id
            self._release_velocity()
            return
        if control.move_id == 0:
            # Held rather than queued: this is a velocity-mode command.
            self._previous_move_id = 0
            self._apply_velocity_command(command)
            return
        self._release_velocity()
        if control.move_id == self._previous_move_id:
            return  # The scanner simply keeps producing the same image.
        self._previous_move_id = control.move_id
        if self._state is SimulatorState.ERROR:
            return

        self._received_move_id = control.move_id
        if VARIABLE_ID_FIRST <= command.command_id <= VARIABLE_ID_LAST:
            # The manual gives this identifier range to variable writes; the
            # arguments are the value.
            self._variables[command.command_id] = tuple(command.arguments)
            return
        name = self._command_names.get(command.command_id)
        if name is None:
            self._enter_error(SimulatorErrorCode.UNKNOWN_MOTION_COMMAND)
            return
        if self._state not in (SimulatorState.IDLE, SimulatorState.MOVING):
            self._enter_error(SimulatorErrorCode.MOVE_WITHOUT_HOMING)
            return

        if name == "StartProgram":
            # A real robot runs the stored program.  Nothing is stored here, so
            # the simulator reports the number it was given and leaves it at
            # that: a scanner sees the acknowledgement and the feedback field
            # without the mock pretending to execute anything.
            self._offline_program_id = int(command.arguments[0])
            return
        if name in ("SetCheckpoint", "SetJointVel"):
            # Accepted and acknowledged without moving, like the real robot.
            self._apply_configuration_command(name, command)
            return
        if name == "Delay":
            # Holds the queue without moving: the same machinery as a move,
            # with the current position as its target.
            self._start_hold(now, float(command.arguments[0]))
            return
        if name not in SUPPORTED_COMMANDS:
            # A real robot accepts every identifier in the table. The
            # acknowledgement is already recorded above; the effect is simply
            # not modelled, which keeps the mock a faithful rehearsal target
            # for a scanner without pretending to be a robot.
            return

        try:
            target = self._resolve_target(name, command)
        except ValueError:
            self._enter_error(SimulatorErrorCode.MOVE_OUT_OF_RANGE)
            return

        distance = max(abs(target[index] - self._joints[index]) for index in range(JOINT_COUNT))
        speed = self._joint_speed_deg_s * self._velocity_percent / 100.0
        self._state = SimulatorState.MOVING
        self._cleared = False
        self._move_start_time = now
        self._move_end_time = now + (distance / speed if speed > 0 else 0.0)
        self._move_start_joints = self._joints
        self._move_target_joints = target

    def _start_hold(self, now: float, duration_s: float) -> None:
        """Keep the robot busy for a while without moving it.

        Args:
            now: Current time.
            duration_s: How long to hold the motion queue.
        """
        self._state = SimulatorState.MOVING
        self._cleared = False
        self._move_start_time = now
        self._move_end_time = now + max(duration_s, 0.0)
        self._move_start_joints = self._joints
        self._move_target_joints = self._joints

    def _apply_configuration_command(self, name: str, command: MotionCommand) -> None:
        """Apply one of the non-moving commands the simulator models.

        Args:
            name: Name the command identifier is registered under.
            command: Motion command decoded from the output assembly.
        """
        if name == "SetCheckpoint":
            # The real robot reports a checkpoint once the motion queue has
            # drained down to it; with no queue to drain, that is immediately.
            self._reached_checkpoint = int(command.arguments[0]) & 0xFFFF
        elif name == "SetJointVel":
            self._velocity_percent = min(max(float(command.arguments[0]), 1.0), 100.0)

    def _release_velocity(self) -> None:
        """Stop holding a velocity-mode command, and settle the state machine.

        Leaving the state at ``MOVING`` would hand the robot back to the
        positional interpolation, which still points at whatever target the
        last queued move had.
        """
        if self._joint_velocity is None:
            return
        self._joint_velocity = None
        if self._state is SimulatorState.MOVING:
            self._state = SimulatorState.IDLE

    def _apply_velocity_command(self, command: MotionCommand) -> None:
        """Hold, change or release a velocity-mode command.

        Args:
            command: Motion command decoded from the output assembly.
        """
        if command.command_id == 0:
            self._release_velocity()
            return
        if self._state is SimulatorState.ERROR:
            return
        name = self._command_names.get(command.command_id)
        if name is None:
            self._enter_error(SimulatorErrorCode.UNKNOWN_MOTION_COMMAND)
            return
        if name != "MoveJointsVel":
            # A declared Cartesian velocity command: accepted, but its effect
            # is not modelled. TODO: model MoveLinVelWrf and MoveLinVelTrf.
            self._joint_velocity = None
            return
        if self._state not in (SimulatorState.IDLE, SimulatorState.MOVING):
            self._enter_error(SimulatorErrorCode.MOVE_WITHOUT_HOMING)
            return
        self._joint_velocity = tuple(float(value) for value in command.arguments[:JOINT_COUNT])

    def _clamp(self, joints: Sequence[float]) -> Tuple[float, ...]:
        """Keep joint angles inside the simulated limits.

        Args:
            joints: Joint angles, in degrees.

        Returns:
            The angles, clamped.
        """
        lower, upper = self._joint_limits_deg
        return tuple(min(max(angle, lower), upper) for angle in joints)

    def _resolve_target(self, name: str, command: MotionCommand) -> Tuple[float, ...]:
        """Convert a motion command into a joint target.

        Args:
            name: Name the command identifier is registered under.
            command: Motion command decoded from the output assembly.

        Returns:
            The joint target, in degrees.

        Raises:
            ValueError: If the command is not one the simulator can execute, or
                if the target falls outside the joint limits.
        """
        if name == "MoveJoints":
            target = tuple(float(value) for value in command.arguments[:JOINT_COUNT])
        elif name == "MoveJointsRel":
            target = tuple(
                angle + float(delta)
                for angle, delta in zip(self._joints, command.arguments[:JOINT_COUNT])
            )
        elif name in ("MovePose", "MoveLin"):
            # The mock draws no distinction between a joint-interpolated and a
            # linear path: it only models where the move ends.
            target = self._fake_inverse_kinematics(command.arguments[:POSE_COUNT])
        elif name in ("MoveLinRelTrf", "MoveLinRelWrf"):
            # With placeholder kinematics that are elementwise, a displacement
            # in the tool frame and one in the world frame coincide.
            current = self._fake_forward_kinematics(self._joints)
            target = self._fake_inverse_kinematics(
                tuple(
                    coordinate + float(delta)
                    for coordinate, delta in zip(current, command.arguments[:POSE_COUNT])
                )
            )
        else:
            raise ValueError("the simulator cannot execute command {!r}".format(name))
        lower, upper = self._joint_limits_deg
        if any(value < lower or value > upper for value in target):
            raise ValueError("move target outside the joint limits")
        return target

    def _interpolate(self, now: float) -> Tuple[float, ...]:
        """Return the joint angles at a given instant of the current move.

        Args:
            now: Current time.

        Returns:
            The interpolated joint angles.
        """
        duration = self._move_end_time - self._move_start_time
        if duration <= 0.0:
            return self._move_target_joints
        ratio = min(max((now - self._move_start_time) / duration, 0.0), 1.0)
        return tuple(
            start + (end - start) * ratio
            for start, end in zip(self._move_start_joints, self._move_target_joints)
        )

    def _enter_error(self, error_code: int) -> None:
        """Move the simulated robot to the error state.

        Args:
            error_code: Error code to report.
        """
        self._state = SimulatorState.ERROR
        self._error_code = SimulatorErrorCode(error_code)

    # ------------------------------------------------------------------
    # Placeholder kinematics
    # ------------------------------------------------------------------
    #: Offsets and scales of the stand-in kinematics.  They are deliberately
    #: trivial and exactly invertible, and have nothing whatsoever to do with
    #: the real robot geometry.
    #: TODO: replace if the mock ever needs to be geometrically credible.
    _FK_OFFSETS = (100.0, 50.0, 200.0, 0.0, 0.0, 0.0)
    _FK_SCALES = (2.0, 2.0, 2.0, 1.0, 1.0, 1.0)

    def _fake_forward_kinematics(self, joints: Sequence[float]) -> Tuple[float, ...]:
        """Convert joint angles into a stand-in Cartesian pose.

        Args:
            joints: Joint angles, in degrees.

        Returns:
            A pose ``(x, y, z, alpha, beta, gamma)``.
        """
        return tuple(
            self._FK_OFFSETS[index] + self._FK_SCALES[index] * value
            for index, value in enumerate(joints)
        )

    def _fake_inverse_kinematics(self, pose: Sequence[float]) -> Tuple[float, ...]:
        """Convert a stand-in Cartesian pose back into joint angles.

        Args:
            pose: Pose ``(x, y, z, alpha, beta, gamma)``.

        Returns:
            The corresponding joint angles, in degrees.

        Raises:
            ValueError: If the pose does not have the expected size.
        """
        if len(pose) != POSE_COUNT:
            raise ValueError("pose must have {} elements".format(POSE_COUNT))
        return tuple(
            (value - self._FK_OFFSETS[index]) / self._FK_SCALES[index]
            for index, value in enumerate(pose)
        )

    def __repr__(self) -> str:
        """Return a short representation including the current state."""
        with self._lock:
            return "<RobotSimulator state={} homed={} error={}>".format(
                self._state.value, self._homed, int(self._error_code)
            )
