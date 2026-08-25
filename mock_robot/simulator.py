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
    JOINT_COUNT,
    POSE_COUNT,
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

__all__ = [
    "RobotSimulator",
    "SimulatorState",
    "SimulatorErrorCode",
    "DEMO_MOTION_COMMAND_IDS",
    "DEFAULT_FIFO_SPACE",
]

#: Motion command identifiers used by the simulator and by the tests.
#:
#: **These are not the real identifiers.**  The EDS, the GSDML and the ESI all
#: defer them to the programming manual, so deliberately implausible values are
#: used here to make sure they are never mistaken for the real thing.  Fill the
#: real ones into the ``motion_commands.ids`` section of the assembly
#: specification once they are known.
DEMO_MOTION_COMMAND_IDS: Dict[str, int] = {
    "MoveJoints": 90001,
    "MovePose": 90002,
}

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
            Defaults to those of the I/O map, falling back to
            :data:`DEMO_MOTION_COMMAND_IDS` when the specification defines none.
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

        if motion_command_ids is not None:
            self._command_ids = dict(motion_command_ids)
        else:
            self._command_ids = io_map.motion_command_ids or dict(DEMO_MOTION_COMMAND_IDS)
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
        with self._lock:
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
                reached_checkpoint_id=0,
                discarded_checkpoint_id=0,
                move_id=self._received_move_id,
                fifo_space=DEFAULT_FIFO_SPACE - (1 if moving else 0),
                offline_program_id=0,
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
            if self._state is SimulatorState.MOVING:
                self._state = SimulatorState.IDLE
            self._cleared = True
            self._paused = True
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
        if not control.setpoint or control.move_id == 0:
            self._previous_move_id = control.move_id
            return
        if control.move_id == self._previous_move_id:
            return  # The scanner simply keeps producing the same image.
        self._previous_move_id = control.move_id
        if self._state is SimulatorState.ERROR:
            return

        self._received_move_id = control.move_id
        name = self._command_names.get(command.command_id)
        if name is None:
            self._enter_error(SimulatorErrorCode.UNKNOWN_MOTION_COMMAND)
            return
        if self._state not in (SimulatorState.IDLE, SimulatorState.MOVING):
            self._enter_error(SimulatorErrorCode.MOVE_WITHOUT_HOMING)
            return

        try:
            target = self._resolve_target(name, command)
        except ValueError:
            self._enter_error(SimulatorErrorCode.MOVE_OUT_OF_RANGE)
            return

        distance = max(abs(target[index] - self._joints[index]) for index in range(JOINT_COUNT))
        self._state = SimulatorState.MOVING
        self._cleared = False
        self._move_start_time = now
        self._move_end_time = now + (
            distance / self._joint_speed_deg_s if self._joint_speed_deg_s > 0 else 0.0
        )
        self._move_start_joints = self._joints
        self._move_target_joints = target

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
        elif name == "MovePose":
            target = self._fake_inverse_kinematics(command.arguments[:POSE_COUNT])
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
