"""Version 1 of the robot assembly mapping.

This class is a thin, typed accessor over the declarative specification
``io_map/spec/assembly_v1.json``, which is generated from the official
Mecademic EDS by ``tools/eds_to_spec.py``.  Offsets are never written in
Python: they are read from the specification and applied by
:class:`~mecademic_fieldbus.io_map.codec.AssemblyCodec`.  Only the *field
names* appear below, as module level constants, using the vendor spelling so
that every value can be traced back to the EDS and the programming manual.

Fields the vendor marks as deprecated -- ``RobotStatus_EStop`` and
``MotionStatus_PStop2``, both superseded by the safety status -- are present in
the specification but deliberately not surfaced here.
"""

from typing import Any, Dict, Optional, Sequence, Tuple

from ..exceptions import FieldbusIoMapError
from ..robot_classes import (
    JOINT_COUNT,
    MOTION_ARGUMENT_COUNT,
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
from .base import IoMap
from .codec import AssemblyCodec
from .spec_loader import AssemblySpec, ConnectionProfile, load_spec

__all__ = ["IoMapV1"]

# --- Input assembly field names (robot -> scanner) -------------------------
_IN_BUSY = "RobotStatus_Busy"
_IN_ACTIVATED = "RobotStatus_Activated"
_IN_HOMED = "RobotStatus_Homed"
_IN_SIM_ACTIVATED = "RobotStatus_SimActivated"
_IN_BRAKES_ENGAGED = "RobotStatus_BrakesEngaged"
_IN_RECOVERY_MODE = "RobotStatus_RecoveryMode"
_IN_COLLISION = "RobotStatus_CollisionStatus"
_IN_WORK_ZONE = "RobotStatus_WorkZoneStatus"
_IN_MONITORING_MODE = "RobotStatus_MonitoringMode"
_IN_ERROR = "RobotStatus_Error"

_IN_PAUSED = "MotionStatus_Paused"
_IN_EOB = "MotionStatus_EOB"
_IN_EOM = "MotionStatus_EOM"
_IN_CLEARED = "MotionStatus_Cleared"
_IN_EXCESSIVE_TORQUE = "MotionStatus_ExcessiveTorque"
_IN_REACHED_CHECKPOINT = "MotionStatus_ReachedCheckpointId"
_IN_DISCARDED_CHECKPOINT = "MotionStatus_DiscardedCheckpointId"
_IN_MOVE_ID = "MotionStatus_MoveID"
_IN_FIFO_SPACE = "MotionStatus_FIFOSpace"
_IN_OFFLINE_PROGRAM_ID = "MotionStatus_OfflineProgramId"

_IN_STOP_ESTOP = "StopMask_EStop"
_IN_STOP_PSTOP2 = "StopMask_PStop2"
_IN_STOP_REBOOT = "StopMask_Reboot"
_IN_STOP_CONNECTION_DROPPED = "StopMask_ConnectionDropped"
_IN_RESETTABLE_ESTOP = "StopResettableMask_EStop"
_IN_RESETTABLE_PSTOP2 = "StopResettableMask_PStop2"
_IN_RESETTABLE_REBOOT = "StopResettableMask_Reboot"
_IN_RESETTABLE_CONNECTION_DROPPED = "StopResettableMask_ConnectionDropped"
_IN_RESET_READY = "SafetyStatus_State_ResetReady"
_IN_MOTOR_VOLTAGE_ON = "SafetyStatus_State_VMotorOn"

#: Joint and pose feedback, one field per component in the vendor layout.
_IN_JOINTS = tuple("JointSet_Joint{}Pos".format(index + 1) for index in range(JOINT_COUNT))
_IN_POSE = (
    "EndEffectorPose_PosX",
    "EndEffectorPose_PosY",
    "EndEffectorPose_PosZ",
    "EndEffectorPose_AngA",
    "EndEffectorPose_AngB",
    "EndEffectorPose_AngG",
)
_IN_SHOULDER = "Configuration_Shoulder"
_IN_ELBOW = "Configuration_Elbow"
_IN_WRIST = "Configuration_Wrist"
_IN_TURN = "Configuration_Turn"

# --- Output assembly field names (scanner -> robot) ------------------------
_OUT_DEACTIVATE = "RobotControl_Deactivate"
_OUT_ACTIVATE = "RobotControl_Activate"
_OUT_HOME = "RobotControl_Home"
_OUT_RESET_ERROR = "RobotControl_ResetError"
_OUT_ENABLE_SIMULATION = "RobotControl_EnableSimulation"
_OUT_ENABLE_RECOVERY_MODE = "RobotControl_EnableRecoveryMode"

_OUT_MOVE_ID = "MotionControl_MoveId"
_OUT_SETPOINT = "MotionControl_Setpoint"
_OUT_PAUSE = "MotionControl_Pause"
_OUT_CLEAR_MOVE = "MotionControl_ClearMove"
_OUT_RESUME_MOTION = "MotionControl_ResumeMotion"
_OUT_USE_VARIABLES = "MotionControl_UseVariables"

_OUT_MOTION_COMMAND = "MotionCommand"
_OUT_MOTION_ARGUMENTS = tuple(
    "MotionCommand_Arg{}".format(index + 1) for index in range(MOTION_ARGUMENT_COUNT)
)


class IoMapV1(IoMap):
    """Assembly mapping version 1, backed by ``spec/assembly_v1.json``.

    Args:
        spec: Already parsed specification to use instead of the shipped one.
            Useful for tests and for trying a candidate layout without touching
            the package.
        motion_command_ids: Motion command identifiers overriding those of the
            specification.  The vendor files do not publish them, so this is
            how a caller supplies the values read from the programming manual
            without editing the shipped file.

    Raises:
        FieldbusSpecError: If the shipped specification cannot be loaded or is
            inconsistent.
    """

    #: Assembly layout version implemented by this class.
    SPEC_VERSION = "1"

    def __init__(
        self,
        spec: Optional[AssemblySpec] = None,
        motion_command_ids: Optional[Dict[str, int]] = None,
    ) -> None:
        self._spec = spec if spec is not None else load_spec(self.SPEC_VERSION)
        self.version = self._spec.assembly_version
        self._motion_command_ids = dict(self._spec.motion_commands)
        if motion_command_ids:
            self._motion_command_ids.update(motion_command_ids)

    # ------------------------------------------------------------------
    # Assembly geometry
    # ------------------------------------------------------------------
    @property
    def spec(self) -> AssemblySpec:
        """The declarative specification backing this map."""
        return self._spec

    @property
    def input_assembly_size(self) -> int:
        """Size of the input assembly (robot to scanner), in bytes."""
        return self._input.size_bytes

    @property
    def output_assembly_size(self) -> int:
        """Size of the output assembly (scanner to robot), in bytes."""
        return self._output.size_bytes

    @property
    def input_assembly_instance(self) -> int:
        """CIP instance number of the input assembly."""
        return int(self._input.instance)

    @property
    def output_assembly_instance(self) -> int:
        """CIP instance number of the output assembly."""
        return int(self._output.instance)

    @property
    def config_assembly_instance(self) -> Optional[int]:
        """CIP instance of the configuration assembly, ``None`` on this robot."""
        return self._spec.config.instance

    @property
    def connection(self) -> ConnectionProfile:
        """Connection parameters the robot advertises for this layout."""
        return self._spec.connection

    @property
    def motion_command_ids(self) -> Dict[str, int]:
        """Known motion command identifiers, keyed by command name."""
        return dict(self._motion_command_ids)

    def empty_input_assembly(self) -> bytes:
        """Return an all-zero image of the input assembly."""
        return self._input.zeros()

    def empty_output_assembly(self) -> bytes:
        """Return an all-zero image of the output assembly."""
        return self._output.zeros()

    # ------------------------------------------------------------------
    # Input assembly: robot to scanner
    # ------------------------------------------------------------------
    def decode_status(self, raw_input: bytes) -> RobotStatus:
        """Decode the general robot status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._input.read_many(
            raw_input,
            (
                _IN_BUSY,
                _IN_ACTIVATED,
                _IN_HOMED,
                _IN_SIM_ACTIVATED,
                _IN_BRAKES_ENGAGED,
                _IN_RECOVERY_MODE,
                _IN_COLLISION,
                _IN_WORK_ZONE,
                _IN_MONITORING_MODE,
                _IN_ERROR,
            ),
        )
        return RobotStatus(
            busy=bool(values[_IN_BUSY]),
            activated=bool(values[_IN_ACTIVATED]),
            homed=bool(values[_IN_HOMED]),
            simulation_mode=bool(values[_IN_SIM_ACTIVATED]),
            brakes_engaged=bool(values[_IN_BRAKES_ENGAGED]),
            recovery_mode=bool(values[_IN_RECOVERY_MODE]),
            collision=bool(values[_IN_COLLISION]),
            out_of_work_zone=bool(values[_IN_WORK_ZONE]),
            monitoring_mode=bool(values[_IN_MONITORING_MODE]),
            error_code=int(values[_IN_ERROR]),
        )

    def decode_motion_status(self, raw_input: bytes) -> MotionStatus:
        """Decode the motion queue status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded motion status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._input.read_many(
            raw_input,
            (
                _IN_PAUSED,
                _IN_EOB,
                _IN_EOM,
                _IN_CLEARED,
                _IN_EXCESSIVE_TORQUE,
                _IN_REACHED_CHECKPOINT,
                _IN_DISCARDED_CHECKPOINT,
                _IN_MOVE_ID,
                _IN_FIFO_SPACE,
                _IN_OFFLINE_PROGRAM_ID,
            ),
        )
        return MotionStatus(
            paused=bool(values[_IN_PAUSED]),
            end_of_block=bool(values[_IN_EOB]),
            end_of_movement=bool(values[_IN_EOM]),
            cleared=bool(values[_IN_CLEARED]),
            excessive_torque=bool(values[_IN_EXCESSIVE_TORQUE]),
            reached_checkpoint_id=int(values[_IN_REACHED_CHECKPOINT]),
            discarded_checkpoint_id=int(values[_IN_DISCARDED_CHECKPOINT]),
            move_id=int(values[_IN_MOVE_ID]),
            fifo_space=int(values[_IN_FIFO_SPACE]),
            offline_program_id=int(values[_IN_OFFLINE_PROGRAM_ID]),
        )

    def decode_safety_status(self, raw_input: bytes) -> RobotSafetyStatus:
        """Decode the safety status from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded safety status.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._input.read_many(
            raw_input,
            (
                _IN_STOP_ESTOP,
                _IN_STOP_PSTOP2,
                _IN_STOP_REBOOT,
                _IN_STOP_CONNECTION_DROPPED,
                _IN_RESETTABLE_ESTOP,
                _IN_RESETTABLE_PSTOP2,
                _IN_RESETTABLE_REBOOT,
                _IN_RESETTABLE_CONNECTION_DROPPED,
                _IN_RESET_READY,
                _IN_MOTOR_VOLTAGE_ON,
            ),
        )
        return RobotSafetyStatus(
            stops=SafetyStopFlags(
                estop=bool(values[_IN_STOP_ESTOP]),
                pstop2=bool(values[_IN_STOP_PSTOP2]),
                reboot=bool(values[_IN_STOP_REBOOT]),
                connection_dropped=bool(values[_IN_STOP_CONNECTION_DROPPED]),
            ),
            resettable_stops=SafetyStopFlags(
                estop=bool(values[_IN_RESETTABLE_ESTOP]),
                pstop2=bool(values[_IN_RESETTABLE_PSTOP2]),
                reboot=bool(values[_IN_RESETTABLE_REBOOT]),
                connection_dropped=bool(values[_IN_RESETTABLE_CONNECTION_DROPPED]),
            ),
            reset_ready=bool(values[_IN_RESET_READY]),
            motor_voltage_on=bool(values[_IN_MOTOR_VOLTAGE_ON]),
        )

    def decode_position(self, raw_input: bytes) -> RobotPosition:
        """Decode the position feedback from the input assembly.

        Args:
            raw_input: Raw input assembly image.

        Returns:
            The decoded position.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._input.read_many(
            raw_input,
            _IN_JOINTS + _IN_POSE + (_IN_SHOULDER, _IN_ELBOW, _IN_WRIST, _IN_TURN),
        )
        return RobotPosition(
            joints=tuple(float(values[name]) for name in _IN_JOINTS),
            pose=tuple(float(values[name]) for name in _IN_POSE),
            shoulder=InverseKinematicsConfiguration.from_raw(int(values[_IN_SHOULDER])),
            elbow=InverseKinematicsConfiguration.from_raw(int(values[_IN_ELBOW])),
            wrist=InverseKinematicsConfiguration.from_raw(int(values[_IN_WRIST])),
            turn=InverseKinematicsConfiguration.from_raw(int(values[_IN_TURN])),
        )

    def encode_status(self, status: RobotStatus, raw_input: Optional[bytes] = None) -> bytes:
        """Write a robot status into an input assembly image.

        Args:
            status: Status to encode.
            raw_input: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated input assembly image.
        """
        return self._input.pack(
            {
                _IN_BUSY: status.busy,
                _IN_ACTIVATED: status.activated,
                _IN_HOMED: status.homed,
                _IN_SIM_ACTIVATED: status.simulation_mode,
                _IN_BRAKES_ENGAGED: status.brakes_engaged,
                _IN_RECOVERY_MODE: status.recovery_mode,
                _IN_COLLISION: status.collision,
                _IN_WORK_ZONE: status.out_of_work_zone,
                _IN_MONITORING_MODE: status.monitoring_mode,
                _IN_ERROR: status.error_code,
            },
            base=raw_input,
        )

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
        return self._input.pack(
            {
                _IN_PAUSED: motion_status.paused,
                _IN_EOB: motion_status.end_of_block,
                _IN_EOM: motion_status.end_of_movement,
                _IN_CLEARED: motion_status.cleared,
                _IN_EXCESSIVE_TORQUE: motion_status.excessive_torque,
                _IN_REACHED_CHECKPOINT: motion_status.reached_checkpoint_id,
                _IN_DISCARDED_CHECKPOINT: motion_status.discarded_checkpoint_id,
                _IN_MOVE_ID: motion_status.move_id,
                _IN_FIFO_SPACE: motion_status.fifo_space,
                _IN_OFFLINE_PROGRAM_ID: motion_status.offline_program_id,
            },
            base=raw_input,
        )

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
        return self._input.pack(
            {
                _IN_STOP_ESTOP: safety_status.stops.estop,
                _IN_STOP_PSTOP2: safety_status.stops.pstop2,
                _IN_STOP_REBOOT: safety_status.stops.reboot,
                _IN_STOP_CONNECTION_DROPPED: safety_status.stops.connection_dropped,
                _IN_RESETTABLE_ESTOP: safety_status.resettable_stops.estop,
                _IN_RESETTABLE_PSTOP2: safety_status.resettable_stops.pstop2,
                _IN_RESETTABLE_REBOOT: safety_status.resettable_stops.reboot,
                _IN_RESETTABLE_CONNECTION_DROPPED: (
                    safety_status.resettable_stops.connection_dropped
                ),
                _IN_RESET_READY: safety_status.reset_ready,
                _IN_MOTOR_VOLTAGE_ON: safety_status.motor_voltage_on,
            },
            base=raw_input,
        )

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
        values: Dict[str, Any] = {
            _IN_SHOULDER: int(position.shoulder),
            _IN_ELBOW: int(position.elbow),
            _IN_WRIST: int(position.wrist),
            _IN_TURN: int(position.turn),
        }
        values.update(zip(_IN_JOINTS, self._fit(position.joints, JOINT_COUNT, "joints")))
        values.update(zip(_IN_POSE, self._fit(position.pose, POSE_COUNT, "pose")))
        return self._input.pack(values, base=raw_input)

    # ------------------------------------------------------------------
    # Output assembly: scanner to robot
    # ------------------------------------------------------------------
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
        return self._output.pack(
            {
                _OUT_DEACTIVATE: control.deactivate,
                _OUT_ACTIVATE: control.activate,
                _OUT_HOME: control.home,
                _OUT_RESET_ERROR: control.reset_error,
                _OUT_ENABLE_SIMULATION: control.enable_simulation,
                _OUT_ENABLE_RECOVERY_MODE: control.enable_recovery_mode,
            },
            base=raw_output,
        )

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
        return self._output.pack(
            {
                _OUT_MOVE_ID: control.move_id,
                _OUT_SETPOINT: control.setpoint,
                _OUT_PAUSE: control.pause,
                _OUT_CLEAR_MOVE: control.clear_move,
                _OUT_RESUME_MOTION: control.resume_motion,
                _OUT_USE_VARIABLES: control.use_variables,
            },
            base=raw_output,
        )

    def encode_motion_command(
        self, command: MotionCommand, raw_output: Optional[bytes] = None
    ) -> bytes:
        """Write a motion command into an output assembly image.

        Args:
            command: Command to write.
            raw_output: Image to update.  Defaults to an all-zero image.

        Returns:
            The updated output assembly image.

        Raises:
            FieldbusIoMapError: If the argument list does not fit.
        """
        values: Dict[str, Any] = {_OUT_MOTION_COMMAND: command.command_id}
        values.update(
            zip(
                _OUT_MOTION_ARGUMENTS,
                self._fit(command.arguments, MOTION_ARGUMENT_COUNT, "motion arguments"),
            )
        )
        return self._output.pack(values, base=raw_output)

    def decode_robot_control(self, raw_output: bytes) -> RobotControl:
        """Decode the robot control bits from an output assembly image.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded control bits.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._output.read_many(
            raw_output,
            (
                _OUT_DEACTIVATE,
                _OUT_ACTIVATE,
                _OUT_HOME,
                _OUT_RESET_ERROR,
                _OUT_ENABLE_SIMULATION,
                _OUT_ENABLE_RECOVERY_MODE,
            ),
        )
        return RobotControl(
            deactivate=bool(values[_OUT_DEACTIVATE]),
            activate=bool(values[_OUT_ACTIVATE]),
            home=bool(values[_OUT_HOME]),
            reset_error=bool(values[_OUT_RESET_ERROR]),
            enable_simulation=bool(values[_OUT_ENABLE_SIMULATION]),
            enable_recovery_mode=bool(values[_OUT_ENABLE_RECOVERY_MODE]),
        )

    def decode_motion_control(self, raw_output: bytes) -> MotionControl:
        """Decode the motion control bits from an output assembly image.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded motion control bits.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._output.read_many(
            raw_output,
            (
                _OUT_MOVE_ID,
                _OUT_SETPOINT,
                _OUT_PAUSE,
                _OUT_CLEAR_MOVE,
                _OUT_RESUME_MOTION,
                _OUT_USE_VARIABLES,
            ),
        )
        return MotionControl(
            move_id=int(values[_OUT_MOVE_ID]),
            setpoint=bool(values[_OUT_SETPOINT]),
            pause=bool(values[_OUT_PAUSE]),
            clear_move=bool(values[_OUT_CLEAR_MOVE]),
            resume_motion=bool(values[_OUT_RESUME_MOTION]),
            use_variables=bool(values[_OUT_USE_VARIABLES]),
        )

    def decode_motion_command(self, raw_output: bytes) -> MotionCommand:
        """Decode a motion command from an output assembly image.

        Args:
            raw_output: Raw output assembly image.

        Returns:
            The decoded command.

        Raises:
            FieldbusIoMapError: If the buffer is too short.
        """
        values = self._output.read_many(raw_output, (_OUT_MOTION_COMMAND,) + _OUT_MOTION_ARGUMENTS)
        return MotionCommand(
            command_id=int(values[_OUT_MOTION_COMMAND]),
            arguments=tuple(float(values[name]) for name in _OUT_MOTION_ARGUMENTS),
        )

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def describe_layout(self) -> str:
        """Return a human readable description of the assembly layout.

        Returns:
            A multi-line description of both assemblies, generated from the
            declarative specification.
        """
        source = self._spec.source
        lines = [
            "IoMap version {} - {} firmware {} (EDS {}, {})".format(
                self.version,
                source.get("product_name", "unknown robot"),
                source.get("firmware_revision", "?"),
                source.get("eds_revision", "?"),
                source.get("file", "?"),
            ),
            "connection path {} - RPI {} to {} us".format(
                self.connection.connection_path or "?",
                self.connection.rpi_microseconds_min,
                self.connection.rpi_microseconds_max,
            ),
        ]
        for codec in (self._input, self._output, self._spec.config):
            lines.append("")
            lines.append(
                "{} assembly - instance {}, {} bytes, {}".format(
                    codec.name,
                    "none" if codec.instance is None else codec.instance,
                    codec.size_bytes,
                    codec.direction or "n/a",
                )
            )
            for name in codec.field_names:
                spec = codec.field(name)
                if spec.is_bool:
                    position = "byte {}, bit {}".format(spec.byte_offset, spec.bit_offset)
                else:
                    position = "byte {}".format(spec.byte_offset)
                count = "" if spec.count == 1 else "[{}]".format(spec.count)
                unit = " ({})".format(spec.unit) if spec.unit else ""
                lines.append("  {:<38} {:<9} {}{}".format(name + count, spec.type, position, unit))
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    @property
    def _input(self) -> AssemblyCodec:
        """Codec of the input assembly."""
        return self._spec.input

    @property
    def _output(self) -> AssemblyCodec:
        """Codec of the output assembly."""
        return self._spec.output

    @staticmethod
    def _fit(values: Sequence[float], count: int, label: str) -> Tuple[float, ...]:
        """Pad an empty sequence to the length the assembly expects.

        An empty sequence is a legitimate "nothing to say" value for the facade
        -- :meth:`MotionCommand.none` for instance -- and is encoded as zeros.
        Any other length mismatch is an error.

        Args:
            values: Values supplied by the caller.
            count: Number of elements the assembly carries.
            label: Name of the vector, for error messages.

        Returns:
            A tuple of exactly ``count`` floats.

        Raises:
            FieldbusIoMapError: If the sequence is neither empty nor of the
                expected length.
        """
        if not values:
            return tuple(0.0 for _ in range(count))
        if len(values) != count:
            raise FieldbusIoMapError(
                "{} expects {} element(s), got {}".format(label, count, len(values))
            )
        return tuple(float(value) for value in values)
