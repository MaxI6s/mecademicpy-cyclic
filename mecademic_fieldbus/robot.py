"""Public facade: drive the robot through a fieldbus, with a familiar vocabulary.

:class:`FieldbusRobot` combines a
:class:`~mecademic_fieldbus.transports.base.FieldbusTransport` (raw bytes on
the wire) with an :class:`~mecademic_fieldbus.io_map.base.IoMap` (bytes to
named fields).  It exposes the subset of robot commands that the cyclic
assemblies can express.

Public methods use the ``PascalCase`` names of the robot TCP/IP API
(``ActivateRobot``, ``Home``, ``GetStatusRobot``, ...) on purpose, so that the
API feels familiar to anyone who already knows that robot family.  The
implementation is entirely local: nothing is imported from a vendor library.
Internal helpers follow the usual ``snake_case`` convention.

Two properties of the cyclic protocol shape everything below:

* The output assembly is a *process image* the transport keeps producing, so a
  command is **latched**, not sent.  Every robot control bit is level
  triggered: it acts for as long as it is set.
* A motion command is only taken into account when the **move id changes**,
  and only while the setpoint bit is set.  That is the entire handshake; there
  is no acknowledgement beyond the robot echoing the id back.
"""

import dataclasses
import time
from types import TracebackType
from typing import Any, Callable, Optional, Sequence, Type

from .exceptions import (
    FieldbusStateError,
    FieldbusTimeoutError,
    FieldbusUnsupportedFeature,
    RobotErrorStatus,
)
from .io_map.base import IoMap
from .robot_classes import (
    JOINT_COUNT,
    POSE_COUNT,
    MotionCommand,
    MotionControl,
    MotionStatus,
    RobotControl,
    RobotPosition,
    RobotSafetyStatus,
    RobotStatus,
)
from .transports.base import FieldbusTransport

__all__ = [
    "FieldbusRobot",
    "DEFAULT_TIMEOUT_S",
    "DEFAULT_POLL_INTERVAL_S",
    "MOVE_JOINTS_COMMAND",
    "MOVE_POSE_COMMAND",
]

#: Default time allowed for the robot to reach a requested state, in seconds.
DEFAULT_TIMEOUT_S = 30.0

#: Default delay between two reads of the input assembly while waiting.
DEFAULT_POLL_INTERVAL_S = 0.02

#: Names under which the motion command identifiers are looked up in the
#: assembly specification.  The vendor files do not publish the identifiers
#: themselves; see ``motion_commands`` in ``io_map/spec/assembly_v1.json``.
MOVE_JOINTS_COMMAND = "MoveJoints"
MOVE_POSE_COMMAND = "MovePose"

#: Highest move identifier before wrapping around.  ``0`` is reserved to mean
#: "no move has been requested yet", so the sequence starts at 1.
_MAX_MOVE_ID = 32767


class FieldbusRobot:
    """Drive a Mecademic robot over a cyclic fieldbus connection.

    The robot must already be running in fieldbus mode.  Switching it into that
    mode is out of scope for this library: use the robot web interface or your
    own tooling first.

    Args:
        transport: Transport carrying the raw assemblies.
        io_map: Mapping between raw assemblies and named fields.
        default_timeout_s: Time allowed for the robot to reach a requested
            state, when a method is called without an explicit timeout.
        poll_interval_s: Delay between two reads of the input assembly while
            waiting for a state change.

    Example:
        >>> from mecademic_fieldbus import FieldbusRobot, get_io_map
        >>> from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport
        >>> io_map = get_io_map()
        >>> transport = EtherNetIpTransport.from_io_map(io_map)
        >>> robot = FieldbusRobot(transport, io_map)          # doctest: +SKIP
        >>> robot.Connect("192.168.0.100")                    # doctest: +SKIP
        >>> robot.ActivateRobot()                             # doctest: +SKIP
        >>> robot.Home()                                      # doctest: +SKIP
    """

    def __init__(
        self,
        transport: FieldbusTransport,
        io_map: IoMap,
        default_timeout_s: float = DEFAULT_TIMEOUT_S,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    ) -> None:
        self._transport = transport
        self._io_map = io_map
        self._default_timeout_s = default_timeout_s
        self._poll_interval_s = poll_interval_s
        self._output_image = io_map.empty_output_assembly()
        self._move_id = 0

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------
    @property
    def transport(self) -> FieldbusTransport:
        """Transport used to exchange the raw assemblies."""
        return self._transport

    @property
    def io_map(self) -> IoMap:
        """I/O map used to interpret the raw assemblies."""
        return self._io_map

    @property
    def is_connected(self) -> bool:
        """Whether the underlying transport is connected."""
        return self._transport.is_connected

    @property
    def latched_output_assembly(self) -> bytes:
        """The output assembly image currently produced towards the robot.

        Exposed for debugging and testing; use the commands below to change it.
        """
        return self._output_image

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def Connect(
        self,
        address: str,
        wait_for_cyclic_data: bool = True,
        timeout_s: Optional[float] = None,
        **kwargs: Any,
    ) -> None:
        """Connect to the robot and start the cyclic exchange.

        The output image is reset to all zeros, so a fresh connection never
        re-asserts a command left over from a previous session, and the move
        identifier sequence resumes from the last identifier the robot
        acknowledged.

        Args:
            address: IPv4 address of the robot.
            wait_for_cyclic_data: Wait for the first frame produced by the
                robot before returning.  The robot publishes free-running
                counters, so anything other than an all-zero image proves the
                cyclic exchange is live.
            timeout_s: Time allowed for that first frame.  Defaults to the
                robot default timeout.
            **kwargs: Passed through to the transport.

        Raises:
            FieldbusConnectionError: If the connection cannot be established.
            FieldbusTimeoutError: If no cyclic data arrives in time.
        """
        self._transport.connect(address, **kwargs)
        self._output_image = self._io_map.empty_output_assembly()
        self._transport.write_output_assembly(self._output_image)
        if wait_for_cyclic_data:
            self._wait_for_cyclic_data(self._resolve_timeout(timeout_s))
        # The robot ignores a move id it has already handled, so resume the
        # sequence where it left it rather than restarting at zero.
        self._move_id = self.GetMotionStatus().move_id

    def Disconnect(self) -> None:
        """Release every command and close the connection.

        Safe to call when already disconnected.
        """
        if self._transport.is_connected:
            try:
                self._output_image = self._io_map.empty_output_assembly()
                self._transport.write_output_assembly(self._output_image)
            except Exception:  # noqa: BLE001 - disconnecting must never fail
                pass
        self._transport.disconnect()

    # ------------------------------------------------------------------
    # Robot state commands
    # ------------------------------------------------------------------
    def ActivateRobot(self, timeout_s: Optional[float] = None) -> None:
        """Power up the motors and wait for the robot to report activation.

        Args:
            timeout_s: Time allowed for the robot to activate.  Defaults to the
                robot default timeout.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusTimeoutError: If the robot does not activate in time.
            RobotErrorStatus: If the robot reports an error while activating.
        """
        self._require_control()
        self._update_robot_control(activate=True, deactivate=False)
        self._wait_for_status(
            lambda status: status.activated and not status.busy,
            timeout_s,
            "robot did not activate",
        )

    def DeactivateRobot(self, timeout_s: Optional[float] = None) -> None:
        """Power down the motors and wait for the robot to report deactivation.

        The deactivation request is released once it has taken effect, since
        holding it would prevent any later activation.

        Args:
            timeout_s: Time allowed for the robot to deactivate.  Defaults to
                the robot default timeout.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusTimeoutError: If the robot does not deactivate in time.
        """
        self._require_control()
        self._update_robot_control(deactivate=True, activate=False, home=False)
        try:
            self._wait_for_status(
                lambda status: not status.activated and not status.busy,
                timeout_s,
                "robot did not deactivate",
                raise_on_robot_error=False,
            )
        finally:
            self._update_robot_control(deactivate=False)

    def Home(self, timeout_s: Optional[float] = None) -> None:
        """Run the homing sequence and wait for it to complete.

        Args:
            timeout_s: Time allowed for homing.  Defaults to the robot default
                timeout.

        Raises:
            FieldbusStateError: If the robot is not activated, or if the
                connection may only monitor it.
            FieldbusTimeoutError: If homing does not complete in time.
            RobotErrorStatus: If the robot reports an error while homing.
        """
        self._require_control()
        status = self.GetStatusRobot()
        if not status.activated:
            raise FieldbusStateError("cannot home a robot that is not activated")
        self._update_robot_control(home=True)
        try:
            self._wait_for_status(
                lambda current: current.homed and not current.busy,
                timeout_s,
                "robot did not complete homing",
            )
        finally:
            self._update_robot_control(home=False)

    def ResetError(self, timeout_s: Optional[float] = None) -> None:
        """Clear the robot error and wait for the error code to drop to zero.

        Args:
            timeout_s: Time allowed for the error to clear.  Defaults to the
                robot default timeout.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusTimeoutError: If the error does not clear in time.
        """
        self._require_control()
        self._update_robot_control(reset_error=True)
        try:
            self._wait_for_status(
                lambda status: not status.error_status,
                timeout_s,
                "robot error did not clear",
                raise_on_robot_error=False,
            )
        finally:
            self._update_robot_control(reset_error=False)

    def GetStatusRobot(self) -> RobotStatus:
        """Return the current robot status.

        Returns:
            The status decoded from the latest input assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        return self._io_map.decode_status(self._transport.read_input_assembly())

    def GetMotionStatus(self) -> MotionStatus:
        """Return the current motion queue status.

        Returns:
            The motion status decoded from the latest input assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        return self._io_map.decode_motion_status(self._transport.read_input_assembly())

    def GetSafetyStatus(self) -> RobotSafetyStatus:
        """Return the current safety status.

        This is diagnostic information only and must never be used to
        implement a safety function.

        Returns:
            The safety status decoded from the latest input assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        return self._io_map.decode_safety_status(self._transport.read_input_assembly())

    def GetRobotPosition(self) -> RobotPosition:
        """Return the current joint angles and end effector pose.

        Returns:
            The position decoded from the latest input assembly.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        return self._io_map.decode_position(self._transport.read_input_assembly())

    # ------------------------------------------------------------------
    # Digital outputs
    # ------------------------------------------------------------------
    def SetOutputState(self, *states: Optional[bool]) -> None:
        """Request a new state for the robot digital outputs.

        Args:
            *states: Requested state of each digital output, ``None`` to leave
                one unchanged.

        Raises:
            FieldbusUnsupportedFeature: If the assembly layout in use carries
                no digital outputs, which is the case of the Meca500 cyclic
                assemblies.
            FieldbusConnectionError: If the transport is not connected.
        """
        count = self._io_map.digital_output_count
        if count == 0:
            raise FieldbusUnsupportedFeature(
                "assembly version {} carries no digital outputs; on the Meca500 they are "
                "reachable through the dynamic data slots or the text API, not through the "
                "cyclic assemblies".format(self._io_map.version)
            )
        current = list(self.GetRtOutputState())
        if len(states) > count:
            raise FieldbusStateError(
                "robot exposes {} digital outputs, got {} values".format(count, len(states))
            )
        for index, state in enumerate(states):
            if state is not None:
                current[index] = bool(state)
        self._output_image = self._io_map.encode_output_state(current, self._output_image)
        self._push_output()

    def GetRtOutputState(self) -> Sequence[bool]:
        """Return the digital output states as reported by the robot.

        Returns:
            One boolean per digital output.

        Raises:
            FieldbusUnsupportedFeature: If the assembly layout in use carries
                no digital outputs.
            FieldbusConnectionError: If the transport is not connected.
        """
        return self._io_map.decode_output_state(  # type: ignore[return-value]
            self._transport.read_input_assembly()
        )

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def SendMotionCommand(self, command_id: int, *arguments: float) -> int:
        """Latch a motion command and hand it to the robot.

        This is the primitive every motion helper is built on.  The robot picks
        the command up because the move id changes; the identifier is returned
        so that :meth:`WaitIdle` can wait for that exact command.

        Args:
            command_id: Identifier of the command, as documented in the
                programming manual.
            *arguments: Up to six arguments, whose meaning depends on the
                command.

        Returns:
            The move identifier assigned to this command.

        Raises:
            FieldbusStateError: If the robot is not activated, homed and error
                free, or if the connection may only monitor it.
            FieldbusConnectionError: If the transport is not connected.
        """
        self._require_control()
        status = self.GetStatusRobot()
        if not status.is_ready:
            raise FieldbusStateError(
                "robot must be activated, homed and error free before moving "
                "(activated={}, homed={}, error_code={})".format(
                    status.activated, status.homed, status.error_code
                )
            )
        self._move_id = self._move_id % _MAX_MOVE_ID + 1
        self._output_image = self._io_map.encode_motion_command(
            MotionCommand.build(command_id, arguments), self._output_image
        )
        self._update_motion_control(setpoint=True, move_id=self._move_id)
        return self._move_id

    def MoveJoints(self, *joints: float) -> int:
        """Request a move to a joint target.

        Args:
            *joints: Target angle of each of the six joints, in degrees.

        Returns:
            The move identifier assigned to this command.

        Raises:
            FieldbusUnsupportedFeature: If the ``MoveJoints`` identifier is not
                defined in the assembly specification.
            FieldbusStateError: If the robot is not ready, or the wrong number
                of joints was given.
        """
        if len(joints) != JOINT_COUNT:
            raise FieldbusStateError(
                "MoveJoints expects {} joint angles, got {}".format(JOINT_COUNT, len(joints))
            )
        return self.SendMotionCommand(self._io_map.motion_command_id(MOVE_JOINTS_COMMAND), *joints)

    def MovePose(self, *pose: float) -> int:
        """Request a move to a Cartesian target.

        Args:
            *pose: Target pose ``(x, y, z, alpha, beta, gamma)``, in
                millimetres and degrees.

        Returns:
            The move identifier assigned to this command.

        Raises:
            FieldbusUnsupportedFeature: If the ``MovePose`` identifier is not
                defined in the assembly specification.
            FieldbusStateError: If the robot is not ready, or the wrong number
                of components was given.
        """
        if len(pose) != POSE_COUNT:
            raise FieldbusStateError(
                "MovePose expects {} pose components, got {}".format(POSE_COUNT, len(pose))
            )
        return self.SendMotionCommand(self._io_map.motion_command_id(MOVE_POSE_COMMAND), *pose)

    def PauseMotion(self) -> None:
        """Pause the robot immediately, without clearing its motion queue.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusConnectionError: If the transport is not connected.
        """
        self._require_control()
        self._update_motion_control(pause=True)

    def ResumeMotion(self) -> None:
        """Resume motion after a pause.

        Unlike every other control bit this one acts on its rising edge only,
        so it is pulsed: cleared first in case it was left set, then raised.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusConnectionError: If the transport is not connected.
        """
        self._require_control()
        self._update_motion_control(pause=False, resume_motion=False)
        self._update_motion_control(resume_motion=True)

    def ClearMotion(self) -> None:
        """Erase the queued motion commands and pause the robot.

        Raises:
            FieldbusStateError: If the connection may only monitor the robot.
            FieldbusConnectionError: If the transport is not connected.
        """
        self._require_control()
        self._update_motion_control(clear_move=True)
        self._update_motion_control(clear_move=False)

    def WaitIdle(self, timeout_s: Optional[float] = None) -> None:
        """Wait for the robot to acknowledge the last command and stop moving.

        Args:
            timeout_s: Time allowed for the move to complete.  Defaults to the
                robot default timeout.

        Raises:
            FieldbusTimeoutError: If the robot is still moving when the timeout
                expires.
            RobotErrorStatus: If the robot reports an error while moving.
        """
        expected_move_id = self._move_id
        timeout = self._resolve_timeout(timeout_s)
        deadline = time.monotonic() + timeout
        while True:
            raw_input = self._transport.read_input_assembly()
            status = self._io_map.decode_status(raw_input)
            if status.error_status:
                raise RobotErrorStatus("robot reported an error while moving", status.error_code)
            motion = self._io_map.decode_motion_status(raw_input)
            acknowledged = expected_move_id == 0 or motion.move_id == expected_move_id
            if acknowledged and motion.end_of_block:
                return
            if time.monotonic() >= deadline:
                raise FieldbusTimeoutError(
                    "robot did not become idle within {:.1f} s".format(timeout)
                )
            time.sleep(self._poll_interval_s)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------
    def __enter__(self) -> "FieldbusRobot":
        """Return the robot itself, for use as a context manager."""
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[TracebackType],
    ) -> None:
        """Disconnect the robot when leaving a ``with`` block.

        Args:
            exc_type: Type of the exception that caused the exit, if any.
            exc_value: Exception that caused the exit, if any.
            traceback: Traceback of the exception, if any.
        """
        self.Disconnect()

    def __repr__(self) -> str:
        """Return a short representation including the connection state."""
        return "<FieldbusRobot connected={} io_map={!r}>".format(
            self.is_connected, self._io_map.version
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _require_control(self) -> None:
        """Verify that this connection is allowed to control the robot.

        Raises:
            FieldbusStateError: If the robot reports that the connection may
                only monitor it.
            FieldbusConnectionError: If the transport is not connected.
        """
        if self.GetStatusRobot().monitoring_mode:
            raise FieldbusStateError(
                "the robot reports this connection as monitoring only; open an "
                "exclusive-owner connection to control it"
            )

    def _update_robot_control(self, **changes: bool) -> None:
        """Change some robot control bits of the latched image and produce it.

        Args:
            **changes: Fields of
                :class:`~mecademic_fieldbus.robot_classes.RobotControl` to
                update.
        """
        control: RobotControl = self._io_map.decode_robot_control(self._output_image)
        control = dataclasses.replace(control, **changes)
        self._output_image = self._io_map.encode_robot_control(control, self._output_image)
        self._push_output()

    def _update_motion_control(self, **changes: Any) -> None:
        """Change some motion control bits of the latched image and produce it.

        Args:
            **changes: Fields of
                :class:`~mecademic_fieldbus.robot_classes.MotionControl` to
                update.
        """
        control: MotionControl = self._io_map.decode_motion_control(self._output_image)
        control = dataclasses.replace(control, **changes)
        self._output_image = self._io_map.encode_motion_control(control, self._output_image)
        self._push_output()

    def _push_output(self) -> None:
        """Hand the latched output image over to the transport.

        Raises:
            FieldbusConnectionError: If the transport is not connected.
        """
        self._transport.write_output_assembly(self._output_image)

    def _resolve_timeout(self, timeout_s: Optional[float]) -> float:
        """Return the timeout to use for an operation.

        Args:
            timeout_s: Timeout requested by the caller, possibly ``None``.

        Returns:
            The caller timeout, or the robot default.
        """
        return self._default_timeout_s if timeout_s is None else timeout_s

    def _wait_for_status(
        self,
        predicate: Callable[[RobotStatus], bool],
        timeout_s: Optional[float],
        message: str,
        raise_on_robot_error: bool = True,
    ) -> RobotStatus:
        """Poll the input assembly until a status predicate holds.

        Args:
            predicate: Condition to wait for.
            timeout_s: Time allowed, or ``None`` for the robot default.
            message: Message used when raising on timeout.
            raise_on_robot_error: Whether a robot error should abort the wait.

        Returns:
            The status that satisfied the predicate.

        Raises:
            FieldbusTimeoutError: If the predicate does not hold in time.
            RobotErrorStatus: If the robot reports an error and
                ``raise_on_robot_error`` is set.
        """
        timeout = self._resolve_timeout(timeout_s)
        deadline = time.monotonic() + timeout
        while True:
            status = self.GetStatusRobot()
            if predicate(status):
                return status
            if raise_on_robot_error and status.error_status:
                raise RobotErrorStatus(
                    "{}: the robot is in error".format(message), status.error_code
                )
            if time.monotonic() >= deadline:
                raise FieldbusTimeoutError("{} within {:.1f} s".format(message, timeout))
            time.sleep(self._poll_interval_s)

    def _wait_for_cyclic_data(self, timeout_s: float) -> None:
        """Wait for the first input assembly produced by the robot.

        The robot publishes free-running counters -- timestamps and a dynamic
        data cycle count -- so any image that is not all zeros proves the
        cyclic exchange is live.  No field is inspected here, only the whole
        image, so this stays free of any layout knowledge.

        Args:
            timeout_s: Time allowed for the first frame to arrive.

        Raises:
            FieldbusTimeoutError: If nothing is received in time.
        """
        empty = self._io_map.empty_input_assembly()
        deadline = time.monotonic() + timeout_s
        while True:
            if self._transport.read_input_assembly() != empty:
                return
            if time.monotonic() >= deadline:
                raise FieldbusTimeoutError(
                    "no cyclic data received from the robot within {:.1f} s; check that it "
                    "runs in fieldbus mode and that the connection was accepted".format(timeout_s)
                )
            time.sleep(self._poll_interval_s)
