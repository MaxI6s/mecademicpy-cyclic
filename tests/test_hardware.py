"""Acceptance suite against a real robot.

This is the only suite that talks to hardware.  It is skipped unless an address
is given, and the tests that actually move the robot need a second, separate
opt-in:

.. code-block:: shell

    # Everything that does not move the robot: connection, layout, activation,
    # the command handshake, pause and resume.
    pytest tests/test_hardware.py --robot-address 192.168.0.100

    # Add homing and the motion tests. READ THE PRECONDITIONS FIRST.
    pytest tests/test_hardware.py --robot-address 192.168.0.100 --allow-motion

    # Move a different joint, by a different amount.
    pytest tests/test_hardware.py --robot-address 192.168.0.100 --allow-motion \\
        --motion-joint 5 --motion-amplitude 2

Without ``--allow-motion`` the suite covers the connection, the whole layout,
activation and the refusal paths.  The command handshake needs a homed robot:
on one that is already homed it runs too, on one that is not it skips, because
homing moves the robot.

**Preconditions, before passing ``--allow-motion``:**

* the workspace is clear and nobody is within the robot's reach;
* an emergency stop is within arm's reach of whoever runs the suite;
* the robot is mounted and, if it carries a tool, the tool is safe to rotate;
* no other scanner owns the exclusive-owner connection.

The suite homes the robot and then moves **one joint** by a small amount --
by default the last one, whose rotation displaces the tool least -- returning
it to where it started.  It never moves in Cartesian space, and it never moves
more than ``--motion-amplitude`` degrees away from the position it found.

What it is for: proving that the specification generated from the vendor EDS
matches the firmware in front of you.  A wrong offset shows up here as an
implausible decoded value long before it shows up as a robot doing something
unexpected.
"""

import time
from typing import Callable, Iterator

import pytest

from mecademic_fieldbus import FieldbusRobot, FieldbusUnsupportedFeature, get_io_map
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot_classes import JOINT_COUNT, POSE_COUNT

pytestmark = pytest.mark.hardware

ethernetip = pytest.importorskip("ethernetip", reason="the 'ethernetip' package is not installed")

from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport  # noqa: E402

#: Time allowed for the robot to reach a requested state.
TIMEOUT_S = 30.0

#: Widest plausible joint angle, in degrees.  The real limits are narrower and
#: differ per joint; this is a sanity bound, not a safety check.
JOINT_SANITY_DEG = 400.0

#: Widest plausible Cartesian coordinate, in millimetres, for a robot whose
#: reach is under a metre.
POSITION_SANITY_MM = 2000.0

#: Motion queue capacity the manual documents.
MAX_FIFO_SPACE = 13000

#: Highest value ``RobotStatus_Error`` can carry, per the EDS.
MAX_ERROR_CODE = 32767


# ----------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------
@pytest.fixture(scope="session")
def hardware_io_map() -> IoMap:
    """Return the shipped I/O map, exactly as an application would get it."""
    return get_io_map()


@pytest.fixture(scope="session")
def robot(
    robot_address: str, hardware_io_map: IoMap, pytestconfig: pytest.Config
) -> Iterator[FieldbusRobot]:
    """Connect to the robot once for the whole session, and leave it safe.

    Teardown always runs: any held velocity command is released, the motion
    queue is cleared, a pending error is reset and the robot is deactivated,
    whatever the tests did.
    """
    overrides = {}
    port = pytestconfig.getoption("--robot-udp-port")
    if port is not None:
        overrides["originator_udp_port"] = int(port)
    transport = EtherNetIpTransport.from_io_map(hardware_io_map, **overrides)
    robot = FieldbusRobot(transport, hardware_io_map, default_timeout_s=TIMEOUT_S)
    robot.Connect(robot_address)
    try:
        yield robot
    finally:
        _leave_safe(robot)
        robot.Disconnect()


@pytest.fixture(autouse=True)
def live_connection(robot: FieldbusRobot, robot_address: str) -> None:
    """Start every test from a live connection.

    The session shares one connection, so without this a single failure that
    drops it turns every later test into a confusing "transport is not
    connected".  Restoring it keeps the rest of the run meaningful, which is
    what you want from an acceptance suite: one run, the whole picture.

    Args:
        robot: The session facade.
        robot_address: Address to reconnect to.
    """
    if not robot.is_connected:
        robot.Connect(robot_address)


def _leave_safe(robot: FieldbusRobot) -> None:
    """Bring the robot back to a resting state, ignoring failures.

    Args:
        robot: Connected facade.
    """
    for action in (
        robot.StopVelocityCommand,
        lambda: robot.ClearMotion(timeout_s=5.0),
        lambda: robot.ResetError(timeout_s=5.0),
        lambda: robot.DeactivateRobot(timeout_s=TIMEOUT_S),
    ):
        try:
            action()
        except Exception:  # noqa: BLE001 - teardown must never mask a failure
            pass


def ensure_activated(robot: FieldbusRobot) -> None:
    """Make sure the robot is powered, activating it if needed.

    Args:
        robot: Connected facade.
    """
    if robot.GetStatusRobot().error_status:
        robot.ResetError()
    if not robot.GetStatusRobot().activated:
        robot.ActivateRobot()


def ensure_ready(robot: FieldbusRobot, allow_motion: bool) -> None:
    """Make sure the robot is activated and homed, or skip the test.

    Homing moves the robot.  On a robot that is already homed this costs
    nothing; on one that is not, it homes only when that was allowed, and skips
    otherwise.

    Args:
        robot: Connected facade.
        allow_motion: Whether the suite may move the robot.
    """
    ensure_activated(robot)
    if robot.GetStatusRobot().homed:
        return
    if not allow_motion:
        pytest.skip("the robot is not homed, and homing it moves it")
    robot.Home()


def wait_until(predicate: Callable[[], bool], description: str, timeout_s: float = 10.0) -> None:
    """Poll a predicate until it holds, or fail the test.

    Args:
        predicate: Condition to wait for.
        description: What is being waited on, for the failure message.
        timeout_s: Time allowed.

    Raises:
        AssertionError: If the predicate never holds.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail("timed out waiting for {}".format(description))


# ----------------------------------------------------------------------
# 1. Connection: does the generated connection profile match the firmware?
# ----------------------------------------------------------------------
def test_connection_is_established(robot: FieldbusRobot) -> None:
    """The Forward Open built from the EDS is accepted by the real firmware.

    Reaching this point already proves the connection path, the assembly
    instances, the negotiated sizes and the RPI are all right: the robot
    validates every one of them and refuses the connection otherwise.
    """
    assert robot.is_connected is True


def test_the_connection_may_control_the_robot(robot: FieldbusRobot) -> None:
    """This is an exclusive-owner connection, not a monitoring one."""
    assert robot.GetStatusRobot().monitoring_mode is False


def test_cyclic_data_is_live(robot: FieldbusRobot, hardware_io_map: IoMap) -> None:
    """The robot produces continuously, not just once.

    The input image must keep changing: the robot publishes free-running
    counters, so two reads a few cycles apart cannot be identical.
    """
    first = robot.transport.read_input_assembly()
    assert first != hardware_io_map.empty_input_assembly()
    wait_until(
        lambda: robot.transport.read_input_assembly() != first,
        "the input assembly to change",
        timeout_s=2.0,
    )


# ----------------------------------------------------------------------
# 2. Layout: are the generated offsets the ones the firmware uses?
# ----------------------------------------------------------------------
def test_robot_status_decodes_plausibly(robot: FieldbusRobot) -> None:
    """Every status field lands in the range its definition allows."""
    status = robot.GetStatusRobot()
    assert 0 <= status.error_code <= MAX_ERROR_CODE
    # Mutually exclusive by construction: the robot cannot be off and homed.
    if not status.activated:
        assert status.homed is False
    # Brakes can only be engaged while the robot is deactivated.
    if status.brakes_engaged:
        assert status.activated is False


def test_motion_status_decodes_plausibly(robot: FieldbusRobot) -> None:
    """The motion queue reports a capacity inside its documented bounds.

    A misplaced offset would almost certainly put this outside the range, so
    it is a cheap and effective check on the second half of the layout.
    """
    motion = robot.GetMotionStatus()
    assert 0 < motion.fifo_space <= MAX_FIFO_SPACE
    assert 0 <= motion.move_id <= MAX_ERROR_CODE
    assert 0 <= motion.reached_checkpoint_id <= 0xFFFF
    assert 0 <= motion.offline_program_id <= 0xFFFF


def test_position_decodes_plausibly(robot: FieldbusRobot) -> None:
    """Joint angles and pose are finite and within a plausible envelope."""
    position = robot.GetRobotPosition()
    assert len(position.joints) == JOINT_COUNT
    assert len(position.pose) == POSE_COUNT
    for angle in position.joints:
        assert angle == angle  # not NaN
        assert abs(angle) < JOINT_SANITY_DEG
    for coordinate in position.pose[:3]:
        assert coordinate == coordinate
        assert abs(coordinate) < POSITION_SANITY_MM
    for sign in (position.shoulder, position.elbow, position.wrist):
        assert int(sign) in (-1, 0, 1)


def test_safety_status_decodes_plausibly(robot: FieldbusRobot) -> None:
    """The safety view is consistent with the robot being reachable.

    Nothing here is a safety function; it checks that the two stop masks and
    the state bits sit where the specification says they do.
    """
    safety = robot.GetSafetyStatus()
    status = robot.GetStatusRobot()
    # A resettable stop can only be resettable if it is actually asserted.
    for name in ("estop", "pstop2", "reboot", "connection_dropped"):
        if getattr(safety.resettable_stops, name):
            assert getattr(safety.stops, name) is True, name
    # Motors cannot be live while the robot reports itself deactivated.
    if safety.motor_voltage_on is False:
        assert status.activated is False


def test_digital_io_is_reported_as_unsupported(robot: FieldbusRobot) -> None:
    """The cyclic assemblies carry no digital I/O, and the library says so.

    The programming manual confirms it by listing SetOutputState among the
    commands not available on this robot.
    """
    with pytest.raises(FieldbusUnsupportedFeature):
        robot.GetRtOutputState()
    with pytest.raises(FieldbusUnsupportedFeature):
        robot.SetOutputState(True)


# ----------------------------------------------------------------------
# 3. Activation: the triggered actions, without moving anything
# ----------------------------------------------------------------------
def test_activation_powers_the_motors(robot: FieldbusRobot) -> None:
    """Activating powers the motors and clears the busy flag when done."""
    ensure_activated(robot)
    status = robot.GetStatusRobot()
    assert status.activated is True
    assert status.busy is False
    assert robot.GetSafetyStatus().motor_voltage_on is True
    assert status.brakes_engaged is False


def test_reset_error_is_harmless_without_an_error(robot: FieldbusRobot) -> None:
    """Resetting when there is nothing to reset is a no-op, not a failure."""
    ensure_activated(robot)
    robot.ResetError()
    assert robot.GetStatusRobot().error_code == 0


# ----------------------------------------------------------------------
# 4. Homing: needs --allow-motion, and everything below needs a homed robot
# ----------------------------------------------------------------------
@pytest.mark.motion
def test_homing(robot: FieldbusRobot) -> None:
    """The robot homes and reports itself ready.

    This runs before the command tests because a robot only accepts motion
    commands once homed.  On a robot that is already homed the tests below run
    without ``--allow-motion``; on one that is not, they skip.
    """
    ensure_activated(robot)
    robot.Home()
    status = robot.GetStatusRobot()
    assert status.homed is True
    assert status.busy is False
    assert status.is_ready is True


@pytest.mark.motion
def test_home_request_is_released(robot: FieldbusRobot, hardware_io_map: IoMap) -> None:
    """The homing bit is dropped once the robot reports the homed state."""
    ensure_activated(robot)
    robot.Home()
    control = hardware_io_map.decode_robot_control(robot.latched_output_assembly)
    assert control.home is False
    assert control.activate is True


# ----------------------------------------------------------------------
# 5. Command handshake: proven without moving a single axis
# ----------------------------------------------------------------------
def test_checkpoint_round_trips_through_the_assemblies(
    robot: FieldbusRobot, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """A queued command is acknowledged, and its argument comes back.

    SetCheckpoint is the ideal probe: it travels the whole output path --
    MotionCommandID, the arguments, the move id and the setpoint bit -- and the
    robot echoes the checkpoint number back through the input assembly, all
    without moving an axis.  If this passes, the command handshake is correct.
    """
    ensure_ready(robot, allow_motion)
    checkpoint = (robot.GetMotionStatus().reached_checkpoint_id % 1000) + 1
    move_id = robot.SendMotionCommand(
        hardware_io_map.motion_command_id("SetCheckpoint"), float(checkpoint)
    )

    wait_until(
        lambda: robot.GetMotionStatus().move_id == move_id,
        "the robot to acknowledge the move id",
    )
    wait_until(
        lambda: robot.GetMotionStatus().reached_checkpoint_id == checkpoint,
        "the robot to report checkpoint {}".format(checkpoint),
    )


def test_configuration_command_is_accepted(
    robot: FieldbusRobot, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """A configuration command is acknowledged, again without moving.

    SetJointVel only changes the speed used by later moves, so it is safe to
    send at any time; the robot acknowledging its move id proves the command
    was understood.
    """
    ensure_ready(robot, allow_motion)
    move_id = robot.SendMotionCommand(hardware_io_map.motion_command_id("SetJointVel"), 10.0)
    wait_until(
        lambda: robot.GetMotionStatus().move_id == move_id,
        "the robot to accept SetJointVel",
    )


def test_move_ids_are_never_reused(
    robot: FieldbusRobot, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """Successive commands carry distinct ids, which is the whole handshake."""
    ensure_ready(robot, allow_motion)
    command = hardware_io_map.motion_command_id("SetJointVel")
    first = robot.SendMotionCommand(command, 10.0)
    wait_until(lambda: robot.GetMotionStatus().move_id == first, "the first acknowledgement")
    second = robot.SendMotionCommand(command, 20.0)
    wait_until(lambda: robot.GetMotionStatus().move_id == second, "the second acknowledgement")
    assert second != first


def test_pause_clear_and_resume(robot: FieldbusRobot, allow_motion: bool) -> None:
    """The motion control bits are confirmed by their status bits.

    Safe with an empty queue: pausing a robot that is not moving changes a flag
    and nothing else.
    """
    ensure_ready(robot, allow_motion)
    robot.PauseMotion()
    assert robot.GetMotionStatus().paused is True

    robot.ClearMotion()
    assert robot.GetMotionStatus().cleared is True

    robot.ResumeMotion()
    assert robot.GetMotionStatus().paused is False


def test_velocity_commands_cannot_be_queued(
    robot: FieldbusRobot, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """The library refuses the mix-up the manual warns about, before sending."""
    from mecademic_fieldbus.exceptions import FieldbusStateError

    ensure_ready(robot, allow_motion)
    with pytest.raises(FieldbusStateError):
        robot.SendMotionCommand(hardware_io_map.motion_command_id("MoveJointsVel"), 0.0)
    with pytest.raises(FieldbusStateError):
        robot.StartVelocityCommand(
            hardware_io_map.motion_command_id("MoveJoints"), *([0.0] * JOINT_COUNT)
        )


# ----------------------------------------------------------------------
# 6. Motion: needs --allow-motion
# ----------------------------------------------------------------------
@pytest.mark.motion
def test_move_joints_reaches_its_target_and_returns(
    robot: FieldbusRobot, motion_joint: int, motion_amplitude: float
) -> None:
    """One joint moves by the configured amount, then comes back.

    The target is computed from the position the robot is actually in, so the
    suite never commands an absolute pose it has not measured, and never moves
    further than the amplitude it was given.
    """
    ensure_activated(robot)
    robot.Home()
    start = list(robot.GetRobotPosition().joints)

    target = list(start)
    target[motion_joint] = start[motion_joint] + motion_amplitude
    move_id = robot.MoveJoints(*target)
    robot.WaitIdle()

    assert robot.GetMotionStatus().move_id == move_id
    reached = robot.GetRobotPosition().joints
    assert reached[motion_joint] == pytest.approx(target[motion_joint], abs=0.5)

    robot.MoveJoints(*start)
    robot.WaitIdle()
    assert robot.GetRobotPosition().joints[motion_joint] == pytest.approx(
        start[motion_joint], abs=0.5
    )


@pytest.mark.motion
def test_end_of_block_tracks_the_move(
    robot: FieldbusRobot, motion_joint: int, motion_amplitude: float
) -> None:
    """The robot reports itself busy during a move and idle afterwards."""
    ensure_activated(robot)
    robot.Home()
    start = list(robot.GetRobotPosition().joints)
    target = list(start)
    target[motion_joint] = start[motion_joint] + motion_amplitude

    robot.MoveJoints(*target)
    robot.WaitIdle()
    motion = robot.GetMotionStatus()
    assert motion.end_of_block is True
    assert motion.end_of_movement is True

    robot.MoveJoints(*start)
    robot.WaitIdle()


@pytest.mark.motion
def test_velocity_mode_moves_and_stops(
    robot: FieldbusRobot,
    hardware_io_map: IoMap,
    motion_joint: int,
    motion_amplitude: float,
) -> None:
    """A held velocity command moves the robot, and releasing it stops it.

    The velocity is chosen so that the joint travels roughly the configured
    amplitude over the hold, and the command is released in a ``finally`` so a
    failure never leaves the robot moving.
    """
    ensure_activated(robot)
    robot.Home()
    start = robot.GetRobotPosition().joints
    hold_s = 0.5
    velocity = [0.0] * JOINT_COUNT
    velocity[motion_joint] = motion_amplitude / hold_s

    try:
        robot.StartVelocityCommand(hardware_io_map.motion_command_id("MoveJointsVel"), *velocity)
        time.sleep(hold_s)
    finally:
        robot.StopVelocityCommand()

    wait_until(
        lambda: robot.GetMotionStatus().end_of_movement,
        "the robot to come to a standstill",
    )
    moved = robot.GetRobotPosition().joints[motion_joint] - start[motion_joint]
    assert abs(moved) > 0.1, "the robot did not move under a velocity command"
    assert abs(moved) < abs(motion_amplitude) * 3.0, "it moved much further than asked"

    settled = robot.GetRobotPosition().joints[motion_joint]
    time.sleep(0.3)
    assert robot.GetRobotPosition().joints[motion_joint] == pytest.approx(settled, abs=0.2)

    robot.MoveJoints(*start)
    robot.WaitIdle()


@pytest.mark.motion
def test_out_of_range_target_is_refused(
    robot: FieldbusRobot, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """A target outside the joint limits is refused rather than attempted.

    If the robot reports an error, the suite clears it; if it silently ignores
    the command instead, the test is skipped rather than failed, since the
    manual does not specify which of the two happens.
    """
    ensure_ready(robot, allow_motion)
    start = list(robot.GetRobotPosition().joints)
    impossible = list(start)
    impossible[0] = 1000.0

    robot.SendMotionCommand(hardware_io_map.motion_command_id("MoveJoints"), *impossible)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if robot.GetStatusRobot().error_status:
            break
        time.sleep(0.05)
    else:
        pytest.skip("the robot ignored an out-of-range target instead of erroring")

    assert robot.GetStatusRobot().error_code != 0
    robot.ResetError()
    assert robot.GetStatusRobot().error_status is False


# ----------------------------------------------------------------------
# 7. Reconnection, and the full power cycle
# ----------------------------------------------------------------------
def test_reconnecting_resumes_the_move_id_sequence(
    robot: FieldbusRobot, robot_address: str, hardware_io_map: IoMap, allow_motion: bool
) -> None:
    """A scanner restarting against a live robot picks up where it left off.

    The robot ignores a move id it has already handled, so a fresh connection
    that restarted the numbering at zero would see its first command silently
    discarded.  This is the regression that motivated seeding the sequence from
    the robot at connection time.
    """
    ensure_ready(robot, allow_motion)
    command = hardware_io_map.motion_command_id("SetJointVel")
    first = robot.SendMotionCommand(command, 10.0)
    wait_until(lambda: robot.GetMotionStatus().move_id == first, "the first acknowledgement")

    robot.Disconnect()
    robot.Connect(robot_address)
    assert robot.is_connected is True
    # The receiving socket must have been released, or this second connection
    # would have failed outright on the standard UDP port.
    assert robot.transport.is_connected is True

    ensure_ready(robot, allow_motion)
    second = robot.SendMotionCommand(command, 20.0)
    assert second != first
    wait_until(
        lambda: robot.GetMotionStatus().move_id == second,
        "the robot to accept a command after reconnecting",
    )


def test_deactivate_and_reactivate(robot: FieldbusRobot, hardware_io_map: IoMap) -> None:
    """The robot powers down and back up, releasing the request bit in between.

    This one runs last on purpose: deactivating clears the homing reference,
    which every command test above depends on.
    """
    ensure_activated(robot)
    robot.DeactivateRobot()
    assert robot.GetStatusRobot().activated is False
    # Holding the deactivate bit would block every later activation.
    control = hardware_io_map.decode_robot_control(robot.latched_output_assembly)
    assert control.deactivate is False

    robot.ActivateRobot()
    assert robot.GetStatusRobot().activated is True

    robot.DeactivateRobot()
    assert robot.GetStatusRobot().activated is False
    assert robot.GetStatusRobot().brakes_engaged is True
