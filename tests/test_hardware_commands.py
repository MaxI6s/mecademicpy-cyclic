"""Coverage of the motion command table, against a real robot.

The specification lists 48 command identifiers.  They are not equally safe to
send: one changes joint limits, another reboots the robot, several drive a
gripper that may be holding something.  Firing all of them blindly at a robot
would be reckless, so every identifier is classified below, with the reason,
and the risky groups need their own opt-in:

.. code-block:: shell

    # Commands that neither move the robot nor change a setting.
    pytest tests/test_hardware_commands.py --robot-address 192.168.0.100

    # Add the ones that change velocities, blending and auto-configuration.
    # The suite restores documented defaults afterwards.
    pytest ... --robot-address 192.168.0.100 --allow-config

    # Add joint-space motion, then Cartesian motion, then the tool.
    pytest ... --allow-motion --allow-cartesian --allow-tool

:data:`NEVER_SENT` and :data:`NEEDS_DELIBERATE_SETUP` are never exercised, by
design.  They are still listed, because
:func:`test_every_command_is_classified` fails when a firmware update adds an
identifier nobody has triaged -- which is how this file stays honest as the
robot evolves.

What "tested" means here: the robot **accepts** the command and acknowledges
its move id.  That catches a wrong identifier, a wrong argument count and a
command the firmware rejects.  It does not confirm the *effect* of a setting,
because the input assembly does not carry settings back; reading them requires
the dynamic data slots, which this library does not surface yet.  Where an
effect is observable with what the assembly does carry -- a checkpoint, a
delay, a position, the duration of a move -- there is a dedicated test that
checks it.
"""

import time
from typing import Tuple

import pytest

from mecademic_fieldbus import FieldbusRobot
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot_classes import JOINT_COUNT

from .command_groups import (
    CONFIGURATION_COMMANDS,
    HARMLESS_COMMANDS,
    TOOL_COMMANDS,
)
from .test_hardware import ensure_ready, wait_until  # noqa: F401  (shared helpers)

pytestmark = pytest.mark.hardware

#: How far the Cartesian tests move the tool, in millimetres.
CARTESIAN_STEP_MM = 1.0

#: Cartesian move commands exercised against an absolute, measured pose.
ABSOLUTE_CARTESIAN = ("MovePose", "MoveLin")

#: Cartesian move commands exercised as a delta, out and back.
RELATIVE_CARTESIAN = ("MoveLinRelTrf", "MoveLinRelWrf")

#: Cartesian velocity commands, held briefly and released.
CARTESIAN_VELOCITY = ("MoveLinVelWrf", "MoveLinVelTrf")


def acknowledge(
    robot: FieldbusRobot, io_map: IoMap, name: str, arguments: Tuple[float, ...]
) -> None:
    """Send one command and wait for the robot to acknowledge its move id.

    Args:
        robot: Connected, homed facade.
        io_map: Map providing the identifier.
        name: Command name.
        arguments: Arguments to send.
    """
    move_id = robot.SendMotionCommand(io_map.motion_command_id(name), *arguments)
    wait_until(
        lambda: robot.GetMotionStatus().move_id == move_id,
        "the robot to acknowledge {}".format(name),
    )
    assert robot.GetStatusRobot().error_code == 0, "{} put the robot in error".format(name)


# ----------------------------------------------------------------------
# Harmless commands, and the effects that are observable today
# ----------------------------------------------------------------------
@pytest.mark.parametrize("name", sorted(HARMLESS_COMMANDS), ids=sorted(HARMLESS_COMMANDS))
def test_harmless_command_is_accepted(
    robot: FieldbusRobot, io_map: IoMap, allow_motion: bool, name: str
) -> None:
    """A command with no lasting effect is accepted and acknowledged."""
    ensure_ready(robot, allow_motion)
    acknowledge(robot, io_map, name, HARMLESS_COMMANDS[name])


def test_checkpoint_value_comes_back(
    robot: FieldbusRobot, io_map: IoMap, allow_motion: bool
) -> None:
    """SetCheckpoint's argument is echoed in the input assembly.

    One of the few commands whose effect the cyclic input actually shows, and
    it needs no motion at all.
    """
    ensure_ready(robot, allow_motion)
    checkpoint = (robot.GetMotionStatus().reached_checkpoint_id % 500) + 7
    acknowledge(robot, io_map, "SetCheckpoint", (float(checkpoint),))
    wait_until(
        lambda: robot.GetMotionStatus().reached_checkpoint_id == checkpoint,
        "checkpoint {} to be reported".format(checkpoint),
    )


def test_delay_keeps_the_robot_busy(
    robot: FieldbusRobot, io_map: IoMap, allow_motion: bool
) -> None:
    """Delay holds the motion queue for its duration, without moving anything.

    This proves a queued command really is *executed*, not merely received:
    the robot leaves the end-of-block state for about as long as it was asked
    to wait.
    """
    ensure_ready(robot, allow_motion)
    before = robot.GetRobotPosition().joints
    started = time.monotonic()
    robot.SendMotionCommand(io_map.motion_command_id("Delay"), 1.0)
    robot.WaitIdle(timeout_s=15.0)
    elapsed = time.monotonic() - started

    assert elapsed > 0.5, "the robot returned to idle too quickly to have waited"
    assert robot.GetRobotPosition().joints == pytest.approx(before, abs=0.01)


# ----------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------
@pytest.mark.config
@pytest.mark.parametrize("name", sorted(CONFIGURATION_COMMANDS), ids=sorted(CONFIGURATION_COMMANDS))
def test_configuration_command_is_accepted(
    robot: FieldbusRobot, io_map: IoMap, allow_motion: bool, name: str
) -> None:
    """A setting is accepted, then restored to its documented default.

    The input assembly carries no settings back, so this confirms acceptance
    rather than effect; reading them back needs the dynamic data slots.
    """
    ensure_ready(robot, allow_motion)
    sent, restore = CONFIGURATION_COMMANDS[name]
    try:
        acknowledge(robot, io_map, name, sent)
    finally:
        acknowledge(robot, io_map, name, restore)


@pytest.mark.config
@pytest.mark.motion
def test_joint_velocity_actually_changes_the_move_duration(
    robot: FieldbusRobot, io_map: IoMap, motion_joint: int, motion_amplitude: float
) -> None:
    """SetJointVel has a measurable effect, not just an acknowledgement.

    The same move is timed at two velocities; the slow one must take longer.
    This is the only way to confirm a setting took hold without reading it
    back, and it is why the command tests above are honest about what they do
    not prove.
    """
    ensure_ready(robot, True)
    start = list(robot.GetRobotPosition().joints)
    target = list(start)
    target[motion_joint] = start[motion_joint] + motion_amplitude

    def timed_move(destination: list, velocity: float) -> float:
        """Run one move at a given velocity and return how long it took."""
        acknowledge(robot, io_map, "SetJointVel", (velocity,))
        began = time.monotonic()
        robot.MoveJoints(*destination)
        robot.WaitIdle(timeout_s=60.0)
        return time.monotonic() - began

    try:
        fast = timed_move(target, 50.0)
        timed_move(start, 50.0)
        slow = timed_move(target, 5.0)
    finally:
        acknowledge(robot, io_map, "SetJointVel", (100.0,))
        robot.MoveJoints(*start)
        robot.WaitIdle(timeout_s=60.0)

    assert slow > fast * 1.5, "a tenfold slower velocity barely changed the duration"


# ----------------------------------------------------------------------
# Joint-space motion
# ----------------------------------------------------------------------
@pytest.mark.motion
def test_move_joints_rel_moves_and_returns(
    robot: FieldbusRobot, io_map: IoMap, motion_joint: int, motion_amplitude: float
) -> None:
    """MoveJointsRel moves by a delta, which is the safest way to move at all.

    A relative command cannot send the robot somewhere unexpected because of a
    stale reference: the delta is all there is.
    """
    ensure_ready(robot, True)
    start = robot.GetRobotPosition().joints
    delta = [0.0] * JOINT_COUNT
    delta[motion_joint] = motion_amplitude

    acknowledge(robot, io_map, "MoveJointsRel", tuple(delta))
    robot.WaitIdle(timeout_s=60.0)
    moved = robot.GetRobotPosition().joints
    assert moved[motion_joint] == pytest.approx(start[motion_joint] + motion_amplitude, abs=0.5)

    delta[motion_joint] = -motion_amplitude
    acknowledge(robot, io_map, "MoveJointsRel", tuple(delta))
    robot.WaitIdle(timeout_s=60.0)
    assert robot.GetRobotPosition().joints[motion_joint] == pytest.approx(
        start[motion_joint], abs=0.5
    )


# ----------------------------------------------------------------------
# Cartesian motion
# ----------------------------------------------------------------------
@pytest.mark.cartesian
@pytest.mark.parametrize("name", ABSOLUTE_CARTESIAN)
def test_absolute_cartesian_move(robot: FieldbusRobot, io_map: IoMap, name: str) -> None:
    """A Cartesian move of one millimetre, computed from the measured pose.

    The target is the pose the robot reports, nudged along Z, so the suite
    never commands a pose it has not measured.
    """
    ensure_ready(robot, True)
    start = list(robot.GetRobotPosition().pose)
    target = list(start)
    target[2] = start[2] + CARTESIAN_STEP_MM

    acknowledge(robot, io_map, name, tuple(target))
    robot.WaitIdle(timeout_s=60.0)
    assert robot.GetRobotPosition().pose[2] == pytest.approx(target[2], abs=0.2)

    acknowledge(robot, io_map, name, tuple(start))
    robot.WaitIdle(timeout_s=60.0)
    assert robot.GetRobotPosition().pose[2] == pytest.approx(start[2], abs=0.2)


@pytest.mark.cartesian
@pytest.mark.parametrize("name", RELATIVE_CARTESIAN)
def test_relative_cartesian_move(robot: FieldbusRobot, io_map: IoMap, name: str) -> None:
    """A relative Cartesian move, out and back."""
    ensure_ready(robot, True)
    start = robot.GetRobotPosition().pose
    out = (0.0, 0.0, CARTESIAN_STEP_MM, 0.0, 0.0, 0.0)
    back = (0.0, 0.0, -CARTESIAN_STEP_MM, 0.0, 0.0, 0.0)

    acknowledge(robot, io_map, name, out)
    robot.WaitIdle(timeout_s=60.0)
    # Assert the outward leg happened: checking only that the robot ends up
    # where it started would pass just as well on a robot that ignored both
    # commands.
    assert robot.GetRobotPosition().pose[2] == pytest.approx(start[2] + CARTESIAN_STEP_MM, abs=0.3)

    acknowledge(robot, io_map, name, back)
    robot.WaitIdle(timeout_s=60.0)
    assert robot.GetRobotPosition().pose == pytest.approx(start, abs=0.3)


@pytest.mark.cartesian
@pytest.mark.parametrize("name", CARTESIAN_VELOCITY)
def test_cartesian_velocity_command(robot: FieldbusRobot, io_map: IoMap, name: str) -> None:
    """A Cartesian velocity is held briefly, then released.

    Released in a ``finally`` so a failure can never leave the robot moving.
    """
    ensure_ready(robot, True)
    start = robot.GetRobotPosition().pose
    velocity = (0.0, 0.0, 2.0, 0.0, 0.0, 0.0)

    try:
        robot.StartVelocityCommand(io_map.motion_command_id(name), *velocity)
        time.sleep(0.3)
    finally:
        robot.StopVelocityCommand()

    wait_until(lambda: robot.GetMotionStatus().end_of_movement, "the robot to come to a standstill")
    assert robot.GetStatusRobot().error_code == 0

    moved = robot.GetRobotPosition().pose[2] - start[2]
    assert abs(moved) < 10.0, "the tool travelled much further than the velocity implies"
    robot.MoveJoints(*robot.GetRobotPosition().joints)
    robot.WaitIdle(timeout_s=30.0)


# ----------------------------------------------------------------------
# Tool
# ----------------------------------------------------------------------
@pytest.mark.tool
@pytest.mark.parametrize("name", sorted(TOOL_COMMANDS), ids=sorted(TOOL_COMMANDS))
def test_tool_command_is_accepted(
    robot: FieldbusRobot, io_map: IoMap, allow_motion: bool, name: str
) -> None:
    """A gripper or valve command is accepted by a robot that carries a tool.

    Only meaningful with a tool attached and nothing held in it; without one
    the robot reports an external tool error, which this reports as a failure
    rather than hiding.
    """
    ensure_ready(robot, allow_motion)
    acknowledge(robot, io_map, name, TOOL_COMMANDS[name])
