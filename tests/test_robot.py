"""Tests of the :class:`~mecademic_fieldbus.robot.FieldbusRobot` facade.

Two setups are used:

* :class:`~tests.fake_transport.LoopbackTransport`, for the nominal command
  sequences against realistic robot behaviour;
* :class:`~tests.fake_transport.FakeTransport`, when a test needs to inject an
  exact input assembly or to inspect the produced output assembly.
"""

import time

import pytest

from mecademic_fieldbus.exceptions import (
    FieldbusConnectionError,
    FieldbusError,
    FieldbusStateError,
    FieldbusTimeoutError,
    FieldbusUnsupportedFeature,
    RobotErrorStatus,
)
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot import FieldbusRobot
from mecademic_fieldbus.robot_classes import DYNAMIC_DATA_SLOTS, MotionStatus, RobotStatus
from mock_robot.loopback import LoopbackTransport
from mock_robot.simulator import DEFAULT_FIFO_SPACE, RobotSimulator

from .fake_transport import FakeTransport

#: A joint target the simulator accepts.
TARGET = (0.0, 0.0, 0.0, 0.0, 45.0, 0.0)


def live_input(io_map: IoMap, status: RobotStatus, **motion: object) -> bytes:
    """Build an input assembly image that looks like a live robot.

    An all-zero image means "no cyclic data yet" to the facade, so a fixture
    that stands for a running robot must carry something -- here the queue
    capacity, which a real robot always reports.

    Args:
        io_map: Map used to encode the image.
        status: Robot status to report.
        **motion: Fields of
            :class:`~mecademic_fieldbus.robot_classes.MotionStatus` to override.

    Returns:
        The raw input assembly image.
    """
    fields = {"fifo_space": DEFAULT_FIFO_SPACE, "end_of_block": True, "end_of_movement": True}
    fields.update(motion)
    raw = io_map.encode_status(status)
    return io_map.encode_motion_status(MotionStatus(**fields), raw)


def connected(io_map: IoMap, raw_input: bytes, timeout_s: float = 1.0) -> FieldbusRobot:
    """Build a facade connected to a transport serving a fixed input image.

    Args:
        io_map: Map used by the facade.
        raw_input: Image the robot is pretending to produce.
        timeout_s: Default timeout of the facade.

    Returns:
        A connected facade.
    """
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    transport.set_input_assembly(raw_input)
    robot = FieldbusRobot(transport, io_map, default_timeout_s=timeout_s)
    robot.Connect("in-memory")
    return robot


# ----------------------------------------------------------------------
# Connection
# ----------------------------------------------------------------------
def test_connect_resets_the_output_image(io_map: IoMap) -> None:
    """Connecting clears every command left over from a previous session."""
    robot = connected(io_map, live_input(io_map, RobotStatus(activated=True)))
    transport = robot.transport
    assert isinstance(transport, FakeTransport)
    robot.ActivateRobot()
    assert io_map.decode_robot_control(transport.last_output_assembly).activate is True
    robot.Disconnect()

    robot.Connect("in-memory")
    assert transport.last_output_assembly == io_map.empty_output_assembly()


def test_connect_times_out_when_no_cyclic_data_arrives(io_map: IoMap) -> None:
    """An all-zero input image means the robot is not producing anything."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    robot = FieldbusRobot(transport, io_map, default_timeout_s=0.1)
    with pytest.raises(FieldbusTimeoutError) as error:
        robot.Connect("in-memory")
    assert "cyclic data" in str(error.value)


def test_connect_can_skip_waiting_for_cyclic_data(io_map: IoMap) -> None:
    """The wait is optional, for callers that want to inspect a dead link."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    robot = FieldbusRobot(transport, io_map, default_timeout_s=0.1)
    robot.Connect("in-memory", wait_for_cyclic_data=False)
    assert robot.is_connected is True


def test_connect_resumes_the_move_id_sequence(io_map: IoMap) -> None:
    """The facade continues the numbering the robot already acknowledged."""
    robot = connected(
        io_map,
        live_input(io_map, RobotStatus(activated=True, homed=True), move_id=41),
    )
    assert robot.SendMotionCommand(1, 0.0) == 42


def test_commands_require_a_connection(io_map: IoMap) -> None:
    """Using the facade before connecting fails loudly."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    robot = FieldbusRobot(transport, io_map)
    with pytest.raises(FieldbusConnectionError):
        robot.GetStatusRobot()


def test_disconnect_is_idempotent(robot: FieldbusRobot) -> None:
    """Disconnecting twice is harmless."""
    robot.Disconnect()
    robot.Disconnect()
    assert robot.is_connected is False


def test_context_manager_disconnects(io_map: IoMap, simulator: RobotSimulator) -> None:
    """Leaving a ``with`` block releases the connection."""
    robot = FieldbusRobot(LoopbackTransport(simulator), io_map, default_timeout_s=2.0)
    with robot:
        robot.Connect("in-memory")
        assert robot.is_connected is True
    assert robot.is_connected is False


def test_monitoring_connection_refuses_to_command(io_map: IoMap) -> None:
    """A robot reporting monitoring mode cannot be controlled."""
    robot = connected(io_map, live_input(io_map, RobotStatus(monitoring_mode=True)))
    with pytest.raises(FieldbusStateError) as error:
        robot.ActivateRobot()
    assert "monitoring" in str(error.value)


# ----------------------------------------------------------------------
# State commands
# ----------------------------------------------------------------------
def test_activate_then_home_then_deactivate(
    robot: FieldbusRobot, simulator: RobotSimulator
) -> None:
    """The nominal start-up sequence drives the simulated robot as expected."""
    robot.ActivateRobot()
    assert robot.GetStatusRobot().activated is True

    robot.Home()
    status = robot.GetStatusRobot()
    assert status.homed is True
    assert status.is_ready is True

    robot.DeactivateRobot()
    assert robot.GetStatusRobot().activated is False


def test_home_releases_the_request_bit(io_map: IoMap, simulator: RobotSimulator) -> None:
    """The homing bit is dropped once the robot reports the homed state."""
    robot = FieldbusRobot(LoopbackTransport(simulator), io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    robot.ActivateRobot()
    robot.Home()

    latched = io_map.decode_robot_control(robot.latched_output_assembly)
    assert latched.home is False
    assert latched.activate is True
    assert robot.GetStatusRobot().homed is True


def test_deactivate_releases_the_request_bit(io_map: IoMap, simulator: RobotSimulator) -> None:
    """Holding the deactivate bit would block any later activation."""
    robot = FieldbusRobot(LoopbackTransport(simulator), io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    robot.ActivateRobot()
    robot.DeactivateRobot()
    assert io_map.decode_robot_control(robot.latched_output_assembly).deactivate is False

    robot.ActivateRobot()
    assert robot.GetStatusRobot().activated is True


def test_home_requires_activation(robot: FieldbusRobot) -> None:
    """Homing a powered-down robot is refused before anything is produced."""
    with pytest.raises(FieldbusStateError):
        robot.Home()


def test_activation_timeout(io_map: IoMap) -> None:
    """A robot that never activates raises a timeout."""
    robot = connected(io_map, live_input(io_map, RobotStatus()), timeout_s=0.1)
    with pytest.raises(FieldbusTimeoutError):
        robot.ActivateRobot()


def test_robot_error_aborts_a_wait(io_map: IoMap) -> None:
    """An error reported while waiting is raised with its code."""
    robot = connected(io_map, live_input(io_map, RobotStatus(error_code=1234)), timeout_s=1.0)
    with pytest.raises(RobotErrorStatus) as error:
        robot.ActivateRobot()
    assert error.value.error_code == 1234
    assert "1234" in str(error.value)


def test_reset_error_clears_the_simulated_error(
    robot: FieldbusRobot, simulator: RobotSimulator
) -> None:
    """Resetting brings the simulated robot out of the error state."""
    simulator.inject_error()
    assert robot.GetStatusRobot().error_status is True

    robot.ResetError()
    assert robot.GetStatusRobot().error_status is False


def test_status_accessors_read_the_same_frame(robot: FieldbusRobot) -> None:
    """Every accessor decodes the current input assembly."""
    robot.ActivateRobot()
    robot.Home()
    assert robot.GetMotionStatus().fifo_space == DEFAULT_FIFO_SPACE
    assert robot.GetSafetyStatus().motor_voltage_on is True
    assert len(robot.GetRobotPosition().joints) == 6


# ----------------------------------------------------------------------
# Digital outputs, which this layout does not carry
# ----------------------------------------------------------------------
def test_set_output_state_reports_the_missing_feature(robot: FieldbusRobot) -> None:
    """The Meca500 cyclic assemblies expose no digital outputs."""
    with pytest.raises(FieldbusUnsupportedFeature) as error:
        robot.SetOutputState(True)
    assert "digital" in str(error.value)


def test_get_output_state_reports_the_missing_feature(robot: FieldbusRobot) -> None:
    """Reading the digital outputs fails the same way, and says why."""
    with pytest.raises(FieldbusUnsupportedFeature):
        robot.GetRtOutputState()


# ----------------------------------------------------------------------
# Motion
# ----------------------------------------------------------------------
def test_move_requires_a_ready_robot(robot: FieldbusRobot) -> None:
    """Moving before homing is refused before anything is produced."""
    with pytest.raises(FieldbusStateError):
        robot.MoveJoints(*TARGET)


def test_move_joints_reaches_the_target(robot: FieldbusRobot, simulator: RobotSimulator) -> None:
    """A joint move completes and leaves the robot at its target."""
    robot.ActivateRobot()
    robot.Home()
    robot.MoveJoints(*TARGET)
    robot.WaitIdle(timeout_s=5.0)
    assert simulator.joint_positions[4] == pytest.approx(45.0)
    assert robot.GetMotionStatus().end_of_block is True


def test_move_joints_checks_the_argument_count(robot: FieldbusRobot) -> None:
    """A joint target of the wrong size is refused before anything is sent."""
    robot.ActivateRobot()
    robot.Home()
    with pytest.raises(FieldbusStateError):
        robot.MoveJoints(0.0, 0.0)


def test_velocity_commands_cannot_be_queued(robot: FieldbusRobot, io_map: IoMap) -> None:
    """Queueing a velocity-mode command is the documented way to flood the queue."""
    robot.ActivateRobot()
    robot.Home()
    with pytest.raises(FieldbusStateError) as error:
        robot.SendMotionCommand(io_map.motion_command_id("MoveJointsVel"), 0.0)
    assert "StartVelocityCommand" in str(error.value)


def test_queued_commands_cannot_be_held(robot: FieldbusRobot, io_map: IoMap) -> None:
    """And the converse: holding a positional command is refused too."""
    robot.ActivateRobot()
    robot.Home()
    with pytest.raises(FieldbusStateError) as error:
        robot.StartVelocityCommand(io_map.motion_command_id("MoveJoints"), *TARGET)
    assert "SendMotionCommand" in str(error.value)


def test_velocity_command_is_held_with_a_zero_move_id(
    robot: FieldbusRobot, io_map: IoMap, simulator: RobotSimulator
) -> None:
    """A velocity-mode command runs continuously while it is held."""
    robot.ActivateRobot()
    robot.Home()
    robot.StartVelocityCommand(io_map.motion_command_id("MoveJointsVel"), 0.0, 0.0, 0.0, 0.0, 30.0)

    control = io_map.decode_motion_control(robot.latched_output_assembly)
    assert control.move_id == 0
    assert control.setpoint is True

    # The facade drives the simulator on the real clock, so let a little time
    # pass rather than injecting one: at 30 deg/s the joint must have moved.
    time.sleep(0.15)
    robot.GetRobotPosition()
    moved = simulator.joint_positions[4]
    assert moved > 1.0

    robot.StopVelocityCommand()
    assert io_map.decode_motion_control(robot.latched_output_assembly).setpoint is False
    time.sleep(0.05)
    robot.GetRobotPosition()
    assert simulator.joint_positions[4] == pytest.approx(moved, abs=0.5)


def test_move_ids_are_unique_and_echoed(robot: FieldbusRobot) -> None:
    """Each move gets its own identifier, echoed back by the robot."""
    robot.ActivateRobot()
    robot.Home()
    first = robot.MoveJoints(0.0, 0.0, 0.0, 0.0, 10.0, 0.0)
    robot.WaitIdle(timeout_s=5.0)
    second = robot.MoveJoints(0.0, 0.0, 0.0, 0.0, 20.0, 0.0)
    robot.WaitIdle(timeout_s=5.0)
    assert second != first
    assert robot.GetMotionStatus().move_id == second


def test_move_pose_is_encoded_as_the_pose_command(robot: FieldbusRobot, io_map: IoMap) -> None:
    """``MovePose`` sends the identifier registered under that name."""
    robot.ActivateRobot()
    robot.Home()
    pose = robot.GetRobotPosition().pose
    move_id = robot.MovePose(*pose)

    command = io_map.decode_motion_command(robot.latched_output_assembly)
    control = io_map.decode_motion_control(robot.latched_output_assembly)
    assert command.command_id == io_map.motion_command_id("MovePose")
    assert command.arguments == pytest.approx(pose)
    assert control.move_id == move_id
    assert control.setpoint is True


def test_wait_idle_times_out_on_a_stalled_robot(io_map: IoMap) -> None:
    """A robot that keeps reporting motion eventually raises a timeout."""
    robot = connected(
        io_map,
        live_input(
            io_map,
            RobotStatus(activated=True, homed=True),
            end_of_block=False,
            end_of_movement=False,
        ),
        timeout_s=0.1,
    )
    robot.SendMotionCommand(1, 0.0)
    with pytest.raises(FieldbusTimeoutError):
        robot.WaitIdle()


def test_pause_and_resume(robot: FieldbusRobot, io_map: IoMap) -> None:
    """Pausing sets the level bit; resuming pulses the rising-edge one."""
    robot.ActivateRobot()
    robot.Home()
    robot.PauseMotion()
    assert io_map.decode_motion_control(robot.latched_output_assembly).pause is True
    assert robot.GetMotionStatus().paused is True

    robot.ResumeMotion()
    assert io_map.decode_motion_control(robot.latched_output_assembly).pause is False
    assert robot.GetMotionStatus().paused is False


def test_clear_motion_empties_the_queue(robot: FieldbusRobot) -> None:
    """Clearing the queue is reported back by the robot."""
    robot.ActivateRobot()
    robot.Home()
    robot.ClearMotion()
    assert robot.GetMotionStatus().cleared is True


# ----------------------------------------------------------------------
# Output assembly production
# ----------------------------------------------------------------------
def test_every_command_produces_a_full_size_assembly(io_map: IoMap) -> None:
    """Whatever the command, the produced image matches the negotiated size."""
    robot = connected(io_map, live_input(io_map, RobotStatus(activated=True, homed=True)))
    transport = robot.transport
    assert isinstance(transport, FakeTransport)
    robot.SendMotionCommand(io_map.motion_command_id("MoveJoints"), *TARGET)
    robot.StopVelocityCommand()
    assert transport.written_assemblies
    for image in transport.written_assemblies:
        assert len(image) == io_map.output_assembly_size


def test_commands_accumulate_in_the_latched_image(io_map: IoMap) -> None:
    """Requesting a move does not cancel a pending activation request."""
    robot = connected(io_map, live_input(io_map, RobotStatus(activated=True, homed=True)))
    robot.ActivateRobot()
    move_id = robot.SendMotionCommand(7, 1.0)
    latched = robot.latched_output_assembly
    assert io_map.decode_robot_control(latched).activate is True
    assert io_map.decode_motion_command(latched).command_id == 7
    assert io_map.decode_motion_control(latched).move_id == move_id


def test_repr_mentions_the_io_map_version(robot: FieldbusRobot) -> None:
    """The representation is useful in a debugger."""
    assert "FieldbusRobot" in repr(robot)
    assert robot.io_map.version in repr(robot)


def test_reconnecting_does_not_reuse_an_acknowledged_move_id(
    io_map: IoMap, simulator: RobotSimulator
) -> None:
    """A scanner restarting against a live robot still gets its moves executed.

    The robot ignores a move id it has already handled, so the facade must
    resume the sequence rather than restart it at zero.
    """
    transport = LoopbackTransport(simulator)
    first = FieldbusRobot(transport, io_map, default_timeout_s=2.0)
    first.Connect("in-memory")
    first.ActivateRobot()
    first.Home()
    first_id = first.MoveJoints(0.0, 0.0, 0.0, 0.0, 10.0, 0.0)
    first.WaitIdle(timeout_s=5.0)
    first.Disconnect()

    second = FieldbusRobot(transport, io_map, default_timeout_s=2.0)
    second.Connect("in-memory")
    second.ActivateRobot()
    second.Home()
    second_id = second.MoveJoints(0.0, 0.0, 0.0, 0.0, 20.0, 0.0)
    assert second_id != first_id

    second.WaitIdle(timeout_s=5.0)
    assert simulator.joint_positions[4] == pytest.approx(20.0)


# ----------------------------------------------------------------------
# Dynamic data and robot variables
# ----------------------------------------------------------------------
def test_a_dynamic_slot_reports_the_type_it_was_asked_for(robot: FieldbusRobot) -> None:
    """The robot echoes the requested type back alongside the values.

    That echo is the only way a reader can tell whether the six values in a
    slot already belong to the new request or still to the previous one, so a
    slot that does not carry it is unusable.
    """
    robot.ActivateRobot()
    robot.Home()
    robot.SetDynamicDataTypes("ActualJointPosition", "FirmwareVersion")

    assert robot.GetDynamicData(0).type_id == robot.io_map.dynamic_data_type_id(
        "ActualJointPosition"
    )
    assert robot.GetDynamicData(1).type_id == robot.io_map.dynamic_data_type_id("FirmwareVersion")


def test_a_dynamic_slot_takes_a_name_or_an_identifier(robot: FieldbusRobot) -> None:
    """Both spellings reach the same slot, because variables have no name."""
    robot.ActivateRobot()
    by_name = robot.io_map.dynamic_data_type_id("ActualJointPosition")

    robot.SetDynamicDataTypes("ActualJointPosition")
    assert robot.GetDynamicData(0).type_id == by_name

    robot.SetDynamicDataTypes(0)
    robot.SetDynamicDataTypes(by_name)
    assert robot.GetDynamicData(0).type_id == by_name


def test_an_unknown_dynamic_type_name_is_refused(robot: FieldbusRobot) -> None:
    """A typo must be reported, not silently requested as some other type."""
    robot.ActivateRobot()
    with pytest.raises(FieldbusError):
        robot.SetDynamicDataTypes("NoSuchTypeAtAll")


def test_a_dynamic_slot_reports_the_measured_joint_positions(
    robot: FieldbusRobot, simulator: RobotSimulator
) -> None:
    """The slots carry what the fixed fields do not: the measured position.

    The input assembly publishes the *target* position, so this is the only
    route to what the robot actually did.
    """
    robot.ActivateRobot()
    robot.Home()
    robot.SetDynamicDataTypes("ActualJointPosition")
    robot.MoveJoints(0.0, 0.0, 0.0, 0.0, 0.0, 20.0)
    robot.WaitIdle(timeout_s=5.0)

    values = robot.GetDynamicData(0).values
    assert values[5] == pytest.approx(simulator.joint_positions[5])
    assert values[5] == pytest.approx(20.0)


def test_a_robot_variable_survives_the_round_trip(robot: FieldbusRobot) -> None:
    """Identifiers 10000-19999 write a variable, and a slot reads it back.

    One handshake, used three ways -- run a command, start a program, write a
    variable -- with only the identifier range deciding which.

    The robot is homed because the facade routes a variable write through
    :meth:`SendMotionCommand`, which gates on the robot being ready to move.
    At the protocol level the write lands without it; see the README's open
    decisions.
    """
    robot.ActivateRobot()
    robot.Home()
    robot.SendMotionCommand(10001, 1.5, 2.5, 3.5)
    robot.SetDynamicDataTypes(10001)

    data = robot.GetDynamicData(0)
    assert data.type_id == 10001
    assert data.values[:3] == pytest.approx((1.5, 2.5, 3.5))


def test_asking_for_more_types_than_there_are_slots_is_refused(robot: FieldbusRobot) -> None:
    """Silently dropping the extras would be the worst of both answers."""
    robot.ActivateRobot()
    with pytest.raises(FieldbusError):
        robot.SetDynamicDataTypes(*range(DYNAMIC_DATA_SLOTS + 1))
