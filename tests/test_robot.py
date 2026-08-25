"""Tests of the :class:`~mecademic_fieldbus.robot.FieldbusRobot` facade.

Two setups are used:

* :class:`~tests.fake_transport.SimulatorTransport`, for the nominal command
  sequences against realistic robot behaviour;
* :class:`~tests.fake_transport.FakeTransport`, when a test needs to inject an
  exact input assembly or to inspect the produced output assembly.
"""

import pytest

from mecademic_fieldbus.exceptions import (
    FieldbusConnectionError,
    FieldbusStateError,
    FieldbusTimeoutError,
    FieldbusUnsupportedFeature,
    RobotErrorStatus,
)
from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot import FieldbusRobot
from mecademic_fieldbus.robot_classes import MotionStatus, RobotStatus
from mock_robot.simulator import DEFAULT_FIFO_SPACE, DEMO_MOTION_COMMAND_IDS, RobotSimulator

from .fake_transport import FakeTransport, SimulatorTransport

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


def test_context_manager_disconnects(demo_io_map: IoMap, simulator: RobotSimulator) -> None:
    """Leaving a ``with`` block releases the connection."""
    robot = FieldbusRobot(SimulatorTransport(simulator), demo_io_map, default_timeout_s=2.0)
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


def test_home_releases_the_request_bit(demo_io_map: IoMap, simulator: RobotSimulator) -> None:
    """The homing bit is dropped once the robot reports the homed state."""
    robot = FieldbusRobot(SimulatorTransport(simulator), demo_io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    robot.ActivateRobot()
    robot.Home()

    latched = demo_io_map.decode_robot_control(robot.latched_output_assembly)
    assert latched.home is False
    assert latched.activate is True
    assert robot.GetStatusRobot().homed is True


def test_deactivate_releases_the_request_bit(demo_io_map: IoMap, simulator: RobotSimulator) -> None:
    """Holding the deactivate bit would block any later activation."""
    robot = FieldbusRobot(SimulatorTransport(simulator), demo_io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    robot.ActivateRobot()
    robot.DeactivateRobot()
    assert demo_io_map.decode_robot_control(robot.latched_output_assembly).deactivate is False

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


def test_move_without_a_known_command_id_says_what_to_do(
    io_map: IoMap, simulator: RobotSimulator
) -> None:
    """Without the identifiers, the failure names the section to fill in."""
    robot = FieldbusRobot(SimulatorTransport(simulator), io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    robot.ActivateRobot()
    robot.Home()
    with pytest.raises(FieldbusUnsupportedFeature) as error:
        robot.MoveJoints(*TARGET)
    assert "motion_commands" in str(error.value)


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


def test_move_pose_is_encoded_as_the_pose_command(robot: FieldbusRobot, demo_io_map: IoMap) -> None:
    """``MovePose`` sends the identifier registered under that name."""
    robot.ActivateRobot()
    robot.Home()
    pose = robot.GetRobotPosition().pose
    move_id = robot.MovePose(*pose)

    command = demo_io_map.decode_motion_command(robot.latched_output_assembly)
    control = demo_io_map.decode_motion_control(robot.latched_output_assembly)
    assert command.command_id == DEMO_MOTION_COMMAND_IDS["MovePose"]
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


def test_pause_and_resume(robot: FieldbusRobot, demo_io_map: IoMap) -> None:
    """Pausing sets the level bit; resuming pulses the rising-edge one."""
    robot.ActivateRobot()
    robot.Home()
    robot.PauseMotion()
    assert demo_io_map.decode_motion_control(robot.latched_output_assembly).pause is True
    assert robot.GetMotionStatus().paused is True

    robot.ResumeMotion()
    assert demo_io_map.decode_motion_control(robot.latched_output_assembly).pause is False
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
    robot.SendMotionCommand(1, 1.0, 2.0, 3.0)
    robot.PauseMotion()
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
    demo_io_map: IoMap, simulator: RobotSimulator
) -> None:
    """A scanner restarting against a live robot still gets its moves executed.

    The robot ignores a move id it has already handled, so the facade must
    resume the sequence rather than restart it at zero.
    """
    transport = SimulatorTransport(simulator)
    first = FieldbusRobot(transport, demo_io_map, default_timeout_s=2.0)
    first.Connect("in-memory")
    first.ActivateRobot()
    first.Home()
    first_id = first.MoveJoints(0.0, 0.0, 0.0, 0.0, 10.0, 0.0)
    first.WaitIdle(timeout_s=5.0)
    first.Disconnect()

    second = FieldbusRobot(transport, demo_io_map, default_timeout_s=2.0)
    second.Connect("in-memory")
    second.ActivateRobot()
    second.Home()
    second_id = second.MoveJoints(0.0, 0.0, 0.0, 0.0, 20.0, 0.0)
    assert second_id != first_id

    second.WaitIdle(timeout_s=5.0)
    assert simulator.joint_positions[4] == pytest.approx(20.0)
