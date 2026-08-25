"""Tests of the robot simulator state machine.

Time is injected explicitly so that the transitions are deterministic and the
suite stays fast.  The control semantics under test are the ones the vendor
files document: level triggered robot control bits, ``Deactivate`` winning over
``Activate``, a rising-edge ``ResumeMotion``, and a motion command taken into
account only when the move id changes while the setpoint bit is set.
"""

import pytest

from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.robot_classes import MotionCommand, MotionControl, RobotControl
from mock_robot.simulator import (
    DEMO_MOTION_COMMAND_IDS,
    RobotSimulator,
    SimulatorErrorCode,
    SimulatorState,
)


@pytest.fixture
def timed_simulator(demo_io_map: IoMap) -> RobotSimulator:
    """Return a simulator with non-zero, deterministic sequence durations."""
    return RobotSimulator(
        demo_io_map, activation_time_s=1.0, homing_time_s=2.0, joint_speed_deg_s=90.0
    )


def drive(simulator: RobotSimulator, io_map: IoMap, now: float, **control: bool) -> None:
    """Apply an output assembly built from robot control bits.

    Args:
        simulator: Simulator to drive.
        io_map: I/O map used to build the assembly.
        now: Simulated time.
        **control: Robot control bits to request.
    """
    simulator.apply_output_assembly(io_map.encode_robot_control(RobotControl(**control)), now=now)


def reach_idle(simulator: RobotSimulator, io_map: IoMap) -> bytes:
    """Bring a zero-delay simulator to the idle state.

    Args:
        simulator: Simulator to drive.
        io_map: I/O map used to build the assemblies.

    Returns:
        The output assembly image holding the activation and homing bits.
    """
    drive(simulator, io_map, 0.0, activate=True)
    raw_output = io_map.encode_robot_control(RobotControl(activate=True, home=True))
    simulator.apply_output_assembly(raw_output, now=0.0)
    # Zero-length sequences still need one tick to be observed as completed.
    simulator.update(now=0.0)
    assert simulator.state is SimulatorState.IDLE
    return raw_output


def request_move(
    simulator: RobotSimulator,
    io_map: IoMap,
    raw_output: bytes,
    move_id: int,
    command: str,
    arguments: tuple,
    now: float,
) -> bytes:
    """Latch a motion command the way the facade does and apply it.

    Args:
        simulator: Simulator to drive.
        io_map: I/O map used to build the assembly.
        raw_output: Current output image to build upon.
        move_id: Move identifier to announce.
        command: Name of the command in the demo identifier table.
        arguments: Command arguments.
        now: Simulated time.

    Returns:
        The output image that was applied.
    """
    raw_output = io_map.encode_motion_command(
        MotionCommand.build(DEMO_MOTION_COMMAND_IDS[command], arguments), raw_output
    )
    raw_output = io_map.encode_motion_control(
        MotionControl(move_id=move_id, setpoint=True), raw_output
    )
    simulator.apply_output_assembly(raw_output, now=now)
    return raw_output


# ----------------------------------------------------------------------
# Activation and homing
# ----------------------------------------------------------------------
def test_starts_deactivated(timed_simulator: RobotSimulator) -> None:
    """A fresh simulator is powered down, not homed, with brakes engaged."""
    status = timed_simulator.get_status()
    assert timed_simulator.state is SimulatorState.DEACTIVATED
    assert status.activated is False
    assert status.homed is False
    assert status.brakes_engaged is True


def test_activation_takes_the_configured_time(
    timed_simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Activation is reported only once the simulated delay has elapsed."""
    drive(timed_simulator, demo_io_map, 0.0, activate=True)
    assert timed_simulator.state is SimulatorState.ACTIVATING
    assert timed_simulator.get_status().activated is False
    assert timed_simulator.get_status().busy is True

    timed_simulator.update(now=0.9)
    assert timed_simulator.state is SimulatorState.ACTIVATING

    timed_simulator.update(now=1.0)
    assert timed_simulator.state is SimulatorState.ACTIVATED
    assert timed_simulator.get_status().activated is True
    assert timed_simulator.get_status().busy is False


def test_deactivate_wins_over_activate(timed_simulator: RobotSimulator, demo_io_map: IoMap) -> None:
    """With both bits set the robot deactivates, as the vendor documents."""
    drive(timed_simulator, demo_io_map, 0.0, activate=True)
    timed_simulator.update(now=1.0)
    assert timed_simulator.get_status().activated is True

    drive(timed_simulator, demo_io_map, 1.0, activate=True, deactivate=True)
    assert timed_simulator.state is SimulatorState.DEACTIVATING
    timed_simulator.update(now=2.0)
    assert timed_simulator.get_status().activated is False


def test_homing_sequence(timed_simulator: RobotSimulator, demo_io_map: IoMap) -> None:
    """Homing runs from the activated state and ends in idle."""
    drive(timed_simulator, demo_io_map, 0.0, activate=True)
    timed_simulator.update(now=1.0)
    drive(timed_simulator, demo_io_map, 1.0, activate=True, home=True)
    assert timed_simulator.state is SimulatorState.HOMING
    assert timed_simulator.get_status().homed is False
    assert timed_simulator.get_status().busy is True

    timed_simulator.update(now=3.0)
    assert timed_simulator.state is SimulatorState.IDLE
    assert timed_simulator.get_status().homed is True


def test_homing_is_ignored_while_deactivated(
    timed_simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Requesting homing without power does not home the robot."""
    drive(timed_simulator, demo_io_map, 0.0, home=True)
    assert timed_simulator.state is SimulatorState.DEACTIVATED
    assert timed_simulator.get_status().homed is False


def test_deactivation_loses_the_homing_reference(
    timed_simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Powering down clears the homed flag."""
    drive(timed_simulator, demo_io_map, 0.0, activate=True)
    timed_simulator.update(now=1.0)
    drive(timed_simulator, demo_io_map, 1.0, activate=True, home=True)
    timed_simulator.update(now=3.0)
    assert timed_simulator.get_status().homed is True

    drive(timed_simulator, demo_io_map, 3.0, deactivate=True)
    timed_simulator.update(now=4.0)
    assert timed_simulator.state is SimulatorState.DEACTIVATED
    assert timed_simulator.get_status().homed is False


def test_simulation_mode_only_toggles_while_deactivated(
    timed_simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Simulation mode is latched while the robot is off, as documented."""
    drive(timed_simulator, demo_io_map, 0.0, enable_simulation=True)
    assert timed_simulator.get_status().simulation_mode is True

    drive(timed_simulator, demo_io_map, 0.0, activate=True, enable_simulation=True)
    timed_simulator.update(now=1.0)
    drive(timed_simulator, demo_io_map, 1.0, activate=True, enable_simulation=False)
    assert timed_simulator.get_status().simulation_mode is True


# ----------------------------------------------------------------------
# Motion
# ----------------------------------------------------------------------
def test_joint_move_interpolates_then_completes(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """A joint move moves through intermediate positions and echoes its id."""
    raw_output = reach_idle(simulator, demo_io_map)
    request_move(simulator, demo_io_map, raw_output, 4, "MoveJoints", (0.0,) * 4 + (90.0, 0.0), 0.0)

    assert simulator.state is SimulatorState.MOVING
    assert simulator.get_motion_status().end_of_block is False
    simulator.update(now=0.5)
    assert 0.0 < simulator.joint_positions[4] < 90.0

    simulator.update(now=1.0)
    assert simulator.state is SimulatorState.IDLE
    assert simulator.joint_positions[4] == pytest.approx(90.0)
    assert simulator.get_motion_status().move_id == 4
    assert simulator.get_motion_status().end_of_block is True


def test_a_move_is_taken_into_account_only_when_the_id_changes(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """The scanner keeps producing the same image; the move must run once."""
    raw_output = reach_idle(simulator, demo_io_map)
    raw_output = request_move(
        simulator, demo_io_map, raw_output, 4, "MoveJoints", (0.0,) * 4 + (90.0, 0.0), 0.0
    )
    simulator.update(now=1.0)
    assert simulator.state is SimulatorState.IDLE

    simulator.apply_output_assembly(raw_output, now=1.0)
    assert simulator.state is SimulatorState.IDLE


def test_a_move_without_the_setpoint_bit_is_ignored(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Clearing the setpoint bit makes the robot ignore the motion fields."""
    raw_output = reach_idle(simulator, demo_io_map)
    raw_output = demo_io_map.encode_motion_command(
        MotionCommand.build(DEMO_MOTION_COMMAND_IDS["MoveJoints"], (0.0,) * 4 + (90.0, 0.0)),
        raw_output,
    )
    raw_output = demo_io_map.encode_motion_control(
        MotionControl(move_id=9, setpoint=False), raw_output
    )
    simulator.apply_output_assembly(raw_output, now=0.0)
    assert simulator.state is SimulatorState.IDLE


def test_pose_move_uses_the_placeholder_kinematics(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """A Cartesian target is reachable and reported back consistently."""
    raw_output = reach_idle(simulator, demo_io_map)
    pose = simulator.get_position().pose
    target = (pose[0] + 10.0,) + tuple(pose[1:])
    request_move(simulator, demo_io_map, raw_output, 1, "MovePose", target, 0.0)
    simulator.update(now=10.0)
    assert simulator.get_position().pose == pytest.approx(target)


def test_unknown_motion_command_raises_a_robot_error(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """A command identifier the robot does not know puts it in error."""
    raw_output = reach_idle(simulator, demo_io_map)
    raw_output = demo_io_map.encode_motion_command(
        MotionCommand.build(4242, (0.0,) * 6), raw_output
    )
    raw_output = demo_io_map.encode_motion_control(
        MotionControl(move_id=1, setpoint=True), raw_output
    )
    simulator.apply_output_assembly(raw_output, now=0.0)
    assert simulator.state is SimulatorState.ERROR
    assert simulator.error_code == int(SimulatorErrorCode.UNKNOWN_MOTION_COMMAND)


def test_move_without_homing_raises_a_robot_error(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Moving before homing puts the simulated robot in error."""
    drive(simulator, demo_io_map, 0.0, activate=True)
    simulator.update(now=0.0)
    raw_output = demo_io_map.encode_robot_control(RobotControl(activate=True))
    request_move(simulator, demo_io_map, raw_output, 1, "MoveJoints", (0.0,) * 6, 0.0)
    assert simulator.state is SimulatorState.ERROR
    assert simulator.error_code == int(SimulatorErrorCode.MOVE_WITHOUT_HOMING)


def test_move_outside_the_joint_limits_raises_a_robot_error(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """A target beyond the joint limits puts the simulated robot in error."""
    raw_output = reach_idle(simulator, demo_io_map)
    request_move(simulator, demo_io_map, raw_output, 1, "MoveJoints", (1000.0,) + (0.0,) * 5, 0.0)
    assert simulator.state is SimulatorState.ERROR
    assert simulator.error_code == int(SimulatorErrorCode.MOVE_OUT_OF_RANGE)


# ----------------------------------------------------------------------
# Pause, clear and resume
# ----------------------------------------------------------------------
def test_clear_move_stops_the_robot_and_pauses_it(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Clearing the queue aborts the move and leaves the robot paused."""
    raw_output = reach_idle(simulator, demo_io_map)
    raw_output = request_move(
        simulator, demo_io_map, raw_output, 1, "MoveJoints", (0.0,) * 4 + (90.0, 0.0), 0.0
    )
    simulator.update(now=0.2)

    raw_output = demo_io_map.encode_motion_control(
        MotionControl(move_id=1, setpoint=True, clear_move=True), raw_output
    )
    simulator.apply_output_assembly(raw_output, now=0.2)
    status = simulator.get_motion_status()
    assert simulator.state is SimulatorState.IDLE
    assert status.paused is True
    assert status.cleared is True


def test_resume_motion_acts_on_its_rising_edge(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """Holding the resume bit does not resume a pause requested afterwards."""
    raw_output = reach_idle(simulator, demo_io_map)
    paused = demo_io_map.encode_motion_control(MotionControl(pause=True), raw_output)
    simulator.apply_output_assembly(paused, now=0.0)
    assert simulator.get_motion_status().paused is True

    resumed = demo_io_map.encode_motion_control(
        MotionControl(pause=False, resume_motion=True), raw_output
    )
    simulator.apply_output_assembly(resumed, now=0.0)
    assert simulator.get_motion_status().paused is False

    # The bit stays set; a new pause must not be undone by the held level.
    paused_again = demo_io_map.encode_motion_control(
        MotionControl(pause=True, resume_motion=True), raw_output
    )
    simulator.apply_output_assembly(paused_again, now=0.0)
    assert simulator.get_motion_status().paused is True


# ----------------------------------------------------------------------
# Errors and assembly production
# ----------------------------------------------------------------------
def test_reset_error_returns_to_deactivated(simulator: RobotSimulator, demo_io_map: IoMap) -> None:
    """Resetting an error powers the robot down and clears the code."""
    simulator.inject_error()
    assert simulator.state is SimulatorState.ERROR

    simulator.apply_output_assembly(
        demo_io_map.encode_robot_control(RobotControl(reset_error=True)), now=0.0
    )
    assert simulator.state is SimulatorState.DEACTIVATED
    assert simulator.error_code == 0


def test_produced_assembly_has_the_expected_size(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """The simulator produces a full-size input assembly."""
    assert len(simulator.build_input_assembly(now=0.0)) == demo_io_map.input_assembly_size


def test_produced_assembly_is_never_all_zeros_once_powered(
    simulator: RobotSimulator, demo_io_map: IoMap
) -> None:
    """A live robot always differs from the empty image, which Connect relies on."""
    reach_idle(simulator, demo_io_map)
    assert simulator.build_input_assembly(now=0.0) != demo_io_map.empty_input_assembly()


def test_home_position_must_match_the_joint_count(demo_io_map: IoMap) -> None:
    """A malformed home position is refused at construction time."""
    with pytest.raises(ValueError):
        RobotSimulator(demo_io_map, home_position_deg=(0.0, 0.0))
