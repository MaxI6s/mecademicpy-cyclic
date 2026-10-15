"""Shared pytest fixtures.

The repository root is added to ``sys.path`` so that both ``mecademic_fieldbus``
and the development-only ``mock_robot`` package are importable from a plain
source checkout, without an editable install.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecademic_fieldbus.io_map import IoMap, get_io_map  # noqa: E402
from mecademic_fieldbus.robot import FieldbusRobot  # noqa: E402
from mock_robot.loopback import LoopbackTransport  # noqa: E402
from mock_robot.simulator import RobotSimulator  # noqa: E402

from .fake_transport import FakeTransport  # noqa: E402

#: Directory holding the recorded assembly fixtures.
FIXTURES_DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


@pytest.fixture
def io_map() -> IoMap:
    """Return the shipped I/O map, exactly as an application would get it."""
    return get_io_map()


@pytest.fixture
def fake_transport(io_map: IoMap) -> FakeTransport:
    """Return a connected in-memory transport sized for the current I/O map."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    transport.connect("in-memory")
    return transport


@pytest.fixture
def simulator(io_map: IoMap) -> RobotSimulator:
    """Return a simulator whose activation and homing sequences are instant."""
    return RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)


@pytest.fixture
def robot(io_map: IoMap, simulator: RobotSimulator) -> FieldbusRobot:
    """Return a facade connected to a simulator through an in-memory transport."""
    robot = FieldbusRobot(LoopbackTransport(simulator), io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    return robot


# ----------------------------------------------------------------------
# Hardware test options
# ----------------------------------------------------------------------
def pytest_addoption(parser: pytest.Parser) -> None:
    """Register the options that enable the hardware test suite.

    Args:
        parser: Parser pytest hands to plugins.
    """
    group = parser.getgroup("robot hardware")
    group.addoption(
        "--robot-address",
        action="store",
        default=None,
        help="IPv4 address of a real robot; enables the tests marked 'hardware'",
    )
    group.addoption(
        "--allow-motion",
        action="store_true",
        default=False,
        help="allow the hardware tests to home and move the robot; WITHOUT IT "
        "they only read, activate and deactivate",
    )
    group.addoption(
        "--allow-cartesian",
        action="store_true",
        default=False,
        help="allow the Cartesian move commands, which can approach a "
        "singularity or a limit in ways a joint move cannot",
    )
    group.addoption(
        "--allow-config",
        action="store_true",
        default=False,
        help="allow the commands that change robot settings (velocities, "
        "blending, auto-configuration); the suite restores documented "
        "defaults afterwards",
    )
    group.addoption(
        "--allow-tool",
        action="store_true",
        default=False,
        help="allow the gripper and valve commands; only with a tool attached "
        "and nothing held in it",
    )
    group.addoption(
        "--motion-joint",
        action="store",
        type=int,
        default=6,
        help="1-based joint the motion tests move (default: %(default)s, the "
        "last one, whose rotation displaces the tool least)",
    )
    group.addoption(
        "--motion-amplitude",
        action="store",
        type=float,
        default=5.0,
        help="how far the motion tests move that joint, in degrees " "(default: %(default)s)",
    )
    group.addoption(
        "--robot-udp-port",
        action="store",
        type=int,
        default=None,
        help="UDP port the scanner listens on; defaults to the standard 2222",
    )


#: Extra opt-in markers, mapped to the option that enables each of them.
OPTIONAL_MARKERS = {
    "motion": ("--allow-motion", "moves the robot"),
    "cartesian": ("--allow-cartesian", "moves the robot in Cartesian space"),
    "config": ("--allow-config", "changes robot settings"),
    "tool": ("--allow-tool", "drives the gripper or the valves"),
}


def pytest_configure(config: pytest.Config) -> None:
    """Declare the markers used by the hardware suite.

    Args:
        config: Configuration pytest hands to plugins.
    """
    config.addinivalue_line("markers", "hardware: needs a real robot, enabled by --robot-address")
    for marker, (option, what) in OPTIONAL_MARKERS.items():
        config.addinivalue_line("markers", "{}: {}, enabled by {}".format(marker, what, option))


def pytest_collection_modifyitems(config: pytest.Config, items: list) -> None:
    """Skip the hardware tests unless the matching options were given.

    Args:
        config: Active configuration.
        items: Collected test items.
    """
    has_robot = config.getoption("--robot-address") is not None
    no_robot = pytest.mark.skip(reason="needs --robot-address=<ip> and a real robot")
    for item in items:
        if "hardware" not in item.keywords:
            continue
        if not has_robot:
            item.add_marker(no_robot)
            continue
        for marker, (option, what) in OPTIONAL_MARKERS.items():
            if marker in item.keywords and not config.getoption(option):
                item.add_marker(
                    pytest.mark.skip(reason="needs {}; this test {}".format(option, what))
                )
                break


@pytest.fixture(scope="session")
def robot_address(pytestconfig: pytest.Config) -> str:
    """Return the address of the robot under test."""
    address = pytestconfig.getoption("--robot-address")
    if address is None:
        pytest.skip("no --robot-address given")
    return str(address)


@pytest.fixture(scope="session")
def allow_motion(pytestconfig: pytest.Config) -> bool:
    """Return whether the hardware tests are allowed to move the robot."""
    return bool(pytestconfig.getoption("--allow-motion"))


@pytest.fixture(scope="session")
def allow_cartesian(pytestconfig: pytest.Config) -> bool:
    """Return whether the Cartesian move commands are allowed."""
    return bool(pytestconfig.getoption("--allow-cartesian"))


@pytest.fixture(scope="session")
def motion_joint(pytestconfig: pytest.Config) -> int:
    """Return the zero-based index of the joint the motion tests move."""
    joint = int(pytestconfig.getoption("--motion-joint"))
    if not 1 <= joint <= 6:
        pytest.fail("--motion-joint must be between 1 and 6")
    return joint - 1


@pytest.fixture(scope="session")
def motion_amplitude(pytestconfig: pytest.Config) -> float:
    """Return how far, in degrees, the motion tests move that joint."""
    return float(pytestconfig.getoption("--motion-amplitude"))
