"""Shared pytest fixtures.

The repository root is added to ``sys.path`` so that both ``mecademic_fieldbus``
and the development-only ``mock_robot`` package are importable from a plain
source checkout, without an editable install.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecademic_fieldbus.io_map import IoMap, IoMapV1, get_io_map  # noqa: E402
from mecademic_fieldbus.robot import FieldbusRobot  # noqa: E402
from mock_robot.simulator import DEMO_MOTION_COMMAND_IDS, RobotSimulator  # noqa: E402

from .fake_transport import FakeTransport, SimulatorTransport  # noqa: E402

#: Directory holding the recorded assembly fixtures.
FIXTURES_DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


@pytest.fixture
def io_map() -> IoMap:
    """Return the shipped I/O map, exactly as an application would get it."""
    return get_io_map()


@pytest.fixture
def demo_io_map() -> IoMap:
    """Return an I/O map carrying the simulator motion command identifiers.

    The vendor files do not publish the real identifiers, so anything that
    needs to actually move the simulated robot uses the synthetic ones the mock
    understands.
    """
    return IoMapV1(motion_command_ids=DEMO_MOTION_COMMAND_IDS)


@pytest.fixture
def fake_transport(io_map: IoMap) -> FakeTransport:
    """Return a connected in-memory transport sized for the current I/O map."""
    transport = FakeTransport(io_map.input_assembly_size, io_map.output_assembly_size)
    transport.connect("in-memory")
    return transport


@pytest.fixture
def simulator(demo_io_map: IoMap) -> RobotSimulator:
    """Return a simulator whose activation and homing sequences are instant."""
    return RobotSimulator(demo_io_map, activation_time_s=0.0, homing_time_s=0.0)


@pytest.fixture
def robot(demo_io_map: IoMap, simulator: RobotSimulator) -> FieldbusRobot:
    """Return a facade connected to a simulator through an in-memory transport."""
    robot = FieldbusRobot(SimulatorTransport(simulator), demo_io_map, default_timeout_s=2.0)
    robot.Connect("in-memory")
    return robot
