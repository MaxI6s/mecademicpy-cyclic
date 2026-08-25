"""Minimal fieldbus session: connect, activate, home, move, deactivate.

The same script runs against a real robot and against the local simulator; only
the address changes.

Against the simulated robot, in two terminals::

    python -m mock_robot
    python examples/minimal_session.py --mock

Against a real Meca500 already switched to EtherNet/IP mode::

    python examples/minimal_session.py --address 192.168.0.100

Switching the robot into fieldbus mode is out of scope for this library: do it
with the robot web interface or your own tooling first.

``--mock`` does two things, both of which only make sense against the local
simulator:

* it loads the synthetic motion command identifiers the simulator understands,
  since the vendor files publish none and the library therefore ships none.
  Against a real robot, fill the real ones from the programming manual into the
  ``motion_commands.ids`` section of
  ``mecademic_fieldbus/io_map/spec/assembly_v1.json``;
* it makes the scanner listen on an ephemeral UDP port instead of the standard
  2222, because the simulated robot already holds 2222 on this machine.  That
  only works against a target that honours the T->O socket address item of the
  Forward Open request; a real robot may ignore it, which is why 2222 is the
  default everywhere else.
"""

import argparse
import logging
import os
import sys
import time
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecademic_fieldbus import (  # noqa: E402
    FieldbusError,
    FieldbusRobot,
    FieldbusUnsupportedFeature,
    get_io_map,
)
from mecademic_fieldbus.io_map import IoMapV1  # noqa: E402
from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport  # noqa: E402

#: Joint target used by the demonstration move, in degrees.
#: TODO: adjust to something safe for your cell before running on real hardware.
DEMO_JOINT_TARGET = (0.0, 0.0, 0.0, 0.0, 30.0, 0.0)


def build_parser() -> argparse.ArgumentParser:
    """Build the command line parser.

    Returns:
        The configured parser.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--address",
        default="127.0.0.1",
        help="IPv4 address of the robot, or of the local mock (default: %(default)s)",
    )
    parser.add_argument(
        "--rpi",
        type=int,
        default=None,
        help="requested packet interval in milliseconds (default: what the robot declares)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="target the local simulator: synthetic motion command ids, ephemeral UDP port",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=None,
        help="UDP port to listen on for the robot data (default: 2222, the standard one)",
    )
    parser.add_argument(
        "--no-move",
        action="store_true",
        help="stop after homing, without moving the robot",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="log what the library is doing",
    )
    return parser


def build_robot(
    address: str, rpi_ms: Optional[int], mock: bool, udp_port: Optional[int]
) -> FieldbusRobot:
    """Assemble the three layers of the library into a ready to use facade.

    The transport is built from the I/O map, so the assembly instances, sizes
    and connection parameters are declared in exactly one place: the
    specification generated from the vendor EDS.

    Args:
        address: IPv4 address of the robot, used only for logging here.
        rpi_ms: Requested packet interval, or ``None`` for the robot default.
        mock: Whether the target is the local simulator.
        udp_port: Port to listen on, or ``None`` to let ``--mock`` decide.

    Returns:
        A facade that is not connected yet.
    """
    if mock:
        from mock_robot.simulator import DEMO_MOTION_COMMAND_IDS

        io_map = IoMapV1(motion_command_ids=DEMO_MOTION_COMMAND_IDS)
    else:
        io_map = get_io_map()

    overrides = {}
    if rpi_ms is not None:
        overrides["rpi_ms"] = rpi_ms
    if udp_port is not None:
        overrides["originator_udp_port"] = udp_port
    elif mock:
        # The simulated robot already holds the standard port on this machine.
        overrides["originator_udp_port"] = 0

    transport = EtherNetIpTransport.from_io_map(io_map, **overrides)
    print(
        "driving {} with I/O map version {}, listening on UDP {}".format(
            address,
            io_map.version,
            transport.originator_udp_port or "an ephemeral port",
        )
    )
    return FieldbusRobot(transport, io_map)


def run_session(robot: FieldbusRobot, address: str, move: bool) -> None:
    """Run the demonstration sequence on a robot.

    Args:
        robot: Facade to drive.
        address: IPv4 address to connect to.
        move: Whether to run the demonstration move.
    """
    print("connecting to {} ...".format(address))
    robot.Connect(address)
    try:
        print("robot status: {}".format(robot.GetStatusRobot()))
        print("safety status: {}".format(robot.GetSafetyStatus()))

        if robot.GetStatusRobot().error_status:
            print("clearing a pre-existing error ...")
            robot.ResetError()

        print("activating ...")
        robot.ActivateRobot()

        print("homing ...")
        robot.Home()
        print("robot is ready: {}".format(robot.GetStatusRobot()))

        if move:
            run_move(robot)

        print("deactivating ...")
        robot.DeactivateRobot()
    finally:
        print("disconnecting ...")
        robot.Disconnect()


def run_move(robot: FieldbusRobot) -> None:
    """Run the demonstration move, if the command identifiers are known.

    Args:
        robot: Connected and homed facade.
    """
    try:
        move_id = robot.MoveJoints(*DEMO_JOINT_TARGET)
    except FieldbusUnsupportedFeature as error:
        print("skipping the move: {}".format(error))
        return
    print("move {} requested, target {} ...".format(move_id, DEMO_JOINT_TARGET))
    robot.WaitIdle()
    position = robot.GetRobotPosition()
    print("joints now at {}".format(tuple(round(value, 3) for value in position.joints)))
    print("pose now at   {}".format(tuple(round(value, 3) for value in position.pose)))


def main(argv: Optional[List[str]] = None) -> int:
    """Run the example.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    args = build_parser().parse_args(argv)
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format="%(levelname)-7s %(name)s: %(message)s")

    robot = build_robot(args.address, args.rpi, args.mock, args.udp_port)
    started = time.monotonic()
    try:
        run_session(robot, args.address, move=not args.no_move)
    except FieldbusError as error:
        print("fieldbus error: {}".format(error), file=sys.stderr)
        return 1
    print("done in {:.1f} s".format(time.monotonic() - started))
    return 0


if __name__ == "__main__":
    sys.exit(main())
