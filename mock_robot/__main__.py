"""Command line entry point of the simulated robot.

Start a robot on the loopback interface and leave it running::

    python -m mock_robot

Then point any scanner at ``127.0.0.1``, including the example shipped in
``examples/minimal_session.py``.
"""

import argparse
import logging
import sys
from typing import List, Optional

from mecademic_fieldbus.io_map import LATEST_VERSION, get_io_map

from .server import DEFAULT_TCP_PORT, DEFAULT_UDP_PORT, MockRobotServer
from .simulator import RobotSimulator


def build_parser() -> argparse.ArgumentParser:
    """Build the command line parser.

    Returns:
        The configured parser.
    """
    parser = argparse.ArgumentParser(
        prog="python -m mock_robot",
        description="Run a simulated Mecademic robot as an EtherNet/IP adapter.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="interface to bind to (default: %(default)s)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=DEFAULT_TCP_PORT,
        help="encapsulation port (default: %(default)s)",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=DEFAULT_UDP_PORT,
        help="implicit I/O port (default: %(default)s)",
    )
    parser.add_argument(
        "--io-map-version",
        default=LATEST_VERSION,
        help="assembly layout version to expose (default: %(default)s)",
    )
    parser.add_argument(
        "--activation-time",
        type=float,
        default=0.5,
        help="simulated activation duration, in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--homing-time",
        type=float,
        default=1.0,
        help="simulated homing duration, in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--permissive",
        action="store_true",
        help="accept a Forward Open even when it does not match the I/O map",
    )
    parser.add_argument(
        "--show-layout",
        action="store_true",
        help="print the assembly layout and exit",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="log every protocol exchange",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Run the simulated robot until interrupted.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    io_map = get_io_map(args.io_map_version)
    if args.show_layout:
        print(io_map.describe_layout())
        return 0

    simulator = RobotSimulator(
        io_map,
        activation_time_s=args.activation_time,
        homing_time_s=args.homing_time,
    )
    if not io_map.motion_command_ids:
        logging.getLogger(__name__).warning(
            "the assembly specification defines no motion command identifiers; "
            "the simulator falls back to its own synthetic ones: %s",
            simulator.motion_command_ids,
        )
    server = MockRobotServer(
        simulator,
        host=args.host,
        tcp_port=args.tcp_port,
        udp_port=args.udp_port,
        strict=not args.permissive,
    )
    try:
        server.serve_forever()
    except OSError as exc:
        print("cannot start the simulated robot: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
