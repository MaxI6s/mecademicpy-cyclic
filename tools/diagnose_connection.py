"""Find out why a robot accepts commands but never produces cyclic data.

This is the tool to reach for when the robot clearly reacts to what the scanner
sends -- its own logs show the commands arriving -- while the scanner keeps
reading an all-zero input assembly.  The output direction works, the input one
does not, and the usual causes look identical from the application side.

The tool takes the third-party stack out of the equation for the *receiving*
side: it opens the session and the Class 1 connection normally, then listens on
its own sockets and reports every datagram exactly as it arrives, without any
filtering.  It watches **two** ports at once:

* the standard UDP 2222, where a target sends its data when it ignores the T->O
  socket address item of the Forward Open request;
* an ephemeral port, which is the one actually advertised in that item.

Which of the two receives data answers the question outright:

* data on 2222 -> the robot ignores the socket address item; the scanner must
  listen on 2222, which is what ``EtherNetIpTransport`` now does by default;
* data on the ephemeral port -> the robot honours the item, so the problem lies
  elsewhere and the report says which check failed;
* nothing on either -> the robot is not producing to this host at all: a
  firewall dropping inbound UDP, a multicast connection, or a robot that never
  entered the run state.

Usage:

.. code-block:: shell

    python tools/diagnose_connection.py --address 192.168.0.100
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecademic_fieldbus.io_map import IoMap, get_io_map  # noqa: E402
from mock_robot.server import (  # noqa: E402
    DEFAULT_UDP_PORT,
    parse_cyclic_frame,
    strip_run_idle_header,
)

#: Size of the run/idle header a scanner prepends to the data it produces.
RUN_IDLE_HEADER = b"\x01\x00\x00\x00"

#: How long to wait, in seconds, for the first datagram before giving up.
DEFAULT_LISTEN_SECONDS = 5.0


class Capture:
    """One listening socket and everything it received.

    Args:
        label: Human readable name of what this port stands for.
        port: UDP port to bind, ``0`` for an ephemeral one.

    Attributes:
        frames: Every datagram received, as ``(source, payload)`` pairs.
        bind_error: Why the socket could not be bound, if that happened.
    """

    def __init__(self, label: str, port: int) -> None:
        self.label = label
        self.requested_port = port
        self.frames: List[Tuple[Tuple[str, int], bytes]] = []
        self.bind_error: Optional[str] = None
        self.socket: Optional[socket.socket] = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
            sock.settimeout(0.2)
            self.socket = sock
        except OSError as error:
            self.bind_error = str(error)

    @property
    def port(self) -> int:
        """Port actually bound, or the requested one when binding failed."""
        if self.socket is None:
            return self.requested_port
        return int(self.socket.getsockname()[1])

    def poll(self, deadline: float, budget: int = 64) -> None:
        """Drain what is waiting on the socket, without blocking long.

        A robot producing at a 10 ms interval keeps the socket permanently
        readable, so the drain is bounded both by a datagram budget and by the
        capture deadline; otherwise this never returns.

        Args:
            deadline: Monotonic time after which to stop draining.
            budget: Maximum number of datagrams to read in one call.
        """
        if self.socket is None:
            return
        for _ in range(budget):
            if time.monotonic() >= deadline:
                return
            try:
                payload, source = self.socket.recvfrom(4096)
            except socket.timeout:
                return
            except OSError:
                return
            self.frames.append((source, payload))

    def close(self) -> None:
        """Close the socket, ignoring errors."""
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
            self.socket = None


def build_output_frame(connection_id: int, sequence: int, assembly: bytes) -> bytes:
    """Build one originator-to-target cyclic frame.

    Args:
        connection_id: Connection identifier the target assigned to this
            direction.
        sequence: 32 bit sequence number of this frame.
        assembly: Raw output assembly image.

    Returns:
        The datagram to send.
    """
    payload = struct.pack("<H", sequence & 0xFFFF) + RUN_IDLE_HEADER + assembly
    return (
        struct.pack("<HHH", 2, 0x8002, 8)
        + struct.pack("<II", connection_id, sequence)
        + struct.pack("<HH", 0x00B1, len(payload))
        + payload
    )


def produce(
    stop: threading.Event,
    address: str,
    connection_id: int,
    assembly: bytes,
    interval_s: float,
) -> None:
    """Send the output assembly cyclically until asked to stop.

    Args:
        stop: Event signalling that production must end.
        address: IPv4 address of the robot.
        connection_id: Originator-to-target connection identifier.
        assembly: Raw output assembly image, produced unchanged.
        interval_s: Delay between two frames.
    """
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    try:
        while not stop.is_set():
            sender.sendto(
                build_output_frame(connection_id, sequence, assembly),
                (address, DEFAULT_UDP_PORT),
            )
            sequence += 1
            time.sleep(interval_s)
    except OSError as error:
        print("  producing stopped: {}".format(error))
    finally:
        sender.close()


def describe_frames(capture: Capture, expected_connection_id: int, io_map: IoMap) -> None:
    """Print what one capture received, and how it compares to expectations.

    Args:
        capture: Capture to describe.
        expected_connection_id: Connection identifier the robot announced for
            the target-to-originator direction.
        io_map: Map used to decode the first usable frame.
    """
    if capture.bind_error is not None:
        print(
            "  {} (port {}): NOT BOUND -- {}".format(
                capture.label, capture.requested_port, capture.bind_error
            )
        )
        return
    if not capture.frames:
        print("  {} (port {}): nothing received".format(capture.label, capture.port))
        return

    sources: Dict[str, int] = {}
    connection_ids: Dict[int, int] = {}
    sizes: Dict[int, int] = {}
    decoded = None
    for source, payload in capture.frames:
        sources[source[0]] = sources.get(source[0], 0) + 1
        try:
            connection_id, data = parse_cyclic_frame(payload)
        except ValueError:
            connection_ids[-1] = connection_ids.get(-1, 0) + 1
            continue
        connection_ids[connection_id] = connection_ids.get(connection_id, 0) + 1
        sizes[len(data)] = sizes.get(len(data), 0) + 1
        if decoded is None:
            assembly = strip_run_idle_header(data, io_map.input_assembly_size)
            if assembly is not None:
                decoded = io_map.decode_status(assembly)

    print("  {} (port {}): {} datagram(s)".format(capture.label, capture.port, len(capture.frames)))
    print(
        "    source addresses  : {}".format(
            ", ".join("{} x{}".format(ip, count) for ip, count in sources.items())
        )
    )
    print(
        "    connection ids    : {}".format(
            ", ".join(
                "0x{:08X} x{}".format(cid, count) if cid >= 0 else "unparsable x{}".format(count)
                for cid, count in connection_ids.items()
            )
        )
    )
    print(
        "    payload sizes     : {} (expected {})".format(
            ", ".join("{} x{}".format(size, count) for size, count in sizes.items()),
            io_map.input_assembly_size,
        )
    )
    if expected_connection_id not in connection_ids:
        print(
            "    !! none carries the T->O connection id 0x{:08X} the robot announced; "
            "the stack filters those out".format(expected_connection_id)
        )
    if decoded is not None:
        print("    decoded status    : {}".format(decoded))


def verdict(
    standard: Capture, advertised: Capture, address: str, expected_connection_id: int
) -> None:
    """Print what the capture means and what to do about it.

    Args:
        standard: Capture bound to the standard port 2222.
        advertised: Capture bound to the port advertised in the Forward Open.
        address: Address the scanner connected to.
        expected_connection_id: T->O connection identifier the robot announced.
    """
    print()
    print("verdict")
    print("-------")
    if advertised.frames:
        print("The robot honours the T->O socket address item: it produced to the")
        print("port advertised in the Forward Open request. The receive path itself")
        print("works, so look at the checks flagged above -- most often the source")
        print("address or the connection id the stack filters on.")
    elif standard.frames:
        print("The robot IGNORES the T->O socket address item: it produced to the")
        print("standard port 2222 instead of the advertised one.")
        print("Fix: have the scanner listen on 2222, which is the default since")
        print("    EtherNetIpTransport(..., originator_udp_port=2222)")
        print("A scanner bound to an ephemeral port will never see this robot.")
    else:
        print("Nothing arrived on either port. The robot is not producing to this")
        print("host. In decreasing order of likelihood:")
        print("  1. a firewall on this machine drops inbound UDP 2222;")
        print("  2. the robot produces multicast rather than unicast, and nothing")
        print("     here joined the group;")
        print("  3. the robot never entered the run state -- check that the O->T")
        print("     frames carry the 32 bit run/idle header, which this tool sends;")
        print("  4. routing: the robot answers on a different interface than the")
        print("     one used to reach {}.".format(address))
    sources = {source[0] for source, _ in standard.frames + advertised.frames}
    if sources and address not in sources:
        print()
        print(
            "Note: the robot answers from {} but the connection was opened to {}.".format(
                ", ".join(sorted(sources)), address
            )
        )
        print("The stack drops frames whose source address differs from the one it")
        print("connected to. Connect using the address the robot answers from.")
    del expected_connection_id


def run(address: str, rpi_ms: int, seconds: float) -> int:
    """Open a connection, capture what comes back, and report.

    Args:
        address: IPv4 address of the robot.
        rpi_ms: Requested packet interval, in milliseconds.
        seconds: How long to listen.

    Returns:
        The process exit code.
    """
    try:
        import ethernetip
    except ImportError:
        print(
            "the 'ethernetip' package is required: pip install 'mecademic-fieldbus[ethernetip]'",
            file=sys.stderr,
        )
        return 1

    io_map = get_io_map()
    print(
        "I/O map version {} -- input {} bytes (instance {}), output {} bytes (instance {})".format(
            io_map.version,
            io_map.input_assembly_size,
            io_map.input_assembly_instance,
            io_map.output_assembly_size,
            io_map.output_assembly_instance,
        )
    )
    print(
        "connection path {}, config assembly {}".format(
            io_map.connection.connection_path or "?",
            "none" if io_map.config_assembly_instance is None else io_map.config_assembly_instance,
        )
    )
    print()

    standard = Capture("standard port", DEFAULT_UDP_PORT)
    advertised = Capture("advertised port", 0)
    if standard.bind_error is not None:
        print("warning: could not bind UDP {} -- {}".format(DEFAULT_UDP_PORT, standard.bind_error))
        print("         something else on this machine already holds it, which on its own")
        print("         explains a scanner never receiving anything.")
        print()

    enip = ethernetip.EtherNetIP(address)
    try:
        connection = enip.explicit_conn(address)
    except OSError as error:
        print("cannot reach {} on TCP 44818: {}".format(address, error), file=sys.stderr)
        standard.close()
        advertised.close()
        return 1

    stop = threading.Event()
    producer: Optional[threading.Thread] = None
    try:
        if connection.registerSession() != 0:
            print("the device refused the session", file=sys.stderr)
            return 1
        print("session registered")

        status = connection.sendFwdOpenReq(
            io_map.input_assembly_instance,
            io_map.output_assembly_instance,
            io_map.config_assembly_instance,
            torpi=rpi_ms,
            otrpi=rpi_ms,
            inputsz=io_map.input_assembly_size,
            outputsz=io_map.output_assembly_size,
            originator_udp_port=advertised.port,
        )
        if status is None:
            print("no Forward Open answer; is the robot in EtherNet/IP mode?", file=sys.stderr)
            return 1
        if status != 0:
            print("Forward Open rejected, extended status 0x{:04X}".format(status), file=sys.stderr)
            return 1

        print("Forward Open accepted")
        print("  O->T connection id : 0x{:08X}".format(connection.otconnid))
        print("  T->O connection id : 0x{:08X}".format(connection.toconnid))
        print(
            "  actual intervals   : O->T {} ms, T->O {} ms".format(
                connection.otapi, connection.toapi
            )
        )
        print(
            "  advertised T->O port: {} (standard port {} is also being watched)".format(
                advertised.port, standard.port
            )
        )
        print()

        producer = threading.Thread(
            target=produce,
            args=(
                stop,
                address,
                connection.otconnid,
                io_map.empty_output_assembly(),
                max(rpi_ms / 1000.0, 0.001),
            ),
            daemon=True,
        )
        producer.start()

        print(
            "producing an all-zero output assembly and listening for {:.1f} s ...".format(seconds)
        )
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            standard.poll(deadline)
            advertised.poll(deadline)

        print()
        print("captured")
        print("--------")
        describe_frames(standard, connection.toconnid, io_map)
        describe_frames(advertised, connection.toconnid, io_map)
        verdict(standard, advertised, address, connection.toconnid)
    finally:
        stop.set()
        if producer is not None:
            producer.join(timeout=1.0)
        try:
            connection.sendFwdCloseReq(
                io_map.input_assembly_instance,
                io_map.output_assembly_instance,
                io_map.config_assembly_instance,
            )
            connection.unregisterSession()
        except OSError:
            pass
        standard.close()
        advertised.close()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    """Run the diagnosis.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", required=True, help="IPv4 address of the robot")
    parser.add_argument(
        "--rpi", type=int, default=10, help="requested packet interval in ms (default: %(default)s)"
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_LISTEN_SECONDS,
        help="how long to listen (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    return run(args.address, args.rpi, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
