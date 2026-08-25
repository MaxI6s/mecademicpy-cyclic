"""Find out why a robot accepts commands but never produces cyclic data.

Reach for this when the robot clearly reacts to what the scanner sends -- its
own logs show the commands arriving -- while the scanner keeps reading an
all-zero input assembly.  The output direction works, the input one does not,
and the usual causes look identical from the application side.

The tool takes the third-party stack out of the picture entirely: it opens the
session and the Class 1 connection with :mod:`tools.eip_originator`, raw sockets
and ``struct`` only.  That buys two things the stack does not give:

* the **socket address items of the Forward Open reply**, where the target says
  where it will send -- including the group address when the connection is
  multicast.  The stack parses those and throws them away;
* unfiltered listening, so every datagram is reported with its real source,
  connection id and size instead of being silently dropped.

It listens on the standard UDP 2222, on an ephemeral port advertised in the
request, and on the multicast group if the reply names one.

Usage:

.. code-block:: shell

    python tools/diagnose_connection.py --address 192.168.0.100
    python tools/diagnose_connection.py --address 192.168.0.100 --multicast
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
from tools.eip_originator import (  # noqa: E402
    DEFAULT_UDP_PORT,
    EipOriginator,
    ForwardOpenReply,
    build_output_frame,
    encode_connection_path,
    is_multicast,
    parse_input_frame,
)

#: How long to listen, in seconds, before drawing a conclusion.
DEFAULT_LISTEN_SECONDS = 5.0

#: Maximum datagrams drained from one socket per polling round.  A robot at a
#: 10 ms interval keeps the socket permanently readable.
_DRAIN_BUDGET = 64


class Capture:
    """One listening socket and everything it received.

    Args:
        label: Human readable name of what this port stands for.
        port: UDP port to bind, ``0`` for an ephemeral one.

    Attributes:
        frames: Every datagram received, as ``(source, payload)`` pairs.
        bind_error: Why the socket could not be bound, if that happened.
        multicast_groups: Multicast groups joined on this socket.
    """

    def __init__(self, label: str, port: int) -> None:
        self.label = label
        self.requested_port = port
        self.frames: List[Tuple[Tuple[str, int], bytes]] = []
        self.bind_error: Optional[str] = None
        self.multicast_groups: List[str] = []
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

    def join_multicast(self, group: str) -> Optional[str]:
        """Subscribe this socket to a multicast group.

        Args:
            group: Group address to join.

        Returns:
            ``None`` on success, otherwise why the join failed.
        """
        if self.socket is None:
            return "socket not bound"
        try:
            request = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton("0.0.0.0"))
            self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, request)
        except OSError as error:
            return str(error)
        self.multicast_groups.append(group)
        return None

    def poll(self, deadline: float) -> None:
        """Drain what is waiting on the socket, bounded by a budget and a deadline.

        Args:
            deadline: Monotonic time after which to stop draining.
        """
        if self.socket is None:
            return
        for _ in range(_DRAIN_BUDGET):
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


def describe(capture: Capture, reply: ForwardOpenReply, io_map: IoMap) -> None:
    """Print what one capture received, and how it compares to expectations.

    Args:
        capture: Capture to describe.
        reply: The Forward Open reply, for the expected connection id.
        io_map: Map used to decode the first usable frame.
    """
    where = "port {}".format(capture.port)
    if capture.multicast_groups:
        where += ", group " + ", ".join(capture.multicast_groups)
    if capture.bind_error is not None:
        print("  {} ({}): NOT BOUND -- {}".format(capture.label, where, capture.bind_error))
        return
    if not capture.frames:
        print("  {} ({}): nothing received".format(capture.label, where))
        return

    sources: Dict[str, int] = {}
    connection_ids: Dict[int, int] = {}
    unusable = 0
    decoded = None
    for source, payload in capture.frames:
        sources[source[0]] = sources.get(source[0], 0) + 1
        parsed = parse_input_frame(payload, io_map.input_assembly_size)
        if parsed is None:
            unusable += 1
            continue
        connection_id, assembly = parsed
        connection_ids[connection_id] = connection_ids.get(connection_id, 0) + 1
        if decoded is None:
            decoded = io_map.decode_status(assembly)

    print("  {} ({}): {} datagram(s)".format(capture.label, where, len(capture.frames)))
    print(
        "    sources        : {}".format(
            ", ".join("{} x{}".format(ip, count) for ip, count in sources.items())
        )
    )
    print(
        "    connection ids : {}".format(
            ", ".join("0x{:08X} x{}".format(cid, n) for cid, n in connection_ids.items())
            or "none usable"
        )
    )
    if unusable:
        print("    unusable       : {} datagram(s) of an unexpected shape".format(unusable))
    if connection_ids and reply.to_connection_id not in connection_ids:
        print(
            "    !! none carries the T->O connection id 0x{:08X} the robot announced; "
            "a stack filtering on it would drop them all".format(reply.to_connection_id)
        )
    if decoded is not None:
        print("    decoded status : {}".format(decoded))


def report_next_steps(address: str) -> None:
    """Print how to tell a firewall from a silent robot.

    Args:
        address: Address the scanner connected to.
    """
    print()
    print("next steps -- is anything reaching this machine at all?")
    print("  Watch the wire itself; tcpdump sees packets before any firewall")
    print("  drops them at the socket layer:")
    print()
    print("    sudo tcpdump -n -i any 'udp and (port 2222 or ip multicast)'")
    print()
    print("  Then, with the robot connected, run this tool again in another")
    print("  terminal and watch what tcpdump prints.")
    print()
    print("  Packets visible in tcpdump but not captured here")
    print("      -> a firewall on this machine drops inbound UDP.")
    if sys.platform == "darwin":
        print("         Check the macOS application firewall:")
        print("           /usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate")
        print("           /usr/libexec/ApplicationFirewall/socketfilterfw --getblockall")
        print("         If block-all is on, turn it off or allow the python binary:")
        print("           sudo /usr/libexec/ApplicationFirewall/socketfilterfw \\")
        print("             --add $(python -c 'import sys; print(sys.executable)')")
        print("         Also check pf, if your organisation enables it:")
        print("           sudo pfctl -s info")
    else:
        print("         Check the host firewall, for instance:")
        print("           sudo iptables -L -n | grep 2222")
        print("           sudo firewall-cmd --list-all")
    print()
    print("  Nothing in tcpdump either")
    print("      -> the robot is not producing. Try, in order:")
    print("         1. this tool with --multicast, in case the robot only")
    print("            produces to a group when the connection asks for one;")
    print("         2. check the robot really is in EtherNet/IP mode and that")
    print("            no other scanner already owns the exclusive connection;")
    print("         3. check the route back: the robot must be able to reach")
    print("            this host's IP. Confirm with, from this machine,")
    print("              ping {}".format(address))
    print("            and check both ends sit on the same subnet, or that the")
    print("            robot has a gateway configured.")


def verdict(
    captures: List[Capture], reply: ForwardOpenReply, address: str, advertised_port: int
) -> None:
    """Print what the capture means and what to do about it.

    Args:
        captures: Every capture, in the order they were set up.
        reply: The Forward Open reply.
        address: Address the scanner connected to.
        advertised_port: Port advertised in the Forward Open request.
    """
    print()
    print("verdict")
    print("-------")
    received = [capture for capture in captures if capture.frames]
    announced = reply.target_to_originator_address

    if not received:
        print("Nothing arrived anywhere, so the robot is not producing to this host.")
        if announced is not None:
            print(
                "It announced it would send to {}, which is where this tool listened.".format(
                    announced
                )
            )
        else:
            print("It announced no target-to-originator socket address in its reply,")
            print("so there is nothing to compare against; the causes below remain.")
        report_next_steps(address)
        return

    winner = received[0]
    print("Data arrived on: {} (port {}).".format(winner.label, winner.port))
    if winner.multicast_groups:
        print()
        print("The robot produces MULTICAST, to {}.".format(", ".join(winner.multicast_groups)))
        print("A scanner that does not join that group never sees a single frame,")
        print("and the stack this library wraps does not join one.")
        print("Fix: ask for a point-to-point connection, which is what")
        print("EtherNetIpTransport does; if the robot insists on multicast, the")
        print("transport needs to join the group -- open an issue with this output.")
    elif winner.port == DEFAULT_UDP_PORT:
        print()
        print(
            "The robot ignored the advertised port {} and produced to the".format(advertised_port)
        )
        print("standard 2222 instead.")
        print("Fix: listen on 2222, which is the default:")
        print("    EtherNetIpTransport.from_io_map(io_map)              # port 2222")
        print("    EtherNetIpTransport.from_io_map(io_map, originator_udp_port=0)  # do NOT")
    else:
        print()
        print("The robot honoured the advertised port, so the receive path works.")
        print("Look at the checks flagged above: a stack filtering on the source")
        print("address or on the connection id would still drop these frames.")

    sources = {source[0] for capture in received for source, _ in capture.frames}
    if sources and address not in sources:
        print()
        print(
            "Note: the robot answers from {} but the connection was opened to {}.".format(
                ", ".join(sorted(sources)), address
            )
        )
        print("The stack drops frames whose source address differs from the one it")
        print("connected to. Connect using the address the robot answers from.")


def run(address: str, rpi_ms: int, seconds: float, multicast: bool) -> int:
    """Open a connection, capture what comes back, and report.

    Args:
        address: IPv4 address of the robot.
        rpi_ms: Requested packet interval, in milliseconds.
        seconds: How long to listen.
        multicast: Whether to request a multicast target-to-originator
            connection.

    Returns:
        The process exit code.
    """
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
        "connection path {}, config assembly {}, requesting {}".format(
            io_map.connection.connection_path or "?",
            "none" if io_map.config_assembly_instance is None else io_map.config_assembly_instance,
            "multicast" if multicast else "point to point",
        )
    )
    print()

    standard = Capture("standard port", DEFAULT_UDP_PORT)
    advertised = Capture("advertised port", 0)
    captures = [standard, advertised]
    if standard.bind_error is not None:
        print("warning: could not bind UDP {} -- {}".format(DEFAULT_UDP_PORT, standard.bind_error))
        print("         something else on this machine already holds it, which on its own")
        print("         explains a scanner never receiving anything.")
        print()

    originator = EipOriginator(address)
    stop = threading.Event()
    producer: Optional[threading.Thread] = None
    path = encode_connection_path(io_map.connection.connection_path)
    try:
        try:
            originator.open_session()
        except (OSError, ValueError) as error:
            print("cannot open a session with {}: {}".format(address, error), file=sys.stderr)
            return 1
        print("session registered (handle 0x{:08X})".format(originator.session))

        try:
            reply = originator.forward_open(
                path,
                io_map.output_assembly_size,
                io_map.input_assembly_size,
                rpi_microseconds=rpi_ms * 1000,
                originator_udp_port=advertised.port,
                multicast=multicast,
                output_run_idle_header=io_map.connection.output_run_idle_header,
            )
        except (OSError, ValueError) as error:
            print("the Forward Open exchange failed: {}".format(error), file=sys.stderr)
            return 1

        if not reply.accepted:
            print(
                "Forward Open REFUSED: general status 0x{:02X}, extended 0x{:04X}".format(
                    reply.general_status, reply.extended_status
                ),
                file=sys.stderr,
            )
            return 1

        print("Forward Open accepted")
        print("  O->T connection id  : 0x{:08X}".format(reply.ot_connection_id))
        print("  T->O connection id  : 0x{:08X}".format(reply.to_connection_id))
        print("  actual intervals    : O->T {} us, T->O {} us".format(reply.ot_api, reply.to_api))
        print("  advertised T->O port: {}".format(advertised.port))
        if reply.socket_addresses:
            for item_type, item in sorted(reply.socket_addresses.items()):
                direction = "O->T" if item_type == 0x8000 else "T->O"
                print(
                    "  reply socket address: {} {} (item 0x{:04X})".format(
                        direction, item, item_type
                    )
                )
        else:
            print("  reply socket address: none returned by the robot")

        announced = reply.target_to_originator_address
        if announced is not None and is_multicast(announced.address):
            print()
            print("  the robot announced a MULTICAST destination: {}".format(announced))
            target = (
                standard
                if standard.port == announced.port
                else Capture("multicast group", announced.port)
            )
            if target not in captures:
                captures.append(target)
            error = target.join_multicast(announced.address)
            if error is None:
                print("  joined {} on port {}".format(announced.address, target.port))
            else:
                print("  could NOT join {}: {}".format(announced.address, error))
        print()

        producer = threading.Thread(
            target=produce,
            args=(
                stop,
                address,
                reply.ot_connection_id,
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
            for capture in captures:
                capture.poll(deadline)

        print()
        print("captured")
        print("--------")
        for capture in captures:
            describe(capture, reply, io_map)
        verdict(captures, reply, address, advertised.port)
    finally:
        stop.set()
        if producer is not None:
            producer.join(timeout=1.0)
        originator.forward_close(path)
        originator.close()
        for capture in captures:
            capture.close()
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
    parser.add_argument(
        "--multicast",
        action="store_true",
        help="request a multicast target-to-originator connection instead of point to point",
    )
    args = parser.parse_args(argv)
    return run(args.address, args.rpi, args.seconds, args.multicast)


if __name__ == "__main__":
    sys.exit(main())
