# mecademic-fieldbus

A **standalone** Python library that drives a Mecademic Meca500 over a
fieldbus, by acting as the EtherNet/IP **scanner (originator)** of the robot's
cyclic IN/OUT assemblies.

The robot must already be running in fieldbus mode. Switching it into that mode
is **out of scope** for this library: do it with the robot web interface or your
own tooling first.

> **Status.** The layering, the mock robot and the test harness are complete and
> working end to end. The assembly layout is the **real one**, generated from
> the official EDS of firmware 11.3 (`Meca500_v11.3.3.12637-official.eds`,
> EDS revision 2.6). What is still missing is listed under
> [Open decisions](#open-decisions) — chiefly the motion command identifiers,
> which no vendor file publishes.

## Why it is built this way

Two constraints shaped the design:

1. **The assembly layout will change.** New firmware means new fields, new
   versions. So no application code — and no transport code — may ever depend on
   a raw offset.
2. **This may be ported to another language.** So the library depends on no
   vendor library, its core needs nothing beyond the standard library, and the
   most valuable part (the bit/word mapping) lives in a **declarative JSON
   specification** rather than in Python.

```
Application
    │  familiar API: ActivateRobot, Home, GetStatusRobot, ...
    ▼
FieldbusRobot ................ robot.py            layer 4: public facade
    │  named, typed fields (RobotStatus, MotionCommand, ...)
    ▼
IoMap (versioned) ............ io_map/            layer 3: bits ↔ logical fields
    │  raw assembly bytes
    ▼
FieldbusTransport ............ transports/        layer 2: protocol
    │  wraps a third-party EtherNet/IP stack
    ▼
Fieldbus network
```

**The central rule of this repository:** *a bit or word offset never appears
outside `mecademic_fieldbus/io_map/`.* Offsets live in
`io_map/spec/assembly_v1.json`, are applied by `io_map/codec.py`, and are exposed
to the rest of the world only as named fields. `IoMap.describe_layout()` is the
one sanctioned way to show offsets to a human.

Each layer is replaceable on its own:

* **Another protocol?** One transport equals one file implementing
  `FieldbusTransport`. Profinet would be `transports/profinet.py` and nothing
  else would change — the firmware ships a GSDML describing the very same
  assemblies. (EtherCAT is the known exception: it needs a real-time master and
  will not fit this interface as-is, even though an ESI file exists.)
* **Another EtherNet/IP stack?** Every call to the third-party stack is confined
  to `transports/ethernetip.py`.
* **New firmware layout?** Re-run the generator, add an `IoMap` subclass if the
  field vocabulary changed; the facade is untouched.

## Install

```bash
pip install -e '.[dev]'
```

The core has **no dependencies**. The EtherNet/IP transport needs
[`python-ethernetip`](https://codeberg.org/paperwork/python-ethernetip)
(published on PyPI as `ethernetip`), pulled in by the `ethernetip` extra. That
stack was chosen because, unlike most Python EtherNet/IP libraries, it supports
Class 1 implicit I/O towards a *generic* adapter — `registerAssembly` plus
`sendFwdOpenReq` plus a `produce` cycle — and not only Rockwell tag messaging.

## Quick start against the simulated robot

No hardware needed. In one terminal:

```bash
python -m mock_robot
```

In another:

```bash
python examples/minimal_session.py --mock
```

The mock is a real EtherNet/IP adapter on TCP 44818 and UDP 2222: it answers
Register Session and Forward Open, validates the requested connection points and
sizes against the specification, then consumes the cyclic output assembly and
produces the cyclic input assembly, driven by a state machine
(`deactivated → activating → activated → homing → idle → moving`, plus `error`).

`--mock` exists because the motion command identifiers are the one thing no
vendor file publishes; see [Open decisions](#open-decisions).

## Quick start against a real robot

```bash
python examples/minimal_session.py --address 192.168.0.100
```

In code, the three layers are wired explicitly. The transport is built *from*
the I/O map, so the assembly instances, sizes and connection parameters are
declared in exactly one place:

```python
from mecademic_fieldbus import FieldbusRobot, get_io_map
from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport

io_map = get_io_map()
transport = EtherNetIpTransport.from_io_map(io_map)

robot = FieldbusRobot(transport, io_map)
robot.Connect("192.168.0.100")
robot.ActivateRobot()
robot.Home()
print(robot.GetStatusRobot())
print(robot.GetRobotPosition().joints)
robot.MoveJoints(0, 0, 0, 0, 30, 0)   # needs the command ids, see below
robot.WaitIdle()
robot.DeactivateRobot()
robot.Disconnect()
```

### What the protocol imposes

There is no request/response here, and the vendor semantics are specific:

* The output assembly is a **process image** the transport keeps producing, so a
  command is *latched*, not sent. Every `RobotControl` bit is **level**
  triggered — it acts "as soon as, and as long as" it is set — and `Deactivate`
  takes precedence over `Activate`.
* `MotionControl_ResumeMotion` is the exception: it acts on its **rising edge**
  only. `ResumeMotion()` pulses it for you.
* A motion command is picked up only when **`MotionControl_MoveId` changes**,
  while `MotionControl_Setpoint` is set. There is no acknowledgement beyond the
  robot echoing the id back in `MotionStatus_MoveID`. `Connect()` therefore
  resumes the numbering where the robot left it, so a scanner restarting against
  a live robot never reuses an id the robot would silently discard.
* There is **no error bit**: a non-zero `RobotStatus_Error` is what signals the
  error state. `RobotStatus.error_status` derives it for you.

## Repository layout

```
mecademic_fieldbus/
    robot.py                  FieldbusRobot, the public facade
    robot_classes.py          RobotStatus, MotionCommand, ... (self-contained)
    exceptions.py             FieldbusError and its subclasses
    transports/
        base.py               FieldbusTransport (ABC)
        ethernetip.py         the only file that touches a third-party stack
    io_map/
        base.py               IoMap (ABC), versioned
        v1.py                 IoMapV1, a typed accessor over the spec
        codec.py              the only code that manipulates offsets
        spec_loader.py        loads and validates the spec
        spec/assembly_v1.json THE SOURCE OF TRUTH, generated from the EDS
mock_robot/
    simulator.py              robot state machine, shares the same io_map
    server.py                 minimal EtherNet/IP adapter, raw sockets
tools/
    eds_to_spec.py            regenerates the spec from a vendor EDS
    make_fixtures.py          regenerates the golden test vectors
    eip_originator.py         dependency-free CIP originator, for diagnosis
    diagnose_connection.py    finds out why a robot produces nothing
examples/minimal_session.py   works against the mock and against a real robot
tests/
    fake_transport.py         in-memory transports for the unit tests
    fixtures/                 golden assembly images, one per scenario
```

## The assembly specification

`io_map/spec/assembly_v1.json` is **generated, not written**:

```bash
python tools/eds_to_spec.py path/to/Meca500_vX.eds \
    -o mecademic_fieldbus/io_map/spec/assembly_v1.json
python tools/make_fixtures.py          # then refresh the golden vectors
```

The generator reads the `[Device]`, `[Params]`, `[Assembly]` and
`[Connection Manager]` sections, expands each CIP bit string into one boolean
field per named bit, drops the reserved and unused ones, and computes every
offset from the ordered member list. It fails loudly if the member sizes
contradict the connection sizes the EDS declares.

What it produces for firmware 11.3:

| | value |
| --- | --- |
| Input assembly (robot → scanner) | instance **100**, **252** bytes, 91 fields |
| Output assembly (scanner → robot) | instance **150**, **60** bytes, 26 fields |
| Connection path | `20 04 2C 96 2C 64` — **no configuration assembly** |
| Real-time format | O→T 32-bit run/idle header, T→O modeless |
| RPI | min **10 ms**, max 10 s, default 10 ms |
| Electronic key | vendor 1565, product type 43, product code 500, rev 11.3 |

Inspect the whole layout with:

```bash
python -m mock_robot --show-layout
```

Field names are the vendor spelling (`RobotStatus_Activated`,
`MotionCommand_Arg1`, …) so any value can be traced straight back to the EDS,
the GSDML and the programming manual. JSON was chosen over YAML deliberately: it
needs no dependency, and every language a port might target parses it out of the
box. A port only has to reimplement the ~200 lines of `codec.py`; the layout
itself travels unchanged.

**Layout mismatch detection.** The assembly carries no revision field, so there
is nothing to compare — but nothing is needed: the robot validates the
negotiated connection sizes at Forward Open time, so a scanner built on the
wrong layout version cannot open a connection at all. `tests/` covers that.

## Testing

```bash
pytest                       # everything
pytest -m "not integration"  # unit tests only, no sockets
pytest -m integration        # the real stack against the mock robot
```

Three levels:

* **Unit** — `FakeTransport` feeds byte fixtures in memory; `SimulatorTransport`
  wires the facade straight to the simulator with no network at all.
* **Golden vectors** — `tests/fixtures/assembly_v1_vectors.json` pins the exact
  bytes each scenario must produce, so an accidental change to the spec is caught
  immediately. These vectors are also the conformance reference a port to another
  language should reproduce byte for byte.
* **Integration** — the real `EtherNetIpTransport` opens a Class 1 connection to
  the mock robot over real sockets, with the real connection path and sizes.
  Needs TCP 44818 and UDP 2222 free; skipped otherwise.

## Troubleshooting

### The robot reacts to commands but the scanner sees nothing back

The output direction works, the input one does not: the robot logs the commands,
while `GetStatusRobot()` keeps returning an all-zero image and `ActivateRobot()`
times out waiting for a bit the robot has already set.

Run:

```bash
python tools/diagnose_connection.py --address 192.168.0.100
```

It opens the connection normally, then listens on **two** ports at once with its
own sockets and no filtering: the standard UDP 2222, and an ephemeral one which
is what it advertises in the T→O socket address item of the Forward Open. Which
one receives data settles it:

| Result | Cause | Fix |
| --- | --- | --- |
| Data on **2222** | The robot ignores the socket address item and produces to the standard port. | Listen on 2222 — the default since `EtherNetIpTransport(..., originator_udp_port=2222)`. An ephemeral port never sees this robot. |
| Data on the **ephemeral** port | The robot honours the item; the receive path works. | Check what the report flags: usually the source address or the connection id the stack filters on. |
| **Nothing** on either | The robot is not producing to this host. | See [Nothing arrives at all](#nothing-arrives-at-all) below. |

The tool opens the connection itself, with raw sockets and no third-party stack,
which buys two things the stack cannot give:

* the **socket address items of the Forward Open reply**, where the target says
  where it will send — including the group address when the connection is
  multicast. The stack parses those items and discards them
  (`ethernetip.py:1801`), which is exactly why a multicast robot looks silent
  rather than misconfigured;
* unfiltered listening. Two filters inside the stack drop frames without a word,
  and the tool reports both: it only accepts datagrams whose **source IP equals
  the address you connected to** (so connect by IP, not by hostname, and use the
  address the robot answers *from*), and whose **connection id matches** the one
  returned by the Forward Open.

### Nothing arrives at all

If the tool captures nothing on any port, the robot is not producing to this
host. Work through these in order — the first step tells you which half of the
problem you have.

**1. Is anything reaching the machine?** `tcpdump` sees packets before any
firewall drops them at the socket layer:

```bash
sudo tcpdump -n -i any 'udp and (port 2222 or ip multicast)'
```

Leave it running and start the diagnostic in another terminal.

**2. Packets in `tcpdump` but nothing captured → a host firewall.** On macOS:

```bash
/usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate
/usr/libexec/ApplicationFirewall/socketfilterfw --getblockall
```

If block-all is on, turn it off, or allow the interpreter:

```bash
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --add $(python -c 'import sys; print(sys.executable)')
```

Check `pf` too if your organisation enables it (`sudo pfctl -s info`). On Linux,
`sudo iptables -L -n | grep 2222` or `sudo firewall-cmd --list-all`.

**3. Nothing in `tcpdump` either → the robot is not sending.** In order:

* retry with `--multicast`: some targets only produce to a group when the
  connection asks for one. If that works, the report names the group;
* check the robot is really in EtherNet/IP mode, and that **no other scanner
  already owns** the exclusive-owner connection — a second originator gets a
  connection that produces nothing, or a monitoring-only one. The facade raises
  on `RobotStatus_MonitoringMode` for that reason;
* check the return route. The Forward Open travelled over TCP, so IP works in
  both directions, but a robot on another subnet with no gateway configured can
  answer TCP and still fail to send unsolicited UDP. Confirm both ends sit on
  the same subnet, or that the robot has a gateway.

### Running a scanner and the simulated robot on one machine

They cannot both hold UDP 2222. Give the scanner an ephemeral port and let the
Forward Open advertise it — which is what `examples/minimal_session.py --mock`
and the integration tests do:

```python
EtherNetIpTransport.from_io_map(io_map, originator_udp_port=0)
```

This only works because the mock honours the socket address item. Never use it
against hardware that has not been checked with the tool above.

## Open decisions

Everything below is marked `# TODO` in the code:

| Topic | What is needed |
| --- | --- |
| **Motion command ids** | The EDS, the GSDML *and* the ESI all defer them to the programming manual. Fill them into `motion_commands.ids` in the spec; until then `MoveJoints`/`MovePose` raise a message naming that section, while `SendMotionCommand(id, *args)` works today. The simulator uses clearly synthetic stand-ins (90001/90002). |
| **Error codes** | Same story: `RobotStatus_Error` is documented as "refer to the programming manual". The simulator invents codes in the 32000s to stay inside the field. |
| **Digital I/O** | The cyclic assemblies carry none, so `SetOutputState`/`GetRtOutputState` raise `FieldbusUnsupportedFeature`. On this robot they are reachable through the four `DynamicData` slots — whose type codes are, again, manual-only — or through the text API. |
| **Dynamic data slots** | `DynamicTypeCfg0..3` select what the robot publishes in `DynamicData0..3`. Present in the spec, not yet surfaced in the facade. |
| **Socket address item** | Whether the robot honours the T→O socket address item is still unconfirmed on hardware; the scanner listens on 2222 by default so that it does not matter. See [Troubleshooting](#troubleshooting). |
| **Connection watchdog** | The mock does not drop the connection when the scanner stops producing, and never sets `StopMask_ConnectionDropped`. |
| **Joint limits** | The simulator applies a flat ±175°; the real per-joint limits are in the manual. |
| **`mock_robot` package name** | Too generic a top-level name to publish as-is. |

## Conventions

* Python 3.8+, type hints and full docstrings throughout.
* `black` and `isort` (line length 100), `flake8` via `setup.cfg`.
* The facade uses `PascalCase` method names on purpose, to match the vocabulary
  of the robot TCP/IP API. Everything else follows PEP 8.
* No dependency on any vendor library, ever.

## License

MIT.
