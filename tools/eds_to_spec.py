"""Generate an I/O map specification from a vendor EDS file.

The declarative specification under ``mecademic_fieldbus/io_map/spec/`` is the
single source of truth for the assembly layout.  Rather than transcribing it by
hand from the EDS -- error prone, and impossible to audit -- this script derives
it mechanically, so a new firmware release is a matter of re-running:

.. code-block:: shell

    python tools/eds_to_spec.py path/to/Meca500_vX.eds \\
        --output mecademic_fieldbus/io_map/spec/assembly_v1.json \\
        --assembly-version 1

What it reads from the EDS:

* ``[Device]`` -- vendor, product and revision, i.e. the electronic key a
  scanner may present in its Forward Open request.
* ``[Params]`` -- the name, CIP type, size, unit and help text of every field,
  plus the ``EnumNNN`` entries that name the individual bits of a bit field.
* ``[Assembly]`` -- the ordered member list of each assembly, from which the
  byte and bit offsets are computed.
* ``[Connection Manager]`` -- the connection path, sizes and RPI limits the
  robot accepts.

Bit fields are expanded into one boolean field per named bit; members named
``Reserved`` and bits named ``Unused`` are dropped, since a field that carries
no meaning has no place in a logical mapping.
"""

import argparse
import collections
import json
import os
import re
import sys
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

#: CIP elementary data types, mapped to the type names used by the codec.
CIP_TYPE_NAMES = {
    0xC1: "bool",
    0xC2: "int8",
    0xC3: "int16",
    0xC4: "int32",
    0xC5: "int64",
    0xC6: "uint8",
    0xC7: "uint16",
    0xC8: "uint32",
    0xC9: "uint64",
    0xCA: "float32",
    0xCB: "float64",
}

#: CIP bit string types, which the EDS pairs with an ``EnumNNN`` entry.
CIP_BITSTRING_TYPES = {0xD1: 8, 0xD2: 16, 0xD3: 32, 0xD4: 64}

#: Member and bit names that carry no meaning and are dropped from the spec.
MEANINGLESS_NAMES = frozenset({"Reserved", "Unused", ""})

#: EDS field positions inside a ``ParamNNN`` entry, once split on commas.
_PARAM_DESCRIPTOR = 3
_PARAM_DATA_TYPE = 4
_PARAM_DATA_SIZE = 5
_PARAM_NAME = 6
_PARAM_UNITS = 7
_PARAM_HELP = 8
_PARAM_MIN = 9
_PARAM_MAX = 10
_PARAM_DEFAULT = 11

#: EDS field position where the member list of an ``AssemN`` entry starts.
_ASSEMBLY_FIRST_MEMBER = 6


def strip_comments(text: str) -> str:
    """Remove the ``$`` comments of an EDS file, leaving quoted text alone.

    Args:
        text: Raw content of the EDS file.

    Returns:
        The same text with every comment removed.
    """
    lines = []
    for line in text.split("\n"):
        kept: List[str] = []
        in_string = False
        for character in line:
            if character == '"':
                in_string = not in_string
            if character == "$" and not in_string:
                break
            kept.append(character)
        lines.append("".join(kept))
    return "\n".join(lines)


def split_entry(body: str) -> List[str]:
    """Split an EDS entry on the commas that are not inside a quoted string.

    Args:
        body: Body of an EDS entry, without its trailing semicolon.

    Returns:
        The comma separated fields, unquoted and stripped.
    """
    fields: List[str] = []
    current: List[str] = []
    in_string = False
    for character in body:
        if character == '"':
            in_string = not in_string
            continue
        if character == "," and not in_string:
            fields.append("".join(current).strip())
            current = []
            continue
        current.append(character)
    fields.append("".join(current).strip())
    return fields


def parse_number(text: str) -> Optional[int]:
    """Parse an EDS integer, in decimal or hexadecimal notation.

    Args:
        text: Text to parse.

    Returns:
        The value, or ``None`` when the field is empty or not a number.
    """
    text = text.strip()
    if not text:
        return None
    try:
        return int(text, 16) if text.lower().startswith("0x") else int(text)
    except ValueError:
        return None


def _split_sections(text: str) -> List[Tuple[str, str]]:
    """Split an EDS file into its bracketed sections.

    Args:
        text: EDS content, comments already removed.

    Returns:
        A list of ``(section name, section body)`` pairs, in file order.
    """
    sections: List[Tuple[str, str]] = []
    matches = list(re.finditer(r"^\s*\[([^\]]+)\]\s*$", text, re.M))
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        sections.append((match.group(1).strip(), text[match.end() : end]))
    return sections


class EdsFile:
    """A parsed EDS file.

    Args:
        text: Raw content of the EDS file.

    Attributes:
        entries: Every ``key = value`` entry of the file, keyed by name.
        params: The ``ParamNNN`` entries, keyed by name.
        enums: The ``EnumNNN`` entries, keyed by the name of the parameter they
            describe, each mapping a bit index to a bit name.
        assemblies: The ``AssemN`` entries, keyed by name.
    """

    def __init__(self, text: str) -> None:
        self.raw_text = text
        self.sections: Dict[str, Dict[str, str]] = {}
        self.entries: Dict[str, str] = {}
        for section, body in _split_sections(strip_comments(text)):
            entries = {
                match.group(1): match.group(2)
                for match in re.finditer(r"^\s*(\w+)\s*=\s*(.*?);", body, re.S | re.M)
            }
            self.sections[section] = entries
            # Several sections reuse the same key names -- "Revision" and
            # "Object_Name" for instance -- so the flat view keeps the first
            # occurrence and section lookups stay authoritative.
            for key, value in entries.items():
                self.entries.setdefault(key, value)

        self.params: Dict[str, Dict[str, Any]] = {}
        self.enums: Dict[str, Dict[int, str]] = {}
        self.assemblies: Dict[str, Dict[str, Any]] = {}
        for key, body in self.entries.items():
            if re.fullmatch(r"Param\d+", key):
                self.params[key] = self._parse_param(body)
            elif re.fullmatch(r"Enum\d+", key):
                self.enums["Param" + key[len("Enum") :]] = self._parse_enum(body)
            elif re.fullmatch(r"Assem\d+", key):
                self.assemblies[key] = self._parse_assembly(body)

    @staticmethod
    def _parse_param(body: str) -> Dict[str, Any]:
        """Parse one ``ParamNNN`` entry.

        Args:
            body: Body of the entry.

        Returns:
            The parameter description.
        """
        fields = split_entry(body)
        return {
            "descriptor": parse_number(fields[_PARAM_DESCRIPTOR]),
            "type_code": parse_number(fields[_PARAM_DATA_TYPE]),
            "size_bytes": parse_number(fields[_PARAM_DATA_SIZE]),
            "name": fields[_PARAM_NAME],
            "units": fields[_PARAM_UNITS],
            "help": fields[_PARAM_HELP],
            "min": parse_number(fields[_PARAM_MIN]),
            "max": parse_number(fields[_PARAM_MAX]),
            "default": parse_number(fields[_PARAM_DEFAULT]),
        }

    @staticmethod
    def _parse_enum(body: str) -> Dict[int, str]:
        """Parse one ``EnumNNN`` entry, which names the bits of a bit field.

        Args:
            body: Body of the entry.

        Returns:
            A mapping of bit index to bit name.
        """
        fields = split_entry(body)
        bits: Dict[int, str] = {}
        for index in range(0, len(fields) - 1, 2):
            bit = parse_number(fields[index])
            if bit is not None:
                bits[bit] = fields[index + 1]
        return bits

    @staticmethod
    def _parse_assembly(body: str) -> Dict[str, Any]:
        """Parse one ``AssemN`` entry.

        Args:
            body: Body of the entry.

        Returns:
            The assembly name and its ordered ``(bit size, parameter)`` members.
        """
        fields = split_entry(body)
        members: List[Tuple[int, str]] = []
        rest = fields[_ASSEMBLY_FIRST_MEMBER:]
        index = 0
        while index + 1 < len(rest):
            size = parse_number(rest[index])
            if size is None:
                index += 1
                continue
            members.append((size, rest[index + 1]))
            index += 2
        return {"name": fields[0], "members": members}

    def device(self) -> Dict[str, Any]:
        """Return the identity of the device described by the file.

        Returns:
            Vendor, product and revision information.
        """
        return {
            "vendor_id": parse_number(self.entries.get("VendCode", "")),
            "vendor_name": split_entry(self.entries.get("VendName", ""))[0],
            "product_type": parse_number(self.entries.get("ProdType", "")),
            "product_code": parse_number(self.entries.get("ProdCode", "")),
            "major_revision": parse_number(self.entries.get("MajRev", "")),
            "minor_revision": parse_number(self.entries.get("MinRev", "")),
            "product_name": split_entry(self.entries.get("ProdName", ""))[0],
            "eds_revision": self.sections.get("File", {}).get("Revision", "").strip(),
        }

    def connection(self, name: str = "Connection1") -> Dict[str, Any]:
        """Return the description of one entry of the Connection Manager section.

        Args:
            name: Name of the connection entry, ``Connection1`` by default,
                which is the exclusive-owner connection used to control the
                robot.

        Returns:
            The connection sizes, path and real-time transfer formats.

        Raises:
            KeyError: If the file has no such connection.
        """
        fields = split_entry(self.sections["Connection Manager"][name])
        return {
            "trigger_and_transport": parse_number(fields[0]),
            "network_connection_parameters": parse_number(fields[1]),
            "output_size_bytes": parse_number(fields[3]),
            "input_size_bytes": parse_number(fields[6]),
            "connection_path": fields[-1].strip(),
            "output_run_idle_header": self._run_idle_header("O->T"),
            "input_run_idle_header": self._run_idle_header("T->O"),
        }

    def _run_idle_header(self, direction: str) -> bool:
        """Tell whether a direction carries a 32 bit run/idle header.

        The network connection parameters word of an EDS ``Connection`` entry
        is a capability mask whose encoding EZ-EDS does not document in the
        file itself; the generated comments next to it do state the negotiated
        real-time transfer format, so they are what is read here.

        Args:
            direction: ``"O->T"`` or ``"T->O"``.

        Returns:
            ``True`` when that direction uses a 32 bit run/idle header.  Falls
            back to the CIP convention for an exclusive-owner connection --
            header on the way out, modeless on the way back -- when the file
            says nothing.
        """
        pattern = re.escape(direction) + r"\s+Real time transfer format\s*=\s*([^\r\n]*)"
        match = re.search(pattern, self.raw_text)
        if match is None:
            return direction == "O->T"
        return "run/idle" in match.group(1).lower()


def connection_points(path: str) -> List[int]:
    """Extract the connection point instances of an EDS connection path.

    Args:
        path: Connection path, as the space separated hexadecimal bytes the EDS
            uses, for example ``"20 04 2C 96 2C 64"``.

    Returns:
        The connection point instances, in path order: output then input.
    """
    tokens = [int(token, 16) for token in path.split()]
    points: List[int] = []
    index = 0
    while index < len(tokens):
        segment = tokens[index]
        if segment == 0x2C and index + 1 < len(tokens):
            points.append(tokens[index + 1])
            index += 2
        elif segment in (0x20, 0x24, 0x30) and index + 1 < len(tokens):
            index += 2
        else:
            index += 1
    return points


def build_fields(eds: EdsFile, members: Sequence[Tuple[int, str]]) -> List[Dict[str, Any]]:
    """Convert the member list of an assembly into specification fields.

    Args:
        eds: Parsed EDS file.
        members: Ordered ``(bit size, parameter name)`` members.

    Returns:
        The fields, each with its computed offsets.

    Raises:
        ValueError: If a member has a type the codec cannot represent, or a
            size that contradicts its parameter definition.
    """
    fields: List[Dict[str, Any]] = []
    bit_position = 0
    for bit_size, param_name in members:
        param = eds.params[param_name]
        name = param["name"]
        type_code = param["type_code"]

        if name in MEANINGLESS_NAMES:
            bit_position += bit_size
            continue
        if bit_position % 8:
            raise ValueError("member {!r} is not byte aligned".format(name))
        byte_offset = bit_position // 8

        if type_code in CIP_BITSTRING_TYPES:
            fields.extend(_expand_bit_field(eds, param_name, name, byte_offset, bit_size))
        elif type_code in CIP_TYPE_NAMES:
            if param["size_bytes"] * 8 != bit_size:
                raise ValueError(
                    "member {!r} occupies {} bits but its parameter declares {}".format(
                        name, bit_size, param["size_bytes"] * 8
                    )
                )
            fields.append(_scalar_field(param, name, byte_offset))
        else:
            raise ValueError(
                "member {!r} has unsupported CIP type 0x{:02X}".format(name, type_code or 0)
            )
        bit_position += bit_size
    return fields


def _scalar_field(param: Dict[str, Any], name: str, byte_offset: int) -> Dict[str, Any]:
    """Build the specification of one scalar field.

    Args:
        param: Parameter description from the EDS.
        name: Field name.
        byte_offset: Offset of the field inside the assembly.

    Returns:
        The field description.
    """
    field = collections.OrderedDict(
        [
            ("name", name),
            ("type", CIP_TYPE_NAMES[param["type_code"]]),
            ("byte_offset", byte_offset),
        ]
    )
    if param["units"]:
        field["unit"] = param["units"]
    if param["help"]:
        field["description"] = param["help"]
    return field


def _expand_bit_field(
    eds: EdsFile, param_name: str, name: str, byte_offset: int, bit_size: int
) -> List[Dict[str, Any]]:
    """Expand a CIP bit string into one boolean field per named bit.

    Args:
        eds: Parsed EDS file.
        param_name: Name of the EDS parameter holding the bit string.
        name: Name of the bit string, used when a bit has no name of its own.
        byte_offset: Offset of the bit string inside the assembly.
        bit_size: Width of the bit string, in bits.

    Returns:
        One field per named bit, dropping the unused ones.
    """
    bits = eds.enums.get(param_name, {})
    fields: List[Dict[str, Any]] = []
    for bit, bit_name in sorted(bits.items()):
        if bit_name in MEANINGLESS_NAMES:
            continue
        if bit >= bit_size:
            raise ValueError(
                "bit {} of {!r} falls outside its {} bit field".format(bit, name, bit_size)
            )
        field = collections.OrderedDict(
            [
                ("name", bit_name),
                ("type", "bool"),
                ("byte_offset", byte_offset + bit // 8),
                ("bit_offset", bit % 8),
            ]
        )
        fields.append(field)
    return fields


#: Hand-maintained companion table merged into every generated specification.
#: The motion command identifiers are the one part of the protocol no vendor
#: definition file publishes, so they cannot be derived from the EDS.
DEFAULT_MOTION_COMMANDS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "motion_commands_v1.json"
)


def load_motion_commands(path: Optional[str]) -> Dict[str, Any]:
    """Load the companion table of motion command identifiers.

    Args:
        path: File to read, or ``None`` to skip the merge.

    Returns:
        The section to place under ``motion_commands`` in the specification.

    Raises:
        ValueError: If the file is unreadable or not valid JSON.
    """
    if path is None:
        return collections.OrderedDict(
            [
                (
                    "description",
                    "Identifiers accepted in the MotionCommand field of the output "
                    "assembly, keyed by command name. No vendor definition file "
                    "publishes them; see tools/motion_commands_v1.json.",
                ),
                ("ids", collections.OrderedDict()),
            ]
        )
    try:
        with open(path, "r") as handle:
            return json.load(handle, object_pairs_hook=collections.OrderedDict)
    except OSError as error:
        raise ValueError("cannot read {}: {}".format(path, error))
    except ValueError as error:
        raise ValueError("invalid JSON in {}: {}".format(path, error))


#: Hand-maintained companion table of dynamic data type identifiers, which no
#: vendor definition file publishes either.
DEFAULT_DYNAMIC_DATA_TYPES = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "dynamic_data_types_v1.json"
)

#: Hand-maintained companion table of per-field documentation.  The EDS keeps
#: the help text of its CIP bit strings in comments this parser strips, so the
#: boolean fields would otherwise come out undocumented.
DEFAULT_FIELD_DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "field_docs_v1.json")


def load_field_docs(path: Optional[str]) -> Dict[str, Any]:
    """Load the companion table of per-field documentation.

    Args:
        path: File to read, or ``None`` to skip the merge.

    Returns:
        A mapping of field name to the keys to apply, empty when skipped.

    Raises:
        ValueError: If the file is unreadable, not valid JSON, or malformed.
    """
    if path is None:
        return {}
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except OSError as error:
        raise ValueError("cannot read {}: {}".format(path, error))
    except ValueError as error:
        raise ValueError("invalid JSON in {}: {}".format(path, error))
    fields = document.get("fields", {})
    if not isinstance(fields, dict):
        raise ValueError("{}: 'fields' must be an object".format(path))
    return fields


def apply_field_docs(
    fields: List[Dict[str, Any]], docs: Dict[str, Any]
) -> Tuple[Set[str], List[str]]:
    """Merge the companion documentation into one assembly's fields.

    The companion table wins over whatever the EDS said, because it is the
    curated source.  Only ``description`` and ``unit`` are applied, which keeps
    this a 1:1 projection onto the field specification and means documentation
    can never move an offset.

    Args:
        fields: Field descriptions of one assembly, updated in place.
        docs: Companion table, keyed by field name.

    Returns:
        A tuple ``(names used, names still undocumented)``.
    """
    used: Set[str] = set()
    undocumented: List[str] = []
    for field in fields:
        entry = docs.get(field["name"])
        if entry:
            used.add(field["name"])
            for key in ("description", "unit"):
                if entry.get(key):
                    field[key] = entry[key]
        if not field.get("description"):
            undocumented.append(field["name"])
    return used, undocumented


def build_spec(
    eds: EdsFile,
    assembly_version: str,
    source_name: str,
    motion_commands: Optional[Dict[str, Any]] = None,
    field_docs: Optional[Dict[str, Any]] = None,
    dynamic_data_types: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Build the complete specification document.

    Args:
        eds: Parsed EDS file.
        assembly_version: Version string to record in the specification.
        source_name: Name of the EDS file, recorded for traceability.
        motion_commands: Companion table of motion command identifiers to merge
            in, as returned by :func:`load_motion_commands`.
        field_docs: Companion table of per-field documentation to merge in, as
            returned by :func:`load_field_docs`.
        dynamic_data_types: Companion table of dynamic data type identifiers.

    Returns:
        The specification, ready to be serialised as JSON.

    Raises:
        ValueError: If the EDS is inconsistent with itself.
    """
    device = eds.device()
    connection = eds.connection()
    points = connection_points(connection["connection_path"])
    if len(points) != 2:
        raise ValueError(
            "expected two connection points in {!r}".format(connection["connection_path"])
        )
    output_instance, input_instance = points

    input_fields = build_fields(eds, eds.assemblies["Assem1"]["members"])
    output_fields = build_fields(eds, eds.assemblies["Assem2"]["members"])

    docs = field_docs or {}
    used_in, undocumented_in = apply_field_docs(input_fields, docs)
    used_out, undocumented_out = apply_field_docs(output_fields, docs)
    report_field_docs(docs, used_in | used_out, undocumented_in + undocumented_out)

    for label, members, declared in (
        ("input", eds.assemblies["Assem1"]["members"], connection["input_size_bytes"]),
        ("output", eds.assemblies["Assem2"]["members"], connection["output_size_bytes"]),
    ):
        actual = sum(bits for bits, _ in members) // 8
        if actual != declared:
            raise ValueError(
                "{} assembly members total {} bytes but the connection declares {}".format(
                    label, actual, declared
                )
            )

    return collections.OrderedDict(
        [
            ("spec_format_version", 1),
            ("assembly_version", assembly_version),
            (
                "description",
                "Assembly layout of the {} robot, generated from {} by "
                "tools/eds_to_spec.py. Do not edit by hand: re-run the generator "
                "against a newer EDS instead.".format(device["product_name"], source_name),
            ),
            (
                "source",
                collections.OrderedDict(
                    [
                        ("file", source_name),
                        ("eds_revision", device["eds_revision"]),
                        ("product_name", device["product_name"]),
                        (
                            "firmware_revision",
                            "{}.{}".format(device["major_revision"], device["minor_revision"]),
                        ),
                    ]
                ),
            ),
            ("byte_order", "little"),
            (
                "connection",
                collections.OrderedDict(
                    [
                        ("connection_path", connection["connection_path"]),
                        (
                            "network_connection_parameters",
                            "0x{:08X}".format(connection["network_connection_parameters"] or 0),
                        ),
                        ("output_run_idle_header", connection["output_run_idle_header"]),
                        ("input_run_idle_header", connection["input_run_idle_header"]),
                        ("rpi_microseconds_min", eds.params["Param11"]["min"]),
                        ("rpi_microseconds_max", eds.params["Param11"]["max"]),
                        ("rpi_microseconds_default", eds.params["Param11"]["default"]),
                        ("vendor_id", device["vendor_id"]),
                        ("product_type", device["product_type"]),
                        ("product_code", device["product_code"]),
                        ("major_revision", device["major_revision"]),
                        ("minor_revision", device["minor_revision"]),
                    ]
                ),
            ),
            ("motion_commands", motion_commands or load_motion_commands(None)),
            ("dynamic_data_types", dynamic_data_types or {"ids": collections.OrderedDict()}),
            (
                "assemblies",
                collections.OrderedDict(
                    [
                        (
                            "input",
                            collections.OrderedDict(
                                [
                                    ("direction", "target_to_originator"),
                                    ("instance", input_instance),
                                    ("size_bytes", connection["input_size_bytes"]),
                                    ("description", "Robot to scanner: cyclic robot state."),
                                    ("fields", input_fields),
                                ]
                            ),
                        ),
                        (
                            "output",
                            collections.OrderedDict(
                                [
                                    ("direction", "originator_to_target"),
                                    ("instance", output_instance),
                                    ("size_bytes", connection["output_size_bytes"]),
                                    ("description", "Scanner to robot: cyclic commands."),
                                    ("fields", output_fields),
                                ]
                            ),
                        ),
                        (
                            "config",
                            collections.OrderedDict(
                                [
                                    ("direction", "originator_to_target"),
                                    ("instance", None),
                                    ("size_bytes", 0),
                                    (
                                        "description",
                                        "The robot connection path carries no configuration "
                                        "assembly; the Forward Open must not include one.",
                                    ),
                                    ("fields", []),
                                ]
                            ),
                        ),
                    ]
                ),
            ),
        ]
    )


def report_field_docs(docs: Dict[str, Any], used: Set[str], undocumented: Sequence[str]) -> None:
    """Warn about a companion table that has drifted from the layout.

    These two warnings are what makes the table survive a regeneration against
    a newer EDS: an entry that matches nothing means a typo or a renamed field,
    and a field with no documentation left means the table needs an entry.

    Args:
        docs: Companion table that was applied.
        used: Field names the table actually matched.
        undocumented: Field names still carrying no description.
    """
    unmatched = sorted(set(docs) - used)
    if unmatched:
        print(
            "warning: {} documentation entr{} matched no field: {}".format(
                len(unmatched), "y" if len(unmatched) == 1 else "ies", ", ".join(unmatched)
            ),
            file=sys.stderr,
        )
    if undocumented:
        print(
            "warning: {} field(s) have no description: {}".format(
                len(undocumented), ", ".join(sorted(undocumented))
            ),
            file=sys.stderr,
        )


def main(argv: Optional[List[str]] = None) -> int:
    """Generate a specification file from an EDS.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("eds", help="path of the vendor EDS file")
    parser.add_argument("--output", "-o", help="path to write; defaults to standard output")
    parser.add_argument(
        "--assembly-version",
        default="1",
        help="assembly version to record in the specification (default: %(default)s)",
    )
    parser.add_argument(
        "--motion-commands",
        default=DEFAULT_MOTION_COMMANDS,
        help="companion table of motion command identifiers to merge in "
        "(default: %(default)s); pass an empty string to leave it out",
    )
    parser.add_argument(
        "--dynamic-data-types",
        default=DEFAULT_DYNAMIC_DATA_TYPES,
        help="companion table of dynamic data type identifiers to merge in "
        "(default: %(default)s); pass an empty string to leave it out",
    )
    parser.add_argument(
        "--field-docs",
        default=DEFAULT_FIELD_DOCS,
        help="companion table of per-field documentation to merge in "
        "(default: %(default)s); pass an empty string to leave it out",
    )
    args = parser.parse_args(argv)

    with open(args.eds, "r", errors="replace") as handle:
        eds = EdsFile(handle.read())
    try:
        commands = load_motion_commands(args.motion_commands or None)
        docs = load_field_docs(args.field_docs or None)
        spec = build_spec(eds, args.assembly_version, os.path.basename(args.eds), commands, docs)
    except (KeyError, ValueError) as error:
        print("cannot convert {}: {}".format(args.eds, error), file=sys.stderr)
        return 1

    document = json.dumps(spec, indent=2) + "\n"
    if args.output:
        with open(args.output, "w") as handle:
            handle.write(document)
        print(
            "wrote {} ({} input fields, {} output fields, {} motion commands)".format(
                args.output,
                len(spec["assemblies"]["input"]["fields"]),
                len(spec["assemblies"]["output"]["fields"]),
                len(spec["motion_commands"].get("ids", {})),
            )
        )
    else:
        sys.stdout.write(document)
    return 0


if __name__ == "__main__":
    sys.exit(main())
