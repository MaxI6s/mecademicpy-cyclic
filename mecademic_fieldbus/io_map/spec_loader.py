"""Loader for the declarative assembly specifications shipped in ``spec/``.

The bit and word layout of the robot assemblies is described in JSON rather
than in Python.  JSON was preferred over YAML for two reasons: it needs no
third-party dependency, and every language this project may be ported to can
parse it out of the box.  The files under ``io_map/spec/`` are therefore the
single source of truth for the layout; the Python classes are only a thin
typed accessor on top of them.

See ``io_map/spec/assembly_v1.json`` for the format:

.. code-block:: json

    {
      "spec_format_version": 1,
      "assembly_version": "1",
      "byte_order": "little",
      "assemblies": {
        "input": {
          "instance": 100,
          "size_bytes": 64,
          "fields": [
            {"name": "activation_state", "type": "bool",
             "byte_offset": 0, "bit_offset": 0}
          ]
        }
      }
    }
"""

import json
import os
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from dataclasses import fields
from typing import Any, Dict, List, Optional

from ..exceptions import FieldbusSpecError
from .codec import AssemblyCodec, FieldSpec

__all__ = [
    "AssemblySpec",
    "ConnectionProfile",
    "SPEC_DIRECTORY",
    "load_spec",
    "load_spec_file",
    "parse_spec",
]

#: Directory holding the declarative specifications shipped with the package.
SPEC_DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spec")

#: Version of the *spec file format* understood by this loader.  This is not
#: the version of the assembly layout, which is carried by ``assembly_version``.
SUPPORTED_SPEC_FORMAT_VERSION = 1

#: Names of the assemblies expected in every specification file.
_INPUT = "input"
_OUTPUT = "output"
_CONFIG = "config"


@dataclass(frozen=True)
class ConnectionProfile:
    """Connection parameters the robot advertises in its EDS.

    These describe how a scanner must open the Class 1 connection.  They are
    *not* offsets: an application may read them and hand them to a transport,
    which is how the assembly geometry stays declared in a single place.

    Attributes:
        connection_path: Connection path of the exclusive-owner connection, as
            the space separated hexadecimal bytes the EDS uses.
        network_connection_parameters: Raw capability word of the EDS entry,
            kept for reference.
        output_run_idle_header: Whether the scanner must prepend a 32 bit
            run/idle header to the data it produces.
        input_run_idle_header: Whether the robot prepends one to the data it
            produces.
        rpi_microseconds_min: Shortest requested packet interval accepted.
        rpi_microseconds_max: Longest requested packet interval accepted.
        rpi_microseconds_default: Interval the vendor file recommends.
        vendor_id: CIP vendor identifier of the robot.
        product_type: CIP product type of the robot.
        product_code: CIP product code of the robot.
        major_revision: Major firmware revision the file describes.
        minor_revision: Minor firmware revision the file describes.
    """

    connection_path: str = ""
    network_connection_parameters: str = ""
    output_run_idle_header: bool = True
    input_run_idle_header: bool = False
    rpi_microseconds_min: int = 10000
    rpi_microseconds_max: int = 10000000
    rpi_microseconds_default: int = 10000
    vendor_id: int = 0
    product_type: int = 0
    product_code: int = 0
    major_revision: int = 0
    minor_revision: int = 0


@dataclass(frozen=True)
class AssemblySpec:
    """A fully parsed and validated assembly specification.

    Attributes:
        assembly_version: Version of the assembly layout, as declared in the
            spec file.  This is the value exposed by ``IoMap.version``.
        description: Free form documentation of this layout version.
        byte_order: ``"little"`` or ``"big"``.
        input: Codec for the input assembly (robot to scanner).
        output: Codec for the output assembly (scanner to robot).
        config: Codec for the configuration assembly, possibly empty.
        connection: Connection parameters advertised by the vendor file.
        motion_commands: Identifiers accepted in the ``MotionCommand`` field,
            keyed by command name.  Empty until filled in from the programming
            manual, which is the only place that documents them.
        source: Provenance of the layout: vendor file, revision, firmware.
        source_path: Path of the file this specification was loaded from.
    """

    assembly_version: str
    description: str
    byte_order: str
    input: AssemblyCodec
    output: AssemblyCodec
    config: AssemblyCodec
    connection: ConnectionProfile = ConnectionProfile()
    motion_commands: Dict[str, int] = dataclass_field(default_factory=dict)
    source: Dict[str, str] = dataclass_field(default_factory=dict)
    source_path: Optional[str] = None


def load_spec(version: str) -> AssemblySpec:
    """Load the specification shipped for a given assembly version.

    Args:
        version: Assembly layout version, for example ``"1"``.

    Returns:
        The parsed specification.

    Raises:
        FieldbusSpecError: If no specification exists for this version, or if
            the file is invalid.
    """
    path = os.path.join(SPEC_DIRECTORY, "assembly_v{}.json".format(version))
    if not os.path.isfile(path):
        raise FieldbusSpecError(
            "no assembly specification for version {!r} (looked for {})".format(version, path)
        )
    return load_spec_file(path)


def load_spec_file(path: str) -> AssemblySpec:
    """Load and validate a specification file.

    Args:
        path: Path of the JSON specification file.

    Returns:
        The parsed specification.

    Raises:
        FieldbusSpecError: If the file cannot be read, is not valid JSON, or
            describes an inconsistent layout.
    """
    try:
        with open(path, "r") as handle:
            document = json.load(handle)
    except OSError as exc:
        raise FieldbusSpecError("cannot read assembly specification {}: {}".format(path, exc))
    except ValueError as exc:
        raise FieldbusSpecError("invalid JSON in assembly specification {}: {}".format(path, exc))
    return parse_spec(document, source_path=path)


def parse_spec(document: Dict[str, Any], source_path: Optional[str] = None) -> AssemblySpec:
    """Validate an already decoded specification document.

    Args:
        document: Decoded JSON document.
        source_path: Path the document was read from, for error messages.

    Returns:
        The parsed specification.

    Raises:
        FieldbusSpecError: If the document is inconsistent.
    """
    format_version = document.get("spec_format_version")
    if format_version != SUPPORTED_SPEC_FORMAT_VERSION:
        raise FieldbusSpecError(
            "unsupported spec_format_version {!r}, this loader understands {!r}".format(
                format_version, SUPPORTED_SPEC_FORMAT_VERSION
            )
        )
    assembly_version = document.get("assembly_version")
    if not isinstance(assembly_version, str):
        raise FieldbusSpecError("missing or invalid 'assembly_version'")
    byte_order = document.get("byte_order", "little")
    assemblies = document.get("assemblies")
    if not isinstance(assemblies, dict):
        raise FieldbusSpecError("missing or invalid 'assemblies' section")

    codecs = {}
    for name in (_INPUT, _OUTPUT, _CONFIG):
        if name not in assemblies:
            raise FieldbusSpecError("missing assembly {!r} in specification".format(name))
        codecs[name] = _parse_assembly(name, assemblies[name], byte_order)

    return AssemblySpec(
        assembly_version=assembly_version,
        description=document.get("description", ""),
        byte_order=byte_order,
        input=codecs[_INPUT],
        output=codecs[_OUTPUT],
        config=codecs[_CONFIG],
        connection=_parse_connection(document.get("connection", {})),
        motion_commands=_parse_motion_commands(document.get("motion_commands", {})),
        source=dict(document.get("source", {})),
        source_path=source_path,
    )


def _parse_connection(document: Dict[str, Any]) -> ConnectionProfile:
    """Build a :class:`ConnectionProfile` from its JSON description.

    Args:
        document: Decoded ``connection`` section, possibly empty.

    Returns:
        The profile, falling back to the CIP defaults for missing keys.

    Raises:
        FieldbusSpecError: If a declared value has the wrong type.
    """
    if not isinstance(document, dict):
        raise FieldbusSpecError("'connection' must be an object")
    known = {field.name for field in fields(ConnectionProfile)}
    unknown = set(document) - known
    if unknown:
        raise FieldbusSpecError(
            "unknown key(s) in 'connection': {}".format(", ".join(sorted(unknown)))
        )
    try:
        return ConnectionProfile(**document)
    except TypeError as exc:
        raise FieldbusSpecError("invalid 'connection' section: {}".format(exc))


def _parse_motion_commands(document: Dict[str, Any]) -> Dict[str, int]:
    """Extract the motion command identifiers from their JSON description.

    Args:
        document: Decoded ``motion_commands`` section, possibly empty.

    Returns:
        A mapping of command name to identifier.

    Raises:
        FieldbusSpecError: If an identifier is not an integer.
    """
    if not isinstance(document, dict):
        raise FieldbusSpecError("'motion_commands' must be an object")
    ids = document.get("ids", {})
    if not isinstance(ids, dict):
        raise FieldbusSpecError("'motion_commands.ids' must be an object")
    commands = {}
    for name, value in ids.items():
        if not isinstance(value, int) or isinstance(value, bool):
            raise FieldbusSpecError(
                "motion command {!r} must map to an integer, got {!r}".format(name, value)
            )
        commands[str(name)] = value
    return commands


def _parse_assembly(name: str, document: Dict[str, Any], byte_order: str) -> AssemblyCodec:
    """Build one :class:`AssemblyCodec` from its JSON description.

    Args:
        name: Name of the assembly.
        document: Decoded JSON description of the assembly.
        byte_order: Byte order inherited from the specification.

    Returns:
        The corresponding codec.

    Raises:
        FieldbusSpecError: If the description is inconsistent.
    """
    if not isinstance(document, dict):
        raise FieldbusSpecError("assembly {!r}: expected an object".format(name))
    try:
        raw_instance = document["instance"]
        # A null instance means the robot connection path carries no such
        # assembly, which is how the Meca500 declares its configuration one.
        instance = None if raw_instance is None else int(raw_instance)
        size_bytes = int(document["size_bytes"])
    except (KeyError, TypeError, ValueError) as exc:
        raise FieldbusSpecError(
            "assembly {!r}: missing or invalid 'instance'/'size_bytes' ({})".format(name, exc)
        )
    raw_fields = document.get("fields", [])
    if not isinstance(raw_fields, list):
        raise FieldbusSpecError("assembly {!r}: 'fields' must be a list".format(name))

    fields: List[FieldSpec] = []
    for raw_field in raw_fields:
        fields.append(_parse_field(name, raw_field))

    return AssemblyCodec(
        name=name,
        instance=instance,
        size_bytes=size_bytes,
        fields=fields,
        byte_order=byte_order,
        direction=document.get("direction", ""),
        description=document.get("description", ""),
    )


def _parse_field(assembly_name: str, document: Dict[str, Any]) -> FieldSpec:
    """Build one :class:`FieldSpec` from its JSON description.

    Args:
        assembly_name: Name of the enclosing assembly, for error messages.
        document: Decoded JSON description of the field.

    Returns:
        The corresponding field specification.

    Raises:
        FieldbusSpecError: If the description is inconsistent.
    """
    if not isinstance(document, dict):
        raise FieldbusSpecError("assembly {!r}: expected field objects".format(assembly_name))
    try:
        return FieldSpec(
            name=str(document["name"]),
            type=str(document["type"]),
            byte_offset=int(document["byte_offset"]),
            bit_offset=int(document.get("bit_offset", 0)),
            count=int(document.get("count", 1)),
            description=str(document.get("description", "")),
            unit=document.get("unit"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise FieldbusSpecError(
            "assembly {!r}: invalid field description {!r} ({})".format(
                assembly_name, document, exc
            )
        )
