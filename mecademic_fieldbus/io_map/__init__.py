"""Versioned mapping between raw assembly bytes and named, typed fields.

This package is the only place in the project that knows about bit and word
positions.  The layout itself lives in declarative JSON files under ``spec/``,
so that it can be reused as-is should the project be ported to another
language.

Typical use::

    from mecademic_fieldbus.io_map import get_io_map

    io_map = get_io_map()          # latest known version
    status = io_map.decode_status(raw_input_assembly_bytes)

The shipped layout is generated from the official vendor EDS by
``tools/eds_to_spec.py``; re-run it against a newer firmware file rather than
editing the JSON by hand.
"""

from typing import Dict, Optional, Type

from ..exceptions import FieldbusSpecError
from .base import IoMap
from .codec import AssemblyCodec, FieldSpec
from .spec_loader import AssemblySpec, ConnectionProfile, load_spec, load_spec_file, parse_spec
from .v1 import IoMapV1

__all__ = [
    "IoMap",
    "IoMapV1",
    "AssemblyCodec",
    "AssemblySpec",
    "ConnectionProfile",
    "FieldSpec",
    "load_spec",
    "load_spec_file",
    "parse_spec",
    "get_io_map",
    "available_versions",
    "LATEST_VERSION",
]

#: Every I/O map version known to this release, keyed by assembly version.
_IO_MAP_CLASSES: Dict[str, Type[IoMap]] = {IoMapV1.SPEC_VERSION: IoMapV1}

#: Most recent assembly version supported by this release.
LATEST_VERSION = IoMapV1.SPEC_VERSION


def available_versions() -> Dict[str, Type[IoMap]]:
    """Return every assembly version supported by this release.

    Returns:
        A mapping of assembly version to the class implementing it.
    """
    return dict(_IO_MAP_CLASSES)


def get_io_map(version: Optional[str] = None) -> IoMap:
    """Instantiate the I/O map for a given assembly version.

    Args:
        version: Assembly version to load.  Defaults to :data:`LATEST_VERSION`.

    Returns:
        A ready to use I/O map.

    Raises:
        FieldbusSpecError: If no implementation exists for this version.
    """
    key = LATEST_VERSION if version is None else version
    try:
        io_map_class = _IO_MAP_CLASSES[key]
    except KeyError:
        raise FieldbusSpecError(
            "unsupported assembly version {!r}, known versions: {}".format(
                key, ", ".join(sorted(_IO_MAP_CLASSES))
            )
        )
    return io_map_class()
