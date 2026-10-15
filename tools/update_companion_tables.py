"""Merge every hand-maintained companion table into an assembly specification.

The bit layout of the assemblies comes from the vendor EDS and is regenerated
by ``tools/eds_to_spec.py``.  Three things do not, because no vendor definition
file publishes them -- not the EDS, not the GSDML, not the ESI:

* the **motion command identifiers** (``tools/motion_commands_v1.json``);
* the **dynamic data type identifiers** (``tools/dynamic_data_types_v1.json``);
* the **documentation of the individual bits** of the CIP bit strings, whose
  help text the EDS keeps in comments the parser strips
  (``tools/field_docs_v1.json``).

All three are transcribed from the programming manual and the GSDML, and merged
into the one specification file.  ``eds_to_spec.py`` does it when it
regenerates everything; this script does it alone, which is what you want when
a table improves but the layout has not moved -- and it needs no EDS file.

.. code-block:: shell

    python tools/update_companion_tables.py
"""

import argparse
import collections
import json
import os
import sys
from typing import Any, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.eds_to_spec import (  # noqa: E402
    DEFAULT_DYNAMIC_DATA_TYPES,
    DEFAULT_FIELD_DOCS,
    DEFAULT_MOTION_COMMANDS,
    apply_field_docs,
    load_field_docs,
    load_motion_commands,
    report_field_docs,
)

#: Specification updated when no other path is given.
DEFAULT_SPEC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "mecademic_fieldbus",
    "io_map",
    "spec",
    "assembly_v1.json",
)


def merge(
    spec_path: str, motion_commands: str, dynamic_data_types: str, field_docs: str
) -> Dict[str, Any]:
    """Apply every companion table to a specification.

    Only the tables and the ``description``/``unit`` of each field are touched,
    so no offset can move.  The golden byte vectors are the proof.

    Args:
        spec_path: Specification file to update.
        motion_commands: Motion command table to merge, or ``""`` to skip.
        dynamic_data_types: Dynamic data type table to merge, or ``""``.
        field_docs: Per-field documentation to merge, or ``""``.

    Returns:
        The updated document.

    Raises:
        ValueError: If a file is missing, not valid JSON, or malformed.
    """
    try:
        with open(spec_path, "r") as handle:
            spec = json.load(handle, object_pairs_hook=collections.OrderedDict)
    except OSError as error:
        raise ValueError("cannot read {}: {}".format(spec_path, error))
    except ValueError as error:
        raise ValueError("invalid JSON in {}: {}".format(spec_path, error))

    if motion_commands:
        spec["motion_commands"] = load_motion_commands(motion_commands)
    if dynamic_data_types:
        spec["dynamic_data_types"] = load_motion_commands(dynamic_data_types)
    if field_docs:
        docs = load_field_docs(field_docs)
        used: set = set()
        undocumented: List[str] = []
        for assembly in spec.get("assemblies", {}).values():
            assembly_used, assembly_undocumented = apply_field_docs(
                assembly.get("fields", []), docs
            )
            used |= assembly_used
            undocumented.extend(assembly_undocumented)
        report_field_docs(docs, used, undocumented)
    return spec


def main(argv: Optional[List[str]] = None) -> int:
    """Update a specification with the current companion tables.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--spec", default=DEFAULT_SPEC, help="specification to update (default: %(default)s)"
    )
    parser.add_argument(
        "--motion-commands",
        default=DEFAULT_MOTION_COMMANDS,
        help="motion command table (default: %(default)s); empty string to skip",
    )
    parser.add_argument(
        "--dynamic-data-types",
        default=DEFAULT_DYNAMIC_DATA_TYPES,
        help="dynamic data type table (default: %(default)s); empty string to skip",
    )
    parser.add_argument(
        "--field-docs",
        default=DEFAULT_FIELD_DOCS,
        help="per-field documentation (default: %(default)s); empty string to skip",
    )
    args = parser.parse_args(argv)

    try:
        spec = merge(args.spec, args.motion_commands, args.dynamic_data_types, args.field_docs)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 1

    with open(args.spec, "w") as handle:
        json.dump(spec, handle, indent=2)
        handle.write("\n")
    print(
        "merged into {}: {} motion command(s), {} dynamic data type(s), "
        "{} documented field(s)".format(
            args.spec,
            len(spec.get("motion_commands", {}).get("ids", {})),
            len(spec.get("dynamic_data_types", {}).get("ids", {})),
            sum(
                1
                for assembly in spec.get("assemblies", {}).values()
                for field in assembly.get("fields", [])
                if field.get("description")
            ),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
