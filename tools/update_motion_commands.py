"""Merge the motion command table into an assembly specification.

The bit layout of the assemblies comes from the vendor EDS and is regenerated
by ``tools/eds_to_spec.py``.  The motion command identifiers do not: no vendor
definition file publishes them, so they live in a hand-maintained companion
table, ``tools/motion_commands_v1.json``, transcribed from the programming
manual.

The two are merged into a single specification file, which stays the one source
of truth.  ``eds_to_spec.py`` does the merge when it regenerates everything;
this script does it alone, which is what you want when the command table gains
an entry but the layout has not moved -- and it needs no EDS file to run.

.. code-block:: shell

    python tools/update_motion_commands.py
"""

import argparse
import collections
import json
import os
import sys
from typing import Any, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.eds_to_spec import DEFAULT_MOTION_COMMANDS, load_motion_commands  # noqa: E402

#: Specification updated when no other path is given.
DEFAULT_SPEC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "mecademic_fieldbus",
    "io_map",
    "spec",
    "assembly_v1.json",
)


def merge(spec_path: str, commands_path: str) -> Dict[str, Any]:
    """Replace the ``motion_commands`` section of a specification.

    Args:
        spec_path: Specification file to update.
        commands_path: Companion table to merge in.

    Returns:
        The updated document.

    Raises:
        ValueError: If either file is missing or not valid JSON.
    """
    try:
        with open(spec_path, "r") as handle:
            spec = json.load(handle, object_pairs_hook=collections.OrderedDict)
    except OSError as error:
        raise ValueError("cannot read {}: {}".format(spec_path, error))
    except ValueError as error:
        raise ValueError("invalid JSON in {}: {}".format(spec_path, error))

    spec["motion_commands"] = load_motion_commands(commands_path)
    return spec


def main(argv: Optional[List[str]] = None) -> int:
    """Update a specification with the current command table.

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
        help="companion table to merge in (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    try:
        spec = merge(args.spec, args.motion_commands)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 1

    with open(args.spec, "w") as handle:
        json.dump(spec, handle, indent=2)
        handle.write("\n")
    print(
        "merged {} motion command(s) into {}".format(
            len(spec["motion_commands"].get("ids", {})), args.spec
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
