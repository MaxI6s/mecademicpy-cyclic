"""Guard rail on the motion command triage.

Every identifier the specification declares must sit in exactly one of the
groups in :mod:`tests.command_groups`.  When a firmware update adds a command,
this fails until somebody decides whether it is safe to send at a robot -- which
is what keeps :mod:`tests.test_hardware_commands` honest as the robot evolves.

No robot is involved, so this runs in the ordinary suite.
"""

from mecademic_fieldbus.io_map import IoMap

from .command_groups import ALL_GROUPS


def test_every_declared_command_is_classified(io_map: IoMap) -> None:
    """No command in the specification is left untriaged."""
    classified = set()
    for _, group in ALL_GROUPS:
        classified |= set(group)
    missing = set(io_map.motion_command_ids) - classified
    assert missing == set(), "untriaged command(s); place them in tests/command_groups.py"


def test_no_group_invents_a_command(io_map: IoMap) -> None:
    """No group names a command the specification does not declare."""
    declared = set(io_map.motion_command_ids)
    for name, group in ALL_GROUPS:
        unknown = set(group) - declared
        assert unknown == set(), "group {!r} names unknown command(s) {}".format(name, unknown)


def test_groups_do_not_overlap() -> None:
    """A command belongs to exactly one group, so its risk tier is unambiguous."""
    for index, (name, group) in enumerate(ALL_GROUPS):
        for other_name, other in ALL_GROUPS[index + 1 :]:
            shared = set(group) & set(other)
            assert shared == set(), "{} and {} share {}".format(name, other_name, shared)


def test_the_destructive_commands_are_never_sent() -> None:
    """The commands that must never be fired at a robot are named as such."""
    from .command_groups import NEVER_SENT

    for name in ("RebootRobot", "ClearRobotHoming", "StartProgram", "SetJointLimits"):
        assert name in NEVER_SENT, "{} must stay in the never-sent group".format(name)
        assert NEVER_SENT[name], "{} must carry a reason".format(name)
