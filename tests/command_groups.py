"""Triage of the motion command table, by how risky each command is to send.

The specification lists 48 command identifiers and they are not equally safe:
one changes joint limits, another reboots the robot, several drive a gripper
that may be holding something.  Sending all of them blindly at a robot would be
reckless, so every identifier is placed in exactly one group here, with the
reason, and :mod:`tests.test_hardware_commands` gates the risky groups behind
their own command line option.

This module holds no tests, so that the classification can be checked -- by
:mod:`tests.test_command_groups` -- without a robot anywhere in sight.
"""

from typing import Dict, Tuple

__all__ = [
    "NEVER_SENT",
    "NEEDS_DELIBERATE_SETUP",
    "HARMLESS_COMMANDS",
    "CONFIGURATION_COMMANDS",
    "JOINT_MOTION_COMMANDS",
    "CARTESIAN_COMMANDS",
    "TOOL_COMMANDS",
    "ALL_GROUPS",
]

#: Commands that are never sent by this suite, and why.
NEVER_SENT: Dict[str, str] = {
    "RebootRobot": "reboots the robot",
    "ClearRobotHoming": "clears the homing reference, forcing a drive reinitialisation",
    "StartProgram": "runs an offline program, which can move the robot arbitrarily",
    "SetJointLimits": "changes the joint limits the robot enforces",
    "SetJointLimitsCfg": "enables or disables those limits",
    "SetWorkZoneLimits": "changes the work zone the robot enforces",
    "SetWorkZoneCfg": "changes how work zone breaches are handled",
    "SetCalibrationCfg": "touches the robot calibration",
}

#: Commands that are safe in themselves but meaningless, or unsafe, without a
#: human deciding the values first.
NEEDS_DELIBERATE_SETUP: Dict[str, str] = {
    "SetTrf": "moves the tool frame, silently relocating every later Cartesian target",
    "SetWrf": "moves the world frame, with the same consequence",
    "SetConf": "pins an inverse kinematics configuration that must suit the cell",
    "SetConfTurn": "pins the last joint turn, likewise",
    "SetTorqueLimits": "sets the thresholds that stop the robot on excessive torque",
    "SetTorqueLimitsCfg": "changes how those thresholds are applied",
    "SetCollisionCfg": "changes the collision detection severity",
    "SetToolSphere": "declares the tool geometry used for collision checking",
    "SetPayload": "declares the payload, which must match the real tool",
    "SetMoveMode": "selects a move mode whose arguments the manual does not spell out",
    "SetMoveDurationCfg": "same, and it interacts with the move mode",
    "SetMoveDuration": "same",
}

#: Commands with no lasting effect, sent by default.  Values are the six
#: arguments; the robot ignores the ones it does not use.
HARMLESS_COMMANDS: Dict[str, Tuple[float, ...]] = {
    "SetCheckpoint": (1.0,),
    "Delay": (0.2,),
}

#: Settings, each with the value to send and the default to restore.  The
#: percentages are the robot power-on defaults.
CONFIGURATION_COMMANDS: Dict[str, Tuple[Tuple[float, ...], Tuple[float, ...]]] = {
    "SetJointVel": ((25.0,), (100.0,)),
    "SetJointAcc": ((50.0,), (100.0,)),
    "SetCartLinVel": ((100.0,), (1000.0,)),
    "SetCartAngVel": ((45.0,), (300.0,)),
    "SetCartAcc": ((50.0,), (100.0,)),
    "SetJointVelLimit": ((100.0,), (100.0,)),
    "SetBlending": ((50.0,), (100.0,)),
    "SetVelTimeout": ((0.5,), (0.05,)),
    "SetTimeScaling": ((100.0,), (100.0,)),
    "SetAutoConf": ((1.0,), (1.0,)),
    "SetAutoConfTurn": ((1.0,), (1.0,)),
}

#: Joint-space motion commands, exercised by their own tests.
JOINT_MOTION_COMMANDS = ("MoveJoints", "MoveJointsRel", "MoveJointsVel")

#: Cartesian motion commands, exercised by their own tests.
CARTESIAN_COMMANDS = (
    "MovePose",
    "MoveLin",
    "MoveLinRelTrf",
    "MoveLinRelWrf",
    "MoveLinVelWrf",
    "MoveLinVelTrf",
)

#: Gripper and valve commands, which need a tool.
TOOL_COMMANDS: Dict[str, Tuple[float, ...]] = {
    "SetGripperVel": (50.0,),
    "SetGripperForce": (50.0,),
    "SetGripperRange": (0.0, 5.8),
    "GripperAction": (1.0,),
    "MoveGripper": (5.8,),
    "SetValveState": (-1.0, -1.0),
}

#: How far the Cartesian tests move the tool, in millimetres.
CARTESIAN_STEP_MM = 1.0

#: Every group, in the order a reader should meet them.
ALL_GROUPS: Tuple[Tuple[str, frozenset], ...] = (
    ("never sent", frozenset(NEVER_SENT)),
    ("needs deliberate setup", frozenset(NEEDS_DELIBERATE_SETUP)),
    ("harmless", frozenset(HARMLESS_COMMANDS)),
    ("configuration", frozenset(CONFIGURATION_COMMANDS)),
    ("joint motion", frozenset(JOINT_MOTION_COMMANDS)),
    ("Cartesian motion", frozenset(CARTESIAN_COMMANDS)),
    ("tool", frozenset(TOOL_COMMANDS)),
)
