"""Tests of the I/O map layer.

Three kinds of checks live here:

* behavioural tests of the generic codec;
* tests of :class:`IoMapV1` against the real Meca500 layout;
* golden vector tests against ``fixtures/assembly_v1_vectors.json``, which pin
  the exact bytes of a handful of scenarios so that an accidental change to the
  declarative specification is caught immediately.  Those vectors are also the
  conformance reference a port of this project to another language should
  reproduce byte for byte.
"""

import dataclasses
import json
import os

import pytest

from mecademic_fieldbus.exceptions import FieldbusIoMapError, FieldbusSpecError
from mecademic_fieldbus.io_map import IoMap, IoMapV1, get_io_map
from mecademic_fieldbus.io_map.codec import AssemblyCodec, FieldSpec
from mecademic_fieldbus.io_map.spec_loader import load_spec, parse_spec
from mecademic_fieldbus.robot_classes import (
    JOINT_COUNT,
    MOTION_ARGUMENT_COUNT,
    POSE_COUNT,
    InverseKinematicsConfiguration,
    MotionCommand,
    MotionControl,
    MotionStatus,
    RobotControl,
    RobotPosition,
    RobotSafetyStatus,
    RobotStatus,
    SafetyStopFlags,
)

from .conftest import FIXTURES_DIRECTORY


def load_vectors() -> dict:
    """Load the golden assembly vectors.

    Returns:
        The decoded fixture document.
    """
    path = os.path.join(FIXTURES_DIRECTORY, "assembly_v1_vectors.json")
    with open(path, "r") as handle:
        return json.load(handle)


VECTORS = load_vectors()


# ----------------------------------------------------------------------
# Specification loading
# ----------------------------------------------------------------------
def test_shipped_spec_describes_the_meca500(io_map: IoMap) -> None:
    """The shipped specification matches the official EDS it was generated from."""
    assert io_map.version == "1"
    assert io_map.input_assembly_instance == 100
    assert io_map.output_assembly_instance == 150
    assert io_map.input_assembly_size == 252
    assert io_map.output_assembly_size == 60
    # The robot connection path carries no configuration assembly.
    assert io_map.config_assembly_instance is None


def test_shipped_spec_carries_the_connection_profile(io_map: IoMap) -> None:
    """The connection parameters a scanner needs come from the vendor file."""
    profile = io_map.connection
    assert profile.connection_path == "20 04 2C 96 2C 64"
    assert profile.output_run_idle_header is True
    assert profile.input_run_idle_header is False
    assert profile.rpi_microseconds_min == 10000
    assert profile.vendor_id == 1565
    assert profile.product_code == 500


def test_shipped_spec_records_its_provenance() -> None:
    """The specification says which vendor file it was generated from."""
    spec = load_spec("1")
    assert spec.source["product_name"] == "Meca500"
    assert spec.source["file"].endswith(".eds")
    assert spec.source["eds_revision"]


def test_get_io_map_rejects_unknown_version() -> None:
    """Asking for a version that has no implementation fails loudly."""
    with pytest.raises(FieldbusSpecError):
        get_io_map("does-not-exist")


def test_load_spec_rejects_unknown_version() -> None:
    """Asking the loader for a missing specification file fails loudly."""
    with pytest.raises(FieldbusSpecError):
        load_spec("42")


def make_document(input_fields: list) -> dict:
    """Build a minimal specification document around some input fields.

    Args:
        input_fields: Field descriptions for the input assembly.

    Returns:
        A document ready for :func:`parse_spec`.
    """
    return {
        "spec_format_version": 1,
        "assembly_version": "test",
        "byte_order": "little",
        "assemblies": {
            "input": {"instance": 1, "size_bytes": 4, "fields": input_fields},
            "output": {"instance": 2, "size_bytes": 0, "fields": []},
            "config": {"instance": None, "size_bytes": 0, "fields": []},
        },
    }


def test_spec_rejects_overlapping_fields() -> None:
    """Two fields sharing a bit are refused at load time."""
    document = make_document(
        [
            {"name": "a", "type": "uint16", "byte_offset": 0},
            {"name": "b", "type": "bool", "byte_offset": 1, "bit_offset": 3},
        ]
    )
    with pytest.raises(FieldbusSpecError) as error:
        parse_spec(document)
    assert "overlap" in str(error.value)


def test_spec_rejects_field_overflowing_the_assembly() -> None:
    """A field that does not fit the declared size is refused at load time."""
    document = make_document([{"name": "a", "type": "float64", "byte_offset": 0}])
    with pytest.raises(FieldbusSpecError):
        parse_spec(document)


def test_spec_rejects_unknown_format_version() -> None:
    """A specification written for another loader is refused."""
    with pytest.raises(FieldbusSpecError):
        parse_spec({"spec_format_version": 99, "assembly_version": "1", "assemblies": {}})


def test_spec_rejects_unknown_connection_key() -> None:
    """A typo in the connection section is refused rather than ignored."""
    document = make_document([])
    document["connection"] = {"rpi_microsecond_min": 1000}
    with pytest.raises(FieldbusSpecError):
        parse_spec(document)


def test_spec_rejects_non_integer_motion_command_id() -> None:
    """A motion command identifier must be an integer."""
    document = make_document([])
    document["motion_commands"] = {"ids": {"MoveJoints": "one"}}
    with pytest.raises(FieldbusSpecError):
        parse_spec(document)


# ----------------------------------------------------------------------
# Codec behaviour
# ----------------------------------------------------------------------
def make_codec() -> AssemblyCodec:
    """Build a small codec covering every supported shape of field.

    Returns:
        A codec with a bit, a bit array, a scalar and a float array.
    """
    return AssemblyCodec(
        name="test",
        instance=1,
        size_bytes=16,
        fields=[
            FieldSpec(name="flag", type="bool", byte_offset=0, bit_offset=5),
            FieldSpec(name="flags", type="bool", byte_offset=1, bit_offset=0, count=12),
            FieldSpec(name="counter", type="uint16", byte_offset=4),
            FieldSpec(name="values", type="float32", byte_offset=8, count=2),
        ],
    )


def test_codec_round_trips_every_field_shape() -> None:
    """Values written into an image are read back unchanged."""
    codec = make_codec()
    flags = tuple(index % 2 == 0 for index in range(12))
    raw = codec.pack({"flag": True, "flags": flags, "counter": 4242, "values": (1.5, -2.25)})
    assert codec.read(raw, "flag") is True
    assert codec.read(raw, "flags") == flags
    assert codec.read(raw, "counter") == 4242
    assert codec.read(raw, "values") == (1.5, -2.25)


def test_codec_pack_preserves_untouched_fields() -> None:
    """Packing over an existing image only rewrites the listed fields."""
    codec = make_codec()
    first = codec.pack({"counter": 7, "flag": True})
    second = codec.pack({"counter": 9}, base=first)
    assert codec.read(second, "counter") == 9
    assert codec.read(second, "flag") is True


def test_codec_pack_without_base_clears_everything() -> None:
    """Packing without a base starts from an all-zero image."""
    codec = make_codec()
    first = codec.pack({"flag": True})
    second = codec.pack({"counter": 1})
    assert codec.read(first, "flag") is True
    assert codec.read(second, "flag") is False


def test_codec_bits_do_not_leak_into_neighbours() -> None:
    """Clearing one bit leaves the surrounding bits untouched."""
    codec = make_codec()
    raw = codec.pack({"flags": tuple(True for _ in range(12))})
    updated = codec.pack({"flags": (False,) + tuple(True for _ in range(11))}, base=raw)
    assert codec.read(updated, "flags") == (False,) + tuple(True for _ in range(11))


def test_codec_rejects_unknown_field() -> None:
    """Reading a field the assembly does not declare fails loudly."""
    codec = make_codec()
    with pytest.raises(FieldbusIoMapError):
        codec.read(codec.zeros(), "nope")


def test_codec_rejects_short_buffer() -> None:
    """Decoding a truncated assembly fails loudly."""
    codec = make_codec()
    with pytest.raises(FieldbusIoMapError):
        codec.read(bytes(4), "counter")


def test_codec_rejects_wrong_element_count() -> None:
    """Writing an array of the wrong length fails loudly."""
    codec = make_codec()
    with pytest.raises(FieldbusIoMapError):
        codec.pack({"values": (1.0,)})


def test_codec_rejects_out_of_range_value() -> None:
    """Writing a value that does not fit the declared type fails loudly."""
    codec = make_codec()
    with pytest.raises(FieldbusIoMapError):
        codec.pack({"counter": 70000})


# ----------------------------------------------------------------------
# IoMapV1 behaviour
# ----------------------------------------------------------------------
def test_status_round_trip(io_map: IoMap) -> None:
    """A robot status survives an encode/decode cycle."""
    status = RobotStatus(
        busy=True,
        activated=True,
        homed=True,
        simulation_mode=True,
        brakes_engaged=True,
        recovery_mode=True,
        collision=True,
        out_of_work_zone=True,
        monitoring_mode=True,
        error_code=1234,
    )
    assert io_map.decode_status(io_map.encode_status(status)) == status


def test_status_reports_the_error_through_the_code(io_map: IoMap) -> None:
    """The layout has no error bit: a non-zero code is what signals the error."""
    assert io_map.decode_status(io_map.encode_status(RobotStatus())).error_status is False
    decoded = io_map.decode_status(io_map.encode_status(RobotStatus(error_code=1)))
    assert decoded.error_status is True


def test_motion_status_round_trip(io_map: IoMap) -> None:
    """A motion status survives an encode/decode cycle."""
    motion = MotionStatus(
        paused=True,
        end_of_block=True,
        end_of_movement=True,
        cleared=True,
        excessive_torque=True,
        reached_checkpoint_id=11,
        discarded_checkpoint_id=12,
        move_id=13,
        fifo_space=12999,
        offline_program_id=0,
    )
    assert io_map.decode_motion_status(io_map.encode_motion_status(motion)) == motion


def test_safety_status_round_trip(io_map: IoMap) -> None:
    """A safety status, with both of its stop masks, survives a cycle."""
    safety = RobotSafetyStatus(
        stops=SafetyStopFlags(estop=True, connection_dropped=True),
        resettable_stops=SafetyStopFlags(pstop2=True, reboot=True),
        reset_ready=True,
        motor_voltage_on=True,
    )
    assert io_map.decode_safety_status(io_map.encode_safety_status(safety)) == safety


def test_position_round_trip(io_map: IoMap) -> None:
    """A position, joints, pose and configuration, survives a cycle."""
    position = RobotPosition(
        joints=(0.0, 10.5, -20.25, 0.0, 90.0, -45.0),
        pose=(100.0, 50.0, 200.0, 0.0, 90.0, 0.0),
        shoulder=InverseKinematicsConfiguration.POSITIVE,
        elbow=InverseKinematicsConfiguration.NEGATIVE,
        wrist=InverseKinematicsConfiguration.POSITIVE,
        turn=InverseKinematicsConfiguration.UNDEFINED,
    )
    assert io_map.decode_position(io_map.encode_position(position)) == position


def test_robot_control_round_trip(io_map: IoMap) -> None:
    """Robot control bits survive an encode/decode cycle."""
    control = RobotControl(deactivate=True, home=True, enable_recovery_mode=True)
    assert io_map.decode_robot_control(io_map.encode_robot_control(control)) == control


def test_motion_control_round_trip(io_map: IoMap) -> None:
    """Motion control bits and the move id survive an encode/decode cycle."""
    control = MotionControl(move_id=1234, setpoint=True, resume_motion=True)
    assert io_map.decode_motion_control(io_map.encode_motion_control(control)) == control


def test_motion_command_round_trip(io_map: IoMap) -> None:
    """A motion command survives an encode/decode cycle."""
    command = MotionCommand.build(4242, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    assert io_map.decode_motion_command(io_map.encode_motion_command(command)) == command


def test_encoders_compose_without_clobbering_each_other(io_map: IoMap) -> None:
    """Successive encoders build one image instead of overwriting it."""
    raw = io_map.empty_output_assembly()
    raw = io_map.encode_robot_control(RobotControl(activate=True), raw)
    raw = io_map.encode_motion_command(MotionCommand.build(7, (1.0,) * 6), raw)
    raw = io_map.encode_motion_control(MotionControl(move_id=5, setpoint=True), raw)
    assert io_map.decode_robot_control(raw).activate is True
    assert io_map.decode_motion_command(raw).command_id == 7
    assert io_map.decode_motion_control(raw).move_id == 5


def test_empty_vectors_are_encoded_as_zeros(io_map: IoMap) -> None:
    """The neutral command and an empty position encode without raising."""
    decoded = io_map.decode_motion_command(io_map.encode_motion_command(MotionCommand.none()))
    assert decoded.command_id == 0
    assert decoded.arguments == (0.0,) * MOTION_ARGUMENT_COUNT
    position = io_map.decode_position(io_map.encode_position(RobotPosition()))
    assert position.joints == (0.0,) * JOINT_COUNT
    assert position.pose == (0.0,) * POSE_COUNT


def test_vector_of_wrong_size_is_rejected(io_map: IoMap) -> None:
    """A position with the wrong number of joints fails loudly."""
    with pytest.raises(FieldbusIoMapError):
        io_map.encode_position(RobotPosition(joints=(1.0, 2.0)))


def test_motion_command_rejects_too_many_arguments() -> None:
    """A command carrying more arguments than the assembly does fails loudly."""
    with pytest.raises(ValueError):
        MotionCommand.build(1, (0.0,) * (MOTION_ARGUMENT_COUNT + 1))


# ----------------------------------------------------------------------
# Optional and unpublished features
# ----------------------------------------------------------------------
def test_layout_carries_no_digital_io(io_map: IoMap) -> None:
    """The Meca500 cyclic assemblies expose no digital inputs or outputs."""
    from mecademic_fieldbus.exceptions import FieldbusUnsupportedFeature

    assert io_map.digital_output_count == 0
    with pytest.raises(FieldbusUnsupportedFeature):
        io_map.decode_output_state(io_map.empty_input_assembly())


def test_motion_command_ids_are_not_published(io_map: IoMap) -> None:
    """The vendor files do not publish the identifiers, so none is known."""
    from mecademic_fieldbus.exceptions import FieldbusUnsupportedFeature

    assert io_map.motion_command_ids == {}
    with pytest.raises(FieldbusUnsupportedFeature) as error:
        io_map.motion_command_id("MoveJoints")
    assert "motion_commands" in str(error.value)


def test_motion_command_ids_can_be_supplied() -> None:
    """A caller can supply the identifiers read from the programming manual."""
    io_map = IoMapV1(motion_command_ids={"MoveJoints": 42})
    assert io_map.motion_command_id("MoveJoints") == 42


def test_describe_layout_mentions_the_source_and_the_fields(io_map: IoMap) -> None:
    """The generated layout description is traceable to the vendor file."""
    description = io_map.describe_layout()
    assert "Meca500" in description
    assert "RobotStatus_Activated" in description
    assert "MotionCommand_Arg1" in description
    assert "input assembly" in description
    assert "output assembly" in description


def test_custom_spec_can_be_injected() -> None:
    """A map can be built on a specification that is not the shipped one."""
    spec = load_spec("1")
    io_map = IoMapV1(spec=spec)
    assert io_map.version == spec.assembly_version


# ----------------------------------------------------------------------
# Golden vectors
# ----------------------------------------------------------------------
@pytest.mark.parametrize("case", VECTORS["input"], ids=lambda case: case["name"])
def test_input_vectors_decode_as_expected(io_map: IoMap, case: dict) -> None:
    """Recorded input assemblies decode to the documented logical values."""
    raw = bytes.fromhex(case["hex"])
    assert len(raw) == io_map.input_assembly_size
    assert dataclasses.asdict(io_map.decode_status(raw)) == case["status"]
    assert dataclasses.asdict(io_map.decode_motion_status(raw)) == case["motion_status"]
    assert dataclasses.asdict(io_map.decode_safety_status(raw)) == case["safety_status"]
    decoded = dataclasses.asdict(io_map.decode_position(raw))
    expected = dict(case["position"])
    expected["joints"] = tuple(expected["joints"])
    expected["pose"] = tuple(expected["pose"])
    assert decoded == expected


@pytest.mark.parametrize("case", VECTORS["output"], ids=lambda case: case["name"])
def test_output_vectors_decode_as_expected(io_map: IoMap, case: dict) -> None:
    """Recorded output assemblies decode to the documented logical values."""
    raw = bytes.fromhex(case["hex"])
    assert len(raw) == io_map.output_assembly_size
    assert dataclasses.asdict(io_map.decode_robot_control(raw)) == case["robot_control"]
    assert dataclasses.asdict(io_map.decode_motion_control(raw)) == case["motion_control"]
    decoded = dataclasses.asdict(io_map.decode_motion_command(raw))
    expected = dict(case["motion_command"])
    expected["arguments"] = tuple(expected["arguments"])
    assert decoded == expected


@pytest.mark.parametrize("case", VECTORS["output"], ids=lambda case: case["name"])
def test_output_vectors_re_encode_byte_for_byte(io_map: IoMap, case: dict) -> None:
    """Re-encoding the documented values reproduces the recorded bytes."""
    raw = io_map.empty_output_assembly()
    raw = io_map.encode_robot_control(RobotControl(**case["robot_control"]), raw)
    raw = io_map.encode_motion_control(MotionControl(**case["motion_control"]), raw)
    raw = io_map.encode_motion_command(
        MotionCommand.build(
            case["motion_command"]["command_id"], case["motion_command"]["arguments"]
        ),
        raw,
    )
    assert raw.hex() == case["hex"]


def test_vectors_match_the_shipped_assembly_version(io_map: IoMap) -> None:
    """The golden vectors were generated for the version under test."""
    assert VECTORS["assembly_version"] == io_map.version
