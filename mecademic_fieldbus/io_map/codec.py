"""Generic codec driven by the declarative assembly specification.

This module, together with :mod:`mecademic_fieldbus.io_map.spec_loader`, is the
**only** place in the whole project where bit and word positions are
manipulated.  Everything above it works with named, typed fields.

The codec is deliberately dumb: it knows how to place a scalar or an array of
scalars at a position described by a :class:`FieldSpec`, and nothing about the
meaning of the fields.  The semantics live in the version specific I/O maps
(:mod:`mecademic_fieldbus.io_map.v1` and successors).
"""

import struct
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

from ..exceptions import FieldbusIoMapError, FieldbusSpecError

__all__ = ["FieldSpec", "AssemblyCodec", "BOOL_TYPE", "STRUCT_FORMATS"]

#: Name of the boolean (single bit) field type in the declarative spec.
BOOL_TYPE = "bool"

#: Mapping of the numeric field types to their :mod:`struct` format character.
STRUCT_FORMATS = {
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "int64": "q",
    "uint64": "Q",
    "float32": "f",
    "float64": "d",
}

#: Mapping of the byte order names used in the spec to :mod:`struct` prefixes.
_BYTE_ORDER_PREFIXES = {"little": "<", "big": ">"}

#: Type of a single decoded value.
ScalarValue = Union[bool, int, float]

#: Type of a decoded field: a scalar, or a tuple of scalars for arrays.
FieldValue = Union[ScalarValue, Tuple[ScalarValue, ...]]


@dataclass(frozen=True)
class FieldSpec:
    """Position and type of one logical field inside an assembly.

    Attributes:
        name: Logical name of the field, unique within its assembly.
        type: Field type, either ``"bool"`` or a key of :data:`STRUCT_FORMATS`.
        byte_offset: Offset of the field, in bytes from the start of the
            assembly.
        bit_offset: Offset of the first bit inside ``byte_offset``.  Only
            meaningful for ``bool`` fields.
        count: Number of consecutive elements.  ``1`` means a scalar field,
            anything greater means an array.
        description: Free form documentation of the field.
        unit: Physical unit of the field, when relevant.
    """

    name: str
    type: str
    byte_offset: int
    bit_offset: int = 0
    count: int = 1
    description: str = ""
    unit: Optional[str] = None

    @property
    def is_array(self) -> bool:
        """Whether the field holds more than one element."""
        return self.count > 1

    @property
    def is_bool(self) -> bool:
        """Whether the field is made of single bits."""
        return self.type == BOOL_TYPE

    @property
    def element_size_bytes(self) -> int:
        """Size of a single element, in bytes (``0`` for ``bool`` fields)."""
        if self.is_bool:
            return 0
        return struct.calcsize(STRUCT_FORMATS[self.type])

    @property
    def first_bit_index(self) -> int:
        """Index of the first bit of the field, counted from the assembly start."""
        return self.byte_offset * 8 + self.bit_offset

    @property
    def size_bits(self) -> int:
        """Total size of the field, in bits."""
        if self.is_bool:
            return self.count
        return self.count * self.element_size_bytes * 8

    @property
    def end_byte_offset(self) -> int:
        """Offset of the first byte *after* the field."""
        return (self.first_bit_index + self.size_bits + 7) // 8


class AssemblyCodec:
    """Encode and decode one assembly according to its declarative spec.

    Args:
        name: Name of the assembly (``"input"``, ``"output"``, ``"config"``).
        instance: CIP assembly instance number, or ``None`` when the robot
            connection path carries no such assembly.
        size_bytes: Total size of the assembly, in bytes.
        fields: Field specifications, in declaration order.
        byte_order: ``"little"`` or ``"big"``.
        direction: Free form description of the data direction.
        description: Free form documentation of the assembly.

    Raises:
        FieldbusSpecError: If two fields overlap, if a field overflows the
            declared size, or if a field uses an unknown type.
    """

    def __init__(
        self,
        name: str,
        instance: Optional[int],
        size_bytes: int,
        fields: Sequence[FieldSpec],
        byte_order: str = "little",
        direction: str = "",
        description: str = "",
    ) -> None:
        if byte_order not in _BYTE_ORDER_PREFIXES:
            raise FieldbusSpecError(
                "assembly {!r}: unsupported byte order {!r}".format(name, byte_order)
            )
        self.name = name
        self.instance = instance
        self.size_bytes = size_bytes
        self.byte_order = byte_order
        self.direction = direction
        self.description = description
        self._prefix = _BYTE_ORDER_PREFIXES[byte_order]
        self._fields: Dict[str, FieldSpec] = {}
        for spec in fields:
            if spec.name in self._fields:
                raise FieldbusSpecError(
                    "assembly {!r}: duplicate field {!r}".format(name, spec.name)
                )
            self._fields[spec.name] = spec
        self._validate()

    def _validate(self) -> None:
        """Check that every field has a known type and a legal position.

        Raises:
            FieldbusSpecError: If the layout is inconsistent.
        """
        occupied: Dict[int, str] = {}
        for spec in self._fields.values():
            if not spec.is_bool and spec.type not in STRUCT_FORMATS:
                raise FieldbusSpecError(
                    "assembly {!r}, field {!r}: unknown type {!r}".format(
                        self.name, spec.name, spec.type
                    )
                )
            if spec.count < 1:
                raise FieldbusSpecError(
                    "assembly {!r}, field {!r}: count must be >= 1".format(self.name, spec.name)
                )
            if spec.byte_offset < 0 or spec.bit_offset < 0 or spec.bit_offset > 7:
                raise FieldbusSpecError(
                    "assembly {!r}, field {!r}: illegal offset".format(self.name, spec.name)
                )
            if not spec.is_bool and spec.bit_offset != 0:
                raise FieldbusSpecError(
                    "assembly {!r}, field {!r}: bit_offset is only allowed on bool fields".format(
                        self.name, spec.name
                    )
                )
            if spec.end_byte_offset > self.size_bytes:
                raise FieldbusSpecError(
                    "assembly {!r}, field {!r}: overflows the {} byte assembly".format(
                        self.name, spec.name, self.size_bytes
                    )
                )
            for bit in range(spec.first_bit_index, spec.first_bit_index + spec.size_bits):
                previous = occupied.get(bit)
                if previous is not None:
                    raise FieldbusSpecError(
                        "assembly {!r}: fields {!r} and {!r} overlap at bit {}".format(
                            self.name, previous, spec.name, bit
                        )
                    )
                occupied[bit] = spec.name

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------
    @property
    def field_names(self) -> Tuple[str, ...]:
        """Names of every field of the assembly, in declaration order."""
        return tuple(self._fields)

    def field(self, name: str) -> FieldSpec:
        """Return the specification of a single field.

        Args:
            name: Logical field name.

        Returns:
            The matching :class:`FieldSpec`.

        Raises:
            FieldbusIoMapError: If the assembly has no such field.
        """
        try:
            return self._fields[name]
        except KeyError:
            raise FieldbusIoMapError("assembly {!r} has no field {!r}".format(self.name, name))

    def has_field(self, name: str) -> bool:
        """Whether the assembly declares a field with this name.

        Args:
            name: Logical field name.

        Returns:
            ``True`` when the field exists.
        """
        return name in self._fields

    def zeros(self) -> bytes:
        """Return an all-zero image of the assembly."""
        return bytes(self.size_bytes)

    # ------------------------------------------------------------------
    # Decoding
    # ------------------------------------------------------------------
    def read(self, buffer: bytes, name: str) -> FieldValue:
        """Decode a single field from a raw assembly image.

        Args:
            buffer: Raw assembly bytes.
            name: Logical field name.

        Returns:
            The decoded value: a scalar for a simple field, a tuple for an
            array field.

        Raises:
            FieldbusIoMapError: If the field is unknown or the buffer is too
                short.
        """
        spec = self.field(name)
        self._check_length(buffer)
        values: List[ScalarValue]
        if spec.is_bool:
            values = [
                bool(
                    buffer[(spec.first_bit_index + index) // 8]
                    & (1 << ((spec.first_bit_index + index) % 8))
                )
                for index in range(spec.count)
            ]
        else:
            fmt = "{}{}{}".format(self._prefix, spec.count, STRUCT_FORMATS[spec.type])
            values = list(struct.unpack_from(fmt, buffer, spec.byte_offset))
        if spec.is_array:
            return tuple(values)
        return values[0]

    def read_many(self, buffer: bytes, names: Sequence[str]) -> Dict[str, FieldValue]:
        """Decode several fields at once.

        Args:
            buffer: Raw assembly bytes.
            names: Logical field names to decode.

        Returns:
            A mapping of field name to decoded value.

        Raises:
            FieldbusIoMapError: If a field is unknown or the buffer is too short.
        """
        return {name: self.read(buffer, name) for name in names}

    # ------------------------------------------------------------------
    # Encoding
    # ------------------------------------------------------------------
    def write(self, buffer: bytearray, name: str, value: Any) -> None:
        """Encode a single field in place into a mutable assembly image.

        Args:
            buffer: Mutable assembly image, at least ``size_bytes`` long.
            name: Logical field name.
            value: Scalar for a simple field, sequence for an array field.

        Raises:
            FieldbusIoMapError: If the field is unknown, the buffer is too
                short, or the value does not fit the declared type.
        """
        spec = self.field(name)
        self._check_length(buffer)
        values = self._as_elements(spec, value)
        if spec.is_bool:
            for index, element in enumerate(values):
                bit_index = spec.first_bit_index + index
                mask = 1 << (bit_index % 8)
                if element:
                    buffer[bit_index // 8] |= mask
                else:
                    buffer[bit_index // 8] &= 0xFF & ~mask
            return
        fmt = "{}{}{}".format(self._prefix, spec.count, STRUCT_FORMATS[spec.type])
        try:
            struct.pack_into(fmt, buffer, spec.byte_offset, *values)
        except struct.error as exc:
            raise FieldbusIoMapError(
                "assembly {!r}, field {!r}: cannot encode {!r} as {}: {}".format(
                    self.name, spec.name, value, spec.type, exc
                )
            )

    def pack(
        self,
        values: Mapping[str, Any],
        base: Optional[bytes] = None,
    ) -> bytes:
        """Return a new assembly image with the given fields written into it.

        Args:
            values: Mapping of logical field name to value.
            base: Image to start from.  Defaults to an all-zero image, which
                means every field not listed in ``values`` is cleared.

        Returns:
            The updated assembly image.

        Raises:
            FieldbusIoMapError: If a field is unknown or a value does not fit.
        """
        buffer = bytearray(self.zeros() if base is None else base)
        self._check_length(buffer)
        for name, value in values.items():
            self.write(buffer, name, value)
        return bytes(buffer)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _check_length(self, buffer: bytes) -> None:
        """Verify that a buffer is large enough to hold this assembly.

        Args:
            buffer: Raw assembly image.

        Raises:
            FieldbusIoMapError: If the buffer is shorter than ``size_bytes``.
        """
        if len(buffer) < self.size_bytes:
            raise FieldbusIoMapError(
                "assembly {!r}: expected at least {} bytes, got {}".format(
                    self.name, self.size_bytes, len(buffer)
                )
            )

    def _as_elements(self, spec: FieldSpec, value: Any) -> List[Any]:
        """Normalise a user supplied value into a list of ``spec.count`` elements.

        Args:
            spec: Specification of the field being written.
            value: Scalar or sequence supplied by the caller.

        Returns:
            A list of exactly ``spec.count`` elements.

        Raises:
            FieldbusIoMapError: If the number of elements does not match.
        """
        if spec.is_array or isinstance(value, (list, tuple)):
            elements = list(value)
        else:
            elements = [value]
        if len(elements) != spec.count:
            raise FieldbusIoMapError(
                "assembly {!r}, field {!r}: expected {} element(s), got {}".format(
                    self.name, spec.name, spec.count, len(elements)
                )
            )
        if spec.is_bool:
            return [bool(element) for element in elements]
        if spec.type.startswith("float"):
            return [float(element) for element in elements]
        return [int(element) for element in elements]
