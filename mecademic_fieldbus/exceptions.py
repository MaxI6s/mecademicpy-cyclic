"""Exception hierarchy of the :mod:`mecademic_fieldbus` package.

Every error raised by this library derives from :class:`FieldbusError`, so an
application can catch the whole library with a single ``except`` clause.  The
hierarchy is intentionally self-contained: it does not derive from, wrap, or
import any exception coming from a third-party or vendor library.  Errors
raised by the underlying protocol stack are translated into these types by the
transport layer.
"""

from typing import Optional

__all__ = [
    "FieldbusError",
    "FieldbusConnectionError",
    "FieldbusTimeoutError",
    "FieldbusProtocolError",
    "FieldbusIoMapError",
    "FieldbusSpecError",
    "FieldbusStateError",
    "FieldbusUnsupportedFeature",
    "RobotErrorStatus",
]


class FieldbusError(Exception):
    """Base class of every error raised by :mod:`mecademic_fieldbus`."""


class FieldbusConnectionError(FieldbusError):
    """The fieldbus connection could not be opened, or was lost.

    Raised for example when the target refuses the TCP session, when the
    Forward Open request is rejected, or when an operation is attempted on a
    transport that is not connected.
    """


class FieldbusTimeoutError(FieldbusError):
    """An operation did not complete within its allotted time.

    Raised both by the transport (no cyclic data received) and by the facade
    (the robot did not reach the expected state in time).
    """


class FieldbusProtocolError(FieldbusError):
    """A malformed or unexpected protocol exchange was detected.

    Raised when the peer answers something that cannot be interpreted, for
    example an assembly whose size does not match the one negotiated at
    Forward Open time.
    """


class FieldbusIoMapError(FieldbusError):
    """A field could not be encoded to, or decoded from, an assembly buffer.

    Typical causes are an unknown field name, a value that does not fit the
    declared type, or a buffer shorter than the declared assembly size.
    """


class FieldbusSpecError(FieldbusIoMapError):
    """The declarative assembly specification is invalid or unusable.

    Raised while loading ``io_map/spec/*.json`` when the file is missing,
    malformed, or describes fields that overlap or overflow the assembly.
    """


class FieldbusStateError(FieldbusError):
    """A command was rejected because the robot is not in a suitable state.

    Raised for example when homing is requested while the robot is not
    activated.
    """


class FieldbusUnsupportedFeature(FieldbusError):
    """The requested feature is not carried by this assembly version.

    Raised when a command exists in the API but has no home in the assembly
    layout in use -- digital inputs and outputs, for instance, which the
    Meca500 cyclic assemblies do not expose.
    """


class RobotErrorStatus(FieldbusError):
    """The robot reports an error condition through its input assembly.

    Args:
        message: Human readable description of the condition.
        error_code: Robot error code as reported in the input assembly, or
            ``None`` when the assembly does not carry one.
    """

    def __init__(self, message: str, error_code: Optional[int] = None) -> None:
        super().__init__(message)
        self.error_code = error_code

    def __str__(self) -> str:
        base = super().__str__()
        if self.error_code is None:
            return base
        return "{} (error code {})".format(base, self.error_code)
