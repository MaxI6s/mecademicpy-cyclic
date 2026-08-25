"""Protocol specific transports.

One fieldbus equals one module equals one class implementing
:class:`~mecademic_fieldbus.transports.base.FieldbusTransport`.  Adding
Profinet later means adding ``profinet.py`` here and nothing else.

:class:`~mecademic_fieldbus.transports.ethernetip.EtherNetIpTransport` is
imported lazily by :func:`get_transport_class` so that the core of the library
stays importable without the optional third-party protocol stack.
"""

from typing import Any, Type

from ..exceptions import FieldbusConnectionError
from .base import FieldbusTransport

__all__ = ["FieldbusTransport", "get_transport_class", "AVAILABLE_TRANSPORTS"]

#: Names of the transports shipped with this release.
AVAILABLE_TRANSPORTS = ("ethernetip",)


def get_transport_class(name: str = "ethernetip") -> Type[FieldbusTransport]:
    """Return the transport class registered under a given name.

    Args:
        name: Transport name, one of :data:`AVAILABLE_TRANSPORTS`.

    Returns:
        The transport class, not instantiated.

    Raises:
        FieldbusConnectionError: If the name is unknown.
    """
    if name != "ethernetip":
        raise FieldbusConnectionError(
            "unknown transport {!r}, available transports: {}".format(
                name, ", ".join(AVAILABLE_TRANSPORTS)
            )
        )
    from .ethernetip import EtherNetIpTransport

    return EtherNetIpTransport


def __getattr__(name: str) -> Any:
    """Expose ``EtherNetIpTransport`` lazily as a module attribute.

    Args:
        name: Attribute being looked up.

    Returns:
        The requested attribute.

    Raises:
        AttributeError: If the module has no such attribute.
    """
    if name == "EtherNetIpTransport":
        from .ethernetip import EtherNetIpTransport

        return EtherNetIpTransport
    raise AttributeError("module {!r} has no attribute {!r}".format(__name__, name))
