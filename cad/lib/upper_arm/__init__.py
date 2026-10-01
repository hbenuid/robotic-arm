"""The upper arm (j1_link) as parametric build123d.

Pure-Python config and layout here (lib/upper_arm/params.py, layout.py); the build123d builder in
lib/upper_arm/link.py (imported explicitly by the part, so importing this package never pulls in OCCT). Never
imports lib/params.py - that module re-exports the interface values from here.
"""
from lib.upper_arm.layout import arm_slide, cove_axes, hub_bolt_points, pad_holes, socket_points  # noqa: F401
from lib.upper_arm.params import (  # noqa: F401
    DEFAULT,
    LEGACY,
    ArmParams,
    BearingParams,
    ElbowParams,
    HubParams,
    PadParams,
    SlabParams,
    SlotParams,
    SocketParams,
    UpperArmConfig,
)
