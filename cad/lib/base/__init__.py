"""The base (the arm's foot and the base_yaw housing) as parametric build123d.

Pure-Python config and layout here (lib/base/params.py, layout.py); the build123d builder in lib/base/body.py
(imported explicitly by the part, so importing this package never pulls in OCCT). Never imports lib/params.py -
that module re-exports the interface values from here.
"""
from lib.base.layout import chamfer_inset, motor_holes, side_stub_x  # noqa: F401
from lib.base.params import (  # noqa: F401
    DEFAULT,
    LEGACY,
    BaseConfig,
    BoreParams,
    CapParams,
    MotorParams,
    PlateParams,
    ShellParams,
)
