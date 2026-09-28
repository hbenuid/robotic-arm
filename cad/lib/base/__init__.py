"""The base (the arm's foot and the base_yaw housing) and its bolt-on motor mount (the +X lobe) as parametric build123d.

Pure-Python config and layout here (lib/base/params.py, layout.py); the build123d builders in lib/base/body.py
(imported explicitly by the part, so importing this package never pulls in OCCT). Never imports lib/params.py -
that module re-exports the interface values from here.
"""
from lib.base.layout import (  # noqa: F401
    chamfer_inset,
    joint_bolt_points,
    joint_stations,
    motor_holes,
    mount_inner_half,
    mount_x1,
    side_stub_x,
)
from lib.base.params import (  # noqa: F401
    DEFAULT,
    LEGACY,
    BaseConfig,
    BoreParams,
    CapParams,
    JointParams,
    MotorParams,
    MountParams,
    PlateParams,
    ShellParams,
)
