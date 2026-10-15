"""The wrist body (wrist_link), the wrist roll drive it carries and the gripper's connector to it, as parametric build123d.

Pure-Python config and layout here (lib/wrist/params.py, layout.py); the build123d builders in lib/wrist/link.py,
roll.py and connector.py (imported explicitly by the parts, so importing this package never pulls in OCCT). Never
imports lib/params.py.
"""
from lib.wrist.layout import end_face_holes, seat_bolt_points, slope_x, stack_positions  # noqa: F401
from lib.wrist.params import (  # noqa: F401
    DEFAULT,
    LEGACY,
    ConnectorParams,
    EndFaceParams,
    PancakeParams,
    PlateParams,
    SeatParams,
    TowerParams,
    WristConfig,
    WristRollParams,
)
