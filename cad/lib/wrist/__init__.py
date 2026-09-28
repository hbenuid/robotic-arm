"""The wrist body (wrist_link) as parametric build123d.

Pure-Python config and layout here (lib/wrist/params.py, layout.py); the build123d builder in lib/wrist/link.py
(imported explicitly by the part, so importing this package never pulls in OCCT). Never imports lib/params.py.
"""
from lib.wrist.layout import end_face_holes, seat_bolt_points, slope_x  # noqa: F401
from lib.wrist.params import (  # noqa: F401
    DEFAULT,
    LEGACY,
    EndFaceParams,
    PlateParams,
    SeatParams,
    TowerParams,
    WristConfig,
)
