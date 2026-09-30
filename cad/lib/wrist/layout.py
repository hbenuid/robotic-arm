"""The wrist body's derived points (pure math, no kernel): what lib/wrist/link.py and the tests share."""
from __future__ import annotations

import math

from lib.wrist.params import DEFAULT, WristConfig

_AXES_DEG = (0.0, 90.0, 180.0, 270.0)


def seat_bolt_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """(x, y, angle_deg) of the coupler flange's 4 bolts: on the axes about the pitch axis at SeatParams.bolt_r."""
    px, r = cfg.plate.pitch_x, cfg.seat.bolt_r
    return [(px + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), a) for a in _AXES_DEG]


def end_face_holes(cfg: WristConfig = DEFAULT) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """((y, z) of the M3 - 2 per row in m3_rows -, (y, z) of the 4 M4) on the end face, about the roll axis."""
    e = cfg.end_face
    h = e.m3_sp / 2.0
    m3 = [(sy * h, e.axis_z + sz * h) for sy in (-1.0, 1.0) for sz in e.m3_rows]
    m4 = [(sy * e.m4_y, e.axis_z + sz * e.m4_dz) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    return m3, m4


def slope_x(z: float, cfg: WristConfig = DEFAULT) -> float:
    """x of the slope plane at height z (the wedge under it is x >= slope_x(z))."""
    t = cfg.tower
    return t.slope_x0 + (z - cfg.plate.thickness) * (t.slope_x1 - t.slope_x0) / (t.z1 - cfg.plate.thickness)
