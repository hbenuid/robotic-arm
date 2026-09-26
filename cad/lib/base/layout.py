"""The base's derived points and stations (pure math, no kernel): what lib/base/body.py and the tests share."""
from __future__ import annotations

import math

from lib.base.params import DEFAULT, BaseConfig
from lib.motors import NEMA17_BOLT_SP


def motor_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the base_yaw motor's 4 holes: the NEMA 17 square about the pattern centre."""
    (cx, cz), h = cfg.motor.centre, NEMA17_BOLT_SP / 2.0
    return [(cx + sx * h, cz + sz * h) for sx in (-1, 1) for sz in (-1, 1)]


def side_stub_x(cfg: BaseConfig = DEFAULT) -> float:
    """Where the straight sides stop above the plate: the inside of a side (z = +/- (r - wall)) meets the outer round."""
    s = cfg.shell
    return math.sqrt(s.r ** 2 - (s.r - s.wall) ** 2)


def chamfer_inset(cfg: BaseConfig = DEFAULT) -> float:
    """How far the 45 degree chamfer under the cap runs in from the wall's inside: its height."""
    return cfg.cap.underside_y - cfg.cap.chamfer_y0
