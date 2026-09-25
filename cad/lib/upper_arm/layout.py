"""Pure-math layout of the upper arm: the hole patterns the builder and the tests share, as (x, z) in j1_link's
part frame (every hole runs along Y)."""
from __future__ import annotations

import math

from lib.upper_arm.params import DEFAULT, UpperArmConfig


def hub_bolt_points(cfg: UpperArmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The 4 hub bolt holes: bolt_angle_deg, then every 90 degrees (the angle measured atan2(z, x))."""
    h = cfg.hub
    r = h.bolt_circle_dia / 2.0
    angles = [math.radians(h.bolt_angle_deg + 90.0 * k) for k in range(4)]
    return [(r * math.cos(a), r * math.sin(a)) for a in angles]


def cove_axes(cfg: UpperArmConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The pad's R10 fills in the opening: ((x, y) of the -X cove's axis, along Z - tangent to the opening's -X wall
    and to the underside's level), ((y, |z|) of the Z coves' axes, along X - tangent to the opening's Z walls and
    through the cavity's edge at the underside's level)."""
    p, oh, y0 = cfg.pad, cfg.hub.opening_half, cfg.slab.y0
    rise = math.sqrt(p.cove_r ** 2 - (p.cavity_half_z - (oh - p.cove_r)) ** 2)
    return (-oh + p.cove_r, y0 + p.cove_r), (y0 + rise, oh - p.cove_r)


def pad_holes(cfg: UpperArmConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The NEMA 17 holes through the pad's floor, (x, z, diameter)."""
    return list(cfg.pad.holes)


def socket_points(cfg: UpperArmConfig = DEFAULT) -> tuple[list, list]:
    """(shoulder-half sockets, elbow-half sockets) in the underside; both empty without sockets."""
    if cfg.sockets is None:
        return [], []
    return list(cfg.sockets.shoulder), list(cfg.sockets.elbow)
