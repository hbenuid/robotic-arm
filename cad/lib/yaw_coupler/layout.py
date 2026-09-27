"""The yaw coupler's derived points and outlines (pure math, no kernel): what lib/yaw_coupler/body.py and the tests
share. Outlines in the YZ plane are (y, z) pairs; the disc's profile is (r, y)."""
from __future__ import annotations

import math

from lib.yaw_coupler.params import DEFAULT, YawCouplerConfig


def disc_r(cfg: YawCouplerConfig, y: float) -> float:
    """The disc's radius at height y: the band, then the draft in to top_r."""
    d = cfg.disc
    if y <= d.band_y1:
        return d.band_r
    return d.band_r + (d.top_r - d.band_r) * (y - d.band_y1) / (d.top_y - d.band_y1)


def disc_profile(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(r, y) of the disc's half section, from the axis at the underside round to the axis at the top face."""
    d = cfg.disc
    band = [(d.band_r, d.band_y1)] if d.y0 < d.band_y1 else []
    return [(0.0, d.y0), (disc_r(cfg, d.y0), d.y0), *band, (d.top_r, d.top_y), (0.0, d.top_y)]


def flare_r(cfg: YawCouplerConfig, y: float) -> float:
    """The flare's radius at height y: the cone through the ring's top edge, its apex on the axis."""
    d, k = cfg.disc, cfg.yoke
    return d.ring_r * (y - k.flare_apex_y) / (d.ring_y1 - k.flare_apex_y)


def below_axis(cfg: YawCouplerConfig, r: float, z: float) -> float:
    """y of the point at |z| on the circle of r about the drive's axis, below the axis."""
    return cfg.yoke.axis_y - math.sqrt(r * r - z * z)


def _mirrored(half: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """A (y, z) outline symmetric about z = 0 from its +Z half (listed bottom up)."""
    return half + [(y, -z) for y, z in reversed(half)]


def _bottom(cfg: YawCouplerConfig) -> float:
    """Where the yoke's prisms start: below the flare, which bounds them."""
    return cfg.disc.ring_y1 - 1.0


def od_point(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float]:
    """(y, z) where the outer face meets the housing's od - the yoke's widest point, both prisms' corner."""
    k = cfg.yoke
    return (below_axis(cfg, k.od_r, k.outer_z), k.outer_z)


def cheek_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the -X cheek: up the outer face to the od point, straight to the cradle at cheek_top_z."""
    k = cfg.yoke
    return _mirrored([(_bottom(cfg), k.outer_z), od_point(cfg), (below_axis(cfg, k.cradle_r, k.cheek_top_z), k.cheek_top_z),
                      (k.axis_y, k.cheek_top_z)])


def middle_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the middle body: up the outer face to the od point, the V-groove round the 45 degree pillar - across
    its end on the od, then down its near side to the cradle."""
    k = cfg.yoke
    near, end = k.groove_z
    return _mirrored([(_bottom(cfg), k.outer_z), od_point(cfg), (below_axis(cfg, k.od_r, end), end),
                      (below_axis(cfg, k.cradle_r, near), near), (k.axis_y, near)])


def channel_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the channel under the bottom pillar: the floor, the walls opening up to the drive's axis."""
    k = cfg.yoke
    top = k.channel_half_z + (k.axis_y - k.channel_floor_y) * k.channel_slope
    return _mirrored([(k.channel_floor_y, k.channel_half_z), (k.axis_y, top)])


def nut_centres(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the housing bolts the cheek's nut pockets sit on: on the bolt circle about the drive's axis."""
    k = cfg.yoke
    return [(k.axis_y - k.bolt_circle_r * math.cos(math.radians(a)), k.bolt_circle_r * math.sin(math.radians(a)))
            for a in k.bolt_deg]


def hole_points(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the hub's 4 holes: on hole_r from hole_deg, 90 degrees apart."""
    h = cfg.hub
    return [(h.hole_r * math.cos(math.radians(h.hole_deg + 90.0 * i)), h.hole_r * math.sin(math.radians(h.hole_deg + 90.0 * i)))
            for i in range(4)]
