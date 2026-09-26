"""build123d builder of j3_coupler from CouplerParams, in the coupler's part frame.

Every feature runs along the part's Y (the joint axis), so the body is built in a working frame whose +Z is the
part's +Y - part (x, y, z) = working (x, -z, y) - and turned into the part frame once at the end (Rot(-90, 0, 0)),
like lib/upper_arm/link.py."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.coupler.layout import flange_bolt_points, pulley_bolt_points
from lib.coupler.params import DEFAULT, CouplerParams
from lib.geom import cylinder, hex_prism, single_solid
from lib.units import NUDGE


def _bore(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0):
    """A cylinder along the part's Y from y0 to y1, its axis through (x, z)."""
    return cylinder(radius, y1 - y0, (x, -z), z0=y0)


def build_coupler(cfg: CouplerParams = DEFAULT):
    body = _bore(cfg.flange_dia / 2.0, 0.0, cfg.flange_y1)
    body = body + (_bore(cfg.lip_dia[1] / 2.0, cfg.flange_y1 - NUDGE, cfg.lip_y1)
                   - _bore(cfg.lip_dia[0] / 2.0, cfg.flange_y1 - 2.0 * NUDGE, cfg.lip_y1 + NUDGE))
    body = body + _bore(cfg.journal_dia / 2.0, cfg.flange_y1 - NUDGE, cfg.journal_y1)
    body = body + _bore(cfg.stub_dia / 2.0, cfg.journal_y1 - NUDGE, cfg.stub_y1)
    body = body - _bore(cfg.bore_dia / 2.0, -NUDGE, cfg.stub_y1 + NUDGE)
    for x, z in pulley_bolt_points(cfg):
        body = body - _bore(cfg.pulley_bolt_dia / 2.0, cfg.nut_depth - NUDGE, cfg.stub_y1 + NUDGE, x, z)
        body = body - bd.Pos(x, -z, -NUDGE) * hex_prism(cfg.nut_af, math.pi / 2.0, cfg.nut_depth + NUDGE)   # a corner along Z
    for x, z in flange_bolt_points(cfg):
        body = body - _bore(cfg.flange_bolt_dia / 2.0, -NUDGE, cfg.counterbore_y0 + NUDGE, x, z)
        body = body - _bore(cfg.counterbore_dia / 2.0, cfg.counterbore_y0, cfg.flange_y1 + NUDGE, x, z)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
