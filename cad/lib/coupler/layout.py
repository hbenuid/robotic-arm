"""The coupler's derived points (pure math, no kernel): what lib/coupler/body.py and the tests share."""
from __future__ import annotations

from lib.coupler.params import DEFAULT, CouplerParams

_AXES = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))


def pulley_bolt_points(cfg: CouplerParams = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the 90T's 4 bolts: on the axes at pulley_bolt_r."""
    return [(cfg.pulley_bolt_r * a, cfg.pulley_bolt_r * b) for a, b in _AXES]


def flange_bolt_points(cfg: CouplerParams = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the flange's 4 bolts: on the axes at flange_bolt_r."""
    return [(cfg.flange_bolt_r * a, cfg.flange_bolt_r * b) for a, b in _AXES]
