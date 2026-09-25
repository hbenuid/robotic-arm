"""The drive's own helpers for the ported cycloidal-drive tests (tests/cycloidal/test_*.py): the config every
drive test checks and its variants. The geometry helpers they share with the other tests are tests/helpers.py.

CadQuery -> build123d: copy.deepcopy + __setattr__ -> dataclasses.replace (no_chamfer).
"""
from __future__ import annotations

from dataclasses import replace

from build123d import Cylinder, Pos

from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, ring_pin_points
from lib.geom import align_min

CFG = DEFAULT_CONFIG   # the configuration every drive test checks (variants: dataclasses.replace, see no_chamfer)


def no_chamfer(cfg: DriveConfig = DEFAULT_CONFIG) -> DriveConfig:
    return replace(cfg, housing=replace(cfg.housing, edge_chamfer=0.0))


def ring_pins(cfg: DriveConfig = DEFAULT_CONFIG, z0: float = 0.0):
    """The 21 ring pins as one compound standing on z0 (the drive's simplified model)."""
    g = cfg.gear
    pins = None
    for x, y in ring_pin_points(cfg):
        pin = Pos(x, y, z0) * Cylinder(g.ring_pin_radius, g.ring_pin_length, align=align_min())
        pins = pin if pins is None else pins + pin
    return pins
