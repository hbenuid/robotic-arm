"""build123d builder of gt2_pulley_90t from PulleyParams, in the pulley's part frame.

Every feature runs along the part's Y (the axis), so the body is built in a working frame whose +Z is the part's +Y -
part (x, y, z) = working (x, -z, y) - and turned into the part frame once at the end (Rot(-90, 0, 0)), like
lib/coupler/body.py. The web and the toothed rim (lib/pulley/teeth.py, an annulus round the web) come first, then the
step, the hub and its ring, then the bore and the bolt holes - every boolean after the rim stays near the axis, far from
its grooves."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.geom import cylinder, single_solid
from lib.pulley.params import DEFAULT, PulleyParams
from lib.pulley.teeth import gt2_ring
from lib.units import NUDGE

_FUSE = 0.2   # the web's diameter past the rim's inside: the two overlap, never just touch


def _disc(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0):
    """A cylinder along the part's Y from y0 to y1, its axis through (x, z)."""
    return cylinder(radius, y1 - y0, (x, -z), z0=y0)


def build_pulley(cfg: PulleyParams = DEFAULT):
    step_dia, step_y0 = cfg.step
    ring_dia, ring_y0, ring_y1 = cfg.ring
    # the web and the rim first: fused onto a web that already carries the step, their outer faces stay 3 faces
    body = _disc((cfg.rim_dia + _FUSE) / 2.0, cfg.web_y0, cfg.face_y)
    body = body + gt2_ring(cfg.teeth, cfg.band_y1, cfg.flange_dia, cfg.flange_t, inner_dia=cfg.rim_dia, chamfer=cfg.chamfer)
    body = body + _disc(step_dia / 2.0, step_y0, cfg.web_y0 + NUDGE)
    body = body + _disc(cfg.hub_dia / 2.0, cfg.end_y, step_y0 + NUDGE)
    body = body + _disc(ring_dia / 2.0, ring_y0, ring_y1)
    body = body - _disc(cfg.bore_dia / 2.0, cfg.end_y - NUDGE, cfg.face_y + NUDGE)
    for x, z in cfg.bolt_points():
        body = body - _disc(cfg.hole_dia / 2.0, cfg.end_y - NUDGE, cfg.face_y + NUDGE, x, z)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
