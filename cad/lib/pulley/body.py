"""build123d builders of gt2_pulley_90t from PulleyParams and gt2_pulley_20_60t from CompoundPulleyParams, each in its
part frame.

Every feature runs along the part's Y (the axis), so the body is built in a working frame whose +Z is the part's +Y -
part (x, y, z) = working (x, -z, y) - and turned into the part frame once at the end (Rot(-90, 0, 0)), like
lib/coupler/body.py. The web and the toothed rim (lib/pulley/teeth.py, an annulus round the web) come first, then the
step, the hub and its ring, then the bore, the bolt holes and their counterbores - every boolean after the rim stays near
the axis, far from its grooves. The compound pulley is its two toothed rings alone, each an annulus on the bore, fused where the upper
band's lower flange stands on the lower band's upper flange - away from both bands' grooves."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.geom import cylinder, single_solid
from lib.pulley.params import COMPOUND, DEFAULT, CompoundPulleyParams, PulleyParams
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
    cb_dia, cb_depth = cfg.counterbore
    for x, z in cfg.bolt_points():
        body = body - _disc(cfg.hole_dia / 2.0, cfg.end_y - NUDGE, cfg.face_y + NUDGE, x, z)
        if cb_depth > 0.0:
            body = body - _disc(cb_dia / 2.0, cfg.face_y - cb_depth, cfg.face_y + NUDGE, x, z)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)


def build_compound(cfg: CompoundPulleyParams = COMPOUND):
    """Each band's toothed ring between its flanges (lib/pulley/teeth.py gt2_ring(), with the band's blend) turned half a
    pitch, so a land - not a groove - lies on +X, as on the export; both annuli on the bore, fused at their flanges."""
    body = None
    for i, (teeth, blend_r) in enumerate(zip(cfg.teeth, cfg.blend_r, strict=True)):
        ring = bd.Rot(0.0, 0.0, 180.0 / teeth) * gt2_ring(teeth, cfg.band_w, cfg.flange_dia(i), cfg.flange_t,
                                                          z0=cfg.band_y0(i), inner_dia=cfg.bore_dia,
                                                          chamfer=cfg.chamfer, blend_r=blend_r)
        body = ring if body is None else body + ring
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
