"""build123d builder of the base from a BaseConfig, in the base's part frame.

Every feature runs along the part's Y (the base_yaw axis), so the body is built in a working frame whose +Z is the
part's +Y - part (x, y, z) = working (x, -z, y), the helpers below take part coordinates - and turned into the part
frame once at the end (Rot(-90, 0, 0)), like lib/upper_arm/link.py."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.base.layout import chamfer_inset, motor_holes, side_stub_x
from lib.base.params import DEFAULT, BaseConfig
from lib.geom import align_min, cylinder, single_solid
from lib.units import NUDGE


def _pt(x: float, z: float) -> tuple[float, float]:
    """A part-frame (x, z) point in the working XY plane."""
    return (x, -z)


def _bore(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0):
    """A cylinder along the part's Y from y0 to y1, its axis through (x, z)."""
    return cylinder(radius, y1 - y0, _pt(x, z), z0=y0)


def _block(x: tuple, z: tuple, y: tuple):
    """A box spanning the part-frame ranges x = (x0, x1), z = (z0, z1), y = (y0, y1)."""
    (x0, x1), (z0, z1), (y0, y1) = x, z, y
    return bd.Pos((x0 + x1) / 2.0, -(z0 + z1) / 2.0, y0) * bd.Box(x1 - x0, z1 - z0, y1 - y0, align=align_min())


def _d(r: float, x1: float, y: tuple, x0: float = 0.0):
    """The D prism from y[0] to y[1]: a round of r about the axis and straight sides z = +/- r from x0 out to x1."""
    return _bore(r, *y) + _block((x0, x1), (-r, r), y)


def _d_section(r: float, x1: float, y: float):
    """The D outline (the round of r on -X, the sides out to x1) as a face at height y - a loft section."""
    edges = [bd.ThreePointArc((0.0, r), (-r, 0.0), (0.0, -r)), bd.Line((0.0, -r), (x1, -r)), bd.Line((x1, -r), (x1, r)),
             bd.Line((x1, r), (0.0, r))]
    return bd.Pos(0.0, 0.0, y) * bd.Face(bd.Wire(edges))


def _sector(r: float, deg: tuple, y: tuple):
    """A pie wedge of radius r about the axis between the part-frame angles deg = (from, to), from y[0] to y[1]."""
    a0, a1 = (math.radians(d) for d in deg)
    p = [_pt(r * math.cos(a), r * math.sin(a)) for a in (a0, (a0 + a1) / 2.0, a1)]
    wire = bd.Wire([bd.Line((0.0, 0.0), p[0]), bd.ThreePointArc(*p), bd.Line(p[2], (0.0, 0.0))])
    return bd.Pos(0.0, 0.0, y[0]) * bd.extrude(bd.Face(wire), amount=y[1] - y[0], dir=(0.0, 0.0, 1.0))   # the wire runs clockwise


def _walls(cfg: BaseConfig):
    """The D wall from the bottom face to the plate's top, hollow; above it the round half and the sides' stubs up to
    the cap's top face, hollow up to the chamfer under the cap."""
    s, c, p = cfg.shell, cfg.cap, cfg.plate
    ri, far = s.r - s.wall, s.r + 1.0
    lower = _d(s.r, s.x1, (s.y0, p.y[1])) - _d(ri, s.x1 - s.wall, (s.y0 - NUDGE, p.y[1] + NUDGE))
    lower = lower - _block((s.x1 - s.wall - NUDGE, s.x1 + NUDGE), (-s.notch_half_z, s.notch_half_z), (s.y0 - NUDGE, s.notch_y1))
    upper = _d(s.r, side_stub_x(cfg), (p.y[1], c.top_y))
    inset = chamfer_inset(cfg)
    cavity = _d(ri, far, (p.y[1] - NUDGE, c.chamfer_y0)) + bd.loft(
        [_d_section(ri, far, c.chamfer_y0), _d_section(ri - inset, far, c.underside_y)], ruled=True)
    return lower + (upper - cavity)


def _cap(cfg: BaseConfig, body):
    """The boss under the cap, the groove and the seat ring on top, the bearing bore, the r 0.5 rounds."""
    c, b = cfg.cap, cfg.bore
    body = body + _bore(c.boss_r, c.boss_y0, c.underside_y + NUDGE)
    body = body - _bore(c.groove_r[1], c.groove_y0, c.top_y + NUDGE)
    body = body + _bore(c.groove_r[0], c.groove_y0 - NUDGE, c.ring_top_y)
    body = body - _bore(b.upper_dia / 2.0, b.lip_y[1], c.ring_top_y + NUDGE)
    body = body - _bore(b.lip_dia / 2.0, b.lip_y[0] - NUDGE, b.lip_y[1] + NUDGE)
    body = body - _bore(b.lower_dia / 2.0, c.boss_y0 - NUDGE, b.lip_y[0])
    rounds = {(c.top_y, c.groove_r[1]), (c.ring_top_y, c.groove_r[0]), (c.ring_top_y, b.upper_dia / 2.0),
              (c.boss_y0, b.lower_dia / 2.0)}
    edges = [e for e in body.edges().filter_by(bd.GeomType.CIRCLE)
             if any(abs(e.arc_center.Z - y) < 1e-6 and abs(e.radius - r) < 1e-6 for y, r in rounds)]
    return body.fillet(c.fillet, edges)


def _plate(cfg: BaseConfig):
    """The motor plate across the D: the central opening (out to the wall on the -X side), the curved slot, the
    motor's holes, window and belt slot."""
    s, p, m = cfg.shell, cfg.plate, cfg.motor
    ri, y = s.r - s.wall, (p.y[0] - NUDGE, p.y[1] + NUDGE)
    plate = _d(ri, s.x1 - s.wall, p.y)
    plate = plate - _bore(p.opening_r, *y) - _sector(ri + 1.0, p.open_deg, y)
    arc = bd.CenterArc((0.0, 0.0), p.arc_slot_r, -p.arc_slot_half_deg, 2.0 * p.arc_slot_half_deg)
    plate = plate - bd.Pos(0.0, 0.0, y[0]) * bd.extrude(bd.SlotArc(arc, p.arc_slot_w), amount=y[1] - y[0])
    cz = m.centre[1]
    plate = plate - _block(m.window_x, (cz - m.window_half_z, cz + m.window_half_z), y)
    plate = plate - (_block((s.r - 5.0, m.slot_x1), (cz - m.slot_half_z, cz + m.slot_half_z), y) - _bore(s.r, y[0] - NUDGE, y[1] + NUDGE))
    for x, z in motor_holes(cfg):
        plate = plate - _bore(m.hole_dia / 2.0, *y, x, z)
    return plate


def _rim(cfg: BaseConfig):
    """The U rim under the plate round the motor's face (open toward the axis, its ends on the shell's outer round)."""
    s, p, m = cfg.shell, cfg.plate, cfg.motor
    cz, h, out = m.centre[1], m.rim_half, m.rim_half + m.rim_wall
    rim = _block((s.r - 10.0, m.slot_x1 + m.rim_wall), (cz - out, cz + out), (m.rim_y0, p.y[0] + NUDGE))
    rim = rim - _block((s.r - 11.0, m.slot_x1), (cz - h, cz + h), (m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE))
    return rim - _bore(s.r, m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE)


def build_base(cfg: BaseConfig = DEFAULT):
    body = _cap(cfg, _walls(cfg))
    body = body + _plate(cfg) + _rim(cfg)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
