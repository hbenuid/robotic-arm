"""build123d builders of the base and its motor mount from a BaseConfig, both in the base's part frame.

Every feature but the joint's bolts runs along the part's Y (the base_yaw axis), so the bodies are built in a working
frame whose +Z is the part's +Y - part (x, y, z) = working (x, -z, y), the helpers below take part coordinates - and
turned into the part frame once at the end (Rot(-90, 0, 0)), like lib/upper_arm/link.py.

With cfg.joint (DEFAULT) the SolidWorks body is cut at x = split_x: build_base keeps the -X side (the tower, the
plate's neck) and ends in two posts with the M4 holes and the nuts' pockets, the window between them opening into the
motor mount; build_motor_mount is a box of its own (cfg.mount) round the motor and its board - walls, end wall, the
plate with the motor's seat, the U rim - with an ear each side at the joint face for the M4. Without it (LEGACY)
build_base is the whole SolidWorks part."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.base.layout import (
    chamfer_inset,
    joint_bolt_points,
    joint_stations,
    motor_holes,
    mount_inner_half,
    mount_x1,
    side_stub_x,
)
from lib.base.params import DEFAULT, BaseConfig
from lib.geom import align_min, cylinder, hex_prism, single_solid
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


def _x_bore(radius: float, x: tuple, z: float, y: float):
    """A cylinder along the part's X from x[0] to x[1], its axis through (z, y)."""
    return bd.Pos(x[0], -z, y) * bd.Rot(0.0, 90.0, 0.0) * bd.Cylinder(radius, x[1] - x[0], align=align_min())


def _x_hex(across_flats: float, x: tuple, z: float, y: float):
    """A hex prism along the part's X from x[0] to x[1], its axis through (z, y), a corner toward +/-Y (up: its roof
    a peak, standing or upside down on the bed)."""
    return bd.Pos(x[0], -z, y) * bd.Rot(0.0, 90.0, 0.0) * hex_prism(across_flats, 0.0, x[1] - x[0])


def _slot(radius: float, x: tuple, z: float, y: tuple):
    """A slot along the part's X (its round ends' centres at x[0], x[1]) through the plate from y[0] to y[1]."""
    return _bore(radius, *y, x[0], z) + _bore(radius, *y, x[1], z) + _block(x, (z - radius, z + radius), y)


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


def _motor_seat(cfg: BaseConfig, plate):
    """`plate` less the motor's window and its 4 holes (slots of +/- travel along X when it has any)."""
    p, m = cfg.plate, cfg.motor
    y, cz = (p.y[0] - NUDGE, p.y[1] + NUDGE), m.centre[1]
    plate = plate - _block(m.window_x, (cz - m.window_half_z, cz + m.window_half_z), y)
    for x, z in motor_holes(cfg):
        plate = plate - (_slot(m.hole_dia / 2.0, (x - m.travel, x + m.travel), z, y) if m.travel > 0.0 else _bore(m.hole_dia / 2.0, *y, x, z))
    return plate


def _plate(cfg: BaseConfig):
    """The motor plate across the D: the central opening (out to the wall on the -X side), the curved slot, the
    motor's seat and, on the SolidWorks base, the belt slot (the motor mount has none)."""
    s, p, m = cfg.shell, cfg.plate, cfg.motor
    ri, y = s.r - s.wall, (p.y[0] - NUDGE, p.y[1] + NUDGE)
    plate = _d(ri, s.x1 - s.wall, p.y)
    plate = plate - _bore(p.opening_r, *y) - _sector(ri + 1.0, p.open_deg, y)
    arc = bd.CenterArc((0.0, 0.0), p.arc_slot_r, -p.arc_slot_half_deg, 2.0 * p.arc_slot_half_deg)
    plate = plate - bd.Pos(0.0, 0.0, y[0]) * bd.extrude(bd.SlotArc(arc, p.arc_slot_w), amount=y[1] - y[0])
    plate = _motor_seat(cfg, plate)
    if cfg.joint is None:
        cz = m.centre[1]
        plate = plate - (_block((s.r - 5.0, m.slot_x1), (cz - m.slot_half_z, cz + m.slot_half_z), y) - _bore(s.r, y[0] - NUDGE, y[1] + NUDGE))
    return plate


def _rim(cfg: BaseConfig):
    """The U rim under the plate round the motor's face (open toward the axis, its ends on the shell's outer round - on
    the joint face in the motor mount)."""
    s, p, m = cfg.shell, cfg.plate, cfg.motor
    cz, h, out = m.centre[1], m.rim_half, m.rim_half + m.rim_wall
    x0 = s.r - 10.0 if cfg.joint is None else cfg.joint.split_x
    rim = _block((x0, m.slot_x1 + m.rim_wall), (cz - out, cz + out), (m.rim_y0, p.y[0] + NUDGE))
    rim = rim - _block((x0 - 1.0, m.slot_x1), (cz - h, cz + h), (m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE))
    return rim - _bore(s.r, m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE) if cfg.joint is None else rim


def _side(cfg: BaseConfig, x: tuple):
    """Everything of the SolidWorks body between x[0] and x[1] (a box past it on every other side)."""
    s = cfg.shell
    return _block(x, (-s.r - 1.0, s.r + 1.0), (s.y0 - 1.0, cfg.cap.top_y + 1.0))


def _posts(cfg: BaseConfig):
    """The base's two posts at the joint: post_t thick behind the joint face, from each side wall in to the motor
    mount's inside (the window between them), from the bottom face up to the plate's underside."""
    s, p, j = cfg.shell, cfg.plate, cfg.joint
    ri, w_in = s.r - s.wall, mount_inner_half(cfg)
    x, y = (j.split_x - j.post_t, j.split_x), (s.y0, p.y[0] + NUDGE)
    return _block(x, (w_in, ri + NUDGE), y) + _block(x, (-ri - NUDGE, -w_in), y)


def _bolt_holes(cfg: BaseConfig, body):
    """`body` less the 4 M4 clearance holes along X through the posts and the ears."""
    j, st = cfg.joint, joint_stations(cfg)
    x = (st["x_post"] - 1.0, st["x_head"] + 1.0)
    for z, y in joint_bolt_points(cfg):
        body = body - _x_bore(j.bolt_dia / 2.0, x, z, y)
    return body


def build_base(cfg: BaseConfig = DEFAULT):
    """The base: the whole SolidWorks part (cfg.joint None), else its -X side up to the joint face, the posts, the M4
    holes and the nuts' hex pockets (from the posts' back face, nut.h deep)."""
    body = _cap(cfg, _walls(cfg)) + _plate(cfg)
    if cfg.joint is None:
        return single_solid(bd.Rot(-90.0, 0.0, 0.0) * (body + _rim(cfg)))
    j, st = cfg.joint, joint_stations(cfg)
    s = cfg.shell
    body = (body & _side(cfg, (-s.r - 1.0, j.split_x))) + _posts(cfg)
    body = _bolt_holes(cfg, body)
    for z, y in joint_bolt_points(cfg):
        body = body - _x_hex(j.nut_pocket_af, (st["x_post"] - NUDGE, st["x_nut_face"]), z, y)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)


def build_motor_mount(cfg: BaseConfig = DEFAULT):
    """The motor mount (cfg.joint, cfg.mount): a box from the joint face to its end wall, room round the motor's board
    inside, open underneath and toward the base; the plate across its top with the motor's seat and the U rim under
    it; an ear outside each side wall at the joint face, and the M4 holes through the ears."""
    s, p, j, mt = cfg.shell, cfg.plate, cfg.joint, cfg.mount
    w_in, x1 = mount_inner_half(cfg), mount_x1(cfg)
    w_out, y = w_in + mt.wall, (s.y0, p.y[1])
    body = _block((j.split_x, x1), (-w_out, w_out), y)
    body = body - _block((j.split_x - 1.0, x1 - mt.wall), (-w_in, w_in), (s.y0 - NUDGE, p.y[0]))
    ears = (j.split_x, j.split_x + mt.ear_t)
    body = body + _block(ears, (w_out - NUDGE, w_out + mt.ear_w), y) + _block(ears, (-w_out - mt.ear_w, -w_out + NUDGE), y)
    body = _motor_seat(cfg, body) + _rim(cfg)
    body = _bolt_holes(cfg, body)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
