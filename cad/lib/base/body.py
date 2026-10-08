"""build123d builders of the base and its motor mount from a BaseConfig, both in the base's part frame.

Every feature but the joint's bolts runs along the part's Y (the base_yaw axis), so the bodies are built in a working
frame whose +Z is the part's +Y - part (x, y, z) = working (x, -z, y), the helpers below take part coordinates - and
turned into the part frame once at the end (Rot(-90, 0, 0)), like lib/upper_arm/link.py.

With cfg.joint (DEFAULT) the SolidWorks body is cut at x = split_x: build_base keeps the -X side (the tower, the
plate's neck) and ends in two posts with the M4 holes and the nuts' pockets, the window between them opening into the
motor mount; build_motor_mount is a box of its own (cfg.mount) round the motor and its board - trussed walls and end
wall, the plate with the motor's seat - with an ear each side at the joint face for the M4. Without it (LEGACY)
build_base is the whole SolidWorks part. With a shell draft (DEFAULT) the walls lean out going down, inside and out:
_d's cylinder becomes a cone round the axis and its block a wedge between leaning planes (_dd); with cfg.foot (DEFAULT)
they stand on a flange with a straight chamfer up the wall (a cone and a wedge again) and the screw holes through it.
With neither (LEGACY) the cones are _d's cylinders and the wedges its blocks."""
from __future__ import annotations

import math
from functools import partial

from cadgen import build123d as bd

from lib.base.layout import (
    chamfer_inset,
    flare_points,
    foot_holes,
    inner_r,
    joint_bolt_points,
    joint_stations,
    motor_holes,
    mount_inner_half,
    mount_x1,
    outer_r,
    side_stub_x,
    truss_panels,
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


def _strut(axis: str, span: tuple, p0: tuple, p1: tuple, width: float):
    """A strut `width` wide along p0 -> p1 in a wall's (u, v) plane (lib/base/layout.py truss_panels(): u the part's x
    for a wall across z, its z for one across x; v the part's y), through the wall's `span` on `axis` and 1 past each
    face, 2 widths past both ends."""
    (u0, v0), (u1, v1) = p0, p1
    du, dv = u1 - u0, v1 - v0
    length, depth = math.hypot(du, dv) + 4.0 * width, span[1] - span[0] + 2.0
    uc, vc, mid = (u0 + u1) / 2.0, (v0 + v1) / 2.0, (span[0] + span[1]) / 2.0
    if axis == "z":   # the wall in the part's XY plane = the working XZ: turned about the working Y
        return bd.Pos(uc, -mid, vc) * bd.Rot(0.0, -math.degrees(math.atan2(dv, du)), 0.0) * bd.Box(length, depth, width)
    # the wall in the part's ZY plane = the working (-Y)Z: turned about the working X
    return bd.Pos(mid, -uc, vc) * bd.Rot(math.degrees(math.atan2(dv, -du)), 0.0, 0.0) * bd.Box(depth, length, width)


def _d(r: float, x1: float, y: tuple, x0: float = 0.0):
    """The D prism from y[0] to y[1]: a round of r about the axis and straight sides z = +/- r from x0 out to x1."""
    return _bore(r, *y) + _block((x0, x1), (-r, r), y)


def _cone(r0: float, r1: float, y: tuple):
    """A cone round the axis from y[0] (radius r0) to y[1] (radius r1) - _bore's cylinder when they are equal."""
    if r0 == r1:
        return _bore(r0, *y)
    return bd.Pos(0.0, 0.0, y[0]) * bd.Cone(r0, r1, y[1] - y[0], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def _wedge(r0: float, r1: float, x: tuple, y: tuple):
    """The block from x[0] to x[1] between the planes through z = +/- r0 at y[0] and z = +/- r1 at y[1]."""
    pts = [(x[0], -r0, y[0]), (x[0], r0, y[0]), (x[0], r1, y[1]), (x[0], -r1, y[1])]
    return bd.extrude(bd.Face(bd.Wire.make_polygon(pts, close=True)), amount=x[1] - x[0], dir=(1.0, 0.0, 0.0))


def _dd(r_of, x1: float, y: tuple):
    """_d with the radius r_of(y) at each end: a cone round the axis and the block out to x1 between the leaning
    planes z = +/- r_of (_d itself when the walls stand straight)."""
    r0, r1 = r_of(y[0]), r_of(y[1])
    return _cone(r0, r1, y) + _wedge(r0, r1, (0.0, x1), y)


def _foot(cfg: BaseConfig):
    """The foot, solid: the flange (_d of foot.r, t thick, its sides out to x1) and the chamfer from its top up to the
    wall - a cone round the axis, the leaning planes on the sides."""
    s, f = cfg.shell, cfg.foot
    (rf, yt), (rw, yw) = flare_points(cfg)
    return _d(f.r, s.x1, (s.y0, yt)) + _cone(rf, rw, (yt, yw)) + _wedge(rf, rw, (0.0, s.x1), (yt, yw))


def _reach(cfg: BaseConfig) -> float:
    """How far the base reaches from the axis: the walls at the bottom face, or the foot."""
    return max(outer_r(cfg, cfg.shell.y0), cfg.foot.r if cfg.foot is not None else 0.0)


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
    """The D wall (on its foot, cfg.foot) from the bottom face to the plate's top, hollow; above it the round half and
    the sides' stubs up to the cap's top face, hollow up to the chamfer under the cap (the cap a full disc). The outside
    is one cone from the bottom face to the top face, the sides' blocks on it: two cones met at the plate's top would
    fuse into a bad face."""
    s, c, p = cfg.shell, cfg.cap, cfg.plate
    r_in = partial(inner_r, cfg)
    r0, rp, rt = (outer_r(cfg, y) for y in (s.y0, p.y[1], c.top_y))
    body = (_cone(r0, rt, (s.y0, c.top_y)) + _wedge(r0, rp, (0.0, s.x1), (s.y0, p.y[1]))
            + _wedge(rp, rt, (0.0, side_stub_x(cfg)), (p.y[1], c.top_y)))
    if cfg.foot is not None:
        body = body + _foot(cfg)
    body = body - _dd(r_in, s.x1 - s.wall, (s.y0 - NUDGE, p.y[1] + NUDGE))
    body = body - _block((s.x1 - s.wall - NUDGE, s.x1 + NUDGE), (-s.notch_half_z, s.notch_half_z), (s.y0 - NUDGE, s.notch_y1))
    inset, ri, far = chamfer_inset(cfg), inner_r(cfg, c.chamfer_y0), r0 + 1.0
    cavity = _dd(r_in, far, (p.y[1] - NUDGE, c.chamfer_y0)) + bd.loft(
        [_d_section(ri, far, c.chamfer_y0), _d_section(ri - inset, far, c.underside_y)], ruled=True)
    return body - cavity


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
    ri, y = inner_r(cfg, p.y[0]), (p.y[0] - NUDGE, p.y[1] + NUDGE)   # the inside at the plate's underside, its widest
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
    """The U rim under the plate round the motor's face (open toward the axis, its ends on the shell's outer round) -
    the SolidWorks base's; the motor mount has none."""
    s, p, m = cfg.shell, cfg.plate, cfg.motor
    cz, h, out = m.centre[1], m.rim_half, m.rim_half + m.rim_wall
    rim = _block((s.r - 10.0, m.slot_x1 + m.rim_wall), (cz - out, cz + out), (m.rim_y0, p.y[0] + NUDGE))
    rim = rim - _block((s.r - 11.0, m.slot_x1), (cz - h, cz + h), (m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE))
    return rim - _bore(s.r, m.rim_y0 - NUDGE, p.y[0] + 2.0 * NUDGE)


def _side(cfg: BaseConfig, x: tuple):
    """Everything of the SolidWorks body between x[0] and x[1] (a box past it on every other side)."""
    s, half = cfg.shell, _reach(cfg) + 1.0
    return _block(x, (-half, half), (s.y0 - 1.0, cfg.cap.top_y + 1.0))


def _posts(cfg: BaseConfig):
    """The base's two posts at the joint: post_t thick behind the joint face, from each side wall in to the motor
    mount's inside (the window between them), from the bottom face up to the plate's underside - out to the inside at
    the bottom face and, when the walls lean, cut back to their outside above."""
    s, p, j = cfg.shell, cfg.plate, cfg.joint
    ri, w_in = inner_r(cfg, s.y0), mount_inner_half(cfg)
    x, y = (j.split_x - j.post_t, j.split_x), (s.y0, p.y[0] + NUDGE)
    posts = _block(x, (w_in, ri + NUDGE), y) + _block(x, (-ri - NUDGE, -w_in), y)
    if s.draft == 0.0:
        return posts
    return posts & _dd(partial(outer_r, cfg), s.x1, (s.y0 - 1.0, p.y[1]))


def _bolt_holes(cfg: BaseConfig, body):
    """`body` less the 4 M4 clearance holes along X through the posts and the ears."""
    j, st = cfg.joint, joint_stations(cfg)
    x = (st["x_post"] - 1.0, st["x_head"] + 1.0)
    for z, y in joint_bolt_points(cfg):
        body = body - _x_bore(j.bolt_dia / 2.0, x, z, y)
    return body


def _foot_holes(cfg: BaseConfig, body):
    """`body` less the foot's screw holes down through the flange and their spot-faces from the flange's top up through
    the chamfer (none without a foot)."""
    f, s = cfg.foot, cfg.shell
    if f is None:
        return body
    yt, y1 = s.y0 + f.t, s.y0 + f.flare_h + NUDGE
    for x, z in foot_holes(cfg):
        body = body - _bore(f.hole_dia / 2.0, s.y0 - NUDGE, yt + NUDGE, x, z) - _bore(f.spot_dia / 2.0, yt, y1, x, z)
    return body


def build_base(cfg: BaseConfig = DEFAULT):
    """The base: the whole SolidWorks part (cfg.joint None), else its -X side up to the joint face, the posts, the M4
    holes and the nuts' hex pockets (from the posts' back face, nut.h deep); the foot's screw holes (cfg.foot)."""
    body = _cap(cfg, _walls(cfg)) + _plate(cfg)
    if cfg.joint is None:
        body = body + _rim(cfg)
    else:
        j, st = cfg.joint, joint_stations(cfg)
        body = (body & _side(cfg, (-_reach(cfg) - 1.0, j.split_x))) + _posts(cfg)
        body = _bolt_holes(cfg, body)
        for z, y in joint_bolt_points(cfg):
            body = body - _x_hex(j.nut_pocket_af, (st["x_post"] - NUDGE, st["x_nut_face"]), z, y)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * _foot_holes(cfg, body))


def _truss_windows(cfg: BaseConfig):
    """The motor mount's truss windows (lib/base/layout.py truss_panels()): per wall the frame's inside less the V's two
    struts - three triangles - through the wall's thickness, 1 past each face."""
    cutter = None
    for pn in truss_panels(cfg):
        (u, v), (a0, a1) = pn["window"], pn["span"]
        across = (a0 - 1.0, a1 + 1.0)
        win = _block(u, across, v) if pn["axis"] == "z" else _block(across, u, v)
        for p0, p1 in pn["struts"]:
            win = win - _strut(pn["axis"], pn["span"], p0, p1, cfg.mount.strut)
        cutter = win if cutter is None else cutter + win
    return cutter


def build_motor_mount(cfg: BaseConfig = DEFAULT):
    """The motor mount (cfg.joint, cfg.mount): a box from the joint face to its end wall, room round the motor's board
    inside, open underneath and toward the base, its walls trussed; the plate across its top with the motor's seat
    (nothing under it: the slots hold the motor); an ear outside each side wall at the joint face, and the M4 holes
    through the ears."""
    s, p, j, mt = cfg.shell, cfg.plate, cfg.joint, cfg.mount
    w_in, x1 = mount_inner_half(cfg), mount_x1(cfg)
    w_out, y = w_in + mt.wall, (s.y0, p.y[1])
    body = _block((j.split_x, x1), (-w_out, w_out), y)
    body = body - _block((j.split_x - 1.0, x1 - mt.wall), (-w_in, w_in), (s.y0 - NUDGE, p.y[0]))
    ears = (j.split_x, j.split_x + mt.ear_t)
    body = body + _block(ears, (w_out - NUDGE, w_out + mt.ear_w), y) + _block(ears, (-w_out - mt.ear_w, -w_out + NUDGE), y)
    body = _motor_seat(cfg, body) - _truss_windows(cfg)
    body = _bolt_holes(cfg, body)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * body)
