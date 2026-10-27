"""build123d builders of the base and its motor mount from a BaseConfig, both in the base's part frame.

Every feature but the joint's bolts runs along the part's Y (the base_yaw axis), so the bodies are built in a working
frame whose +Z is the part's +Y - part (x, y, z) = working (x, -z, y), the helpers below take part coordinates - and
turned into the part frame once at the end (Rot(-90, 0, 0)), like lib/upper_arm/link.py.

With cfg.joint (DEFAULT) the SolidWorks body is cut at x = split_x: build_base keeps the -X side (the tower, the
plate's neck) and ends in two posts with the M4 holes and the nuts' pockets, the window between them opening into the
motor mount; build_motor_mount (cfg.mount) is the base's other half - the same leaning walls carried on from the
joint face to a round end about mount_end_x() (_mount_d: the wedge and a cone round that centre), the foot round it,
the plate with the motor's seat and the belt's idler's seat (cfg.idler), and two posts at the joint facing the base's,
the M4 through both; build_yaw_idler_spacer is that idler's printed spacer. Without it (LEGACY)
build_base is the whole SolidWorks part. With a shell draft (DEFAULT) the walls lean out going down, inside and out:
_d's cylinder becomes a cone round the axis and its block a wedge between leaning planes (_dd); with cfg.foot (DEFAULT)
they stand on a flange with a straight chamfer up the wall (a cone and a wedge again) and the screw holes through it;
with cfg.slits (DEFAULT) a band of upright slits runs through both parts' walls (_slit_cuts).
With neither (LEGACY) the cones are _d's cylinders and the wedges its blocks."""
from __future__ import annotations

import math
from functools import partial

import build123d as bd

from lib.base.layout import (
    chamfer_inset,
    flare_points,
    foot_holes,
    inner_r,
    joint_bolt_points,
    joint_stations,
    motor_face_y,
    motor_holes,
    motor_screw_y,
    mount_end_x,
    mount_foot_holes,
    mount_inner_half,
    outer_r,
    side_stub_x,
    slit_band,
    slits,
    yaw_idler_slot,
    yaw_idler_spacer_h,
)
from lib.base.params import DEFAULT, BaseConfig
from lib.geom import align_min, cylinder, hex_circumdiameter, hex_prism, single_solid, through
from lib.motors import NEMA17_FACE
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


def _cone(r0: float, r1: float, y: tuple, x: float = 0.0):
    """A cone round the axis (or round the vertical through (x, 0)) from y[0] (radius r0) to y[1] (radius r1) - _bore's
    cylinder when they are equal."""
    if r0 == r1:
        return _bore(r0, *y, x)
    return bd.Pos(x, 0.0, y[0]) * bd.Cone(r0, r1, y[1] - y[0], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


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
    """`plate` less the motor's window and its 4 holes (slots of +/- travel along X when it has any) - or, with
    motor.holes (DEFAULT), with the pad under it round the motor's face (pad_t, the NEMA 17 face's square), less the
    pilot's round hole and the 4 round holes through both, each with its counterbore (pocket_dia, the screw's head_h
    deep) from the plate's top: the heads flush."""
    p, m = cfg.plate, cfg.motor
    y, (cx, cz) = (p.y[0] - NUDGE, p.y[1] + NUDGE), m.centre
    if m.holes is None:
        plate = plate - _block(m.window_x, (cz - m.window_half_z, cz + m.window_half_z), y)
        for x, z in motor_holes(cfg):
            plate = plate - (_slot(m.hole_dia / 2.0, (x - m.travel, x + m.travel), z, y) if m.travel > 0.0 else _bore(m.hole_dia / 2.0, *y, x, z))
        return plate
    h, face, half = m.holes, motor_face_y(cfg), NEMA17_FACE / 2.0
    if m.pad_t > 0.0:
        plate = plate + _block((cx - half, cx + half), (cz - half, cz + half), (face, p.y[0] + NUDGE))
    y = (face - NUDGE, p.y[1] + NUDGE)
    plate = plate - _bore(h.pilot_dia / 2.0, *y, cx, cz)
    for x, z in motor_holes(cfg):
        plate = plate - _bore(m.hole_dia / 2.0, *y, x, z) - _bore(h.pocket_dia / 2.0, motor_screw_y(cfg), y[1], x, z)
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


def _foot_holes(cfg: BaseConfig, body, holes=None):
    """`body` less the foot's screw holes (the base's, or `holes`) down through the flange and their spot-faces from the
    flange's top up through the chamfer (none without a foot)."""
    f, s = cfg.foot, cfg.shell
    if f is None:
        return body
    yt, y1 = s.y0 + f.t, s.y0 + f.flare_h + NUDGE
    for x, z in foot_holes(cfg) if holes is None else holes:
        body = body - _bore(f.hole_dia / 2.0, s.y0 - NUDGE, yt + NUDGE, x, z) - _bore(f.spot_dia / 2.0, yt, y1, x, z)
    return body


def _slit_cuts(cfg: BaseConfig, body, part: str):
    """`body` less its slits (cfg.slits; lib/base/layout.py slits(): `part` "base" or "mount"): each a round-ended slot
    w x h in the plane across its normal at the band's middle, cut horizontally through the leaning wall - the wall's
    half and the lean over half the slit's height each side of the mid-surface, 1 past."""
    if cfg.slits is None:
        return body
    sl, s = cfg.slits, cfg.shell
    y0, y1 = slit_band(cfg)
    depth = s.draft * sl.h / 2.0 + s.wall / 2.0 + 1.0
    cutter = None
    for x, z, deg in slits(cfg, part):
        a = math.radians(deg)
        plane = bd.Plane(origin=(x, -z, (y0 + y1) / 2.0), x_dir=(math.sin(a), math.cos(a), 0.0),
                         z_dir=(math.cos(a), -math.sin(a), 0.0))
        cut = bd.extrude(plane * bd.SlotOverall(sl.h, sl.w, rotation=90.0), amount=depth, both=True)
        cutter = cut if cutter is None else cutter + cut
    return body - cutter


def build_base(cfg: BaseConfig = DEFAULT):
    """The base: the whole SolidWorks part (cfg.joint None), else its -X side up to the joint face, the posts, the M4
    holes and the nuts' hex pockets (from the posts' back face, nut.h deep); the foot's screw holes (cfg.foot), the
    slits (cfg.slits)."""
    body = _cap(cfg, _walls(cfg)) + _plate(cfg)
    if cfg.joint is None:
        body = body + _rim(cfg)
    else:
        j, st = cfg.joint, joint_stations(cfg)
        body = (body & _side(cfg, (-_reach(cfg) - 1.0, j.split_x))) + _posts(cfg)
        body = _bolt_holes(cfg, body)
        for z, y in joint_bolt_points(cfg):
            body = body - _x_hex(j.nut_pocket_af, (st["x_post"] - NUDGE, st["x_nut_face"]), z, y)
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * _slit_cuts(cfg, _foot_holes(cfg, body), "base"))


def _band(p0: tuple, p1: tuple, half_w: float, past: float, y: tuple):
    """A box half_w either side of the line p0 -> p1 (part-frame (x, z) points), `past` beyond each end, from y[0] to
    y[1]."""
    (a0, b0), (a1, b1) = _pt(*p0), _pt(*p1)
    length = math.hypot(a1 - a0, b1 - b0) + 2.0 * past
    box = bd.Box(length, 2.0 * half_w, y[1] - y[0], align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return bd.Pos((a0 + a1) / 2.0, (b0 + b1) / 2.0, y[0]) * bd.Rot(0.0, 0.0, math.degrees(math.atan2(b1 - b0, a1 - a0))) * box


def _stadium(p0: tuple, p1: tuple, radius: float, y: tuple):
    """A slot of `radius` round the line p0 -> p1 (part-frame (x, z): its round ends' centres), from y[0] to y[1]."""
    return _band(p0, p1, radius, 0.0, y) + _bore(radius, *y, *p0) + _bore(radius, *y, *p1)


def _idler_seat(cfg: BaseConfig, body):
    """`body` (the mount) with the base_yaw idler's seat (YawIdlerParams), both across the belt between its slot's ends
    (lib/base/layout.py yaw_idler_slot()): the axle's slot (slot_w) through the plate, and under the plate two rails
    either side of the nut's slot - its flats + nut_fit across, its corners + nut_fit past each end, closed there by
    rail_wall - rail_h deep (the nut bears up on the plate's underside between them), inside the walls."""
    i, (y0, y1) = cfg.idler, cfg.plate.y
    ends = yaw_idler_slot(cfg)
    half_nut, reach = (i.nut.af + i.nut_fit) / 2.0, hex_circumdiameter(i.nut.af) / 2.0 + i.nut_fit
    rails = _band(*ends, half_nut + i.rail_wall, reach + i.rail_wall, (y0 - i.rail_h, y0 + NUDGE))
    rails = rails & _mount_d(cfg, partial(inner_r, cfg), (y0 - i.rail_h - 1.0, y0 + 1.0), cfg.joint.split_x)
    body = body + rails - _band(*ends, half_nut, reach, (y0 - i.rail_h - 1.0, y0))
    return body - _stadium(*ends, i.slot_w / 2.0, (y0 - i.rail_h - 1.0, y1 + 1.0))


def _mount_d(cfg: BaseConfig, r_of, y: tuple, x0: float):
    """The motor mount's D from y[0] to y[1]: the straight sides z = +/- r_of(y) from x0 out to the end round's centre
    (mount_end_x()) and the end round about it - a wedge and a cone, the radius r_of at each end -, cut at x0."""
    cx, r0, r1 = mount_end_x(cfg), r_of(y[0]), r_of(y[1])
    far = max(r0, r1) + 1.0
    return (_wedge(r0, r1, (x0, cx), y) + _cone(r0, r1, y, cx)) & _block((x0, cx + far), (-far, far), (y[0] - 1.0, y[1] + 1.0))


def _mount_foot(cfg: BaseConfig):
    """The foot round the motor mount, solid: the flange (straight sides z = +/- foot.r from the joint face to the end's
    centre, its round about it) and the chamfer from its top up to the wall - the base's foot (_foot), mirrored."""
    s, f, x0, cx = cfg.shell, cfg.foot, cfg.joint.split_x, mount_end_x(cfg)
    (rf, yt), (rw, yw) = flare_points(cfg)
    foot = (_block((x0, cx), (-f.r, f.r), (s.y0, yt)) + _bore(f.r, s.y0, yt, cx)
            + _wedge(rf, rw, (x0, cx), (yt, yw)) + _cone(rf, rw, (yt, yw), cx))
    return foot & _block((x0, cx + f.r + 1.0), (-f.r - 1.0, f.r + 1.0), (s.y0 - 1.0, yw + 1.0))


def build_motor_mount(cfg: BaseConfig = DEFAULT):
    """The motor mount (cfg.joint, cfg.mount): the base's D carried on from the joint face to a round end (_mount_d),
    hollow, `shell.wall` thick, open underneath and toward the base, on the foot (_mount_foot) and its screw holes; the
    plate across its top with the motor's seat (_motor_seat: under it only the pad, the screws hold the motor) and, with cfg.idler, the
    belt's tensioner's seat (_idler_seat); two posts at the joint face, post_t deep, from the window out to the walls,
    and the M4 holes through them; the slits (cfg.slits)."""
    s, p, j, mt = cfg.shell, cfg.plate, cfg.joint, cfg.mount
    x0, r_out, r_in = j.split_x, partial(outer_r, cfg), partial(inner_r, cfg)
    body = _mount_d(cfg, r_out, (s.y0, p.y[1]), x0)
    if cfg.foot is not None:
        body = body + _mount_foot(cfg)
    body = body - _mount_d(cfg, r_in, (s.y0 - NUDGE, p.y[0]), x0 - 1.0)
    ri, w_in, x, y = inner_r(cfg, s.y0), mount_inner_half(cfg), (x0, x0 + mt.post_t), (s.y0, p.y[0] + NUDGE)
    posts = _block(x, (w_in, ri + NUDGE), y) + _block(x, (-ri - NUDGE, -w_in), y)
    body = body + (posts & _mount_d(cfg, r_out, (s.y0 - 1.0, p.y[1]), x0))
    body = _motor_seat(cfg, body)
    if cfg.idler is not None:
        body = _idler_seat(cfg, body)
    body = _foot_holes(cfg, _bolt_holes(cfg, body), mount_foot_holes(cfg))
    return single_solid(bd.Rot(-90.0, 0.0, 0.0) * _slit_cuts(cfg, body, "mount"))


def build_yaw_idler_spacer(cfg: BaseConfig = DEFAULT):
    """The base_yaw idler's printed spacer (YawIdlerParams; parts/base/yaw_idler_spacer), axis on Z: its land
    (land_dia x land_h) on z=0, on the idler's bearing inner ring; its riser (riser_dia) on up to
    z = yaw_idler_spacer_h(), on the mount's plate; the axle's slot_w through both."""
    i, height = cfg.idler, yaw_idler_spacer_h(cfg)
    body = cylinder(i.land_dia / 2.0, i.land_h) + cylinder(i.riser_dia / 2.0, height - i.land_h, z0=i.land_h)
    return single_solid(body - through(i.slot_w / 2.0, height))
