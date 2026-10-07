"""build123d builder of j1_link (the upper arm) from an UpperArmConfig, in j1_link's part frame.

Every feature runs along the part's Y (its thickness), so the body is built in a working frame whose +Z is the
part's +Y - part (x, y, z) = working (x, -z, y), the helpers below take part coordinates - and turned into the part
frame once at the end (Rot(-90, 0, 0)). DEFAULT's arm (ArmParams) - one flat slab in place of the plate and the pad,
the elbow motor's hole through it - adds the cycloidal drive's shell body, built in the drive's frame
(lib/cycloidal/housing.py build_shell_body), through drive_frame()."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.cycloidal.housing import build_shell_body
from lib.forearm.link import stadium, x_cylinder
from lib.geom import align_min, cylinder, single_solid
from lib.units import NUDGE
from lib.upper_arm.layout import arm_slide, cove_axes, hub_bolt_points, pad_holes, socket_points
from lib.upper_arm.params import DEFAULT, UpperArmConfig

_FAR = 400.0   # past the part: the span of a cutter only its radius limits


def _bore(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0):
    """A cylinder along the part's Y from y0 to y1, its axis through (x, z)."""
    return cylinder(radius, y1 - y0, (x, -z), z0=y0)


def _block(x: tuple, z: tuple, y: tuple):
    """A box spanning the part-frame ranges x = (x0, x1), z = (z0, z1), y = (y0, y1)."""
    (x0, x1), (z0, z1), (y0, y1) = x, z, y
    return bd.Pos((x0 + x1) / 2.0, -(z0 + z1) / 2.0, y0) * bd.Box(x1 - x0, z1 - z0, y1 - y0, align=align_min())


def _slot(x: float, half_len: float, width: float, y0: float, y1: float):
    """A stadium slot along the part's Z (end centres at z = +/- half_len) through x, from y0 to y1."""
    return bd.Pos(x, 0.0, y0) * bd.Rot(0.0, 0.0, 90.0) * bd.extrude(bd.SlotCenterToCenter(2.0 * half_len, width), amount=y1 - y0)


def _cut_stepped_slot(body, cfg: UpperArmConfig, top: float):
    """`body` less the slot at SlotParams.stepped_x: through from its counterbore's floor up to `top`, the counterbore
    from the elbow half's underside."""
    sl, e = cfg.slots, cfg.elbow
    body = body - _slot(sl.stepped_x, sl.stepped_half_len, sl.width, sl.counterbore_y - NUDGE, top)
    return body - _slot(sl.stepped_x, sl.stepped_half_len, sl.counterbore_w, e.y0 - NUDGE, sl.counterbore_y)


def _cut_elbow_stack(body, cfg: UpperArmConfig, top: float):
    """`body` less the elbow axis's bearing stack: the recess from `top`, the bore, the lip, the seat from below."""
    e, x = cfg.elbow, cfg.slab.elbow_x
    body = body - _bore(e.recess_dia / 2.0, e.recess_y, top, x)
    body = body - _bore(e.bore_dia / 2.0, e.lip_y[1], e.recess_y + NUDGE, x)
    body = body - _bore(e.lip_dia / 2.0, e.lip_y[0] - NUDGE, e.lip_y[1] + NUDGE, x)
    return body - _bore(e.seat_dia / 2.0, e.y0 - NUDGE, e.lip_y[0], x)


def _plate(cfg: UpperArmConfig):
    """The stadium plate with its lip and rounded edge, the elbow half's step below, the square opening over the pad,
    the slots, the second stage's bearing seats (if any) and the elbow bearing stack."""
    s, e, sl, b = cfg.slab, cfg.elbow, cfg.slots, cfg.bearing
    top = s.lip_top + NUDGE
    body = stadium(s.elbow_x, 2.0 * s.r, s.y1 - e.y0, s.elbow_x / 2.0, e.y0)
    body = body.fillet(s.round_r, body.faces().sort_by(bd.Axis.Z)[-1].edges())
    body = body + stadium(s.elbow_x, 2.0 * s.lip_r, s.lip_top - s.y1, s.elbow_x / 2.0, s.y1)
    if e.relief_r:
        body = body - _bore(e.relief_r, e.relief_y, top, s.elbow_x)
    # the shoulder half is thinner: its underside, the 45 degree chamfer, the step down to the elbow half
    drop = e.step_x - e.chamfer_x
    step = bd.Plane.XZ * bd.Polygon((-2.0 * s.r, s.y0), (e.chamfer_x, s.y0), (e.step_x, s.y0 - drop), (e.step_x, e.y0 - 1.0),
                                    (-2.0 * s.r, e.y0 - 1.0), align=None)
    body = body - bd.extrude(step, amount=2.0 * s.r, both=True)
    # the square opening over the pad
    oh, px = cfg.hub.opening_half, cfg.pad.x
    body = body - _block((px - oh, px + oh), (-oh, oh), (s.y0 - 1.0, top))
    # the slots: two through, one with a counterbore from below
    for x in sl.through_x:
        body = body - _slot(x, sl.through_half_len, sl.width, s.y0 - NUDGE, top)
    body = _cut_stepped_slot(body, cfg, top)
    # the second stage (BearingParams.x): the boss under the plate, a seat from each side, the hole through the web
    if b is not None:
        body = body + _bore(b.boss_dia / 2.0, b.boss_y, s.y0 + NUDGE, b.x)
        body = body - _bore(b.seat_dia / 2.0, b.web_y[1], top, b.x)
        body = body - _bore(b.hole_dia / 2.0, b.web_y[0] - NUDGE, b.web_y[1] + NUDGE, b.x)
        body = body - _bore(b.seat_dia / 2.0, b.boss_y - NUDGE, b.web_y[0], b.x)
    return _cut_elbow_stack(body, cfg, top)


def _pad(cfg: UpperArmConfig):
    """The motor pad: the tube with its root flare and the R10 fills, hollowed, the floor's pilot opening and holes,
    the four windows."""
    p, oh, y0 = cfg.pad, cfg.hub.opening_half, cfg.slab.y0
    h, wh, out = p.half, p.window_half, p.half + p.flare + 1.0
    pad = _block((-h, h), (-h, h), (p.face_y, y0))
    pad = pad + bd.loft([bd.Plane.XY.offset(y0 - p.flare) * bd.Rectangle(2.0 * h, 2.0 * h),
                         bd.Plane.XY.offset(y0) * bd.Rectangle(2.0 * (h + p.flare), 2.0 * (h + p.flare))], ruled=True)
    # the fills in the opening: a cove along Z on the -X side, one along X on the +X half of each Z side
    (cx, cy), (zy, zz) = cove_axes(cfg)
    cove = _block((-oh, cx), (-oh, oh), (y0, cy))
    pad = pad + (cove - bd.Pos(cx, 0.0, cy) * bd.Rot(90.0, 0.0, 0.0) * bd.Cylinder(p.cove_r, 2.0 * oh + 2.0))
    for sz in (1.0, -1.0):
        cove = _block((wh, oh), sorted((sz * p.cavity_half_z, sz * oh)), (y0, zy))
        pad = pad + (cove - x_cylinder(p.cove_r, oh - wh + 2.0, (-sz * zz, zy), wh - 1.0))
    # hollow it: the cavity down to the floor, the pilot opening through the floor
    pad = pad - _block(p.cavity_x, (-p.cavity_half_z, p.cavity_half_z), (p.floor_y, cfg.slab.lip_top + 1.0))
    pad = pad - _block((-p.pilot_half, p.pilot_half), (-p.pilot_half, p.pilot_half), (p.face_y - 1.0, p.floor_y + 1.0))
    # the windows: -X and +/-Z through the wall, +X through the wall and the floor into the pilot opening
    full = (p.face_y - 1.0, cfg.slab.lip_top + 1.0)
    pad = pad - _block((-out, p.cavity_x[0]), (-wh, wh), full)
    pad = pad - _block((p.pilot_half, out), (-wh, wh), full)
    for z in ((p.cavity_half_z, out), (-out, -p.cavity_half_z)):
        pad = pad - _block((-wh, wh), z, full)
    for x, z, dia in pad_holes(cfg):
        pad = pad - _bore(dia / 2.0, p.face_y - NUDGE, p.floor_y + NUDGE, x, z)
    return pad


def _slab(cfg: UpperArmConfig):
    """The arm rising off the drive's shell (ArmParams), in the unslid plate's frame: one flat slab of the plate's
    outline (the stadium from the shoulder axis to the elbow axis) from the elbow half's underside up to the relief's
    floor, end to end; the stepped slot and the elbow bearing stack; the elbow motor's square hole through it about the
    pad's axis, a web left across the hole under the motor's face (SlabParams.y0: y_outer once slid) with the hole for
    its pilot and its 4 holes (PadParams.holes), as the drive's motor plate holds its motor: motor_plate_t of web under
    the screws' heads, the heads flush in pockets in its underside (ArmParams.motor_screw, motor_pocket_dia)."""
    s, e, a, px = cfg.slab, cfg.elbow, cfg.arm, cfg.pad.x
    top = e.relief_y + NUDGE
    body = stadium(s.elbow_x, 2.0 * s.r, e.relief_y - e.y0, s.elbow_x / 2.0, e.y0)
    body = _cut_elbow_stack(_cut_stepped_slot(body, cfg, top), cfg, top)
    hh, face, t, head = a.motor_hole_half, s.y0, a.motor_plate_t, a.motor_screw.head_h
    body = body - _block((px - hh, px + hh), (-hh, hh), (e.y0 - 1.0, top + 1.0))
    web = _block((px - hh, px + hh), (-hh, hh), (face - t - head, face))
    web = web - _bore(a.motor_pilot_dia / 2.0, face - t - head - 1.0, face + 1.0, px)
    for x, z, dia in pad_holes(cfg):
        web = web - _bore(dia / 2.0, face - t - head - 1.0, face + 1.0, px + x, z)
        web = web - _bore(a.motor_pocket_dia / 2.0, face - t - head - 1.0, face - t, px + x, z)
    return body + web


def drive_frame(cfg: UpperArmConfig = DEFAULT) -> bd.Location:
    """The cycloidal drive's frame in this one (ArmParams): its axis this frame's Y, its +Z this frame's -Y, its z
    drive_z_at_y0 at y = 0, its +X at drive_x_deg."""
    a = cfg.arm
    t = math.radians(a.drive_x_deg)
    return bd.Location(bd.Plane(origin=(0.0, a.drive_z_at_y0, 0.0), x_dir=(math.cos(t), 0.0, math.sin(t)), z_dir=(0.0, -1.0, 0.0)))


def build_link(cfg: UpperArmConfig = DEFAULT):
    s, e = cfg.slab, cfg.elbow
    if cfg.arm is not None:
        # the slab, slid along Y with the plate (its shoulder half's underside onto y_outer), cut back inside the
        # shell's wall
        body = bd.Pos(0.0, 0.0, arm_slide(cfg)) * _slab(cfg) - _bore(cfg.arm.fuse_r, -_FAR, _FAR)
    else:
        body = _plate(cfg) + bd.Pos(cfg.pad.x, 0.0, 0.0) * _pad(cfg)
        # the drive's hub bolts, through the plate (the pad's windows lie under them)
        for x, z in hub_bolt_points(cfg):
            body = body - _bore(cfg.hub.bolt_dia / 2.0, s.y0 - NUDGE, s.lip_top + NUDGE, x, z)
    # the cap's locating sockets (LEGACY)
    shoulder, elbow = socket_points(cfg)
    if cfg.sockets is not None:
        r, depth = cfg.sockets.dia / 2.0, cfg.sockets.depth
        for x, z in shoulder:
            body = body - _bore(r, s.y0 - NUDGE, s.y0 + depth, x, z)
        for x, z in elbow:
            body = body - _bore(r, e.y0 - NUDGE, e.y0 + depth, x, z)
    part = bd.Rot(-90.0, 0.0, 0.0) * body
    if cfg.arm is not None:   # one print with the shell's body round the discs
        part = part + drive_frame(cfg) * build_shell_body(DRIVE)
    return single_solid(part)
