"""build123d builder of j1_coupler (and of DEFAULT's motor leg, j1_motor_leg) from YawCouplerConfig, in the coupler's
part frame.

Built in the part frame directly: the disc, the flare and the hub turn about the part's Y (the base_yaw axis), the
yoke is prisms along X (the drive's axis) - their outlines in the YZ plane, lib/yaw_coupler/layout.py; DEFAULT's disc is
faceted (DiscParams.sides: a tapered prism about Y, every flat a plane), its fork (ForkParams) boxes and cylinders along
X on the drive's own axis, each leg's root the convex hull of its post's section and the disc's corners (planar faces:
lib/geom.py hull_solid)."""
from __future__ import annotations

import math

import build123d as bd

from lib.fasteners import M4_CLEAR, M4_NUT
from lib.geom import align_min, faceted_end, hex_prism, hull_solid, single_solid
from lib.style import EDGE_CHAMFER, SIDES
from lib.units import NUDGE
from lib.yaw_coupler.layout import (
    COUNTERBORE,
    HEAD_SEAT,
    NUT_SLOT,
    channel_outline,
    cheek_outline,
    disc_profile,
    disc_r,
    flare_r,
    fork_hub_bolts,
    fork_hub_leg_x,
    fork_motor_bolt_x,
    fork_motor_bolts,
    fork_motor_leg_x,
    fork_post_y,
    fork_root_points,
    fork_sleeve_bolt_deg,
    fork_sleeve_bolt_x,
    hole_points,
    middle_outline,
    nut_centres,
    socket_outline,
)
from lib.yaw_coupler.params import DEFAULT, YawCouplerConfig

_FAR = 200.0   # beyond the part: the span of a cutter or a bounding box that only one side limits


def _ycyl(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0):
    """A cylinder along the part's Y from y0 to y1, its axis through (x, z)."""
    return bd.Pos(x, y0, z) * bd.Rot(-90.0, 0.0, 0.0) * bd.Cylinder(radius, y1 - y0, align=align_min())


def _xcyl(radius: float, x0: float, x1: float, y: float, z: float = 0.0):
    """A cylinder along the part's X from x0 to x1, its axis through (y, z)."""
    return bd.Pos(x0, y, z) * bd.Rot(0.0, 90.0, 0.0) * bd.Cylinder(radius, x1 - x0, align=align_min())


def _box(x: tuple, y: tuple, z: tuple):
    """A box spanning the ranges x = (x0, x1), y = (y0, y1), z = (z0, z1)."""
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def _x_prism(outline: list, x0: float, x1: float):
    """The (y, z) outline as a prism along X from x0 to x1."""
    face = bd.Plane.YZ.offset(x0) * bd.Polygon(*outline, align=None)
    return bd.extrude(face, amount=x1 - x0, dir=(1.0, 0.0, 0.0))


def _revolved(profile: list):
    """The (r, y) profile turned about the part's Y."""
    return bd.revolve(bd.Polygon(*profile, align=None), bd.Axis.Y)


def _ycone(r0: float, r1: float, y0: float, y1: float, sides: int):
    """A faceted frustum about the part's Y from y0 (its flats r0 off the axis) to y1 (r1): a tapered prism, every flat a
    plane (a ruled loft's B-spline faces integrate per machine: lib/AGENTS.md), a flat facing each of +/-X and +/-Z."""
    h = y1 - y0
    face = faceted_end(r0, sides=sides)
    return bd.Pos(0.0, y0, 0.0) * bd.Rot(-90.0, 0.0, 0.0) * bd.extrude(face, amount=h, taper=math.degrees(math.atan2(r0 - r1, h)))


def _disc(cfg: YawCouplerConfig):
    """The drafted disc between its flats, the ears beyond them, the ring on top - or, faceted (DiscParams.sides), the
    whole frustum and the ring as many flats."""
    d = cfg.disc
    ring_x = _box((d.ring_x0, d.flat_x), (-_FAR, _FAR), (-_FAR, _FAR))
    if d.sides:
        disc = _ycone(disc_r(cfg, d.y0), d.top_r, d.y0, d.top_y, d.sides)
        return disc + (_ycone(d.ring_r, d.ring_r, d.top_y - NUDGE, d.ring_y1, d.sides) & ring_x)
    disc = _revolved(disc_profile(cfg)) & _box((-d.flat_x, d.flat_x), (-_FAR, _FAR), (-_FAR, _FAR))
    disc = disc + _ycyl(d.ear_r, d.y0, d.ear_y1)
    return disc + (_ycyl(d.ring_r, d.top_y - NUDGE, d.ring_y1) & ring_x)


def _yoke(cfg: YawCouplerConfig):
    """The cheek and the middle body, each inside the flare (the cone out of the ring's top edge); the wall round each
    socket, whole even where the flare trims the middle body."""
    d, k = cfg.disc, cfg.yoke
    y0 = d.ring_y1 - NUDGE   # a hair into the ring, so the two fuse
    flare = _revolved([(0.0, y0), (flare_r(cfg, y0), y0), (flare_r(cfg, k.axis_y), k.axis_y), (0.0, k.axis_y)])
    yoke = _x_prism(cheek_outline(cfg), *k.cheek_x) & flare
    yoke = yoke + (_x_prism(middle_outline(cfg), k.cheek_x[1] - NUDGE, k.body_x1) & flare)
    for deg in k.socket_deg:
        yoke = yoke + _x_prism(socket_outline(cfg, deg, k.socket_wall), k.cheek_x[1] - NUDGE, k.body_x1)
    return yoke


def _yoke_cuts(cfg: YawCouplerConfig):
    """The cradle, the pocket over the hub, the channel and the notch under the bottom pillar, the sockets round the
    pillars, the nut pockets."""
    d, k = cfg.disc, cfg.yoke
    (px, pz), x_out = k.pocket_half, k.cheek_x[0]
    cuts = [_xcyl(k.cradle_r, -_FAR, _FAR, k.axis_y),
            _box((-px, px), (k.pocket_y0, k.axis_y), (-pz, pz))]
    if k.channel:
        cuts += [_x_prism(channel_outline(cfg), k.cheek_x[1], k.channel_x1),
                 _xcyl(k.notch_r, k.channel_x1, d.flat_x + NUDGE, k.notch_y)]
    for deg in k.socket_deg:
        # from the cheek's inner face (the housing's output face bears on it) on through the middle body
        cuts.append(_x_prism(socket_outline(cfg, deg), k.cheek_x[1], k.body_x1 + NUDGE))
    for y, z in nut_centres(cfg):
        # a hex prism along X, a corner along Z (hex_prism's first vertex on +X turns onto -Z)
        cuts.append(bd.Pos(x_out - NUDGE, y, z) * bd.Rot(0.0, 90.0, 0.0) * hex_prism(k.nut_af, 0.0, k.nut_depth + NUDGE))
        cuts.append(_xcyl(k.bolt_hole_dia / 2.0, x_out + k.nut_depth - NUDGE, k.cheek_x[1] + NUDGE, y, z))
    return cuts


def _top(cfg: YawCouplerConfig):
    """A leg's faceted top about the drive's axis (lib/geom.py faceted_end: its flats plate_r off it), a face in the
    plane x = 0 drawn as (u, v) = (y, z)."""
    f = cfg.fork
    return faceted_end(f.plate_r, (f.axis_y, f.axis_z))


def _leg_prism(cfg: YawCouplerConfig, x0: float, x1: float, y_foot: float, seats: tuple = ()):
    """A leg from x0 to x1 along X: its faceted top on a post as wide (plate_r either side) down to y_foot, the outline of both its
    faces chamfered (lib/style.py EDGE_CHAMFER) - but the post's foot, where it stands on (or runs into) the disc, and
    the flats at `seats` (deg about the drive's axis, fork_sleeve_bolt_deg's: from this frame's +Z toward its +Y), where
    a screw's head seats across the leg's whole thickness: they stay sharp, the joint flush and the seat whole."""
    f = cfg.fork
    post = bd.Pos((y_foot + f.axis_y) / 2.0, f.axis_z) * bd.Rectangle(f.axis_y - y_foot, 2.0 * f.plate_r)
    face = bd.Plane.YZ.offset(x0) * (_top(cfg) + post).face()
    prism = single_solid(bd.extrude(face, amount=x1 - x0, dir=(1.0, 0.0, 0.0)))
    normals = [(math.sin(math.radians(a)), math.cos(math.radians(a))) for a in seats]

    def sharp(edge) -> bool:
        c = edge.center()
        return abs(c.Y - y_foot) < 1e-6 or any(
            abs((c.Y - f.axis_y) * ny + (c.Z - f.axis_z) * nz - f.plate_r) < 1e-6 for ny, nz in normals)
    ends = prism.faces().sort_by(bd.Axis.X)
    edges = [e for end in (ends[0], ends[-1]) for e in end.outer_wire().edges() if not sharp(e)]
    return prism.chamfer(EDGE_CHAMFER, None, edges)


def _root(cfg: YawCouplerConfig, x_in: float, x_out: float):
    """The leg's faceted root between its inner face x_in and outer face x_out: the hull of its post's section at
    fork_post_y and the disc's corners outboard of the inner face (lib/yaw_coupler/layout.py fork_root_points)."""
    return hull_solid(fork_root_points(cfg, x_in, x_out))


def _fork(cfg: YawCouplerConfig):
    """The hub leg - its faceted top round the drive's axis on a post as wide down to fork_post_y (_leg_prism), its
    faceted root under it -, and the motor leg's root under its post and inboard of its outer face, the disc's there."""
    d = cfg.disc
    (hx0, hx1), (mx0, mx1) = fork_hub_leg_x(cfg), fork_motor_leg_x(cfg)
    leg = _leg_prism(cfg, hx0, hx1, fork_post_y(cfg)) + _root(cfg, hx1, hx0)
    return leg + (_root(cfg, mx0, mx1) & _box((-_FAR, mx1), (-_FAR, d.top_y), (-_FAR, _FAR)))


def _fork_cuts(cfg: YawCouplerConfig):
    """The pocket over the hub; the hub's bolts through the hub leg, counterbored from outside; the motor side cut back
    for the motor leg (the ring motor_leg_gap off it, the disc at its outer face, where the foot butts on); the motor
    leg's 2 M4s and their nuts' slots, from the disc's top face down past the screws, flats across z."""
    f, d, k = cfg.fork, cfg.disc, cfg.yoke
    (hx0, hx1), (mx0, mx1), (px, pz) = fork_hub_leg_x(cfg), fork_motor_leg_x(cfg), k.pocket_half
    cuts = [_box((-px, px), (k.pocket_y0, f.axis_y), (-pz, pz)),
            _box((mx0 - f.motor_leg_gap, _FAR), (d.top_y, _FAR), (-_FAR, _FAR)),
            _box((mx1, _FAR), (-_FAR, _FAR), (-_FAR, _FAR))]
    for y, z in fork_hub_bolts(cfg):
        cuts += [_xcyl(M4_CLEAR / 2.0, hx0 - NUDGE, hx1 + NUDGE, y, z), _xcyl(COUNTERBORE / 2.0, hx0 - NUDGE, hx0 + HEAD_SEAT, y, z)]
    _, nut_x1, nut_x0 = fork_motor_bolt_x(cfg)
    half_af, corner = (M4_NUT.af + NUT_SLOT) / 2.0, (M4_NUT.af + NUT_SLOT) / math.sqrt(3.0)
    for y, z in fork_motor_bolts(cfg):
        cuts += [_xcyl(M4_CLEAR / 2.0, nut_x0 - 1.0, mx1 + NUDGE, y, z),
                 _box((nut_x0, nut_x1), (y - corner, d.top_y + NUDGE), (z - half_af, z + half_af))]
    return cuts


def _sleeve_slot(cfg: YawCouplerConfig, deg: float):
    """A radial hole through the ring for an M4 into the sleeve at deg about the drive's axis (from its +X, this frame's
    +Z, toward +Y): M4_CLEAR wide, sleeve_slot_len along the axis - built along +Y, then turned about the axis."""
    f = cfg.fork
    corner = f.plate_r / math.cos(math.pi / SIDES)   # out through the faceted top, its corners too
    x, half, r0, r1 = fork_sleeve_bolt_x(cfg), (f.sleeve_slot_len - M4_CLEAR) / 2.0, f.ring_bore_dia / 2.0 - 1.0, corner + 1.0
    slot = _ycyl(M4_CLEAR / 2.0, r0, r1, x - half) + _ycyl(M4_CLEAR / 2.0, r0, r1, x + half)
    slot = slot + _box((x - half, x + half), (r0, r1), (-M4_CLEAR / 2.0, M4_CLEAR / 2.0))
    return bd.Pos(0.0, f.axis_y, f.axis_z) * bd.Rot(90.0 - deg, 0.0, 0.0) * slot


def build_motor_leg(cfg: YawCouplerConfig):
    """DEFAULT's motor leg (parts/base/j1_motor_leg), in the coupler's frame: a solid faceted top round the drive's
    axis (_leg_prism: flats plate_r off it, its outline chamfered but on the flats the sleeve's screws seat on), bored
    to the sleeve, on a post as wide down to fork_post_y, its faceted root under it standing on the disc's top face; its
    foot the root's part past the leg's outer face, down to the disc's underside; the 2 M4s through the foot from
    counterbores; the sleeve's radial M4s' slotted holes through the top."""
    f, d = cfg.fork, cfg.disc
    x0, x1 = fork_motor_leg_x(cfg)
    leg = _leg_prism(cfg, x0, x1, fork_post_y(cfg), seats=fork_sleeve_bolt_deg(cfg))
    above = _box((-_FAR, _FAR), (d.top_y, _FAR), (-_FAR, _FAR)) + _box((x1, _FAR), (-_FAR, _FAR), (-_FAR, _FAR))
    leg = leg + (_root(cfg, x0, x1) & above)
    leg = leg - _xcyl(f.ring_bore_dia / 2.0, x0 - NUDGE, _FAR, f.axis_y, f.axis_z)
    seat_x = fork_motor_bolt_x(cfg)[0]
    for y, z in fork_motor_bolts(cfg):
        leg = leg - _xcyl(M4_CLEAR / 2.0, x1 - NUDGE, _FAR, y, z) - _xcyl(COUNTERBORE / 2.0, seat_x, _FAR, y, z)
    for deg in fork_sleeve_bolt_deg(cfg):
        leg = leg - _sleeve_slot(cfg, deg)
    return single_solid(leg)


def toward_axis(shape, deg: float, r: float):
    """`shape`, built at the origin along +Z from z 0, set to point in at the axis from radius r at deg (from +X toward
    +Y, in the z = 0 plane): its +Z radially in, its +X along the tangent - the sleeve's radial screws and their nuts in
    their frame on the drive's axis (lib/yaw_coupler/layout.py fork_sleeve_screw_frame)."""
    ux, uy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return shape.moved(bd.Location(bd.Plane(origin=(r * ux, r * uy, 0.0), x_dir=(-uy, ux, 0.0), z_dir=(-ux, -uy, 0.0))))


def _stub(cfg: YawCouplerConfig):
    """The stub from its end up into the recess's ceiling, the round on its end's outer edge."""
    h = cfg.hub
    stub = _ycyl(h.stub_dia / 2.0, h.stub_y0, h.recess_y1 + NUDGE)
    if h.stub_round > 0.0:
        stub = stub.fillet(h.stub_round, [stub.edges().sort_by(bd.Axis.Y)[0]])
    return stub


def build_yaw_coupler(cfg: YawCouplerConfig = DEFAULT):
    d, h, k = cfg.disc, cfg.hub, cfg.yoke
    if cfg.fork is None:
        body, cuts = _disc(cfg) + _yoke(cfg), _yoke_cuts(cfg)
    else:
        body, cuts = _disc(cfg) + _fork(cfg), _fork_cuts(cfg)
    for cut in cuts:
        body = body - cut
    body = body - _ycyl(h.recess_dia / 2.0, d.y0 - NUDGE, h.recess_y1)
    body = body + _stub(cfg)
    body = body - _ycyl(h.bore_dia / 2.0, h.stub_y0 - NUDGE, k.pocket_y0 + NUDGE)
    for x, z in hole_points(cfg):
        body = body - _ycyl(h.hole_dia / 2.0, h.stub_y0 - NUDGE, k.pocket_y0 + NUDGE, x, z)
        if h.nut_af is not None:
            # a hex prism along Y, a corner along Z (hex_prism's vertex at +90 deg turns onto -Z)
            body = body - bd.Pos(x, k.pocket_y0 - h.nut_depth, z) * bd.Rot(-90.0, 0.0, 0.0) * hex_prism(
                h.nut_af, math.pi / 2.0, h.nut_depth + NUDGE)
    return single_solid(body)
