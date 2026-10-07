"""build123d builder of j1_coupler (and of DEFAULT's motor leg, j1_motor_leg) from YawCouplerConfig, in the coupler's
part frame.

Built in the part frame directly: the disc, the flare and the hub turn about the part's Y (the base_yaw axis), the
yoke is prisms along X (the drive's axis) - their outlines in the YZ plane, lib/yaw_coupler/layout.py; DEFAULT's fork
(ForkParams) is boxes and cylinders along X on the drive's own axis, and the disc's drafted side carried up the legs'
outer faces, a cone about Y."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.fasteners import M4_CLEAR, M4_NUT
from lib.geom import align_min, hex_prism, single_solid
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
    fork_flare_top,
    fork_hub_bolts,
    fork_hub_leg_x,
    fork_motor_bolt_x,
    fork_motor_bolts,
    fork_motor_leg_x,
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


def _disc(cfg: YawCouplerConfig):
    """The drafted disc between its flats, the ears beyond them, the ring on top."""
    d = cfg.disc
    disc = _revolved(disc_profile(cfg)) & _box((-d.flat_x, d.flat_x), (-_FAR, _FAR), (-_FAR, _FAR))
    disc = disc + _ycyl(d.ear_r, d.y0, d.ear_y1)
    ring = _ycyl(d.ring_r, d.top_y - NUDGE, d.ring_y1) & _box((d.ring_x0, d.flat_x), (-_FAR, _FAR), (-_FAR, _FAR))
    return disc + ring


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


def _cone(cfg: YawCouplerConfig, y0: float, y1: float):
    """The disc's drafted side as a solid about the part's Y from y0 to y1 - past the disc's top face, the legs' outer
    faces."""
    return _revolved([(0.0, y0), (disc_r(cfg, y0), y0), (disc_r(cfg, y1), y1), (0.0, y1)])


def _post_z(cfg: YawCouplerConfig) -> tuple[float, float]:
    """A leg's post, across: leg_half_z either side of the drive's axis."""
    f = cfg.fork
    return (f.axis_z - f.leg_half_z, f.axis_z + f.leg_half_z)


def _fork(cfg: YawCouplerConfig):
    """The hub leg - its disc round the drive's axis on a post down to the disc, the disc's draft up its outer face - and
    the disc drafted round to its rim under both legs (its flats and ears filled): on the motor side to the leg's outer
    face, the motor leg's foot past it."""
    f, d = cfg.fork, cfg.disc
    (hx0, hx1), (mx0, mx1) = fork_hub_leg_x(cfg), fork_motor_leg_x(cfg)
    leg = _xcyl(f.hub_plate_r, hx0, hx1, f.axis_y, f.axis_z) + _box((hx0, hx1), (d.y0, f.axis_y), _post_z(cfg))
    leg = leg + (_cone(cfg, d.y0, fork_flare_top(cfg)) & _box((-_FAR, hx1), (d.y0, _FAR), _post_z(cfg)))
    rim = _box((-_FAR, hx1), (d.y0, d.top_y), (-_FAR, _FAR)) + _box((mx0, mx1), (d.y0, d.top_y), (-_FAR, _FAR))
    return leg + (_cone(cfg, d.y0, d.top_y) & rim)


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


def build_motor_leg(cfg: YawCouplerConfig):
    """DEFAULT's motor leg (parts/base/j1_motor_leg), in the coupler's frame: a solid plate_r ring round the drive's
    axis, bored to the sleeve, on a post standing on the disc's top face, the disc's draft up its outer face; its foot
    the disc's rim past the leg's outer face, down to the underside; the 2 M4s through the foot from counterbores."""
    f, d = cfg.fork, cfg.disc
    x0, x1 = fork_motor_leg_x(cfg)
    leg = _xcyl(f.plate_r, x0, x1, f.axis_y, f.axis_z) + _box((x0, x1), (d.top_y, f.axis_y), _post_z(cfg))
    # the draft up the post and the foot: one cut of the cone (two meeting on the top face leave the solid invalid)
    reach = _box((x0, _FAR), (d.top_y, _FAR), _post_z(cfg)) + _box((x1, _FAR), (d.y0, d.top_y), (-_FAR, _FAR))
    leg = leg + (_cone(cfg, d.y0, fork_flare_top(cfg)) & reach)
    leg = leg - _xcyl(f.ring_bore_dia / 2.0, x0 - NUDGE, _FAR, f.axis_y, f.axis_z)
    seat_x = fork_motor_bolt_x(cfg)[0]
    for y, z in fork_motor_bolts(cfg):
        leg = leg - _xcyl(M4_CLEAR / 2.0, x1 - NUDGE, _FAR, y, z) - _xcyl(COUNTERBORE / 2.0, seat_x, _FAR, y, z)
    return single_solid(leg)


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
