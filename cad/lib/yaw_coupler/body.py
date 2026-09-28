"""build123d builder of j1_coupler from YawCouplerConfig, in the coupler's part frame.

Built in the part frame directly: the disc, the flare and the hub turn about the part's Y (the base_yaw axis), the
yoke is prisms along X (the drive's axis) - their outlines in the YZ plane, lib/yaw_coupler/layout.py."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.geom import align_min, hex_prism, single_solid
from lib.units import NUDGE
from lib.yaw_coupler.layout import (
    channel_outline,
    cheek_outline,
    disc_profile,
    flare_r,
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


def _stub(cfg: YawCouplerConfig):
    """The stub from its end up into the recess's ceiling, the round on its end's outer edge."""
    h = cfg.hub
    stub = _ycyl(h.stub_dia / 2.0, h.stub_y0, h.recess_y1 + NUDGE)
    if h.stub_round > 0.0:
        stub = stub.fillet(h.stub_round, [stub.edges().sort_by(bd.Axis.Y)[0]])
    return stub


def build_yaw_coupler(cfg: YawCouplerConfig = DEFAULT):
    d, h, k = cfg.disc, cfg.hub, cfg.yoke
    body = _disc(cfg) + _yoke(cfg)
    for cut in _yoke_cuts(cfg):
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
