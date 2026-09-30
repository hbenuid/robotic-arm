"""build123d builder of j2_link (the forearm web) from a ForearmConfig, in j2_link's part frame."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.forearm.layout import (
    disc_bolt_angles,
    disc_bolt_points,
    elbow_end_x,
    flange_bolt_points,
    link_socket_points,
    neck_tangent,
    screw_channel,
    screws_under_the_web,
)
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import align_min, cylinder, hex_prism, single_solid, through
from lib.units import NUDGE


def stadium(length: float, width: float, height: float, x_centre: float, z0: float):
    """A slot-shaped prism (semicircular ends, centre-to-centre `length`, `width` = its diameter) along x,
    centred at (x_centre, 0), standing on z0."""
    return bd.Pos(x_centre, 0.0, z0) * bd.extrude(bd.SlotCenterToCenter(length, width), amount=height)


def slab(length: float, width: float, height: float, x_centre: float, z0: float, y_centre: float = 0.0):
    """A box along x, centred at (x_centre, y_centre), standing on z0 (bd.Box itself is centred in z)."""
    return bd.Pos(x_centre, y_centre, z0) * bd.Box(length, width, height, align=align_min())


def x_cylinder(radius: float, length: float, yz, x0: float):
    """A cylinder along +X from x0 to x0 + length, its axis through (y, z) = yz - the roll-axis features."""
    return bd.Pos(x0, yz[0], yz[1]) * bd.Rot(0.0, 90.0, 0.0) * bd.Cylinder(radius, length, align=align_min())


def yz_prism(face, x0: float, x1: float):
    """A face drawn as (u, v) = (y, z), extruded along +X from x0 to x1."""
    return bd.extrude(bd.Plane.YZ.offset(x0) * face, amount=x1 - x0, dir=(1.0, 0.0, 0.0))


def wall_outline(cfg: ForearmConfig = DEFAULT):
    """The flange wall's outline, a face in (y, z): the hull of its round flange about the roll axis and its foot, the
    necked web's end section."""
    r, w = cfg.roll_end, cfg.web
    flange = bd.Pos(0.0, r.axis_z) * bd.Circle(r.wall_od / 2.0)
    foot = bd.Pos(0.0, (w.z0 + w.z1) / 2.0) * bd.Rectangle(2.0 * r.neck_half_w, w.thickness)
    return bd.make_hull((flange + foot).edges())


def roll_wall(cfg: ForearmConfig = DEFAULT):
    """The forearm's elbow end with the roll joint: the flange wall (a round flange about the roll axis on a foot as
    wide as the web's neck) with the rotor flange's locating recess on its elbow face, the bolt holes and the cable
    bore on the roll axis."""
    r = cfg.roll_end
    x0, x1 = r.wall_x
    axis = (0.0, r.axis_z)
    wall = yz_prism(wall_outline(cfg), x0, x1)
    wall = wall - x_cylinder((r.flange_dia + r.flange_recess_add) / 2.0, r.flange_recess_depth + NUDGE, axis, x1 - r.flange_recess_depth)
    wall = wall - x_cylinder(r.cable_bore / 2.0, x1 - x0 + 2 * NUDGE, axis, x0 - NUDGE)
    for yz in flange_bolt_points(cfg):
        wall = wall - x_cylinder(r.bolt_dia / 2.0, x1 - x0 + 2 * NUDGE, yz, x0 - NUDGE)
    return wall


def necked_web(cfg: ForearmConfig = DEFAULT):
    """The web with the roll joint: from the wall's foot (neck_half_w either side) it widens to the wrist boss, its
    sides tangent to the boss (the round wrist end)."""
    w, b, n = cfg.web, cfg.boss, cfg.roll_end.neck_half_w
    (tx, ty), x_end = neck_tangent(cfg), elbow_end_x(cfg)
    plan = bd.Polygon((tx, ty), (x_end, n), (x_end, -n), (tx, -ty), align=None)
    web = bd.Pos(0.0, 0.0, w.z0) * bd.extrude(plan, amount=w.thickness, dir=(0.0, 0.0, 1.0))
    return web + cylinder(b.dia / 2.0, w.thickness, (w.wrist_x, 0.0), z0=w.z0)


def wall_gussets(cfg: ForearmConfig = DEFAULT):
    """The two gussets that brace the wall on the web: a triangle in the x-z plane (rib_h high at the wall's wrist face,
    down to the web's top face rib_len along it), |y| inside rib_y, its top cut by the wall's round outline."""
    r, w = cfg.roll_end, cfg.web
    x0 = r.wall_x[0]
    under_the_outline = yz_prism(wall_outline(cfg), x0 - r.rib_len, x0)
    triangle = bd.Polygon((x0, w.z1), (x0, w.z1 + r.rib_h), (x0 - r.rib_len, w.z1), align=None)
    wedge = bd.extrude(bd.Plane.XZ * triangle, amount=r.rib_y[1], both=True)
    ribs = []
    for side in (1.0, -1.0):
        band = slab(r.rib_len, r.rib_y[1] - r.rib_y[0], r.rib_h, x0 - r.rib_len / 2.0, w.z1, side * sum(r.rib_y) / 2.0)
        ribs.append(under_the_outline & wedge & band)
    return ribs


def elbow_disc(cfg: ForearmConfig = DEFAULT):
    """The j3_coupler interface: the disc with its bore, the 4x M4 clearance holes and their hex nut pockets
    (the material only; the web is added by build_link, the elbow block reuses this in M3)."""
    d = cfg.disc
    body = cylinder(d.dia / 2.0, d.z1 - d.z0, z0=d.z0)
    body = body - through(d.bore_dia / 2.0, d.z1 - d.z0, z0=d.z0)
    for angle, xy in zip(disc_bolt_angles(cfg), disc_bolt_points(cfg), strict=True):
        body = body - cylinder(d.bolt_dia / 2.0, d.bolt_z1 - d.z0 + NUDGE, xy, z0=d.z0 - NUDGE)
        body = body - bd.Pos(xy[0], xy[1], d.nut_z0) * hex_prism(d.nut_af, angle, d.z1 - d.nut_z0 + NUDGE)
    return body


def build_link(cfg: ForearmConfig = DEFAULT):
    w, d, b, s = cfg.web, cfg.disc, cfg.boss, cfg.slot
    if cfg.roll:
        # the necked web from the flange wall to the wrist pivot (round wrist end); the wall and its gussets; the wrist
        # boss over the web
        body = necked_web(cfg) + roll_wall(cfg)
        for rib in wall_gussets(cfg):
            body = body + rib
        # the way in for a wall screw whose head would land in the web: a channel in the web's underside, into the
        # motor slot
        for y, z in screws_under_the_web(cfg):
            x0, x1, half_w, z_top = screw_channel(z, cfg)
            body = body - slab(x1 - x0, 2.0 * half_w, z_top - w.z0 + NUDGE, (x0 + x1) / 2.0, w.z0 - NUDGE, y)
    else:
        # the web: a stadium from the elbow pivot to the wrist pivot; the elbow disc under it
        body = stadium(abs(w.wrist_x), 2.0 * w.half_w, w.thickness, w.wrist_x / 2.0, w.z0)
        body = body + cylinder(d.dia / 2.0, d.z1 - d.z0, z0=d.z0)
    body = body + cylinder(b.dia / 2.0, b.z1 - w.z0, (w.wrist_x, 0.0), z0=w.z0)
    if not cfg.roll:
        # the elbow interface (the disc's bore and bolts run through the web too)
        body = body - through(d.bore_dia / 2.0, d.z1 - d.z0, z0=d.z0)
        for angle, xy in zip(disc_bolt_angles(cfg), disc_bolt_points(cfg), strict=True):
            body = body - cylinder(d.bolt_dia / 2.0, d.bolt_z1 - d.z0 + NUDGE, xy, z0=d.z0 - NUDGE)
            body = body - bd.Pos(xy[0], xy[1], d.nut_z0) * hex_prism(d.nut_af, angle, d.z1 - d.nut_z0 + NUDGE)
    # the wrist bearing seat: the lip's bore all the way, the seat's bore below and above the lip, the recess on top
    wx = (w.wrist_x, 0.0)
    body = body - cylinder(b.lip_dia / 2.0, b.seat_z1 - w.z0 + NUDGE, wx, z0=w.z0 - NUDGE)
    body = body - cylinder(b.seat_dia / 2.0, b.lip_z[0] - w.z0 + NUDGE, wx, z0=w.z0 - NUDGE)
    body = body - cylinder(b.seat_dia / 2.0, b.seat_z1 - b.lip_z[1], wx, z0=b.lip_z[1])
    body = body - cylinder(b.recess_dia / 2.0, b.z1 - b.recess_z0 + NUDGE, wx, z0=b.recess_z0)
    # the motor slide: the central slot and the two side slots through the web
    body = body - stadium(s.centre_x[1] - s.centre_x[0], s.centre_w, w.thickness + 2 * NUDGE, sum(s.centre_x) / 2.0, w.z0 - NUDGE)
    for sy in (s.side_y, -s.side_y):
        body = body - bd.Pos(0.0, sy, 0.0) * stadium(s.side_x[1] - s.side_x[0], s.side_w, w.thickness + 2 * NUDGE, sum(s.side_x) / 2.0, w.z0 - NUDGE)
    # the locating sockets, both faces (LEGACY)
    if cfg.sockets is not None:
        top, bottom = link_socket_points(cfg)
        r, depth = cfg.sockets.dia / 2.0, cfg.sockets.depth
        for xy in top:
            body = body - cylinder(r, depth + NUDGE, xy, z0=w.z1 - depth)
        for xy in bottom:
            body = body - cylinder(r, depth + NUDGE, xy, z0=w.z0 - NUDGE)
    return single_solid(body)
