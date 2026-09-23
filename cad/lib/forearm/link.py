"""build123d builder of j2_link (the forearm web) from a ForearmConfig, in j2_link's part frame."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.cycloidal.geom import align_min, cylinder, single_solid, through
from lib.cycloidal.housing import hex_prism
from lib.forearm.layout import disc_bolt_angles, disc_bolt_points, link_socket_points
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.units import NUDGE


def stadium(length: float, width: float, height: float, x_centre: float, z0: float):
    """A slot-shaped prism (semicircular ends, centre-to-centre `length`, `width` = its diameter) along x,
    centred at (x_centre, 0), standing on z0."""
    return bd.Pos(x_centre, 0.0, z0) * bd.extrude(bd.SlotCenterToCenter(length, width), amount=height)


def slab(length: float, width: float, height: float, x_centre: float, z0: float, y_centre: float = 0.0):
    """A box along x, centred at (x_centre, y_centre), standing on z0 (bd.Box itself is centred in z)."""
    return bd.Pos(x_centre, y_centre, z0) * bd.Box(length, width, height, align=align_min())


def elbow_disc(cfg: ForearmConfig = DEFAULT):
    """The j3_coupler interface: the disc with its bore, the 4x M4 clearance holes and their hex nut pockets
    (the material only; the web is added by build_link, the elbow block reuses this in M3)."""
    d = cfg.disc
    body = cylinder(d.dia / 2.0, d.z1 - d.z0, z0=d.z0)
    body = body - through(d.bore_dia / 2.0, d.z1 - d.z0, z0=d.z0)
    for angle, xy in zip(disc_bolt_angles(cfg), disc_bolt_points(cfg)):
        body = body - cylinder(d.bolt_dia / 2.0, d.bolt_z1 - d.z0 + NUDGE, xy, z0=d.z0 - NUDGE)
        body = body - bd.Pos(xy[0], xy[1], d.nut_z0) * hex_prism(d.nut_af, angle, d.z1 - d.nut_z0 + NUDGE)
    return body


def build_link(cfg: ForearmConfig = DEFAULT):
    w, d, b, s = cfg.web, cfg.disc, cfg.boss, cfg.slot
    # the web: a stadium from the elbow pivot to the wrist pivot; the elbow disc under it; the wrist boss over it
    body = stadium(abs(w.wrist_x), 2.0 * w.half_w, w.thickness, w.wrist_x / 2.0, w.z0)
    body = body + cylinder(d.dia / 2.0, d.z1 - d.z0, z0=d.z0)
    body = body + cylinder(b.dia / 2.0, b.z1 - w.z0, (w.wrist_x, 0.0), z0=w.z0)
    # the elbow interface (the disc's bore and bolts run through the web too)
    body = body - through(d.bore_dia / 2.0, d.z1 - d.z0, z0=d.z0)
    for angle, xy in zip(disc_bolt_angles(cfg), disc_bolt_points(cfg)):
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
    # the locating sockets, both faces
    top, bottom = link_socket_points(cfg)
    r, depth = cfg.sockets.dia / 2.0, cfg.sockets.depth
    for xy in top:
        body = body - cylinder(r, depth + NUDGE, xy, z0=w.z1 - depth)
    for xy in bottom:
        body = body - cylinder(r, depth + NUDGE, xy, z0=w.z0 - NUDGE)
    return single_solid(body)
