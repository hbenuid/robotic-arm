"""build123d builder of j2_link (the forearm web) from a ForearmConfig, in j2_link's part frame."""
from __future__ import annotations

import math

import build123d as bd

from lib.forearm.layout import (
    disc_bolt_angles,
    disc_bolt_points,
    elbow_end_x,
    flange_bolt_points,
    lap_nut_z,
    lap_z,
    link_socket_points,
    motor_hole_points,
    screw_channel,
    screws_under_the_web,
    wrist_idler_spacer_h,
    wrist_idler_xy,
)
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import (
    align_min,
    chamfer_outline,
    cylinder,
    faceted_end,
    hex_circumdiameter,
    hex_prism,
    hull_solid,
    single_solid,
    through,
)
from lib.style import EDGE_CHAMFER, SIDES
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
    """The flange wall's outline, a face in (y, z) about the roll axis: faceted (lib/geom.py faceted_end: flats
    wall_od / 2 off it) with the roll joint - the roll frame's tower carried on through -, round on LEGACY."""
    r = cfg.roll_end
    if cfg.roll:
        return faceted_end(r.wall_od / 2.0, (0.0, r.axis_z))
    return bd.Pos(0.0, r.axis_z) * bd.Circle(r.wall_od / 2.0)


def roll_wall(cfg: ForearmConfig = DEFAULT):
    """The forearm's elbow end with the roll joint: the flange wall (the faceted flange about the roll axis through its
    whole thickness, its corners inside the roll tensioner's screw head's path as the forearm rolls) with the rotor
    flange's locating recess on its elbow face, the bolt holes - their heads' counterbores in its wrist face (head_seat
    deep: flush) - and the cable bore on the roll axis."""
    r = cfg.roll_end
    x0, x1 = r.wall_x
    axis = (0.0, r.axis_z)
    wall = yz_prism(wall_outline(cfg), x0, x1)
    depth = r.flange_recess_depth + r.flange_recess_clear
    wall = wall - x_cylinder((r.flange_dia + r.flange_recess_add) / 2.0, depth + NUDGE, axis, x1 - depth)
    wall = wall - x_cylinder(r.cable_bore / 2.0, x1 - x0 + 2 * NUDGE, axis, x0 - NUDGE)
    for yz in flange_bolt_points(cfg):
        wall = wall - x_cylinder(r.bolt_dia / 2.0, x1 - x0 + 2 * NUDGE, yz, x0 - NUDGE)
        wall = wall - x_cylinder(r.head_dia / 2.0, r.head_seat + NUDGE, yz, x0 - NUDGE)
    return wall


def straight_web(cfg: ForearmConfig = DEFAULT):
    """The web with the roll joint: straight from the wall's wrist face to the wrist pivot, half_w either side, its
    wrist end faceted (lib/geom.py faceted_end: flats half_w off the pivot), the outline of both faces chamfered - run
    EDGE_CHAMFER on into the wall, so the chamfer round its end there is buried in it (its corners outside the flange
    there go with corner_facets)."""
    w = cfg.web
    x_end = elbow_end_x(cfg) + EDGE_CHAMFER
    rect = bd.Pos((x_end + w.wrist_x) / 2.0, 0.0) * bd.Rectangle(x_end - w.wrist_x, 2.0 * w.half_w)
    plan = (rect + faceted_end(w.half_w, (w.wrist_x, 0.0))).face()
    return chamfer_outline(bd.Pos(0.0, 0.0, w.z0) * bd.extrude(plan, amount=w.thickness), (0.0, 0.0, 1.0))


def wrist_boss(cfg: ForearmConfig = DEFAULT):
    """The wrist boss over the web, as wide as it: faceted about the pivot like the web's end, from the web's
    underside up to its top (boss.z1), its top's outline chamfered (the web's chamfer takes the underside's)."""
    w, b = cfg.web, cfg.boss
    boss = bd.Pos(0.0, 0.0, w.z0) * bd.extrude(faceted_end(b.dia / 2.0, (w.wrist_x, 0.0)), amount=b.z1 - w.z0)
    return chamfer_outline(boss, (0.0, 0.0, 1.0))


def wall_gussets(cfg: ForearmConfig = DEFAULT):
    """The two gussets that brace the wall on the web: a triangle in the x-z plane (rib_h high at the wall's wrist face,
    down to the web's top face rib_len along it), |y| inside rib_y - under the flange's side flats, their feet cut by
    corner_facets with the web's corners."""
    r, w = cfg.roll_end, cfg.web
    x0 = r.wall_x[0]
    triangle = bd.Polygon((x0, w.z1), (x0, w.z1 + r.rib_h), (x0 - r.rib_len, w.z1), align=None)
    wedge = bd.extrude(bd.Plane.XZ * triangle, amount=r.rib_y[1], both=True)
    ribs = []
    for side in (1.0, -1.0):
        band = slab(r.rib_len, r.rib_y[1] - r.rib_y[0], r.rib_h, x0 - r.rib_len / 2.0, w.z1, side * sum(r.rib_y) / 2.0)
        ribs.append(wedge & band)
    return ribs


def corner_facets(cfg: ForearmConfig = DEFAULT) -> list:
    """What the web grows out of the flange through, one solid per side, to cut from the web and the gussets: under the
    plane through the flange's lower diagonal facet at the wall's wrist face and the web's bottom edge chamfer (its
    inner edge, EDGE_CHAMFER in from the side) facet_len along the web - the flange's facet carried on along the web,
    running out into the web's own chamfer, both at 45 deg on the octagon (lib/style.py SIDES) -; and under the facet
    itself on into the wall past the web's buried end. A convex hull of the sections at the three stations (the facet's
    planes fold over at the wall face, the region under them stays convex)."""
    r, w = cfg.roll_end, cfg.web
    x0, apo = r.wall_x[0], r.wall_od / 2.0
    corner_y = apo * math.tan(math.pi / SIDES)               # the flange's bottom flat's corner (its z: w.z0)
    slope = math.tan(2.0 * math.pi / SIDES)                  # the diagonal facet: z = z0 + (|y| - corner_y) * slope
    drop = (w.half_w - EDGE_CHAMFER - corner_y) * slope      # at facet_len the plane through the chamfer's inner edge
    z_lo, y_out = w.z0 - 1.0, w.half_w + 1.0
    cuts = []
    for side in (1.0, -1.0):
        points = []
        for x, d in ((x0 - r.facet_len, drop), (x0, 0.0), (x0 + EDGE_CHAMFER + 1.0, 0.0)):
            # the section under the plane z = z0 + (|y| - corner_y) * slope - d, down to z_lo, out to y_out
            y_lo = corner_y + (z_lo - w.z0 + d) / slope
            z_out = w.z0 + (y_out - corner_y) * slope - d
            points += [(x, side * y_lo, z_lo), (x, side * y_out, z_lo), (x, side * y_out, z_out)]
        cuts.append(hull_solid(points))
    return cuts


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


def idler_mount(body, cfg: ForearmConfig = DEFAULT):
    """`body` with the wrist idler's M5 slot through the web and the nut's channel (flats + nut_fit wide across x) from
    nut_floor up through the top face - each on the idler's axis' line, running along y between IdlerParams.slot_y's
    ends. No rib under the web: the web prints on its underside, the idler hangs under a spacer of its own
    (build_idler_spacer)."""
    w, i = cfg.web, cfg.idler
    xi, _ = wrist_idler_xy(cfg)
    y0, y1 = i.slot_y

    def along_y(width: float, z0: float, height: float):
        return bd.Pos(xi, (y0 + y1) / 2.0, z0) * bd.Rot(0.0, 0.0, 90.0) * bd.extrude(bd.SlotCenterToCenter(y1 - y0, width), amount=height)

    body = body - along_y(i.slot_w, w.z0 - NUDGE, w.thickness + 2.0 * NUDGE)
    channel = (i.nut.af + i.nut_fit, y1 - y0 + hex_circumdiameter(i.nut.af) + i.nut_fit)
    return body - slab(channel[0], channel[1], w.z1 - i.nut_floor + NUDGE, xi, i.nut_floor, (y0 + y1) / 2.0)


def build_idler_spacer(cfg: ForearmConfig = DEFAULT):
    """The wrist idler's printed spacer (IdlerParams; parts/joints/wrist_idler_spacer), axis on Z: its land
    (land_dia x land_h) on z=0, on the idler's bearing inner ring; its riser (riser_dia) on up to
    z = wrist_idler_spacer_h(), on the web's underside; the axle's slot_w through both. Prints riser down."""
    i, height = cfg.idler, wrist_idler_spacer_h(cfg)
    body = cylinder(i.land_dia / 2.0, i.land_h) + cylinder(i.riser_dia / 2.0, height - i.land_h, z0=i.land_h)
    return single_solid(body - through(i.slot_w / 2.0, height))


def wall_bracket_region(cfg: ForearmConfig = DEFAULT):
    """What the wall bracket takes of the whole forearm (WallSplitParams): the wall (from its wrist face on), the
    tongue (the web over lap_z() from the wall back to lap_x) and the gussets (everything over the web's top face as far
    along as they run)."""
    r, w, s = cfg.roll_end, cfg.web, cfg.split
    x0, top, y = r.wall_x[0], r.axis_z + r.wall_od, 2.0 * (w.half_w + r.wall_od)
    z_lap = lap_z(cfg)
    wall = slab(r.wall_x[1] + 10.0 - x0, y, top - w.z0 + 20.0, (x0 + r.wall_x[1] + 10.0) / 2.0, w.z0 - 10.0)
    tongue = slab(x0 - s.lap_x, y, top - z_lap, (x0 + s.lap_x) / 2.0, z_lap)
    gussets = slab(r.rib_len + 1.0, y, top - w.z1, x0 - (r.rib_len + 1.0) / 2.0, w.z1)
    return wall + tongue + gussets


def lap_cuts(cfg: ForearmConfig = DEFAULT):
    """The lap's fastener cuts (WallSplitParams), at each screw point: the screw's clearance through both halves, its
    head's counterbore head_h up from the web's underside, the nut's hex pocket down from the tongue's top face (a
    corner along x)."""
    w, s = cfg.web, cfg.split
    cuts = []
    for xy in s.screw_points:
        cuts.append(through(s.bolt_dia / 2.0, w.thickness, xy, z0=w.z0))
        cuts.append(cylinder(s.head_dia / 2.0, s.screw.head_h + NUDGE, xy, z0=w.z0 - NUDGE))
        cuts.append(bd.Pos(xy[0], xy[1], lap_nut_z(cfg)) * hex_prism(s.nut.af + s.nut_fit, 0.0, s.nut.h + NUDGE))
    return cuts


def build_wall_bracket(cfg: ForearmConfig = DEFAULT):
    """The wall bracket (WallSplitParams; parts/joints/j2_wall_bracket) in j2_link's part frame: the whole forearm's
    wall, gussets and tongue (wall_bracket_region()), the lap's holes and nut pockets through the tongue. Prints on the
    wall's elbow face."""
    body = _build_whole(cfg) & wall_bracket_region(cfg)
    for cut in lap_cuts(cfg):
        body = body - cut
    return single_solid(body)


def build_link(cfg: ForearmConfig = DEFAULT):
    """j2_link: the whole forearm (LEGACY: the SolidWorks part), or with cfg.split the web the wall bracket is lapped
    onto - the whole less wall_bracket_region(), the lap's holes and the screws' counterbores through its end."""
    body = _build_whole(cfg)
    if cfg.split is not None:
        body = body - wall_bracket_region(cfg)
        for cut in lap_cuts(cfg):
            body = body - cut
    return single_solid(body)


def _build_whole(cfg: ForearmConfig = DEFAULT):
    """The whole forearm, one solid: the web, the elbow end (LEGACY's disc, DEFAULT's roll wall), the wrist boss, the motor's
    seat, the idler's slot and channel (build_link splits it when cfg.split)."""
    w, d, b, s = cfg.web, cfg.disc, cfg.boss, cfg.slot
    if cfg.roll:
        # the straight web from the flange wall to the wrist pivot (faceted wrist end) and the wall's gussets on it, grown
        # out of the flange through its carried-on lower facets; the wall; the wrist boss over the web
        body = straight_web(cfg)
        for rib in wall_gussets(cfg):
            body = body + rib
        for cut in corner_facets(cfg):
            body = body - cut
        body = body + roll_wall(cfg)
        # the way in for a wall screw whose head would land in the web: a channel in the web's underside, into the
        # motor slot
        for y, z in screws_under_the_web(cfg):
            x0, x1, half_w, z_top = screw_channel(z, cfg)
            body = body - slab(x1 - x0, 2.0 * half_w, z_top - w.z0 + NUDGE, (x0 + x1) / 2.0, w.z0 - NUDGE, y)
    else:
        # the web: a stadium from the elbow pivot to the wrist pivot; the elbow disc under it
        body = stadium(abs(w.wrist_x), 2.0 * w.half_w, w.thickness, w.wrist_x / 2.0, w.z0)
        body = body + cylinder(d.dia / 2.0, d.z1 - d.z0, z0=d.z0)
    body = body + (wrist_boss(cfg) if cfg.roll else cylinder(b.dia / 2.0, b.z1 - w.z0, (w.wrist_x, 0.0), z0=w.z0))
    if not cfg.roll:
        # the elbow interface (the disc's bore and bolts run through the web too)
        body = body - through(d.bore_dia / 2.0, d.z1 - d.z0, z0=d.z0)
        for angle, xy in zip(disc_bolt_angles(cfg), disc_bolt_points(cfg), strict=True):
            body = body - cylinder(d.bolt_dia / 2.0, d.bolt_z1 - d.z0 + NUDGE, xy, z0=d.z0 - NUDGE)
            body = body - bd.Pos(xy[0], xy[1], d.nut_z0) * hex_prism(d.nut_af, angle, d.z1 - d.nut_z0 + NUDGE)
    # the wrist bearing seat: the lip's bore all the way, the seat's bore below and above the lip (on through the top
    # when the boss ends at the seat's top), the recess on top
    wx = (w.wrist_x, 0.0)
    top_out = NUDGE if b.z1 <= b.seat_z1 else 0.0
    body = body - cylinder(b.lip_dia / 2.0, b.seat_z1 - w.z0 + NUDGE + top_out, wx, z0=w.z0 - NUDGE)
    body = body - cylinder(b.seat_dia / 2.0, b.lip_z[0] - w.z0 + NUDGE, wx, z0=w.z0 - NUDGE)
    body = body - cylinder(b.seat_dia / 2.0, b.seat_z1 - b.lip_z[1] + top_out, wx, z0=b.lip_z[1])
    if b.recess_dia is not None:
        body = body - cylinder(b.recess_dia / 2.0, b.z1 - b.recess_z0 + NUDGE, wx, z0=b.recess_z0)
    # the motor: on a slide - the central slot and the two side slots through the web - or on round holes, the pilot
    # boss's and the 31 mm square's, their screws' heads flush in counterbores in the underside
    if s is not None:
        body = body - stadium(s.centre_x[1] - s.centre_x[0], s.centre_w, w.thickness + 2 * NUDGE, sum(s.centre_x) / 2.0, w.z0 - NUDGE)
        for sy in (s.side_y, -s.side_y):
            body = body - bd.Pos(0.0, sy, 0.0) * stadium(s.side_x[1] - s.side_x[0], s.side_w, w.thickness + 2 * NUDGE, sum(s.side_x) / 2.0, w.z0 - NUDGE)
    if cfg.holes is not None:
        h = cfg.holes
        body = body - cylinder(h.pilot_dia / 2.0, w.thickness + 2 * NUDGE, (cfg.motor_x, 0.0), z0=w.z0 - NUDGE)
        for xy in motor_hole_points(cfg):
            body = body - cylinder(h.bolt_dia / 2.0, w.thickness + 2 * NUDGE, xy, z0=w.z0 - NUDGE)
            body = body - cylinder(h.pocket_dia / 2.0, h.screw.head_h + NUDGE, xy, z0=w.z0 - NUDGE)
    # the wrist belt's idler (IdlerParams): the axle's slot through the web, the nut's channel down from the top face -
    # both along y over the idler's travel
    if cfg.roll and cfg.idler is not None:
        body = idler_mount(body, cfg)
    # the locating sockets, both faces (LEGACY)
    if cfg.sockets is not None:
        top, bottom = link_socket_points(cfg)
        r, depth = cfg.sockets.dia / 2.0, cfg.sockets.depth
        for xy in top:
            body = body - cylinder(r, depth + NUDGE, xy, z0=w.z1 - depth)
        for xy in bottom:
            body = body - cylinder(r, depth + NUDGE, xy, z0=w.z0 - NUDGE)
    return single_solid(body)
