"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the frame (stator, forearm_roll_block - one body round the motor: the octagonal elbow end with the elbow's
output flange under it, the motor's pocket open through its bottom and the plate, the tower with the 6806 pair and the
bearing boss on its front face), the pulley (rotor, the output - its integral 90T is the GT2 ring of lib/pulley/teeth.py
- its hub on through the tower to bearing 1), the shaft (rotor, a collar that holds bearing 1) and the back panel (stator,
a plate flush with the frame's rear face over the shaft's bay - its step, the opening and the magnets on the rear face:
back_face / back_point)."""
from __future__ import annotations

import math

import build123d as bd

from lib.datum import to_location
from lib.forearm.layout import (
    clamp_nut_pocket,
    clamp_points,
    coupler_steps,
    end_nut_pocket,
    flange_bolt_points_module,
    module_frame_in_host,
    motor_pocket,
    nut_channel_end,
    pad_bolt_points,
    plug_slots,
    pulley_bolt_points,
    roll_idler_xy,
    stack_positions,
)
from lib.forearm.link import x_cylinder, yz_prism
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import box, cylinder, hex_prism, nut_slot, single_solid, stop_wedge
from lib.pulley.teeth import gt2_ring
from lib.units import NUDGE

FAR = 500.0   # past every part: the half-plane behind the frame's front face


def host_to_module(cfg: ForearmConfig = DEFAULT) -> bd.Location:
    """j2_link's frame before the forearm's turn -> the module frame (the inverse of the module's mount frame)."""
    return to_location(module_frame_in_host(cfg)).inverse()


def _x_hex(across_flats: float, length: float, yz, x0: float, flat_deg: float):
    """A hex prism along +X from x0, its axis through (y, z) = yz, one flat facing flat_deg in the y-z plane (0 = +Y)."""
    return bd.Pos(x0, yz[0], yz[1]) * bd.Rot(0.0, 90.0, 0.0) * hex_prism(across_flats, math.radians(flat_deg), length)


def rear_line(cfg: ForearmConfig = DEFAULT) -> tuple[tuple[float, float], float]:
    """The outline's rear line, from the end up to the tower's top, as ((n_u, n_v), distance): the frame keeps to
    n . (p - elbow axis) <= distance, n its outward normal in (u, v) = (module y, module z). Tangent to the end_r circle
    about the elbow axis and to a rear_line_r circle in the tower's rear-top corner."""
    d, S = cfg.drive, stack_positions(cfg)
    cu, cv = d.tower_y - d.rear_line_r - S["y_elbow"], S["z_tower_rear"] + d.rear_line_r   # that circle, from the elbow axis
    th = math.atan2(cv, cu) - math.acos((d.end_r - d.rear_line_r) / math.hypot(cu, cv))
    return (math.cos(th), math.sin(th)), d.end_r


def back_face(cfg: ForearmConfig = DEFAULT) -> tuple[float, tuple[float, float], tuple[float, float]]:
    """The rear face's frame (the back panel's, BackPanelParams): (z0, n, t) - the roll axis meets the face at module
    (0, 0, z0), n is its outward normal and t the way up it toward the tower's top, both as (module y, module z). A point
    on the face is (x, s): module x, and s along t from that point."""
    (nu, nv), dist = rear_line(cfg)
    return (dist + nu * stack_positions(cfg)["y_elbow"]) / nv, (nu, nv), (-nv, nu)


def back_point(x: float, s: float, depth: float = 0.0, cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """The module point at (x, s) on the rear face (back_face), `depth` in from it."""
    z0, (nu, nv), (tu, tv) = back_face(cfg)
    return (x, tu * s - nu * depth, z0 + tv * s - nv * depth)


def _back_plane(depth: float, cfg: ForearmConfig = DEFAULT) -> bd.Plane:
    """The rear face's plane `depth` in from it: local (x, s) as back_face's, +Z into the frame."""
    _z0, (nu, nv), _t = back_face(cfg)
    return bd.Plane(origin=back_point(0.0, 0.0, depth, cfg), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, -nu, -nv))


def _on_back(sketch, depth: float, amount: float, cfg: ForearmConfig = DEFAULT):
    """`sketch` (local x, s) on the rear face's plane `depth` in from it, extruded `amount` further in."""
    return bd.extrude(_back_plane(depth, cfg) * sketch, amount=amount)


def back_opening(cfg: ForearmConfig = DEFAULT, inset: float = 0.0):
    """The back opening seen square to the rear face (local x, s), `inset` inside it: the bay's bore where it meets the
    face (an ellipse) and, below the roll axis' point, a rectangle as wide down to opening_bottom, its corners there
    rounded opening_r."""
    d, b = cfg.drive, cfg.drive.back
    _z0, (_nu, nv), _t = back_face(cfg)
    a, r = d.bay_r - inset, b.opening_r - inset
    x_r, s_r = d.bay_r - b.opening_r, b.opening_bottom + b.opening_r      # the corners' centres
    shape = bd.Ellipse(a, d.bay_r / -nv - inset) + bd.Pos(0.0, s_r / 2.0) * bd.Rectangle(2.0 * a, -s_r)
    shape = shape + bd.Pos(0.0, (s_r + b.opening_bottom + inset) / 2.0) * bd.Rectangle(2.0 * x_r, s_r - b.opening_bottom - inset)
    return shape + bd.Pos(-x_r, s_r) * bd.Circle(r) + bd.Pos(x_r, s_r) * bd.Circle(r)


def _back_top(cfg: ForearmConfig = DEFAULT) -> float:
    """An s past the tower's top along the rear face (the step and the panel are cut off at the top)."""
    _z0, _n, (tu, _tv) = back_face(cfg)
    return (cfg.drive.tower_y + 5.0) / tu


def back_magnet(x: float, s: float, depth: float, cfg: ForearmConfig = DEFAULT):
    """One of the back panel's disc magnets (BackPanelParams magnet_*) on the rear face at (x, s), its outer face `depth`
    in from it."""
    b = cfg.drive.back
    return _on_back(bd.Pos(x, s) * bd.Circle(b.magnet_dia / 2.0), depth, b.magnet_t, cfg)


def back_magnet_depths(cfg: ForearmConfig = DEFAULT) -> tuple[float, float]:
    """The outer faces' depths in from the rear face of each pair's two magnets: the frame's, pressed to the bottom of
    its pocket in the step's floor, and the panel's, pressed to the bottom of its pocket in the panel's inner face."""
    b = cfg.drive.back
    return b.t + b.pocket_depth - b.magnet_t, b.t - b.pocket_depth


def profile(cfg: ForearmConfig = DEFAULT):
    """The frame's outline seen along the elbow axis, as (u, v) = (module y, module z): an octagon about the elbow axis
    (end_sides, a flat on each axis end_r off it) and the tower round the roll axis (its top tower_y, from its rear to the
    front face), their hull, cut off at the front face and behind the rear line (rear_line()) - straight sides, sharp
    corners."""
    d, S = cfg.drive, stack_positions(cfg)
    y_e, n = S["y_elbow"], d.end_sides
    end = bd.Pos(y_e, 0.0) * bd.RegularPolygon(d.end_r, n, major_radius=False, rotation=180.0 / n)
    z0, z1 = S["z_tower_rear"], S["z_front"]
    tower = bd.Pos(0.0, (z0 + z1) / 2.0) * bd.Rectangle(2.0 * d.tower_y, z1 - z0)
    hull = bd.make_hull((end + tower).edges())
    (nu, nv), dist = rear_line(cfg)
    behind = (bd.Pos(y_e + nu * (dist - FAR), nv * (dist - FAR)) * bd.Rot(0.0, 0.0, math.degrees(math.atan2(nv, nu)))
              * bd.Rectangle(2.0 * FAR, 2.0 * FAR))
    return hull & (bd.Pos(0.0, z1 - FAR) * bd.Rectangle(2.0 * FAR, 2.0 * FAR)) & behind


def build_block(cfg: ForearmConfig = DEFAULT):
    """The frame (forearm_roll_block) in the module frame: ONE body round the motor - the profile (an octagon about the
    elbow axis, hulled up to the tower round the roll axis) from the underside (block_x[0], the elbow flange's lip / boss /
    journal / stub under it) to the side (+N) face, closed; the motor's pocket between the floor and the side wall, open
    down through the octagon's bottom (the motor slides up into it), a groove inside each wall at the board's side plugs;
    its front wall the plate (the pilot hole, the motor's 4 M3 holes with their heads' counterbores, the shaft's slit down
    out through the bottom), the plate's face the frame's one front face - on it the roll belt's tensioner's riser, its
    M5's slot and its nut's channel under the face, the nut in from the pocket -; the tower: bearing 1's seat from the
    shaft's bay behind it - bored on through the frame's back -, the lip (the spacer), bearing 2's seat in the bearing
    boss - round on the front face, the ring's flange turning over it -, the stop post on the tower's face in the bay;
    the rear face: the back panel's step across it, the opening grown down below the bay's bore, the magnets' pockets."""
    d, S = cfg.drive, stack_positions(cfg)
    y_e = S["y_elbow"]
    # the material first: the profile across the frame's depth, the elbow flange under it, the bearing boss (from 1 mm
    # inside the front face: one body) up to bearing 2's front face
    body = yz_prism(profile(cfg), d.block_x[0], d.block_x[1])
    for dia, a, b in coupler_steps(cfg):
        body = body + x_cylinder(dia / 2.0, b - a, (y_e, 0.0), a)
    body = body + cylinder(d.bearing_boss_dia / 2.0, S["z_boss"] - S["z_front"] + 1.0, z0=S["z_front"] - 1.0)
    # the roll belt's tensioner's riser on the front face (from 1 mm inside it: one body), its land on top under the idler's
    # bearing inner ring; both stadiums along X, the idler's push, +/- travel
    i, (ix, iy) = d.idler, roll_idler_xy(cfg)
    land0 = S["z_idler"] - i.land_h
    body = body + bd.Pos(ix, iy, S["z_front"] - 1.0) * bd.extrude(bd.SlotCenterToCenter(2.0 * i.travel, i.riser_w), amount=land0 - S["z_front"] + 1.0)
    body = body + bd.Pos(ix, iy, land0 - NUDGE) * bd.extrude(bd.SlotCenterToCenter(2.0 * i.travel, i.land_w), amount=i.land_h + NUDGE)
    # the motor's pocket, between the floor and the side wall, from the board's back to the plate, open on down out through
    # the octagon's bottom; the plate in front of it: the pilot hole and the motor's four M3 holes, each screw's head flush
    # in a counterbore in the front face, the motor centred on the elbow axis
    _y0, y1, z0, z1 = motor_pocket(cfg)
    body = body - box((S["x_cradle"], S["x_side"]), (S["y_elbow"] - d.end_r - 1.0, y1), (z0, z1))
    # the board's two side plugs, at the walls: a groove inside each from the plug down out through the bottom
    for x0, x1, sy0, sy1, sz0, sz1 in plug_slots(cfg):
        body = body - box((x0, x1), (sy0, sy1), (sz0, sz1))
    h = d.motor_holes
    body = body - cylinder(h.pilot_dia / 2.0, d.pad_t + 2 * NUDGE, (S["x_motor"], S["y_motor"]), z0=S["z_motor_face"] - NUDGE)
    for xy in pad_bolt_points(cfg):
        body = body - cylinder(h.bolt_dia / 2.0, d.pad_t + 2 * NUDGE, xy, z0=S["z_motor_face"] - NUDGE)
        body = body - cylinder(h.pocket_dia / 2.0, h.screw.head_h + NUDGE, xy, z0=S["z_motor_screws"])
    # the shaft's slit through the plate, from the pilot hole down out through the bottom: the motor + board slide up from
    # there motor_drop back (the shaft in the slit, the pilot boss under the plate), then forward into the pilot hole
    slit = (S["x_motor"] - d.shaft_slit / 2.0, S["x_motor"] + d.shaft_slit / 2.0)
    body = body - box(slit, (S["y_elbow"] - d.end_r - 1.0, S["y_motor"]), (S["z_motor_face"] - NUDGE, S["z_front"] + NUDGE))
    # the tower, from the bay to the boss's top: bearing 1's seat, the lip, bearing 2's seat (open through the top)
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_1"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + 2 * NUDGE, z0=S["z_lip"] - NUDGE)
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_2"])
    # the shaft's bay behind bearing 1, bored on through the frame's back (the shaft, bearing 1 on it, goes in along the
    # axis from behind); the back panel's step across the rear face, full width, from the tower's top down to its bottom
    # edge, and the opening grown on down below the bay's bore, opening_depth in; the frame's magnets' pockets in the
    # step's floor; the stop post on the tower's face inside the bay, at -X
    body = body - cylinder(d.bay_r, S["z_bearing_1"] + FAR, z0=-FAR)
    b, s_top = d.back, _back_top(cfg)
    body = body - _on_back(bd.Pos(0.0, (b.bottom + s_top) / 2.0) * bd.Rectangle(2.0 * FAR, s_top - b.bottom), -1.0, b.t + 1.0, cfg)
    body = body - _on_back(back_opening(cfg), -1.0, b.opening_depth + 1.0, cfg)
    for x, s in b.magnet_points:
        body = body - _on_back(bd.Pos(x, s) * bd.Circle(b.pocket_dia / 2.0), b.t - NUDGE, b.pocket_depth + NUDGE, cfg)
    body = body + stop_wedge(d.stop_post_r[0], d.stop_post_r[1], S["z_stop_post"], d.stop_post_t, d.stop_deg_width, 180.0)
    # the elbow flange: the pin bore, the elbow pulley's 4x M4 - clearance up through the stub to the nut seat, then each
    # nut's hex channel (a flat toward the elbow axis) on up through the floor into the motor's pocket, where the nut
    # drops in before the motor
    body = body - x_cylinder(d.pin_bore_dia / 2.0, d.pin_bore_x[1] - d.pin_bore_x[0] + NUDGE, (y_e, 0.0), d.pin_bore_x[0] - NUDGE)
    x_end = nut_channel_end(cfg)
    for y, z in pulley_bolt_points(cfg):
        body = body - x_cylinder(d.pulley_bolt_dia / 2.0, d.nut_seat_x - d.stub_x[0] + 2 * NUDGE, (y_e + y, z), d.stub_x[0] - NUDGE)
        body = body - _x_hex(d.nut_af, x_end - d.nut_seat_x, (y_e + y, z), d.nut_seat_x, math.degrees(math.atan2(z, y)))
    # the tensioner's M5: its slot through the land, the riser and the plate, on down past the screw's tip; the nut's
    # channel under the face along X, past the slot's ends both ways (the nut flats-on, sliding with it), and its way in:
    # a recess in the plate's inner face from the motor's pocket (entry inside its top edge) along +Y into the channel
    tip = S["z_idler_top"] - i.screw_len
    body = body - bd.Pos(ix, iy, tip - i.tip_clear) * bd.extrude(bd.SlotCenterToCenter(2.0 * i.travel, i.slot_w), amount=S["z_idler"] - tip + i.tip_clear + NUDGE)
    half, corner = i.nut.af / 2.0 + i.nut_fit, i.nut.af / math.sqrt(3.0) + i.nut_fit
    nut_z = (S["z_idler_roof"] - i.nut.h - i.nut_fit, S["z_idler_roof"])
    body = body - box((ix - i.travel - corner, ix + i.travel + corner), (iy - half, iy + half), nut_z)
    body = body - box((ix - corner, ix + corner), (y1 - i.entry, iy), nut_z)
    return single_solid(body)


def build_pulley(cfg: ForearmConfig = DEFAULT):
    """The pulley (forearm_roll_pulley) in the module frame - the roll's output: the integral 90T ring (two flanges) on
    a Ø44 core, its hub - a journal through bearing 2, then the neck (neck_dia, under the bearing's bore: bearing 2 slides
    over it) on through the lip to bearing 1's front face, where the shaft's journal meets it -, the Ø33 shoulder on
    bearing 2's inner ring, the Ø39.7 spigot on its front face in the forearm wall's recess (the wall bolts onto the ring's
    face, into nuts in pockets in the core, pushed in from the bore); the cable bore, the rotor clamp's 4 holes with their
    heads' counterbores in the front face."""
    d, w, S = cfg.drive, cfg.roll_end, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    z_end = S["z_spigot_end"]
    # the core: the neck, the hub, the shoulder, the core under the ring, the spigot - then every cut, then the ring fused
    # on LAST (an annulus on the core, so no boolean ever runs through its 90 grooves)
    body = cylinder(d.neck_dia / 2.0, S["z_bearing_2"] - S["z_meet"], z0=S["z_meet"])
    body = body + cylinder(j_r, S["z_boss"] - S["z_bearing_2"], z0=S["z_bearing_2"])
    body = body + cylinder(d.shoulder_dia / 2.0, d.run_gap, z0=S["z_boss"])
    body = body + cylinder(d.ring_core_dia / 2.0, S["z_ring_end"] - S["z_ring_flange_1"], z0=S["z_ring_flange_1"])
    body = body + cylinder(w.flange_dia / 2.0, z_end - S["z_wall"], z0=S["z_wall"])
    body = body - cylinder(d.bore / 2.0, z_end - S["z_meet"] + 2 * NUDGE, z0=S["z_meet"] - NUDGE)
    # the rotor clamp: clearance end to end, each head's counterbore from the front face
    head_r = d.clamp_screw.head_dia / 2.0 + d.clamp_head_clear
    for x, y in clamp_points(cfg):
        body = body - cylinder(w.bolt_dia / 2.0, z_end - S["z_meet"] + 2 * NUDGE, (x, y), z0=S["z_meet"] - NUDGE)
        body = body - cylinder(head_r, z_end - S["z_clamp_head"] + NUDGE, (x, y), z0=S["z_clamp_head"])
    # the forearm wall's 4x M3: clearance from the front face on past each nut to the screw's tip, and each nut's pocket -
    # a slot from the cable bore outward along the screw's angle, a flat either side - in the core
    r_in, r_out, z0, z1 = end_nut_pocket(cfg)
    z_hole = S["z_end_tip"] - d.end_nut_fit
    for x, y in flange_bolt_points_module(cfg):
        body = body - cylinder(w.bolt_dia / 2.0, z_end - z_hole + NUDGE, (x, y), z0=z_hole)
        body = body - nut_slot(r_in, r_out, d.end_nut.af, z0, z1, math.atan2(y, x))
    ring = gt2_ring(d.ring_teeth, d.ring_width, d.ring_flange_dia, d.ring_flange_t, z0=S["z_ring"], inner_dia=d.ring_core_dia - 0.2)
    return single_solid(body + ring)


def build_shaft(cfg: ForearmConfig = DEFAULT):
    """The shaft (forearm_roll_shaft) in the module frame - a collar that holds bearing 1: its journal through bearing 1
    to the bearing's front face, where the pulley's neck meets it, the Ø33 shoulder on bearing 1's inner ring, the flange
    behind it with the rotor clamp's four nuts (pockets open to its rear face and into the bore) and the hard-stop lug;
    the cable bore."""
    d, w, S = cfg.drive, cfg.roll_end, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    z0 = S["z_shaft_end"]
    body = cylinder(j_r, S["z_meet"] - S["z_bearing_1"], z0=S["z_bearing_1"])
    body = body + cylinder(d.shoulder_dia / 2.0, d.run_gap, z0=S["z_shoulder_1"])
    body = body + cylinder(d.collar_od / 2.0, S["z_shoulder_1"] - z0, z0=z0)
    body = body + stop_wedge(d.stop_lug_r[0], d.stop_lug_r[1], S["z_stop_lug"], d.stop_lug_t, d.stop_deg_width, 0.0)
    body = body - cylinder(d.bore / 2.0, S["z_meet"] - z0 + 2 * NUDGE, z0=z0 - NUDGE)
    r_in, r_out, p0, p1 = clamp_nut_pocket(cfg)
    for x, y in clamp_points(cfg):
        body = body - cylinder(w.bolt_dia / 2.0, S["z_meet"] - z0 + 2 * NUDGE, (x, y), z0=z0 - NUDGE)
        body = body - nut_slot(r_in, r_out, d.clamp_nut.af, p0 - NUDGE, p1, math.atan2(y, x))
    return single_solid(body)


def build_back_panel(cfg: ForearmConfig = DEFAULT):
    """The back panel (forearm_roll_back_panel) in the module frame: a plate in the frame's step on the rear face (its
    outer face on the rear line), as wide as the frame less side_clear, from gap over the step's bottom edge up to
    side_clear under the tower's top; relieved where it passes over the turning shaft - run_gap off its rear face and its
    lug's sweep, inside the bay's bore -; the U rim on its inner face down into the opening's lower part (it locates the
    panel); a pocket for each magnet in its inner face."""
    d, b, S = cfg.drive, cfg.drive.back, stack_positions(cfg)
    s_top, half = _back_top(cfg), d.block_x[1] - b.side_clear
    body = _on_back(bd.Pos(0.0, (b.bottom + b.gap + s_top) / 2.0) * bd.Rectangle(2.0 * half, s_top - b.bottom - b.gap), 0.0, b.t, cfg)
    body = body - box((-FAR, FAR), (d.tower_y - b.side_clear, FAR), (-FAR, FAR))
    body = body - cylinder(d.bay_r, FAR, z0=S["z_shaft_end"] - d.run_gap)
    rim = back_opening(cfg, b.rim_clear) - back_opening(cfg, b.rim_clear + b.rim_w)
    rim = rim & bd.Pos(0.0, (b.rim_top + b.opening_bottom - 10.0) / 2.0) * bd.Rectangle(2.0 * FAR, b.rim_top - b.opening_bottom + 10.0)
    body = body + _on_back(rim, b.t - NUDGE, b.rim_h + NUDGE, cfg)
    for x, s in b.magnet_points:
        body = body - _on_back(bd.Pos(x, s) * bd.Circle(b.pocket_dia / 2.0), b.t - b.pocket_depth, b.pocket_depth + NUDGE, cfg)
    return single_solid(body)
