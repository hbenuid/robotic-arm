"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the frame (stator, forearm_roll_block - one body round the motor: the round elbow end with the elbow's
output flange under it, the motor's pocket and plate, the tower with the 6806 pair and the bearing boss on its front face), the
pulley (rotor, the output - its integral 90T is the GT2 ring of lib/pulley/teeth.py) and the shaft (rotor, a collar
behind bearing 1)."""
from __future__ import annotations

import math

from cadgen import build123d as bd

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
    pulley_bolt_points,
    roll_idler_xy,
    stack_positions,
)
from lib.forearm.link import x_cylinder, yz_prism
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import box, cylinder, hex_prism, nut_slot, single_solid, slot_y, stop_wedge
from lib.pulley.teeth import gt2_ring
from lib.units import NUDGE

FAR = 500.0   # past every part: the half-plane behind the frame's front face


def host_to_module(cfg: ForearmConfig = DEFAULT) -> bd.Location:
    """j2_link's frame before the forearm's turn -> the module frame (the inverse of the module's mount frame)."""
    return to_location(module_frame_in_host(cfg)).inverse()


def _x_hex(across_flats: float, length: float, yz, x0: float, flat_deg: float):
    """A hex prism along +X from x0, its axis through (y, z) = yz, one flat facing flat_deg in the y-z plane (0 = +Y)."""
    return bd.Pos(x0, yz[0], yz[1]) * bd.Rot(0.0, 90.0, 0.0) * hex_prism(across_flats, math.radians(flat_deg), length)


def profile(cfg: ForearmConfig = DEFAULT):
    """The frame's outline seen along the elbow axis, as (u, v) = (module y, module z): the round end about the elbow axis
    (end_r) and the tower round the roll axis (its top tower_y, from its rear to the front face), their hull, cut off
    at the front face."""
    d, S = cfg.drive, stack_positions(cfg)
    end = bd.Pos(S["y_elbow"], 0.0) * bd.Circle(d.end_r)
    z0, z1 = S["z_tower_rear"], S["z_front"]
    tower = bd.Pos(0.0, (z0 + z1) / 2.0) * bd.RectangleRounded(2.0 * d.tower_y, z1 - z0, d.tower_corner_r)
    hull = bd.make_hull((end + tower).edges())
    return hull & (bd.Pos(0.0, z1 - FAR) * bd.Rectangle(2.0 * FAR, 2.0 * FAR))


def build_block(cfg: ForearmConfig = DEFAULT):
    """The frame (forearm_roll_block) in the module frame: ONE body round the motor - the profile (round about the elbow
    axis, tangent up to the tower round the roll axis) from the underside (block_x[0], the elbow flange's lip / boss /
    journal / stub under it) to the open +N face; the motor's pocket open on that face, its front wall the plate (the
    tension slots, the pilot slot, the shaft's slit out to the open face), the plate's face the frame's one front face - on
    it the roll belt's tensioner's riser, its M5's slot and its nut's channel under the face -; the tower: bearing 1's seat from the
    shaft's bay behind it, the lip, bearing 2's seat in the bearing boss - round on the front face, the ring's flange
    turning over it -, the stop post on the tower's rear face in the bay."""
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
    # the motor's pocket, from the floor out through the open face; the plate in front of it: the motor's four tension
    # slots and the pilot slot, the motor centred on the elbow axis
    y0, y1, z0, z1 = motor_pocket(cfg)
    body = body - box((S["x_cradle"], d.block_x[1] + 1.0), (y0, y1), (z0, z1))
    for x, y in pad_bolt_points(cfg):
        body = body - slot_y(x, y, d.pad_bolt_dia, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    body = body - slot_y(S["x_motor"], S["y_motor"], d.pad_pilot_w, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    # the shaft's slit through the plate, from the pilot slot out through the +N face: the motor + board slide in from
    # that face motor_drop low (the shaft in the slit, the pilot boss under the plate), then lift into the pilot slot
    slit = (S["y_motor"] - d.shaft_slit / 2.0, S["y_motor"] + d.shaft_slit / 2.0)
    body = body - box((S["x_motor"], d.block_x[1] + 1.0), slit, (S["z_motor_face"] - NUDGE, S["z_front"] + NUDGE))
    # the tower, from the bay to the boss's top: bearing 1's seat, the lip, bearing 2's seat (open through the top)
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_1"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + 2 * NUDGE, z0=S["z_lip"] - NUDGE)
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_2"])
    # the shaft's bay behind bearing 1, open through the +N face (the shaft and bearing 1 go in from it); the stop post on
    # the tower's rear face inside it, at -X
    bay = (S["z_bay"], S["z_bearing_1"])
    body = body - cylinder(d.bay_r, bay[1] - bay[0], z0=bay[0])
    body = body - box((0.0, d.block_x[1] + 1.0), (-d.bay_r, d.bay_r), bay)
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
    # channel under the face, from the open +N face along X past the slot's far end (the nut flats-on, sliding with it)
    tip = S["z_idler_top"] - i.screw_len
    body = body - bd.Pos(ix, iy, tip - i.tip_clear) * bd.extrude(bd.SlotCenterToCenter(2.0 * i.travel, i.slot_w), amount=S["z_idler"] - tip + i.tip_clear + NUDGE)
    half = i.nut.af / 2.0 + i.nut_fit
    x0 = ix - i.travel - i.nut.af / math.sqrt(3.0) - i.nut_fit
    body = body - box((x0, d.block_x[1] + 1.0), (iy - half, iy + half), (S["z_idler_roof"] - i.nut.h - i.nut_fit, S["z_idler_roof"]))
    return single_solid(body)


def build_pulley(cfg: ForearmConfig = DEFAULT):
    """The pulley (forearm_roll_pulley) in the module frame - the roll's output: the integral 90T ring (two flanges) on
    a Ø44 core, its hub (a journal) down through bearing 2 to the lip's middle where the shaft's meets it, the Ø33
    shoulder on bearing 2's inner ring between them, the Ø39.7 spigot on its front face in the forearm wall's recess
    (the wall bolts onto the ring's face, into nuts in pockets in the core, pushed in from the bore); the cable bore,
    the rotor clamp's 4 holes with their heads' counterbores in the front face."""
    d, w, S = cfg.drive, cfg.roll_end, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    z_end = S["z_spigot_end"]
    # the core: the hub, the shoulder, the core under the ring, the spigot - then every cut, then the ring fused on LAST
    # (an annulus on the core, so no boolean ever runs through its 90 grooves)
    body = cylinder(j_r, S["z_boss"] - S["z_meet"], z0=S["z_meet"])
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
    """The shaft (forearm_roll_shaft) in the module frame - a collar behind bearing 1: its hub (a journal) up through
    bearing 1 to the lip's middle where the pulley's meets it, the Ø33 shoulder on bearing 1's inner ring, the flange
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
