"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the frame (stator, forearm_roll_block - the roll housing, the web down to the elbow's output flange and
the motor's plate in one part), the hollow roll shaft with its integral 90T (rotor; the GT2 ring of
lib/pulley/teeth.py) and the bolt-on end cap carrying bearing 2."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.datum import to_location
from lib.forearm.layout import (
    belt_window,
    cap_bolt_points,
    coupler_steps,
    end_nut_pocket,
    flange_bolt_points_module,
    module_frame_in_host,
    nut_channel_end,
    pad_bolt_points,
    pulley_bolt_points,
    stack_positions,
)
from lib.forearm.link import x_cylinder
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import align_min, cylinder, hex_prism, single_solid
from lib.pulley.teeth import gt2_ring
from lib.units import NUDGE


def host_to_module(cfg: ForearmConfig = DEFAULT) -> bd.Location:
    """j2_link's frame -> the module frame (the inverse of the module's mount frame)."""
    return to_location(module_frame_in_host(cfg)).inverse()


def _rounded_box(x, y, z, r: float):
    """A box over the x / y / z ranges with its four edges along Z rounded."""
    box = bd.Pos((x[0] + x[1]) / 2.0, (y[0] + y[1]) / 2.0, z[0]) * bd.Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=align_min())
    return box.fillet(r, box.edges().filter_by(bd.Axis.Z))


def _box(x, y, z):
    """A plain box over the x / y / z ranges."""
    return bd.Pos(x[0], y[0], z[0]) * bd.Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))


def _wedge(r_in: float, r_out: float, z0: float, height: float, deg_width: float, at_deg: float):
    """A hard-stop lug: a radial box from r_in to r_out, deg_width wide at r_out, standing on z0, turned to at_deg."""
    half = r_out * math.tan(math.radians(deg_width / 2.0))
    box = bd.Pos(r_in, 0.0, z0) * bd.Box(r_out - r_in, 2 * half, height, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    return bd.Rot(0.0, 0.0, at_deg) * box


def _x_hex(across_flats: float, length: float, yz, x0: float, flat_deg: float):
    """A hex prism along +X from x0, its axis through (y, z) = yz, one flat facing flat_deg in the y-z plane (0 = +Y)."""
    return bd.Pos(x0, yz[0], yz[1]) * bd.Rot(0.0, 90.0, 0.0) * hex_prism(across_flats, math.radians(flat_deg), length)


def _slot_y(x: float, y: float, width: float, length: float, z0: float, depth: float):
    """A slot `length` long along Y, `width` wide, through a plate from z0 (the plate's tension slots + pilot slot)."""
    return bd.Pos(x, y, z0 - NUDGE) * bd.Rot(0.0, 0.0, 90.0) * bd.extrude(bd.SlotCenterToCenter(length, width), amount=depth + 2 * NUDGE)


def build_block(cfg: ForearmConfig = DEFAULT):
    """The frame (forearm_roll_block) in the module frame: the housing round the roll axis, the web under the motor
    (on the upper arm's side, x block_x[0] .. x_cradle) from the housing down past the elbow axis with the elbow flange
    on its underside, and the motor's plate in front of the motor - one printed part."""
    d, S = cfg.drive, stack_positions(cfg)
    x0, y_e, y_in = d.block_x[0], S["y_elbow"], d.block_y[0] + d.block_corner_r + 2.0   # y_in: where the web and plate root in the housing
    # the material first: the housing, the web, the plate, the coupler's lip / boss / journal / stub on the web's
    # underside about the elbow axis (module X through y_elbow)
    body = _rounded_box(d.block_x, d.block_y, d.block_z, d.block_corner_r)
    body = body + _box((x0, S["x_cradle"]), (S["y_web_low"], y_in), d.web_z)
    body = body + _box((x0, d.plate_w / 2.0), (S["y_plate_low"], y_in), (S["z_motor_face"], S["z_pad_top"]))
    for dia, a, b in coupler_steps(cfg):
        body = body + x_cylinder(dia / 2.0, b - a, (y_e, 0.0), a)
    # the housing's bore, rear to front: the cable exit through the end wall, the lip, bearing 1's seat, the clearance
    # bore round the core, the cavity the ring runs in - open through the front face
    body = body - cylinder(d.cable_exit_dia / 2.0, d.end_wall + 2 * NUDGE, z0=S["z_end"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + NUDGE, z0=S["z_lip"])
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, d.bearing_width + NUDGE, z0=S["z_seat"])
    body = body - cylinder(d.core_bore_dia / 2.0, S["z_cavity"] - S["z_bore"] + NUDGE, z0=S["z_bore"])
    body = body - cylinder(d.cavity_dia / 2.0, S["z_face"] - S["z_cavity"] + 2 * NUDGE, z0=S["z_cavity"])
    # the belt window down through the housing's bottom wall round the ring (the belt runs down to the motor's 20T)
    z0, z1, half_x, y0 = belt_window(cfg)
    body = body - _box((-half_x, half_x), (d.block_y[0] - NUDGE, y0), (z0, z1))
    # the cap's 4x M3 (self-tapping) in the front face
    for x, y in cap_bolt_points(cfg):
        body = body - cylinder(d.cap_tap_dia / 2.0, d.cap_tap_depth + NUDGE, (x, y), z0=S["z_face"] - d.cap_tap_depth)
    # the elbow flange: the pin bore, the elbow pulley's 4x M4 - clearance up through the stub to the nut seat, then each
    # nut's hex channel (a flat toward the elbow axis) on up through the web into the motor's cradle, where the nut
    # drops in before the motor
    body = body - x_cylinder(d.pin_bore_dia / 2.0, d.pin_bore_x[1] - d.pin_bore_x[0] + NUDGE, (y_e, 0.0), d.pin_bore_x[0] - NUDGE)
    x_end = nut_channel_end(cfg)
    for y, z in pulley_bolt_points(cfg):
        body = body - x_cylinder(d.pulley_bolt_dia / 2.0, d.nut_seat_x - d.stub_x[0] + 2 * NUDGE, (y_e + y, z), d.stub_x[0] - NUDGE)
        body = body - _x_hex(d.nut_af, x_end - d.nut_seat_x, (y_e + y, z), d.nut_seat_x, math.degrees(math.atan2(z, y)))
    # the plate: the motor's four tension slots and the pilot slot, the motor centred on the elbow axis
    for x, y in pad_bolt_points(cfg):
        body = body - _slot_y(x, y, d.pad_bolt_dia, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    body = body - _slot_y(S["x_motor"], S["y_motor"], d.pad_pilot_w, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    return single_solid(body)


def build_shaft(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    # the core: journal 1 (from the rear end to shoulder 1), the Ø44 core through the clearance bore and the cavity
    # (under the ring), journal 2, the neck bearing 2 slides over, the end spigot into the wall's recess, the
    # hard-stop lug - then every cut, then the ring fused on LAST (an annulus on the core, so no boolean ever runs
    # through its 90 grooves)
    body = cylinder(j_r, S["z_bore"] - S["z_shaft_end"], z0=S["z_shaft_end"])
    body = body + cylinder(d.shoulder_od / 2.0, S["z_face"] - S["z_bore"], z0=S["z_bore"])
    body = body + cylinder(j_r, d.bearing_width, z0=S["z_bearing_2"])
    body = body + cylinder(d.neck_od / 2.0, S["z_wall"] - S["z_neck"], z0=S["z_neck"])
    body = body + cylinder(cfg.roll_end.flange_dia / 2.0, S["z_spigot_end"] - S["z_wall"], z0=S["z_wall"])
    body = body + _wedge(d.stop_lug_r[0], d.stop_lug_r[1], S["z_stop_lug"], d.stop_t, d.stop_deg_width, 0.0)
    body = body - cylinder(d.bore / 2.0, S["z_spigot_end"] - S["z_shaft_end"] + 2 * NUDGE, z0=S["z_shaft_end"] - NUDGE)   # the cable bore
    # the forearm wall's 4x M3: clearance from the end face on past each nut to the screw's tip, and each nut's pocket - a
    # slot from the cable bore outward along the screw's angle, a flat either side - behind bearing 2
    r_in, r_out, z0, z1 = end_nut_pocket(cfg)
    for x, y in flange_bolt_points_module(cfg):
        z_hole = S["z_end_tip"] - d.end_nut_fit
        body = body - cylinder(cfg.roll_end.bolt_dia / 2.0, S["z_spigot_end"] - z_hole + NUDGE, (x, y), z0=z_hole)
        pocket = bd.Pos((r_in + r_out) / 2.0, 0.0, z0) * bd.Box(r_out - r_in, d.end_nut.af, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
        body = body - bd.Rot(0.0, 0.0, math.degrees(math.atan2(y, x))) * pocket
    ring = gt2_ring(d.ring_teeth, d.ring_width, d.ring_flange_dia, d.ring_flange_t, z0=S["z_ring"], inner_dia=d.shoulder_od - 0.2)
    return single_solid(body + ring)


def build_retainer(cfg: ForearmConfig = DEFAULT):
    """The end cap at its LOCAL origin (standing on z=0; the module places it on the housing's front face): the housing's
    rounded outline, bearing 2's seat then the lip, four M3 clearance holes at the corners, the hard-stop post on
    its outer face at -X (the rotor's lug on the neck meets it at +/- stop_deg)."""
    d = cfg.drive
    cap_t = d.bearing_width + d.cap_lip
    body = _rounded_box(d.block_x, d.block_y, (0.0, cap_t), d.block_corner_r)
    body = body + _wedge(d.stop_post_r[0], d.stop_post_r[1], cap_t, d.stop_t, d.stop_deg_width, 180.0)
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, d.bearing_width + NUDGE, z0=-NUDGE)
    body = body - cylinder(d.lip_id / 2.0, cap_t + 2 * NUDGE, z0=-NUDGE)
    for x, y in cap_bolt_points(cfg):
        body = body - cylinder(d.cap_bolt_dia / 2.0, cap_t + 2 * NUDGE, (x, y), z0=-NUDGE)
    return single_solid(body)
