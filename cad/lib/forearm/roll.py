"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the elbow block (stator - the housing that is also the elbow's output flange), the bolt-on motor mount on
its top, the hollow roll shaft with its integral 90T (rotor; the GT2 ring of lib/pulley/teeth.py) and the bolt-on end
cap carrying bearing 2."""
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
    mount_bolt_points,
    mount_nut_pocket_open_y,
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


def _y_cylinder(radius: float, height: float, xz, y0: float):
    """A cylinder along +Y from y0, its axis through (x, z) = xz."""
    return bd.Pos(xz[0], y0, xz[1]) * bd.Rot(-90.0, 0.0, 0.0) * cylinder(radius, height)


def _y_hex(across_flats: float, length: float, xz, y0: float):
    """A hex prism along +Y from y0, its axis through (x, z) = xz, a flat toward +/-X (its corners along Z)."""
    return bd.Pos(xz[0], y0, xz[1]) * bd.Rot(-90.0, 0.0, 0.0) * hex_prism(across_flats, math.pi / 2.0, length)


def _pocket(cfg: ForearmConfig, half_x: float, z1: float, y1: float, rear_pad: float = 0.0):
    """The box the motor mount's base occupies on the block's flat top: x +/- half_x, y_step .. y1, from the rear face
    (rear_pad past it: a cutter's overshoot) to z1."""
    d, S = cfg.drive, stack_positions(cfg)
    return bd.Pos(0.0, S["y_step"], d.block_z[0] - rear_pad) * bd.Box(
        2 * half_x, y1 - S["y_step"], z1 - d.block_z[0] + rear_pad, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))


def build_block(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    # the material first: the rounded box round the roll axis, the coupler's lip / boss / journal / stub on its
    # underside (about the elbow axis, module X)
    body = _rounded_box(d.block_x, d.block_y, d.block_z, d.block_corner_r)
    for dia, x0, x1 in coupler_steps(cfg):
        body = body + x_cylinder(dia / 2.0, x1 - x0, (0.0, 0.0), x0)
    y_top = d.block_y[1]
    # the bore, rear to front: the cable exit through the end wall, the lip, bearing 1's seat, the clearance bore
    # round the core, the cavity the ring runs in - open through the front face
    body = body - cylinder(d.cable_exit_dia / 2.0, d.end_wall + 2 * NUDGE, z0=S["z_end"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + NUDGE, z0=S["z_lip"])
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, d.bearing_width + NUDGE, z0=S["z_seat"])
    body = body - cylinder(d.core_bore_dia / 2.0, S["z_cavity"] - S["z_bore"] + NUDGE, z0=S["z_bore"])
    body = body - cylinder(d.cavity_dia / 2.0, S["z_face"] - S["z_cavity"] + 2 * NUDGE, z0=S["z_cavity"])
    # the belt window through the top wall round the ring
    z0, z1, half_x, y0 = belt_window(cfg)
    body = body - bd.Pos(0.0, y0, z0) * bd.Box(2 * half_x, y_top - y0 + NUDGE, z1 - z0, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))
    # the coupler side: the pin bore, the elbow pulley's 4x M4 - clearance up through the stub to the nut seat, then
    # each nut's hex channel (a flat toward the elbow axis) on up into the core bore, where the nut drops in
    body = body - x_cylinder(d.pin_bore_dia / 2.0, d.pin_bore_x[1] - d.pin_bore_x[0] + NUDGE, (0.0, 0.0), d.pin_bore_x[0] - NUDGE)
    for y, z in pulley_bolt_points(cfg):
        body = body - x_cylinder(d.pulley_bolt_dia / 2.0, d.nut_seat_x - d.stub_x[0] + 2 * NUDGE, (y, z), d.stub_x[0] - NUDGE)
        x_end = nut_channel_end(y, cfg)
        body = body - _x_hex(d.nut_af, x_end - d.nut_seat_x, (y, z), d.nut_seat_x, math.degrees(math.atan2(z, y)))
    # the cap's 4x M3 (self-tapping) in the front face
    for x, y in cap_bolt_points(cfg):
        body = body - cylinder(d.cap_tap_dia / 2.0, d.cap_tap_depth + NUDGE, (x, y), z0=S["z_face"] - d.cap_tap_depth)
    # the motor mount's seat: the pocket its base fills in the flat top (mount_fit round it, open at the rear face; the
    # rounded edges stay), the 4 screws' clearance holes down through the top wall to the nuts' seats, and each nut's
    # hex pocket (a flat toward +/-X) on down into the core bore, where the nut goes in before the shaft
    body = body - _pocket(cfg, d.plate_w / 2.0 + d.mount_fit, S["z_step_riser"], y_top + 1.0, rear_pad=1.0)
    for x, z in mount_bolt_points(cfg):
        body = body - _y_cylinder(d.mount_bolt_dia / 2.0, S["y_step"] - S["y_mount_nut"] + 2 * NUDGE, (x, z), S["y_mount_nut"] - NUDGE)
        y_open = mount_nut_pocket_open_y(x, cfg)
        body = body - _y_hex(d.mount_nut_pocket_af, S["y_mount_nut"] - y_open, (x, z), y_open)
    return single_solid(body)


def build_motor_mount(cfg: ForearmConfig = DEFAULT):
    """The motor mount in the module frame, one thickness throughout (mount_base_t = pad_t): a plain base plate_w wide
    in the pocket on the block's flat top (y_step .. the block's top, the rear face .. the plate's front face - the
    motor's clearance unchanged), the vertical plate rooted in it (the four tension slots, the pilot slot) and the 4
    countersunk clearance holes (the heads flush with the base's top, under the motor)."""
    d, S = cfg.drive, stack_positions(cfg)
    y_top = d.block_y[1]
    body = _pocket(cfg, d.plate_w / 2.0, S["z_pad_top"], y_top)
    body = body + bd.Pos(0.0, S["y_step"], S["z_motor_face"]) * bd.Box(
        d.plate_w, S["y_plate_top"] - S["y_step"], d.pad_t, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))
    for x, y in pad_bolt_points(cfg):
        body = body - _slot_y(x, y, d.pad_bolt_dia, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    body = body - _slot_y(S["x_motor"], S["y_motor"], d.pad_pilot_w, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    # the 90 deg countersinks: the screw head's own cone from its top (dk at the base's top) down to the hole
    r_hole, r_head = d.mount_bolt_dia / 2.0, d.mount_screw.head_dia / 2.0
    for xz in mount_bolt_points(cfg):
        body = body - _y_cylinder(r_hole, d.mount_base_t + 2 * NUDGE, xz, S["y_step"] - NUDGE)
        sink = bd.Cone(r_hole, r_head + NUDGE, r_head - r_hole + NUDGE, align=align_min())
        body = body - bd.Pos(xz[0], y_top - (r_head - r_hole), xz[1]) * bd.Rot(-90.0, 0.0, 0.0) * sink
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
    """The end cap at its LOCAL origin (standing on z=0; the module places it on the block's front face): the block's
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
