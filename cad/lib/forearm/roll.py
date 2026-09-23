"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the elbow block (stator - the housing), the hollow roll shaft with its integral 90T (rotor) and the
bolt-on end cap carrying bearing 2."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.cycloidal.geom import align_min, cylinder, single_solid
from lib.datum import to_location
from lib.forearm.layout import (
    cap_bolt_points, flange_bolt_points_module, module_frame_in_host, pad_bolt_points, pad_slot_angle_deg, stack_positions,
)
from lib.forearm.link import elbow_disc
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.forearm.pulley import gt2_ring
from lib.units import NUDGE


def host_to_module(cfg: ForearmConfig = DEFAULT) -> bd.Location:
    """j2_link's frame -> the module frame (the inverse of the module's mount frame)."""
    return to_location(module_frame_in_host(cfg)).inverse()


def _lug(cfg: ForearmConfig, x: float, y: float, z0: float, height: float):
    """A lug / ear box from inside the housing wall (lug_root) out to the bolt at (x, y) + lug_w / 2: on +X or on -Y."""
    d = cfg.drive
    reach = math.hypot(x, y) + d.lug_w / 2.0 - d.lug_root
    if x > 0:
        return bd.Pos(d.lug_root, 0.0, z0) * bd.Box(reach, d.lug_w, height, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    return bd.Pos(0.0, -d.lug_root, z0) * bd.Box(d.lug_w, reach, height, align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN))


def _flat(cfg: ForearmConfig, z0: float, height: float):
    """The cutter that flattens a part's underside (module -X) at flat_x - the host's upper-arm slab clearance."""
    d = cfg.drive
    return bd.Pos(d.flat_x, 0.0, z0 - NUDGE) * bd.Box(60.0, 120.0, height + 2 * NUDGE, align=(bd.Align.MAX, bd.Align.CENTER, bd.Align.MIN))


def _slot(cfg: ForearmConfig, x: float, y: float, width: float, z0: float):
    """A tension slot (pad_slot_len long, `width` wide) through the pad plate, along the axis -> motor direction."""
    d = cfg.drive
    return bd.Pos(x, y, z0 - NUDGE) * bd.Rot(0.0, 0.0, pad_slot_angle_deg(cfg)) * bd.extrude(
        bd.SlotCenterToCenter(d.pad_slot_len, width), amount=d.pad_t + 2 * NUDGE)


def build_block(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    # the material first: the SolidWorks disc's j3_coupler interface (lib/forearm/link.py elbow_disc, re-expressed in the
    # module frame), the housing round the roll axis, the two cap lugs on its wrist end, the motor pad tower (the plate
    # above the motor's mounting face, up in the swing plane, + the filler down to the housing's wall)
    body = elbow_disc(cfg).moved(host_to_module(cfg))
    body = body + cylinder(d.housing_od / 2.0, S["z_face"] - S["z_end"], z0=S["z_end"])
    for x, y in cap_bolt_points(cfg):
        body = body + _lug(cfg, x, y, S["z_face"] - d.lug_len, d.lug_len)
    body = body + bd.Pos(S["x_motor"], S["y_motor"], S["z_motor_face"]) * bd.Box(d.pad_w, d.pad_w, d.pad_t, align=align_min())
    body = body + bd.Pos(S["x_motor"] - 15.0, d.pad_root_y, S["z_lip"]) * bd.Box(
        30.0, S["y_motor"] - d.pad_w / 2.0 + 2.0 - d.pad_root_y, S["z_pad_top"] - S["z_lip"],
        align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))
    # then every cut through the union: the lip bearing 1 stops on, its seat, the cavity the ring runs in, the cable exit
    # (+X, end wall + lip only - the seat stays round), the belt window (+Y, round the ring), the flat underneath, the
    # lugs' tap holes, the pad's four tension slots and the pilot boss slot
    body = body - cylinder(d.lip_id / 2.0, S["z_seat"] - S["z_lip"] + NUDGE, z0=S["z_lip"])
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, d.bearing_width + NUDGE, z0=S["z_seat"])
    body = body - cylinder(d.cavity_dia / 2.0, S["z_face"] - S["z_cavity"] + 2 * NUDGE, z0=S["z_cavity"])
    win_len = min(d.cable_window_len, S["z_seat"] - S["z_end"])
    body = body - bd.Pos(0.0, 0.0, S["z_end"] - NUDGE) * bd.Box(d.housing_od / 2.0 + NUDGE, d.cable_window_w, win_len + NUDGE,
                                                                align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    y0, y1 = d.belt_window_y
    ring_span = 2 * d.ring_flange_t + d.ring_width
    body = body - bd.Pos(0.0, y0, S["z_ring_flange_1"] - d.belt_window_margin) * bd.Box(
        2 * d.belt_window_half_x, y1 - y0, ring_span + 2 * d.belt_window_margin, align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN))
    body = body - _flat(cfg, S["z_end"], S["z_face"] - S["z_end"])
    for x, y in cap_bolt_points(cfg):
        body = body - cylinder(d.cap_tap_dia / 2.0, d.cap_tap_depth + NUDGE, (x, y), z0=S["z_face"] - d.cap_tap_depth)
    for x, y in pad_bolt_points(cfg):
        body = body - _slot(cfg, x, y, d.pad_bolt_dia, S["z_motor_face"])
    body = body - _slot(cfg, S["x_motor"], S["y_motor"], d.pad_pilot_w, S["z_motor_face"])
    return single_solid(body)


def build_shaft(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    s_r = d.shoulder_od / 2.0
    # the core: journal 1, the Ø44 core through the cavity (both shoulders + under the ring), journal 2, the neck bearing 2
    # slides over, the end spigot into the wall's recess, the hard-stop lug - then every cut, then the ring fused on LAST
    # (an annulus on the core, so no boolean ever runs through its 90 grooves)
    body = cylinder(j_r, S["z_cavity"] - S["z_shaft_end"], z0=S["z_shaft_end"])
    body = body + cylinder(s_r, S["z_face"] - S["z_cavity"], z0=S["z_cavity"])
    body = body + cylinder(j_r, d.bearing_width, z0=S["z_bearing_2"])
    body = body + cylinder(d.neck_od / 2.0, S["z_wall"] - S["z_neck"], z0=S["z_neck"])
    body = body + cylinder(cfg.roll_end.flange_dia / 2.0, S["z_spigot_end"] - S["z_wall"], z0=S["z_wall"])
    half_w = d.stop_lug_r[1] * math.tan(math.radians(d.stop_deg_width / 2.0))
    body = body + bd.Pos(d.stop_lug_r[0], 0.0, S["z_stop_lug"]) * bd.Box(d.stop_lug_r[1] - d.stop_lug_r[0], 2 * half_w, d.stop_t,
                                                                          align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    body = body - cylinder(d.bore / 2.0, S["z_spigot_end"] - S["z_shaft_end"] + 2 * NUDGE, z0=S["z_shaft_end"] - NUDGE)   # the cable bore
    for xy in flange_bolt_points_module(cfg):                                                      # 4x M3 self-tapping from the end face
        body = body - cylinder(d.end_bolt_tap_dia / 2.0, d.end_bolt_depth + NUDGE, xy, z0=S["z_spigot_end"] - d.end_bolt_depth)
    ring = gt2_ring(d.ring_teeth, d.ring_width, d.ring_flange_dia, d.ring_flange_t, z0=S["z_ring"], inner_dia=d.shoulder_od - 0.2)
    return single_solid(body + ring)


def build_retainer(cfg: ForearmConfig = DEFAULT):
    """The end cap at its LOCAL origin (standing on z=0; the module places it on the housing's wrist face): bearing 2's
    seat then the lip, the housing's outline with the same flat, two ears for the M3s into the housing's lugs, the
    hard-stop post on its outer face at -X (the rotor's lug on the neck meets it at +/- stop_deg)."""
    d, S = cfg.drive, stack_positions(cfg)
    cap_t = d.bearing_width + d.cap_lip
    body = cylinder(d.housing_od / 2.0, cap_t)
    for x, y in cap_bolt_points(cfg):
        body = body + _lug(cfg, x, y, 0.0, cap_t)
    half_w = d.stop_post_r[1] * math.tan(math.radians(d.stop_deg_width / 2.0))
    body = body + bd.Pos(-d.stop_post_r[1], 0.0, cap_t) * bd.Box(d.stop_post_r[1] - d.stop_post_r[0], 2 * half_w, d.stop_t,
                                                                  align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, d.bearing_width + NUDGE, z0=-NUDGE)
    body = body - cylinder(d.lip_id / 2.0, cap_t + 2 * NUDGE, z0=-NUDGE)
    body = body - _flat(cfg, 0.0, cap_t + d.stop_t)
    for x, y in cap_bolt_points(cfg):
        body = body - cylinder(d.cap_bolt_dia / 2.0, cap_t + 2 * NUDGE, (x, y), z0=-NUDGE)
    return single_solid(body)
