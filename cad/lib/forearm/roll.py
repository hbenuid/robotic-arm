"""build123d builders of the forearm roll drive's printed parts (RollDriveParams, module frame, at their stack
stations): the elbow block (stator), the hollow roll shaft with its integral 90T (rotor) and the bearing retainer."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.cycloidal.geom import align_min, cylinder, single_solid
from lib.cycloidal.housing import hex_prism
from lib.datum import to_location
from lib.forearm.layout import (
    flange_bolt_points_module, module_frame_in_host, pad_bolt_points, retainer_bolt_points, stack_positions,
)
from lib.forearm.link import elbow_disc, stadium, x_cylinder
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.forearm.pulley import gt2_ring
from lib.units import NUDGE


def host_to_module(cfg: ForearmConfig = DEFAULT) -> bd.Location:
    """j2_link's frame -> the module frame (the inverse of the module's mount frame)."""
    return to_location(module_frame_in_host(cfg)).inverse()


def build_block(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    # the material first: the SolidWorks disc's j3_coupler interface (bore, 4x M4 with hex nut pockets - lib/forearm/link.py
    # elbow_disc, re-expressed in the module frame), the tube round the roll axis, the two retainer lugs on the wrist
    # face, the motor pad tower (the plate the motor bolts to + the filler down to the tube)
    body = elbow_disc(cfg).moved(host_to_module(cfg))
    body = body + cylinder(d.tube_od / 2.0, S["z_face"] - S["z_end"], z0=S["z_end"])
    lug_len = 12.0
    for x, y in retainer_bolt_points(cfg):
        body = body + bd.Pos(x, y, S["z_face"] - lug_len) * bd.Box(d.retainer_ear_w, d.retainer_ear_w, lug_len, align=align_min())
    z_pad = S["z_motor_face"]                                   # the plate sits on the mounting face, on the shaft side
    x_tube_top = d.tube_od / 2.0
    x_plate_edge = S["x_motor"] - d.pad_w / 2.0
    body = body + bd.Pos(S["x_motor"], 0.0, z_pad) * bd.Box(d.pad_w, d.pad_w, d.pad_t, align=align_min())
    body = body + bd.Pos(x_tube_top - 2.0, 0.0, S["z_lip"]) * bd.Box(x_plate_edge - x_tube_top + 4.0, d.pad_w - 6.0, S["z_pad_top"] - S["z_lip"],
                                                                     align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    # then every cut through the union (the disc's rim reaches inside the tube's bores): the lip bearing 1 stops on, one
    # seat for both bearings, the cable exit from the bore's end out through the tube top (+X; end wall + lip only, the
    # seat stays round), the lugs' blind M3 holes, the pad's four tension slots and the pilot boss slot
    body = body - cylinder(d.lip_id / 2.0, S["z_seat"] - S["z_lip"] + NUDGE, z0=S["z_lip"])
    body = body - cylinder((d.bearing_od + d.seat_add) / 2.0, S["z_face"] - S["z_seat"] + NUDGE, z0=S["z_seat"])
    win_len = min(d.cable_window_len, S["z_seat"] - S["z_end"])
    body = body - bd.Pos(0.0, 0.0, S["z_end"] - NUDGE) * bd.Box(d.tube_od / 2.0 + NUDGE, d.cable_window_w, win_len + NUDGE,
                                                                align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    for x, y in retainer_bolt_points(cfg):
        body = body - cylinder((d.retainer_bolt_dia - 0.6) / 2.0, 8.0 + NUDGE, (x, y), z0=S["z_face"] - 8.0)   # M3 self-tapping
    for x, y in pad_bolt_points(cfg):
        body = body - bd.Pos(0.0, y, 0.0) * stadium(d.pad_slot_len, d.pad_bolt_dia, d.pad_t + 2 * NUDGE, x, z_pad - NUDGE)
    body = body - stadium(d.pad_slot_len, d.pad_pilot_w, d.pad_t + 2 * NUDGE, S["x_motor"], z_pad - NUDGE)
    return single_solid(body)


def build_shaft(cfg: ForearmConfig = DEFAULT):
    d, S = cfg.drive, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    s_r = d.shoulder_od / 2.0
    # the core: journals, shoulders, the flange + spigot, the stop-pin boss - then every cut, then the ring fused on
    # LAST (an annulus on the Ø44 core, so no boolean ever runs through its 90 grooves)
    body = cylinder(j_r, S["z_shoulder_mid"] - S["z_shaft_end"], z0=S["z_shaft_end"])            # journal 1 (bearing 1 from the elbow end)
    body = body + cylinder(s_r, d.bearing_gap, z0=S["z_shoulder_mid"])                            # the middle shoulder between the inner races
    body = body + cylinder(j_r, d.bearing_width, z0=S["z_bearing_2"])                             # journal 2
    body = body + cylinder(s_r, S["z_flange"] - S["z_face"], z0=S["z_face"])                      # shoulder 2 .. through the retainer, under the ring, the bare shaft
    body = body + cylinder(d.flange_dia / 2.0, d.flange_t + d.spigot_len, z0=S["z_flange"])       # the flange + its spigot into the wall's recess
    x_boss = (d.bore + d.shoulder_od) / 4.0                                                        # the boss starts inside the core's wall (never on the axis or the bore)
    body = body + x_cylinder(d.stop_pin_boss_dia / 2.0, d.stop_r - 4.0 - x_boss, (0.0, S["z_stop_pin"]), x_boss)   # the hard-stop pin boss (radial, +X)
    body = body - x_cylinder(1.25, d.stop_r - 4.0 - s_r + 2.0 + NUDGE, (0.0, S["z_stop_pin"]), s_r - 2.0)   # its M3 pilot hole (self-tapping)
    body = body - cylinder(d.bore / 2.0, S["z_spigot_end"] - S["z_shaft_end"] + 2 * NUDGE, z0=S["z_shaft_end"] - NUDGE)   # the cable bore
    for xy in flange_bolt_points_module(cfg):                                                      # 4x M3 through the flange, nuts captive from its elbow face
        body = body - cylinder(cfg.roll_end.bolt_dia / 2.0, d.flange_t + d.spigot_len + 2 * NUDGE, xy, z0=S["z_flange"] - NUDGE)
        body = body - bd.Pos(xy[0], xy[1], S["z_flange"] - NUDGE) * hex_prism(d.flange_nut_af, 0.0, d.flange_nut_depth + NUDGE)
    ring = gt2_ring(d.ring_teeth, d.ring_width, d.ring_flange_dia, d.ring_flange_t, z0=S["z_ring"], inner_dia=d.shoulder_od - 0.2)
    return single_solid(body + ring)


def build_retainer(cfg: ForearmConfig = DEFAULT):
    """The bearing retainer at its LOCAL origin (standing on z=0; the module places it on the block's wrist face):
    an annulus over bearing 2's outer race, two ears for the M3s into the block's lugs, the hard-stop post on the +Y
    ear rising past the ring to the stop pin's station."""
    d, S = cfg.drive, stack_positions(cfg)
    body = cylinder(d.tube_od / 2.0, d.retainer_t)
    for x, y in retainer_bolt_points(cfg):
        ear_x = 26.0 if y > 0 else d.retainer_ear_w          # the +Y ear reaches out to carry the stop post
        body = body + bd.Pos(x - d.retainer_ear_w / 2.0, y, 0.0) * bd.Box(ear_x, d.retainer_ear_w, d.retainer_t,
                                                                          align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
        body = body - cylinder(d.retainer_bolt_dia / 2.0, d.retainer_t + 2 * NUDGE, (x, y), z0=-NUDGE)
    post_h = S["z_stop_pin"] + d.stop_pin_boss_dia / 2.0 + 2.0 - S["z_face"]
    post_x = (d.stop_r ** 2 - d.retainer_ear_y ** 2) ** 0.5 - 3.0            # the post's inner face at stop_r from the axis
    body = body + bd.Pos(post_x, d.retainer_ear_y, 0.0) * bd.Box(6.0, d.retainer_ear_w, post_h, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    body = body - cylinder(d.retainer_id / 2.0, d.retainer_t + 2 * NUDGE, z0=-NUDGE)
    return single_solid(body)
