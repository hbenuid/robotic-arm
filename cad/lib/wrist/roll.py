"""build123d builders of the wrist roll drive's printed geometry (WristRollParams, module frame, at their stack
stations): the frame wrist_link carries on its plate's free face (the stator - one body round the pancake and the tower
round R'; lib/wrist/link.py puts it on the plate), the pulley (rotor, the output - its 40T band the GT2 ring of
lib/pulley/teeth.py, its face the connector bolts to) and the shaft (rotor, a collar behind bearing 1 with the stop lug)."""
from __future__ import annotations

import math

import build123d as bd

from lib.datum import to_location
from lib.geom import box, cylinder, nut_slot, single_solid, slot_y, stop_wedge
from lib.pulley.teeth import gt2_ring
from lib.units import NUDGE
from lib.wrist.layout import (
    board_window,
    clamp_nut_pocket,
    clamp_points,
    connector_bolt_points_module,
    face_nut_pocket,
    frame_y_span,
    module_frame_in_host,
    motor_pocket,
    pad_bolt_points,
    stack_positions,
)
from lib.wrist.params import DEFAULT, WristConfig


def module_to_host(cfg: WristConfig = DEFAULT) -> bd.Location:
    """The module frame -> wrist_link's part frame (the module's mount frame)."""
    return to_location(module_frame_in_host(cfg))


def profile(cfg: WristConfig = DEFAULT):
    """The frame's outline seen along R', as (module x, module y): the hull of a circle about R' (frame_r - the open +X
    face and the tower's far side) and the pocket's box with its far wall (frame_y_span: on the motor's side of R'), from
    the plate's coupler face (x_coupler: the frame runs down through the plate) up."""
    d, S = cfg.roll, stack_positions(cfg)
    y_far = frame_y_span(cfg)[0 if d.side > 0 else 1]
    tower = bd.Circle(d.frame_r)
    side = bd.Pos((S["x_coupler"] + d.frame_r) / 2.0, y_far / 2.0) * bd.Rectangle(d.frame_r - S["x_coupler"], abs(y_far))
    hull = bd.make_hull((tower + side).edges())
    above = bd.Pos(S["x_coupler"], 0.0) * bd.Rectangle(4.0 * d.frame_r, 4.0 * abs(y_far), align=(bd.Align.MIN, bd.Align.CENTER))
    return hull & above


def build_frame(cfg: WristConfig = DEFAULT):
    """The frame in the module frame: the profile from the frame's rear to its front face (the plate's), the bearing boss
    on that face round R'; the pancake's pocket from the plate's free face out through the open +X face, its front wall
    the plate (the 4 bolt slots and the pilot slot along Y - the belt's tension -, the shaft's slit out to the open
    face), behind it the MKS board's window out through the frame's rear; the tower: bearing 1's seat from the shaft's
    bay behind it, the lip, bearing 2's seat in the boss; the bay open on +X, the stop post on the tower's rear face in
    it."""
    d, S = cfg.roll, stack_positions(cfg)
    outline = profile(cfg)
    y0, y1, z0, z1 = motor_pocket(cfg)
    # the profile from the frame's rear (the tower's: what the wrist pitch's lower limit first meets) to the front face
    body = bd.extrude(bd.Pos(0.0, 0.0, S["z_rear"]) * outline, amount=S["z_front"] - S["z_rear"])
    body = body + cylinder(d.bearing_boss_dia / 2.0, S["z_boss"] - S["z_front"] + 1.0, z0=S["z_front"] - 1.0)
    # the pocket, from the free face out through the open face; the plate in front of it; behind it the board's window,
    # out through the frame's rear (no wall behind the pancake: the board on its rear face stands 11 proud of the frame)
    body = body - box((S["x_free"], d.frame_r + 1.0), (y0, y1), (z0, z1))
    w0, w1, wz0, wz1 = board_window(cfg)
    body = body - box((S["x_free"], d.frame_r + 1.0), (w0, w1), (wz0 - 1.0, wz1))
    for x, y in pad_bolt_points(cfg):
        body = body - slot_y(x, y, d.pad_bolt_dia, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    body = body - slot_y(S["x_motor"], S["y_motor"], d.pad_pilot_w, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    # the shaft's slit through the plate, from the pilot slot out through the open face: the pancake slides in from that
    # face motor_drop back (its shaft in the slit, its pilot behind the plate), then forward into the pilot slot
    slit = (S["y_motor"] - d.shaft_slit / 2.0, S["y_motor"] + d.shaft_slit / 2.0)
    body = body - box((S["x_motor"], d.frame_r + 1.0), slit, (S["z_motor_face"] - NUDGE, S["z_front"] + NUDGE))
    # the tower: bearing 1's seat, the lip, bearing 2's seat (open through the boss's top)
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_1"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + 2 * NUDGE, z0=S["z_lip"] - NUDGE)
    body = body - cylinder(r_seat, d.bearing_width + 2 * NUDGE, z0=S["z_bearing_2"])
    # the shaft's bay behind bearing 1, open on +X (the shaft and bearing 1 go in from it, the cable leaves by it); the
    # stop post on the tower's rear face inside it, at -X
    bay = (S["z_bay"], S["z_bearing_1"])
    body = body - cylinder(d.bay_r, bay[1] - bay[0], z0=bay[0])
    body = body - box((0.0, d.frame_r + 1.0), (-d.bay_r, d.bay_r), bay)
    body = body + stop_wedge(d.stop_post_r[0], d.stop_post_r[1], S["z_stop_post"], d.stop_post_t, d.stop_deg_width, 180.0)
    return single_solid(body)


def build_pulley(cfg: WristConfig = DEFAULT):
    """The pulley (wrist_roll_pulley) in the module frame - the roll's output: its hub (a journal) through bearing 2 to
    the lip's middle, where the shaft's meets it, the Ø33 shoulder on bearing 2's inner ring, the 40T band (two flanges)
    on its core, the face gripper_j3_connector bolts to (its 4 nuts in pockets from the face's rim); the cable bore, the
    rotor clamp's 4 holes with their heads' counterbores in the face."""
    d, S = cfg.roll, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    z_end = S["z_face"]
    # the core: the hub, the shoulder, the core under the band, the face - then every cut, then the band fused on LAST
    # (an annulus on the core, so no boolean ever runs through its grooves)
    body = cylinder(j_r, S["z_boss"] - S["z_meet"], z0=S["z_meet"])
    body = body + cylinder(d.shoulder_dia / 2.0, d.run_gap, z0=S["z_boss"])
    body = body + cylinder(d.ring_core_dia / 2.0, S["z_ring_end"] - S["z_ring_flange_1"], z0=S["z_ring_flange_1"])
    body = body + cylinder(d.face_dia / 2.0, z_end - S["z_ring_end"], z0=S["z_ring_end"])
    body = body - cylinder(d.bore / 2.0, z_end - S["z_meet"] + 2 * NUDGE, z0=S["z_meet"] - NUDGE)
    head_r = d.clamp_screw.head_dia / 2.0 + d.clamp_head_clear
    for x, y in clamp_points(cfg):
        body = body - cylinder(d.bolt_dia / 2.0, z_end - S["z_meet"] + 2 * NUDGE, (x, y), z0=S["z_meet"] - NUDGE)
        body = body - cylinder(head_r, z_end - S["z_clamp_head"] + NUDGE, (x, y), z0=S["z_clamp_head"])
    # the connector's 4x M3: clearance from the face down through it, each nut's pocket a slot from the face's rim inward
    # past the nut's inner corner (a flat either side), the nut under the face
    r_in, r_out, z0, z1 = face_nut_pocket(cfg)
    for x, y in connector_bolt_points_module(cfg):
        body = body - cylinder(d.bolt_dia / 2.0, z_end - S["z_ring_end"] + 2 * NUDGE, (x, y), z0=S["z_ring_end"] - NUDGE)
        body = body - nut_slot(r_in, r_out, d.nut.af + 2 * d.nut_fit, z0, z1, math.atan2(y, x))
    ring = gt2_ring(d.ring_teeth, d.ring_width, d.ring_flange_dia, d.ring_flange_t, z0=S["z_ring"], inner_dia=d.ring_core_dia - 0.2)
    return single_solid(body + ring)


def build_shaft(cfg: WristConfig = DEFAULT):
    """The shaft (wrist_roll_shaft) in the module frame - a collar behind bearing 1: its hub (a journal) up through
    bearing 1 to the lip's middle where the pulley's meets it, the Ø33 shoulder on bearing 1's inner ring, the flange
    behind it with the rotor clamp's 4 nuts (pockets open to its rear face and into the bore) and the hard-stop lug; the
    cable bore."""
    d, S = cfg.roll, stack_positions(cfg)
    j_r = (d.bearing_bore + d.journal_add) / 2.0
    z0 = S["z_shaft_end"]
    body = cylinder(j_r, S["z_meet"] - S["z_bearing_1"], z0=S["z_bearing_1"])
    body = body + cylinder(d.shoulder_dia / 2.0, d.run_gap, z0=S["z_shoulder_1"])
    body = body + cylinder(d.collar_od / 2.0, S["z_shoulder_1"] - z0, z0=z0)
    body = body + stop_wedge(d.stop_lug_r[0], d.stop_lug_r[1], S["z_stop_lug"], d.stop_lug_t, d.stop_deg_width, 0.0)
    body = body - cylinder(d.bore / 2.0, S["z_meet"] - z0 + 2 * NUDGE, z0=z0 - NUDGE)
    r_in, r_out, p0, p1 = clamp_nut_pocket(cfg)
    for x, y in clamp_points(cfg):
        body = body - cylinder(d.bolt_dia / 2.0, S["z_meet"] - z0 + 2 * NUDGE, (x, y), z0=z0 - NUDGE)
        body = body - nut_slot(r_in, r_out, d.nut.af, p0 - NUDGE, p1, math.atan2(y, x))
    return single_solid(body)
