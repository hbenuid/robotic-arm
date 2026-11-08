"""build123d builders of the wrist roll drive's printed geometry (WristRollParams, module frame, at their stack
stations): the frame wrist_link carries on its plate's free face (the stator - laid out as the forearm roll's frame: one
body round the pancake and the tower round R', the pancake in from the bottom, the far face closed, the shaft's bay
bored through the rear under a removable back panel; lib/wrist/link.py puts it on the plate), the back panel, the pulley
(rotor, the output - its 40T band the GT2 ring of lib/pulley/teeth.py, its face the connector bolts to) and the shaft
(rotor, a collar behind bearing 1 with the stop lug)."""
from __future__ import annotations

import math

import build123d as bd

from lib.datum import to_location
from lib.geom import box, cylinder, faceted_end, nut_slot, single_solid, slot_y, stop_wedge
from lib.pulley.teeth import gt2_ring
from lib.style import EDGE_CHAMFER, SIDES
from lib.units import NUDGE
from lib.wrist.layout import (
    board_plug_rooms,
    board_window,
    clamp_nut_pocket,
    clamp_points,
    connector_bolt_points_module,
    face_nut_pocket,
    module_frame_in_host,
    motor_pocket,
    pad_bolt_points,
    pancake_plug_groove,
    rear_line,
    rear_plane,
    rear_z,
    side_wall_x,
    stack_positions,
    toward_motor,
)
from lib.wrist.params import DEFAULT, WristConfig

FAR = 500.0   # past every part: a cut or a half-plane bounded on one side only


def module_to_host(cfg: WristConfig = DEFAULT) -> bd.Location:
    """The module frame -> wrist_link's part frame (the module's mount frame)."""
    return to_location(module_frame_in_host(cfg))


def _half_space(point, normal):
    """The half-space n . (p - point) >= 0 as a solid (the side a cut removes)."""
    n = bd.Vector(normal).normalized()
    return bd.Plane(origin=point, z_dir=n) * bd.Box(4 * FAR, 4 * FAR, 2 * FAR, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def _x_prism(face_yz, x0: float, x1: float):
    """A face drawn in module (y, z) extruded along X from x0 to x1."""
    plane = bd.Plane(origin=(x0, 0.0, 0.0), x_dir=(0.0, 1.0, 0.0), z_dir=(1.0, 0.0, 0.0))
    return bd.extrude(plane * face_yz, amount=x1 - x0)


def _z_prism(face_xy):
    """A face drawn in module (x, y) extruded along Z through everything."""
    return bd.Pos(0.0, 0.0, -FAR) * bd.extrude(face_xy, amount=2.0 * FAR)


def _bottom(y_from: float, cfg: WristConfig = DEFAULT) -> tuple[float, float]:
    """A y span from y_from on down out through the frame's bottom (the end beyond the motor)."""
    s = toward_motor(cfg)
    return (y_from, s * FAR) if s > 0 else (-FAR, y_from)


def side_profile(cfg: WristConfig = DEFAULT):
    """The frame seen along the pitch axis, a face in module (y, z): the plate's octagon about the pitch axis (lib/geom.py
    faceted_end, flats pitch_r - the forearm roll frame's octagonal end about its motor's axis) cut at rear_low, hulled to
    the tower round R' (frame_r either side, z_rear to the front face) - cut at the front face and on the rear's slanted
    plane (rear_line) - straight sides, sharp corners."""
    d, S, p = cfg.roll, stack_positions(cfg), cfg.plate
    octagon = faceted_end(p.pitch_r, (S["y_motor"], 0.0)) & bd.Pos(0.0, (d.rear_low + FAR) / 2.0) * bd.Rectangle(4 * FAR, FAR - d.rear_low)
    tower = bd.Pos(0.0, (S["z_rear"] + S["z_front"]) / 2.0) * bd.Rectangle(2.0 * d.frame_r, S["z_front"] - S["z_rear"])
    hull = bd.make_hull((octagon + tower).edges())
    hull = hull & bd.Pos(0.0, (S["z_front"] - FAR) / 2.0) * bd.Rectangle(4 * FAR, FAR + S["z_front"])
    (ya, za), (yb, zb) = rear_line(cfg)
    k = (zb - za) / (yb - ya)
    y0, y1 = min(ya, yb) - FAR / 4.0, max(ya, yb) + FAR / 4.0
    above = bd.Polygon((y0, za + k * (y0 - ya)), (y1, za + k * (y1 - ya)), (y1, FAR), (y0, FAR), align=None)
    return hull & above


def tower_section(cfg: WristConfig = DEFAULT):
    """The frame seen along R', a face in module (x, y): the tower the octagon about R' (flats frame_r, so its far side and
    the far face's flat are the octagon's) over the half-plane on the motor's side, where the plate and the far face
    bound the frame."""
    s = toward_motor(cfg)
    return faceted_end(cfg.roll.frame_r) + bd.Pos(0.0, s * FAR / 2.0) * bd.Rectangle(4 * FAR, FAR)


def front_facets(cfg: WristConfig = DEFAULT):
    """What the front face's two long edges' facets take away (front_facet at 45 deg, along the frame end to end): at the
    far face and at the plate's coupler face - the plate itself fills that one back where it runs."""
    d, S = cfg.roll, stack_positions(cfg)
    f = d.front_facet
    return (_half_space((d.frame_r, 0.0, S["z_front"] - f), (1.0, 0.0, 1.0))
            + _half_space((S["x_coupler"], 0.0, S["z_front"] - f), (-1.0, 0.0, 1.0)))


def envelope(cfg: WristConfig = DEFAULT):
    """The frame's outer body before any cut: the side profile from the coupler face to the far face, cut to the tower's
    section, the front facets off, the outline of both side faces (x_coupler, frame_r) chamfered (lib/style.py
    EDGE_CHAMFER) - the outline every face of the frame and the back panel follows."""
    d, S = cfg.roll, stack_positions(cfg)
    body = single_solid((_x_prism(side_profile(cfg), S["x_coupler"], d.frame_r) & _z_prism(tower_section(cfg))) - front_facets(cfg))
    edges = []
    for f in body.faces():
        if f.geom_type == bd.GeomType.PLANE and abs(abs(f.normal_at().X) - 1.0) < 1e-6 and any(abs(f.center().X - x) < 1e-3 for x in (S["x_coupler"], d.frame_r)):
            edges += list(f.outer_wire().edges())
    return body.chamfer(EDGE_CHAMFER, None, edges)


def _rear_slab(cfg: WristConfig, d0: float, d1: float):
    """The slab between the planes d0 and d1 in from the rear plane (along its inward normal), everywhere."""
    (px, py, pz), (nx, ny, nz) = rear_plane(cfg)
    a = (px - nx * d0, py - ny * d0, pz - nz * d0)
    b = (px - nx * d1, py - ny * d1, pz - nz * d1)
    return _half_space(a, (-nx, -ny, -nz)) & _half_space(b, (nx, ny, nz))


def _plate_prism(cfg: WristConfig, grow: float = 0.0):
    """The plate's octagon (its coupler face to its free face + grow), in the module frame."""
    S = stack_positions(cfg)
    return _x_prism(faceted_end(cfg.plate.pitch_r, (S["y_motor"], 0.0)), S["x_coupler"] - 1.0, S["x_free"] + grow)


def _panel_region(cfg: WristConfig, y_to: float, grow: float = 0.0):
    """Within back.t of the rear plane, over the tower: from its far side to y_to (toward the motor), off the plate."""
    s = toward_motor(cfg)
    ys = (-FAR, y_to) if s > 0 else (y_to, FAR)
    return (envelope(cfg) & _rear_slab(cfg, -1.0, cfg.roll.back.t) & box((-FAR, FAR), ys, (-FAR, FAR))) - _plate_prism(cfg, grow)


def back_point(x: float, y: float, depth: float = 0.0, cfg: WristConfig = DEFAULT) -> tuple[float, float, float]:
    """The module point at (x, y) on the rear plane, `depth` in from it."""
    _p, (_nx, ny, nz) = rear_plane(cfg)
    return (x, y - ny * depth, rear_z(y, cfg) - nz * depth)


def _on_rear(x: float, y: float, radius: float, depth: float, amount: float, cfg: WristConfig = DEFAULT):
    """A disc at (x, y) on the rear plane, `depth` in from it, `amount` further in along the plane's inward normal."""
    _p, (nx, ny, nz) = rear_plane(cfg)
    return bd.extrude(bd.Plane(origin=back_point(x, y, depth, cfg), z_dir=(-nx, -ny, -nz)) * bd.Circle(radius), amount=amount)


def back_magnet(x: float, y: float, depth: float, cfg: WristConfig = DEFAULT):
    """One of the back panel's disc magnets (WristBackPanelParams magnet_*) at (x, y), its outer face `depth` in from the
    rear plane."""
    b = cfg.roll.back
    return _on_rear(x, y, b.magnet_dia / 2.0, depth, b.magnet_t, cfg)


def back_magnet_depths(cfg: WristConfig = DEFAULT) -> tuple[float, float]:
    """The outer faces' depths in from the rear plane of each pair's two magnets: the frame's, pressed to the bottom of
    its pocket in the step's floor, and the panel's, pressed to the bottom of its pocket in the panel's inner face."""
    b = cfg.roll.back
    return b.t + b.pocket_depth - b.magnet_t, b.t - b.pocket_depth


def _notches(cfg: WristConfig):
    """What the back panel leaves open over the bay: the gripper servo's cable's notch (from notch_reach inside the bay's
    bore out through its edge toward the motor) and the board's near side plug's room (plug_clear round it)."""
    d, b, s = cfg.roll, cfg.roll.back, toward_motor(cfg)
    cut = box((-b.notch_w / 2.0, b.notch_w / 2.0), _bottom(s * (d.bay_r - b.notch_reach), cfg), (-FAR, FAR))
    w0, w1, _, _ = board_window(cfg)
    near = w0 if s > 0 else w1
    for x0, x1, y0, y1, _z0, _z1 in board_plug_rooms(cfg).values():
        if (y0 - near) * s < 0.0 < (y1 - near) * s or (y1 - near) * s < 0.0 < (y0 - near) * s:   # it reaches under the panel
            cut = cut + box((x0 - d.plug_clear, x1 + d.plug_clear), _bottom(min(y0, y1, key=lambda v: v * s) - s * d.plug_clear, cfg), (-FAR, FAR))
    return cut


def back_grill_field(cfg: WristConfig = DEFAULT) -> tuple[float, float]:
    """The back panel's grill field (WristBackPanelParams grill_clear), in module (x, y): (flats, edge) - an octagon about
    R' with its corners grill_clear inside the ring rim's inner edge, and its edge toward the motor (y) grill_clear short
    of the cable's notch."""
    d, b, s = cfg.roll, cfg.roll.back, toward_motor(cfg)
    corner = d.bay_r - b.rim_clear - b.rim_w - b.grill_clear
    return corner * math.cos(math.pi / SIDES), s * (d.bay_r - b.notch_reach - b.grill_clear)


def back_grill(cfg: WristConfig = DEFAULT):
    """What the back panel's grill takes away (WristBackPanelParams grill_*, lib/style.py GRILL_SLOT / GRILL_PITCH):
    straight slots along y, one on R', clipped to the field (back_grill_field) and cut through along R' - as the panel's
    notches are, so the field keeps grill_clear off the ring rim at every depth."""
    b, s = cfg.roll.back, toward_motor(cfg)
    flats, edge = back_grill_field(cfg)
    field = faceted_end(flats) & bd.Pos(0.0, edge - s * FAR / 2.0) * bd.Rectangle(4 * FAR, FAR)
    n = int((flats - b.grill_w / 2.0) // b.grill_pitch)
    slots = [bd.Pos(k * b.grill_pitch, 0.0) * bd.Rectangle(b.grill_w, 4 * flats) for k in range(-n, n + 1)]
    return _z_prism(sum(slots[1:], slots[0]) & field)


def build_frame(cfg: WristConfig = DEFAULT):
    """The frame in the module frame: the envelope (seen along the pitch axis the plate's octagon hulled to the tower round
    R', from the plate's coupler face to the closed far face, the front facets off, both side faces' outline chamfered),
    the bearing boss from 1 mm under the facets up to bearing 2's front face; the back panel's step on the rear over the
    tower; the pancake's pocket from the plate's free face to the side wall, on down out through the bottom - it slides
    in from there, motor_drop back -, its front wall the plate (the 4 bolt slots and the pilot slot along Y - the belt's
    tension -, the shaft's slit on down out through the bottom), a groove inside the side wall for its plug (and the
    board's motor plug's room); behind it the board's window, out through the rear and the bottom (its side plugs' rooms:
    in it, and under the panel's notch); the tower: bearing 1's seat, the lip, bearing 2's seat in the boss; the shaft's bay behind bearing 1 bored on through
    the rear, the stop post on the tower's rear face in it; the magnets' pockets in the step's floor; the cable's groove
    across it into the window."""
    d, b, S, s = cfg.roll, cfg.roll.back, stack_positions(cfg), toward_motor(cfg)
    x_wall = side_wall_x(cfg)
    body = envelope(cfg)
    boss_z0 = S["z_front"] - d.front_facet - 1.0
    body = body + cylinder(d.bearing_boss_dia / 2.0, S["z_boss"] - boss_z0, z0=boss_z0)
    # the back panel's step: back.t in from the rear plane over the tower, from its far side to the board's window
    w0, w1, _wz0, wz1 = board_window(cfg)
    w_near = w0 if s > 0 else w1
    body = body - _panel_region(cfg, w_near)
    # the pocket and the board's window, on down out through the bottom; the plug's groove inside the side wall
    y0, y1, z0, z1 = motor_pocket(cfg)
    body = body - box((S["x_free"], x_wall), _bottom(y0 if s > 0 else y1, cfg), (z0, z1))
    body = body - box((S["x_free"], x_wall), _bottom(w_near, cfg), (-FAR, wz1))
    gx0, gx1, gy0, _gy1, _gz0, gz1 = pancake_plug_groove(cfg)
    body = body - box((gx0 - NUDGE, gx1), _bottom(gy0, cfg), (-FAR, gz1))
    for x, y in pad_bolt_points(cfg):
        body = body - slot_y(x, y, d.pad_bolt_dia, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    body = body - slot_y(S["x_motor"], S["y_motor"], d.pad_pilot_w, d.pad_slot_len, S["z_motor_face"], d.pad_t)
    # the shaft's slit through the plate, from the pilot slot down out through the bottom: the pancake slides in from
    # there motor_drop back (its shaft up the slit, its pilot behind the plate), then forward into the pilot slot
    slit = (S["x_motor"] - d.shaft_slit / 2.0, S["x_motor"] + d.shaft_slit / 2.0)
    body = body - box(slit, _bottom(S["y_motor"], cfg), (S["z_motor_face"] - NUDGE, S["z_front"] + NUDGE))
    # the tower: bearing 1's seat, the lip, bearing 2's seat (open through the boss's top); the shaft's bay behind bearing
    # 1 bored on through the rear (the shaft and bearing 1 go in from behind, the panel off); the stop post in it at -X
    r_seat = (d.bearing_od + d.seat_add) / 2.0
    body = body - cylinder(r_seat, d.bearing_width + NUDGE, z0=S["z_bearing_1"] - NUDGE)
    body = body - cylinder(d.lip_id / 2.0, d.lip + 2 * NUDGE, z0=S["z_lip"] - NUDGE)
    body = body - cylinder(r_seat, d.bearing_width + 2 * NUDGE, z0=S["z_bearing_2"])
    body = body - cylinder(d.bay_r, S["z_bearing_1"] + FAR, z0=-FAR)
    body = body + stop_wedge(d.stop_post_r[0], d.stop_post_r[1], S["z_stop_post"], d.stop_post_t, d.stop_deg_width, 180.0)
    # the magnets' pockets in the step's floor; the cable's groove across it from the bay into the window
    for x, y in b.magnet_points:
        body = body - _on_rear(x, y, b.pocket_dia / 2.0, b.t - NUDGE, b.pocket_depth + NUDGE, cfg)
    groove = box((-b.notch_w / 2.0, b.notch_w / 2.0), _bottom(s * (d.bay_r - 1.0), cfg), (-FAR, FAR))
    body = body - (groove & _rear_slab(cfg, b.t - NUDGE, b.t + b.notch_groove))
    return single_solid(body)


def build_back_panel(cfg: WristConfig = DEFAULT):
    """The back panel (wrist_back_panel) in the module frame: the step's plate - back.t in from the rear plane over the
    tower, from its far side to gap short of the board's window, side_clear off the plate's free face, its outline the
    frame's (chamfered with it) -, its locating rim (a ring inside the bay's bore, rim_h in from its inner face), the
    magnets' pockets in its inner face, its notches over the bay (the cable's, the board's near plug's), the grill's slots
    through it over the bay (back_grill)."""
    d, b, s = cfg.roll, cfg.roll.back, toward_motor(cfg)
    w0, w1, _, _ = board_window(cfg)
    w_near = w0 if s > 0 else w1
    body = _panel_region(cfg, w_near - s * b.gap, b.side_clear)
    r_out = d.bay_r - b.rim_clear
    ring = cylinder(r_out, 2.0 * FAR, z0=-FAR) - cylinder(r_out - b.rim_w, 2.0 * FAR + 2.0, z0=-FAR - 1.0)
    body = body + (ring & _rear_slab(cfg, b.t - NUDGE, b.t + b.rim_h))
    for x, y in b.magnet_points:
        body = body - _on_rear(x, y, b.pocket_dia / 2.0, b.t - b.pocket_depth, b.pocket_depth + NUDGE, cfg)
    return single_solid(body - _notches(cfg) - back_grill(cfg))


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
