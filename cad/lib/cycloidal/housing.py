"""Shared outer profile of the housing parts (the port's motor plate + ring gear body; the turning shell's shell ring
+ body), and the turning shell's body (build_shell_body).

Port of cycloidal_drive@2f1f67d src/helpers/housing_profile.py. Both parts present the same
silhouette: trapezoidal pillars (one per M4 housing bolt, HousingParams.bolt_count) joined only at
the inner annulus, with reveal windows between them. ``reveal_window_cutter`` is subtracted from an
annular housing solid to imprint that profile; ``chamfer_outer_silhouette`` bevels the result's outer edges.

Pillar (radially flipped trapezoid, lib/cycloidal/layout.py pillar_corners): ``pillar_inner_w`` (18)
tangential at the bore, ``pillar_outer_w`` (10) at the OD; the radial bounds overshoot the bore (-1)
and the OD (+1) so the bore / base-cylinder cuts trim every pillar flush.
"""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.cycloidal.layout import (
    arm_zone,
    housing_bolt_points,
    pillar_corners,
    ring_pin_hole_dia,
    ring_pin_points,
    shell_ends,
    stack_positions,
)
from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig, compute_housing_bolt_angles
from lib.geom import align_min, cylinder, hex_prism, single_solid
from lib.units import NUDGE

CUTTER_OVERSHOOT = 0.1    # [DESIGN] cutter annulus past the OD so the base cylinder trims it flush
BARREL_EDGE_MARGIN = 1.0  # [DESIGN] barrel corners are the vertical edges at r >= od/2 - this (skips r<=66 bolt artefacts)


def reveal_window_cutter(cfg: DriveConfig = DEFAULT_CONFIG, height: float = 0.0, z_offset: float = 0.0):
    """Wall-removal volume leaving a trapezoidal pillar at each bolt angle: bolt_count disjoint
    solids spanning z_offset..z_offset+height. The annulus inside the bore is untouched."""
    h = cfg.housing
    housing_r, bore_r = h.od / 2.0, h.bore_dia / 2.0
    cutter = bd.Cylinder(housing_r + CUTTER_OVERSHOOT, height, align=align_min()) - bd.Cylinder(bore_r, height, align=align_min())
    for a in compute_housing_bolt_angles(cfg):
        pts = pillar_corners(cfg, a)
        pillar = bd.extrude(bd.make_face(bd.Polyline(*pts, close=True)), amount=height, dir=(0, 0, 1))
        cutter = cutter - pillar
    return bd.Pos(0, 0, z_offset) * cutter


def end_face(solid: bd.Shape, z: float, tol: float = 1e-4) -> bd.Face:
    """The largest planar face lying in the plane z (normal along +/-Z)."""
    faces = [
        f for f in solid.faces().filter_by(bd.GeomType.PLANE)
        if abs(f.center().Z - z) <= tol and abs(abs(f.normal_at().Z) - 1.0) <= 1e-6
    ]
    if not faces:
        raise ValueError(f"no planar face at z={z}")
    return max(faces, key=lambda f: f.area)


def chamfer_outer_silhouette(shape: bd.Shape, cfg: DriveConfig = DEFAULT_CONFIG, external_z: float | None = None) -> bd.Solid:
    """Bevel the outer silhouette of a finished housing part (call AFTER the windows are cut).

    Always chamfers the pillars' outer vertical corners (the barrel edges). When ``external_z``
    is the height of the part's externally facing end face, that face's whole outer perimeter
    (outer arcs, window arcs, pillar sides - the outer wire, so internal holes are excluded) is
    chamfered too. Mating faces stay sharp. Unchanged when ``edge_chamfer <= 0``."""
    solid = single_solid(shape)
    ch = cfg.housing.edge_chamfer
    if ch <= 0:
        return solid
    rmin = cfg.housing.od / 2.0 - BARREL_EDGE_MARGIN
    sel = []
    for e in solid.edges():
        if e.geom_type != bd.GeomType.LINE:
            continue
        p0, p1 = e.position_at(0.0), e.position_at(1.0)
        vertical = abs(p1.Z - p0.Z) > 1e-6 and math.hypot(p0.X - p1.X, p0.Y - p1.Y) < 1e-6
        if vertical and math.hypot((p0.X + p1.X) / 2.0, (p0.Y + p1.Y) / 2.0) >= rmin:
            sel.append(e)
    if external_z is not None:
        sel += list(end_face(solid, external_z).outer_wire().edges())
    return solid.chamfer(ch, None, sel) if sel else solid


def hex_pocket(cfg: DriveConfig, xy, angle_rad: float, depth: float, z0: float = 0.0):
    """Captive M4 nut pocket (pocket AF) at xy, from z0 up by ``depth``, hex keyed along ``angle_rad``."""
    return bd.Pos(xy[0], xy[1], z0) * hex_prism(cfg.housing.bolt_nut_pocket_af, angle_rad, depth)


PIN_END_CLEAR = 0.5   # [DESIGN] the ring pins' blind holes this far past the pins' ends


def build_shell_body(cfg: DriveConfig = DEFAULT_CONFIG) -> bd.Solid:
    """The turning shell's body round the discs (ShellParams), at its stack position: z = arm_zone's start (9, on the
    shell ring) .. the shell's hub end (61) - printed as one with j1_link, the upper arm rising off it
    (lib/upper_arm/link.py). The housing's od with its pillars and windows from end to end; inside, the shell ring's
    mirror: the bore round the discs (to the hub's flange, arm_zone's end), a ring_bore_dia pin ring round the flange
    with the 21 ring pins' blind holes from its face on the bore, the hub-end 6814's seat, the end_lip; the housing
    bolts' holes through it and their nuts' hex pockets from the hub end, a flat outward (bolt_nut_turn_deg), their
    floors where the nuts sit on the bolts' ends (stack_positions z_housing_nuts). The hub end chamfered like the port's
    output face; the face on the shell ring sharp."""
    h, sh, b, tol, s = cfg.housing, cfg.shell, cfg.bearings, cfg.tolerances, cfg.stack_up
    (z0, flange), z1 = arm_zone(cfg), shell_ends(cfg)[1]            # 9, 39, 61
    seat = s.z_output_bearings                                       # 48
    body = cylinder(h.od / 2.0, z1 - z0, z0=z0)
    body = body - cylinder(h.bore_dia / 2.0, flange - z0 + NUDGE, z0=z0 - NUDGE)                       # round the discs
    body = body - cylinder(sh.ring_bore_dia / 2.0, seat - flange + NUDGE, z0=flange)                   # the pin ring
    body = body - cylinder(h.output_bearing_seat_dia / 2.0, b.out_width + NUDGE, z0=seat)               # the 6814's seat
    body = body - cylinder(h.lip_bore_dia / 2.0, sh.end_lip + 2 * NUDGE, z0=z1 - sh.end_lip - NUDGE)    # the lip
    at = stack_positions(cfg)
    pins_top = at["z_ring_pins"] + cfg.gear.ring_pin_length + PIN_END_CLEAR
    for xy in ring_pin_points(cfg):
        body = body - cylinder(ring_pin_hole_dia(cfg) / 2.0, pins_top - flange + NUDGE, xy, z0=flange - NUDGE)
    turn = math.radians(h.bolt_nut_turn_deg)
    for angle, xy in zip(compute_housing_bolt_angles(cfg), housing_bolt_points(cfg), strict=True):
        body = body - cylinder((h.bolt_dia + tol.bolt_clearance_add) / 2.0, z1 - z0 + 2 * NUDGE, xy, z0=z0 - NUDGE)
        body = body - hex_pocket(cfg, xy, angle + turn, z1 - at["z_housing_nuts"] + NUDGE, z0=at["z_housing_nuts"])
    body = body - reveal_window_cutter(cfg, z1 - z0, z_offset=z0)
    return chamfer_outer_silhouette(single_solid(body), cfg, external_z=z1)
