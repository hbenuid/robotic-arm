"""Ring gear body — housing cylinder, bearing seat, integral retention lip, nut pockets.

3D-printed PETG housing part.  Spans from the motor plate inner face
(global Z=9mm) to the housing output face (global Z=60mm).  Local Z=0 at
the input face.  This is the output end of the housing — the separate
output cap has been removed and its two structural jobs folded in here.

Stepped internal bore:
  Z=0  to Z=28  — 116mm bore (disc orbit clearance + 2mm output gap)
  Z=28 to Z=48  — 90.15mm bore (press-fit seat for 2× 6814-2RS output
                   bearings)
  Z=48 to Z=51  — 86.15mm bore (output hub clearance).  The 90.15→86.15
                   step at Z=48 is the integral retention lip (2mm radial)
                   that blocks the 90mm 6814 outer races from the output side.

Output-bearing retention: press-fit into the 90.15mm seat plus the integral
lip from the output side.  No input shoulder ring is used — the disc
envelope (~108mm) exceeds the 6814 OD (90mm), so a shoulder cannot
simultaneously clear the disc and block the bearing.  Bearings insert from
the input side and seat up against the lip; they stay removable toward the
disc side (not trapped).

Captive housing nuts: 8× hex pockets (4mm deep) on the output face seat the
M4 nuts for the M4×55 housing bolts (heads counterbored in the motor plate).
Each pocket is backed by the full-height bolt pillar, so the thin output wall
does not limit the pocket depth.

Ring-pin retention (dual-end):
  35mm pins sit 3.5mm in motor plate through-holes and 3.5mm into the
  ring gear body bearing zone.  The middle 28mm spans the disc +
  clearance zone (116mm bore — pins in air).  Blind holes (31.5mm deep)
  from the input face avoid cutting through the bearing seat wall.

Other features:
  - 21 ring-pin blind holes on 108mm circle (4.20mm clearance dia, 31.5mm deep)
  - 8 M4 housing-bolt through-holes on 125mm circle + output-face nut pockets

Assembly: insert ring pins through the motor plate's through-holes,
then slide the ring gear body onto the protruding pin ends.  Chamfered
entries on both parts guide the pins in.  No need to align all 21
pins simultaneously — insert and seat them one at a time.
"""

import math
import sys
import os

import cadquery as cq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, DEFAULT_CONFIG, compute_housing_bolt_angles
from src.helpers.housing_profile import (
    build_reveal_window_cutter,
    chamfer_outer_silhouette,
)


def build_ring_gear_body(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """Build the ring gear body housing cylinder.

    Return a valid CadQuery Workplane with exactly one solid.
    """
    g = cfg.gear
    h = cfg.housing
    tol = cfg.tolerances
    stack = cfg.stack_up

    # ── Dimensions ────────────────────────────────────────────────
    body_height = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm (now includes the output-end wall the cap used to provide)
    housing_r = h.od / 2.0  # 67mm
    bore_r = h.bore_dia / 2.0  # 58mm (116mm bore)
    bearing_seat_r = h.output_bearing_seat_dia / 2.0  # 45.075mm

    # Zone boundaries (local Z)
    disc_zone_end = (
        stack.input_clearance
        + stack.disc_thickness * 2
        + stack.inter_disc_spacer
    )  # 26mm
    bore_zone_end = disc_zone_end + stack.output_clearance  # 28mm (2mm disc clearance)
    bearing_z = bore_zone_end  # 28mm
    bearing_h = stack.output_bearing_total  # 20mm

    # Pin hole dimensions
    pin_hole_dia = g.ring_pin_dia - tol.ring_pin_press_sub  # 4.20mm clearance
    pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm

    # ── 1. Base cylinder ─────────────────────────────────────────
    result = (
        cq.Workplane("XY")
        .circle(housing_r)
        .extrude(body_height)
    )

    # ── 2. Main bore: Z=0 to Z=28 (116mm, disc orbit + clearance) ─
    main_bore = (
        cq.Workplane("XY")
        .circle(bore_r)
        .extrude(bore_zone_end)
    )
    result = result.cut(main_bore)

    # ── 3. Output bearing seat: Z=28 to Z=48 (90.15mm) ──────────
    bearing_bore = (
        cq.Workplane("XY")
        .workplane(offset=bearing_z)
        .circle(bearing_seat_r)
        .extrude(bearing_h)
    )
    result = result.cut(bearing_bore)

    # ── 3b. Hub-clearance bore + integral retention lip: Z=48 to top (86.15mm) ──
    # Above the bearing seat the bore narrows to 86.15mm, leaving a 2mm radial
    # lip.  The 90.15→86.15 step at Z=48 retains the 6814 outer races from the
    # output side (the removed cap's job, now printed in); 86.15mm still clears
    # the 70.3mm output hub with ~7.9mm radial gap.
    lip_bore_dia = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
    bearing_seat_top = bearing_z + bearing_h  # 48mm (local)
    lip_bore = (
        cq.Workplane("XY")
        .workplane(offset=bearing_seat_top)
        .circle(lip_bore_dia / 2.0)
        .extrude(body_height - bearing_seat_top)
    )
    result = result.cut(lip_bore)

    # ── 4. Ring-pin blind holes (21×, press-fit) ──────────────────
    # Holes go from Z=0 (input face) through the bore zone (28mm,
    # air at 54mm radius inside 58mm bore) plus 3.5mm of press-fit
    # engagement into the bearing zone.  Total depth 31.5mm.
    pin_engagement = (g.ring_pin_length - bore_zone_end) / 2.0  # 3.5mm
    pin_hole_depth = bore_zone_end + pin_engagement  # 31.5mm
    pin_pts = [
        (
            pin_circle_r * math.cos(2 * math.pi * i / g.num_ring_pins),
            pin_circle_r * math.sin(2 * math.pi * i / g.num_ring_pins),
        )
        for i in range(g.num_ring_pins)
    ]
    pin_holes = (
        cq.Workplane("XY")
        .pushPoints(pin_pts)
        .circle(pin_hole_dia / 2.0)
        .extrude(pin_hole_depth)
    )
    result = result.cut(pin_holes)

    # Chamfer at bore/bearing transition — pins are in air (116mm bore)
    # from Z=0 to Z=28, then enter solid bearing-zone material.
    chamfer_depth = 1.0  # mm
    chamfer_dia = pin_hole_dia + 1.0  # mm, funnel entry
    for pt in pin_pts:
        cone = (
            cq.Workplane("XY")
            .workplane(offset=bore_zone_end)
            .center(pt[0], pt[1])
            .circle(chamfer_dia / 2.0)
            .workplane(offset=chamfer_depth)
            .circle(pin_hole_dia / 2.0)
            .loft(ruled=True)
        )
        result = result.cut(cone)

    # ── 5. M4 housing-bolt through-holes ─────────────────────────
    # Bolt angles are offset to sit at midpoints between adjacent
    # ring pins, preventing hole overlap on the shared annular wall.
    m4_clearance_dia = h.bolt_dia + 0.4  # 4.4mm
    bolt_r = h.bolt_circle_dia / 2.0  # 62.5mm
    bolt_angles = compute_housing_bolt_angles(cfg)
    bolt_pts = [
        (bolt_r * math.cos(a), bolt_r * math.sin(a))
        for a in bolt_angles
    ]
    bolt_holes = (
        cq.Workplane("XY")
        .pushPoints(bolt_pts)
        .circle(m4_clearance_dia / 2.0)
        .extrude(body_height)
    )
    result = result.cut(bolt_holes)

    # ── 6. Reveal windows — expose ring pins between bolt pillars ──
    # Cut away the outer wall over the full body height, keeping only
    # trapezoidal pillars around each housing bolt.  The motor plate seats
    # against the 8 pillar top faces (no continuous rim).  Shared with the
    # motor plate so both parts present the same outer silhouette.
    result = result.cut(build_reveal_window_cutter(cfg, body_height))

    # ── 7. Captive hex nut pockets on the output face (8×, M4 housing nuts) ──
    # Ported from the removed output cap: each pocket opens on the output face
    # (Z=body_height) and seats an M4 nut for the housing bolt.  Unlike a
    # standalone cap, each pocket is backed by the full-height bolt pillar, so
    # the thin output wall imposes no floor constraint.  Hex flat faces radially
    # outward to maximise the pocket-to-OD wall.
    nut_circ_dia = h.bolt_nut_pocket_af / math.cos(math.radians(30))  # ~8.31mm
    for angle, pt in zip(bolt_angles, bolt_pts):
        hex_pocket = (
            cq.Workplane("XY")
            .workplane(offset=body_height - h.bolt_nut_depth)
            .center(pt[0], pt[1])
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .polygon(6, nut_circ_dia)
            .extrude(h.bolt_nut_depth)
        )
        result = result.cut(hex_pocket)

    # ── 8. Bevel the outer silhouette ──────────────────────────────
    # The output end face (Z=body_height) is now external (cap removed) → bevel
    # its full outer perimeter + barrel verticals.  The motor-side face (Z=0)
    # mates against the motor plate → left sharp.
    result = chamfer_outer_silhouette(result, cfg, external_face=">Z")

    return result


if __name__ == "__main__":
    from ocp_vscode import show_object

    body = build_ring_gear_body()
    show_object(body, name="ring_gear_body", options={"color": "slategray", "alpha": 0.5})
