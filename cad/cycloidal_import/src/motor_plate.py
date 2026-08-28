"""Motor plate — mounts the NEMA 17 and supports the input shaft.

3D-printed PETG housing part. Sits at Z=0 (outer/motor face) to Z=9mm
(inner face). Features:
  - Motor pilot recess and M3 bolt pattern on the outer face
  - Central shaft bore for motor shaft pass-through (direct D-shaft engagement)
  - 21 ring-pin through-holes on 108mm circle (clearance fit)
  - 8 M4 housing-bolt through-holes on the perimeter

No motor-side 625 bearing — the motor shaft passes through the plate
and engages the eccentric shaft D-bore directly. Motor internal bearings
and 6003 disc bearings provide adequate radial support.

Ring-pin through-holes (not blind) allow pins to be inserted one at a
time from the outer face. Ring gear body holes provide the retention
fit; motor plate provides lateral location only.
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


def build_motor_plate(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """Build the motor-side housing plate.

    Return a valid CadQuery Workplane with exactly one solid.
    """
    h = cfg.housing
    m = cfg.motor
    tol = cfg.tolerances
    stack = cfg.stack_up

    plate_thickness = stack.motor_plate_wall + stack.motor_plate_inner_wall  # 9mm
    housing_r = h.od / 2.0  # 67mm

    # ── 1. Base disc ────────────────────────────────────────────────
    result = (
        cq.Workplane("XY")
        .circle(housing_r)
        .extrude(plate_thickness)
    )

    # ── 2. Motor pilot recess (outer face, Z=0 side) ───────────────
    pilot_recess_dia = m.pilot_dia + tol.mating_surface_add * 2  # 22.30mm
    pilot_depth = 2.0  # matches NEMA 17 boss height
    pilot_cut = (
        cq.Workplane("XY")
        .circle(pilot_recess_dia / 2.0)
        .extrude(pilot_depth)
    )
    result = result.cut(pilot_cut)

    # ── 3. Central shaft bore (through full plate) ──────────────────
    shaft_bore_dia = 15.0  # mm, enlarged motor shaft pass-through
    shaft_bore = (
        cq.Workplane("XY")
        .circle(shaft_bore_dia / 2.0)
        .extrude(plate_thickness)
    )
    result = result.cut(shaft_bore)

    # ── 4. M3 motor bolt holes (through, clearance fit) ──────────────
    m3_clearance_dia = m.bolt_dia + 0.4  # 3.4mm
    half_pat = m.bolt_pattern_square / 2.0  # 15.5mm
    motor_bolt_pts = [
        (half_pat, half_pat),
        (-half_pat, half_pat),
        (-half_pat, -half_pat),
        (half_pat, -half_pat),
    ]
    motor_bolts = (
        cq.Workplane("XY")
        .pushPoints(motor_bolt_pts)
        .circle(m3_clearance_dia / 2.0)
        .extrude(plate_thickness)
    )
    result = result.cut(motor_bolts)

    # ── 4a. Counterbore pockets for M3 bolt heads (inner face, Z=9mm) ──
    # Recesses bolt heads flush with the inner face so they don't intrude
    # into the input clearance zone.  With a 9mm plate the formula yields
    # 3mm — bolt head (3mm tall) sits flush, no extra pocket above.
    m3_cb_dia = m.motor_bolt_head_dia + 0.4  # 5.7mm clearance
    m3_cb_depth = plate_thickness - (m.motor_bolt_thread_length - (m.bolt_hole_depth - 0.5))  # 3mm
    motor_bolt_cb = (
        cq.Workplane("XY")
        .workplane(offset=plate_thickness)
        .pushPoints(motor_bolt_pts)
        .circle(m3_cb_dia / 2.0)
        .extrude(-m3_cb_depth)
    )
    result = result.cut(motor_bolt_cb)

    # ── 5. Ring-pin through-holes (21×, clearance fit) ─────────────
    # Through-holes let pins be inserted one at a time from the outer
    # face. Chamfered lead-in on the inner face guides entry during
    # assembly. Ring gear body provides retention; motor plate locates.
    g = cfg.gear
    pin_hole_dia = g.ring_pin_dia - tol.ring_pin_press_sub  # 4.20mm
    pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm
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
        .extrude(plate_thickness)
    )
    result = result.cut(pin_holes)

    # ── 6. M4 housing bolt holes (through, clearance fit) ─────────
    # Bolt angles match ring gear body — offset to avoid ring pin holes.
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
        .extrude(plate_thickness)
    )
    result = result.cut(bolt_holes)

    # ── 7. Counterbore pockets for M4 bolt heads (outer face, Z=0) ──
    counterbores = (
        cq.Workplane("XY")
        .pushPoints(bolt_pts)
        .circle(h.bolt_counterbore_dia / 2.0)
        .extrude(h.bolt_counterbore_depth)
    )
    result = result.cut(counterbores)

    # ── 8. Reveal windows — match shared housing profile ────────────
    # Cuts the outer ring (58→70mm radius) into 8 trapezoidal pillars
    # aligned with the ring gear body pillars.  The central disc (motor
    # mount, pilot, ring-pin holes, inner pocket) is untouched.
    result = result.cut(build_reveal_window_cutter(cfg, plate_thickness))

    # ── 9. Bevel the outer silhouette ──────────────────────────────
    # Full outer perimeter of the motor face (Z=0, external) + barrel verticals.
    # Inner face (Z=9) seats on the ring gear body → left sharp.
    result = chamfer_outer_silhouette(result, cfg, external_face="<Z")

    return result


if __name__ == "__main__":
    from ocp_vscode import show_object

    plate = build_motor_plate()
    show_object(plate, name="motor_plate", options={"color": "slategray", "alpha": 0.7})
