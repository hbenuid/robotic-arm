"""Simplified purchased-part models for assembly visualization.

These are not manufacturing models — just enough geometry to verify
fitment, clearances, and the axial stack-up.
"""

import math
import sys
import os

import cadquery as cq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, DEFAULT_CONFIG, compute_housing_bolt_angles


# -------------------------------------------------------------------
# Bearings — annular cylinders (bore/OD/width)
# -------------------------------------------------------------------


def build_bearing_6003(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """6003-2RS eccentric bearing: 17×35×10mm."""
    b = cfg.bearings
    return (
        cq.Workplane("XY")
        .circle(b.ecc_od / 2.0)
        .circle(b.ecc_bore / 2.0)
        .extrude(b.ecc_width)
    )


def build_bearing_6814(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """6814-2RS output bearing: 70×90×10mm."""
    b = cfg.bearings
    return (
        cq.Workplane("XY")
        .circle(b.out_od / 2.0)
        .circle(b.out_bore / 2.0)
        .extrude(b.out_width)
    )


def build_bearing_625(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """625-2RS shaft support bearing (output side): 5×16×5mm."""
    b = cfg.bearings
    return (
        cq.Workplane("XY")
        .circle(b.inp_od / 2.0)
        .circle(b.inp_bore / 2.0)
        .extrude(b.inp_width)
    )


# -------------------------------------------------------------------
# NEMA 17 motor — square body + cylindrical shaft
# -------------------------------------------------------------------


def build_nema17_motor(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """NEMA 17 stepper: 42.3mm square × 48mm body with 5mm × 20mm shaft.

    Built so that the mounting face is at Z=0 and the body extends in -Z.
    The shaft extends in +Z from the mounting face.
    The shaft has a D-cut flat (4.5mm across) over the top 18mm.
    4× M3 threaded blind holes on the 31mm square bolt pattern.
    """
    m = cfg.motor
    shaft_r = m.shaft_dia / 2.0

    # Body: square prism extending in -Z
    body = (
        cq.Workplane("XY")
        .rect(m.body_width, m.body_width)
        .extrude(-m.body_length)
    )

    # Centering pilot / boss on the mounting face
    pilot = (
        cq.Workplane("XY")
        .circle(m.pilot_dia / 2.0)
        .extrude(2.0)  # typical 2mm boss height
    )

    # Shaft: full 5mm round base section (bottom 2mm), then D-cut section
    round_length = m.shaft_length - m.shaft_dcut_length  # 4mm round base
    shaft_round = (
        cq.Workplane("XY")
        .circle(shaft_r)
        .extrude(round_length)
    )

    # D-cut section: 5mm circle intersected with a half-space to create the flat
    # The flat is at distance dcut_flat/2 from center, cutting the +Y side
    dcut_offset = m.shaft_dcut_flat / 2.0  # 2.25mm from center
    shaft_dcut = (
        cq.Workplane("XY")
        .workplane(offset=round_length)
        .circle(shaft_r)
        .extrude(m.shaft_dcut_length)
    )
    # Cut block to create the D-flat: remove material beyond the flat
    cut_depth = shaft_r - dcut_offset  # 2.5 - 2.25 = 0.25mm
    cut_block = (
        cq.Workplane("XY")
        .workplane(offset=round_length)
        .transformed(offset=(0, shaft_r - cut_depth / 2.0, 0))
        .rect(m.shaft_dia, cut_depth)
        .extrude(m.shaft_dcut_length)
    )
    shaft_dcut = shaft_dcut.cut(cut_block)

    result = body.union(pilot).union(shaft_round).union(shaft_dcut)

    # M3 threaded blind holes on 31mm square bolt pattern (into mounting face)
    # Holes go from Z=0 into -Z (into the body)
    half_pat = m.bolt_pattern_square / 2.0
    bolt_positions = [
        (half_pat, half_pat),
        (-half_pat, half_pat),
        (-half_pat, -half_pat),
        (half_pat, -half_pat),
    ]
    holes = (
        cq.Workplane("XY")
        .pushPoints(bolt_positions)
        .circle(m.bolt_dia / 2.0)
        .extrude(-m.bolt_hole_depth)
    )
    result = result.cut(holes)

    return result


# -------------------------------------------------------------------
# Motor mounting bolts — 4× M3 × 10mm SHCS on 31mm square pattern
# -------------------------------------------------------------------


def build_motor_bolts(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """4× M3 × 10mm socket head cap screws for NEMA 17 mounting.

    Built with thread tip at Z=0, thread extends in +Z (10mm),
    then head extends in +Z (3mm more).  Total height = 13mm.
    Caller positions so thread tip sits 4mm into the motor body.
    """
    m = cfg.motor
    half_pat = m.bolt_pattern_square / 2.0
    positions = [
        (half_pat, half_pat),
        (-half_pat, half_pat),
        (-half_pat, -half_pat),
        (half_pat, -half_pat),
    ]

    # Thread / shank: 3mm ⌀ × 10mm, from Z=0 to Z=10mm
    shanks = (
        cq.Workplane("XY")
        .pushPoints(positions)
        .circle(m.bolt_dia / 2.0)
        .extrude(m.motor_bolt_thread_length)
    )

    # Head: 5.3mm ⌀ × 3mm, from Z=10mm to Z=13mm
    heads = (
        cq.Workplane("XY")
        .workplane(offset=m.motor_bolt_thread_length)
        .pushPoints(positions)
        .circle(m.motor_bolt_head_dia / 2.0)
        .extrude(m.motor_bolt_head_height)
    )

    return shanks.union(heads)


# -------------------------------------------------------------------
# Ring pins — 21 cylinders on 108mm circle
# -------------------------------------------------------------------


def build_ring_pins(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """21× 4mm × 35mm ring pins on the 108mm pin circle."""
    g = cfg.gear
    r = g.ring_pin_circle_dia / 2.0
    positions = [
        (
            r * math.cos(2 * math.pi * i / g.num_ring_pins),
            r * math.sin(2 * math.pi * i / g.num_ring_pins),
        )
        for i in range(g.num_ring_pins)
    ]
    return (
        cq.Workplane("XY")
        .pushPoints(positions)
        .circle(g.ring_pin_dia / 2.0)
        .extrude(g.ring_pin_length)
    )


# -------------------------------------------------------------------
# Output pins — 4 cylinders on 60mm circle
# -------------------------------------------------------------------


def build_output_pins(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """4× 4mm × 45mm h6 ground steel dowel pins on the 60mm output pin circle.

    Captured in blind clearance holes (4.20mm × 19mm) in the output hub —
    closed hub ceiling above the pins, motor plate inner face below.
    Free-floating through the discs (8mm clearance accommodates the
    1.5mm eccentric motion).
    """
    d = cfg.disc
    r = d.output_pin_circle_dia / 2.0
    positions = [
        (
            r * math.cos(2 * math.pi * i / d.output_pin_count),
            r * math.sin(2 * math.pi * i / d.output_pin_count),
        )
        for i in range(d.output_pin_count)
    ]
    return (
        cq.Workplane("XY")
        .pushPoints(positions)
        .circle(d.output_pin_dia / 2.0)
        .extrude(d.output_pin_length)
    )


# -------------------------------------------------------------------
# Housing bolts — 8× M4 × 55mm SHCS on 125mm bolt circle
# -------------------------------------------------------------------


def build_housing_bolts(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """8× M4 × 55mm socket head cap screws on the housing bolt circle.

    Built with head top at Z=0 (motor plate outer face).  Head sits in
    the counterbore; shank extends in +Z through the housing.
    """
    h = cfg.housing
    bolt_r = h.bolt_circle_dia / 2.0
    bolt_angles = compute_housing_bolt_angles(cfg)
    positions = [
        (bolt_r * math.cos(a), bolt_r * math.sin(a))
        for a in bolt_angles
    ]

    # Cylindrical head: 7mm ⌀ × 4mm, from Z=0 to Z=head_height
    heads = (
        cq.Workplane("XY")
        .pushPoints(positions)
        .circle(h.bolt_head_dia / 2.0)
        .extrude(h.bolt_head_height)
    )

    # Cylindrical shank: 4mm ⌀ × 55mm, from Z=head_height onward
    shanks = (
        cq.Workplane("XY")
        .workplane(offset=h.bolt_head_height)
        .pushPoints(positions)
        .circle(h.bolt_dia / 2.0)
        .extrude(h.bolt_length)
    )

    return heads.union(shanks)


# -------------------------------------------------------------------
# Housing nuts — 8× M4 hex nuts on 125mm bolt circle
# -------------------------------------------------------------------


def build_housing_nuts(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """8× M4 hex nuts on the housing bolt circle.

    Built at Z=0; caller translates to the nut pocket floor position.
    """
    h = cfg.housing
    bolt_r = h.bolt_circle_dia / 2.0
    bolt_angles = compute_housing_bolt_angles(cfg)
    nut_circ_dia = 7.0 / math.cos(math.radians(30))  # nominal M4 nut

    result = None
    for a in bolt_angles:
        x = bolt_r * math.cos(a)
        y = bolt_r * math.sin(a)
        nut = (
            cq.Workplane("XY")
            .center(x, y)
            .transformed(rotate=(0, 0, math.degrees(a)))
            .polygon(6, nut_circ_dia)
            .extrude(h.bolt_nut_thickness)
        )
        result = nut if result is None else result.union(nut)

    return result


# -------------------------------------------------------------------
# Output shaft pin — 5mm steel dowel for eccentric shaft support
# -------------------------------------------------------------------


def build_shaft_support_pin(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """5mm × 20mm steel dowel pin for eccentric shaft output support.

    Built at Z=0 with full pin length extending in +Z.
    Caller positions it so the insertion end sits inside the shaft hole.
    """
    shaft = cfg.shaft
    return (
        cq.Workplane("XY")
        .circle(shaft.support_pin_dia / 2.0)
        .extrude(shaft.support_pin_length)
    )


if __name__ == "__main__":
    from ocp_vscode import show_object

    show_object(build_bearing_6003(), name="bearing_6003")
    show_object(build_bearing_6814(), name="bearing_6814")
    show_object(build_bearing_625(), name="bearing_625")
    show_object(build_nema17_motor(), name="nema17_motor")
    show_object(build_ring_pins(), name="ring_pins")
    show_object(build_output_pins(), name="output_pins")
    show_object(build_housing_bolts(), name="housing_bolts")
    show_object(build_housing_nuts(), name="housing_nuts")
    show_object(build_motor_bolts(), name="motor_bolts")
    show_object(build_shaft_support_pin(), name="shaft_support_pin")
