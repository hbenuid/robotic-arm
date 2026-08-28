"""Eccentric shaft — converts motor rotation into disc wobble.

3D-printed PETG (100% infill, 0.16mm layer height per spec). Two 17.10mm
OD lobes offset 180° from each other (one per disc), connected by a 5mm
OD spine. The lobe centers are offset ±eccentricity from the shaft axis.

The input end has a D-bore socket that receives the motor shaft directly
(no coupler). A 10mm OD collar at the input provides wall thickness
around the D-bore.
"""

import sys
import os

import cadquery as cq

# Allow running directly or as a module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, DEFAULT_CONFIG


def build_eccentric_shaft(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """Build the eccentric shaft with two offset lobes and a central spine."""
    shaft = cfg.shaft
    motor = cfg.motor
    tol = cfg.tolerances
    stack = cfg.stack_up
    disc_t = cfg.disc.thickness

    e = shaft.eccentricity
    lobe_r = shaft.bearing_seat_od / 2.0
    spine_r = shaft.spine_od / 2.0
    collar_r = shaft.input_collar_od / 2.0

    # Z positions from stack-up
    z_start = stack.z_motor_plate_inner  # 9mm — shaft starts at motor plate inner face
    z_lobe1 = stack.z_disc1  # 13mm
    z_lobe2 = stack.z_disc2  # 25mm
    z_end = z_lobe2 + disc_t  # 35mm (steel dowel pin extends beyond)
    total_length = z_end - z_start

    # ── Spine: 5mm OD cylinder from collar end to output ────────────
    spine = (
        cq.Workplane("XY")
        .workplane(offset=z_start)
        .circle(spine_r)
        .extrude(total_length)
    )

    # ── Input collar: enlarged section for D-bore wall thickness ────
    collar_length = z_lobe1 - z_start  # 4mm (input_clearance)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=z_start)
        .circle(collar_r)
        .extrude(collar_length)
    )

    # ── Lobe 1: 17mm OD, center offset +e in X ─────────────────────
    lobe1 = (
        cq.Workplane("XY")
        .workplane(offset=z_lobe1)
        .center(e, 0)
        .circle(lobe_r)
        .extrude(disc_t)
    )

    # ── Lobe 2: 17mm OD, center offset -e in X (180° from lobe 1) ──
    lobe2 = (
        cq.Workplane("XY")
        .workplane(offset=z_lobe2)
        .center(-e, 0)
        .circle(lobe_r)
        .extrude(disc_t)
    )

    # ── Bridge + retention flange: loft between lobes with enlarged OD ──
    # Transitions the eccentric center from (+e, 0) to (-e, 0) through
    # the inter-disc spacer zone. The entire bridge is oversized (23.10mm)
    # so it acts as a bearing retention flange, preventing the two 6003
    # bearings from sliding toward each other.
    spacer_t = cfg.disc.inter_disc_spacer
    z_bridge_start = z_lobe1 + disc_t  # where lobe 1 ends
    flange_r = (shaft.bearing_seat_od + 6.0) / 2.0  # 11.55mm
    bridge = (
        cq.Workplane("XY")
        .workplane(offset=z_bridge_start)
        .center(e, 0)
        .circle(flange_r)
        .workplane(offset=spacer_t)
        .center(-2 * e, 0)
        .circle(flange_r)
        .loft(ruled=True)
    )

    result = spine.union(collar).union(lobe1).union(lobe2).union(bridge)

    # ── Output pin hole: blind hole for steel dowel press-fit ─────
    pin_bore_dia = shaft.support_pin_dia + tol.dowel_bore_clearance_add * 2  # 5.15mm
    pin_hole_depth = shaft.support_pin_hole_depth  # 12mm
    z_output_face = z_lobe2 + disc_t  # 35mm

    pin_hole = (
        cq.Workplane("XY")
        .workplane(offset=z_output_face - pin_hole_depth)
        .circle(pin_bore_dia / 2.0)
        .extrude(pin_hole_depth)
    )
    result = result.cut(pin_hole)

    # ── D-bore: motor shaft socket from input face ──────────────────
    # Round bore + D-flat key matching the motor shaft profile
    bore_dia = shaft.d_bore_dia + tol.d_bore_clearance_add * 2  # 5.13mm
    bore_r = bore_dia / 2.0
    bore_depth = shaft.d_bore_depth  # 14mm

    d_bore_round = (
        cq.Workplane("XY")
        .workplane(offset=z_start)
        .circle(bore_r)
        .extrude(bore_depth)
    )
    result = result.cut(d_bore_round)

    # D-flat key: the motor shaft has a flat at 4.5mm across. After cutting
    # a full round bore, we add back a key block so the D-cut locks in.
    dcut_offset = shaft.d_bore_flat / 2.0 + tol.d_bore_clearance_add  # 2.315mm
    cut_depth = bore_r - dcut_offset  # amount to NOT cut (the key)
    if cut_depth > 0:
        # Add back a key block (material the D-bore leaves in place)
        key_block = (
            cq.Workplane("XY")
            .workplane(offset=z_start)
            .transformed(offset=(0, bore_r - cut_depth / 2.0, 0))
            .rect(bore_dia, cut_depth)
            .extrude(bore_depth)
        )
        result = result.union(key_block)

    return result


if __name__ == "__main__":
    from ocp_vscode import show_object

    shaft = build_eccentric_shaft()
    show_object(shaft, name="eccentric_shaft")
