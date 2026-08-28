"""Output hub — passes through 2× 6814 inner races, carries output pins.

3D-printed PETG part.  Sits at global Z=37mm (z_output_bearings) and rises to
Z=65mm — 5mm proud of the housing output face (z=60) so the arm link mounts
above the stationary chassis without rubbing.  Local Z=0 is the inner face
(disc-facing side).

Features:
  - 625 bearing pocket on inner face (supports eccentric shaft output stub)
  - 4× output pin holes on 60mm circle — 4.20mm clearance, blind from the
    output side; dowels are captured between the closed hub ceiling
    and the motor plate inner face
  - Central shaft clearance bore (6.0mm), capped above the bearing-grip zone;
    the proud arm-mount face carries a central lightening recess (arm link bears
    on the r=25→35mm annulus around the bolt circle)
  - 4× M4 arm-link mounting holes (50mm circle, 45° off the output pins) running
    through the hub into captive M4 hex-nut pockets on the inner face

Print orientation: print output-face-down so the proud arm-mount face is a clean
bed face; drop the 4 M4 nuts into the inner-face pockets before pressing the hub
through the 6814 bearings (no access afterward).

The hub OD is sized for a light press into the 6814 bearing bores (70mm
nominal, reduced by PETG inner-shaft tolerance).
"""

import math
import sys
import os

import cadquery as cq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, DEFAULT_CONFIG


def build_output_hub(cfg: DriveConfig = DEFAULT_CONFIG) -> cq.Workplane:
    """Build the output hub/plate that passes through 2× 6814 bearings.

    Return a valid CadQuery Workplane with exactly one solid.
    """
    hub = cfg.output_hub
    d = cfg.disc
    b = cfg.bearings
    tol = cfg.tolerances
    stack = cfg.stack_up
    h = cfg.housing

    # ── Dimensions ──────────────────────────────────────────────────
    hub_od = hub.od  # 70.3mm (0.3mm interference grip on the 70mm 6814 inner race)
    bearing_grip = stack.output_bearing_total  # 20mm — section that grips the 2× 6814 inner races
    # Total height extends past the housing so the arm-mount face is proud of the chassis.
    hub_height = bearing_grip + stack.output_wall + hub.proud_above_housing  # 28mm → top face at z=65
    shaft_bore_dia = hub.shaft_clearance_bore  # 6.0mm

    # 625 bearing pocket (outer race press-fit seat on inner face)
    bearing_pocket_dia = b.inp_od + tol.bearing_seat_bore_add  # 16.2mm
    bearing_pocket_depth = b.inp_width  # 5mm

    # Output pin blind clearance holes (ring-pin convention).  Depth references the
    # bearing-grip zone (not the taller total height) so the pins + 1mm ceiling stay
    # pinned at z=37→57 regardless of the proud extension.
    pin_circle_r = d.output_pin_circle_dia / 2.0  # 30mm
    pin_hole_dia = d.output_pin_dia - tol.ring_pin_press_sub  # 4.20mm clearance
    pin_hole_depth = bearing_grip - hub.output_hub_pin_ceiling  # 19mm, leaves 1mm closed top

# ── 1. Base cylinder ────────────────────────────────────────────
    result = (
        cq.Workplane("XY")
        .circle(hub_od / 2.0)
        .extrude(hub_height)
    )

    # ── 2. Central shaft clearance bore (bearing-grip zone only) ────
    # Runs z=37→57 over the 625 pocket / shaft region; the proud section
    # (z=57→65) stays solid apart from the central lightening recess (step 6).
    shaft_bore = (
        cq.Workplane("XY")
        .circle(shaft_bore_dia / 2.0)
        .extrude(bearing_grip)
    )
    result = result.cut(shaft_bore)

    # ── 3. 625 bearing pocket (inner face, Z=0) ────────────────────
    bearing_pocket = (
        cq.Workplane("XY")
        .circle(bearing_pocket_dia / 2.0)
        .extrude(bearing_pocket_depth)
    )
    result = result.cut(bearing_pocket)

    # ── 4. Output pin holes (4×, 60mm circle) ──────────────────────
    pin_pts = [
        (
            pin_circle_r * math.cos(2 * math.pi * i / d.output_pin_count),
            pin_circle_r * math.sin(2 * math.pi * i / d.output_pin_count),
        )
        for i in range(d.output_pin_count)
    ]
    pin_holes = (
        cq.Workplane("XY")
        .pushPoints(pin_pts)
        .circle(pin_hole_dia / 2.0)
        .extrude(pin_hole_depth)
    )
    result = result.cut(pin_holes)

    # ── 5. Arm-link mount: M4 clearance through-holes + captive nut pockets ──
    # Bolt drops from the arm link, through the hub, into an M4 hex nut trapped on
    # the inner face.  The bolt circle is offset 45° from the output pins so the
    # nut pockets clear them.  Reuses the housing M4 nut spec.
    arm_r = hub.arm_mount_bolt_circle_dia / 2.0  # 25mm
    arm_hole_dia = h.bolt_dia + 0.4  # 4.4mm M4 clearance (matches housing bolt holes)
    arm_off = math.radians(hub.arm_mount_angle_offset_deg)  # 45° → holes at 45/135/225/315
    arm_angles = [
        arm_off + 2 * math.pi * i / hub.arm_mount_bolt_count
        for i in range(hub.arm_mount_bolt_count)
    ]
    arm_pts = [(arm_r * math.cos(a), arm_r * math.sin(a)) for a in arm_angles]

    # Through clearance holes (full height, z=37→65)
    arm_holes = (
        cq.Workplane("XY")
        .pushPoints(arm_pts)
        .circle(arm_hole_dia / 2.0)
        .extrude(hub_height)
    )
    result = result.cut(arm_holes)

    # Captive M4 hex-nut pockets on the inner face (Z=0), each hex oriented along
    # its radial so the nut is held against rotation while the bolt is tightened.
    nut_circ_dia = h.bolt_nut_pocket_af / math.cos(math.radians(30))  # ~8.31mm across-corners
    for angle, pt in zip(arm_angles, arm_pts):
        hex_pocket = (
            cq.Workplane("XY")
            .center(pt[0], pt[1])
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .polygon(6, nut_circ_dia)
            .extrude(h.bolt_nut_depth)  # 4mm deep, z=37→41
        )
        result = result.cut(hex_pocket)

    # ── 6. Central lightening pocket (proud arm-mount face) ─────────
    # The block above the bearing-grip zone is dead material at the center
    # (shaft bore stops at z=bearing_grip, pins at r=30, arm bolts at r=25).
    # Open a blind recess from the top face, leaving a solid floor above the
    # grip zone.  Set arm_mount_pocket_dia=0 to keep a fully sealed face.
    if hub.arm_mount_pocket_dia > 0.0:
        pocket_z0 = bearing_grip + hub.arm_mount_pocket_floor  # 21mm
        pocket_depth = hub_height - pocket_z0  # up to the top (output) face
        lightening = (
            cq.Workplane("XY")
            .workplane(offset=pocket_z0)
            .circle(hub.arm_mount_pocket_dia / 2.0)
            .extrude(pocket_depth)
        )
        result = result.cut(lightening)

    return result


if __name__ == "__main__":
    from ocp_vscode import show_object

    hub = build_output_hub()
    show_object(hub, name="output_hub", options={"color": "goldenrod", "alpha": 0.7})
