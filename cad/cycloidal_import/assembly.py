"""Interactive viewer — compose and display selected parts.

Run this file to visualize parts together in OCP CAD Viewer.
Comment/uncomment sections to show what you want to see.
"""

import cadquery as cq
from ocp_vscode import show_object

from src.params import DEFAULT_CONFIG
from src.cycloidal_disc import build_cycloidal_disc
from src.eccentric_shaft import build_eccentric_shaft
from src.motor_plate import build_motor_plate
from src.ring_gear_body import build_ring_gear_body
from src.output_hub import build_output_hub
from src.purchased_parts import (
    build_bearing_6003,
    build_bearing_6814,
    build_bearing_625,
    build_nema17_motor,
    build_ring_pins,
    build_output_pins,
    build_housing_bolts,
    build_housing_nuts,
    build_motor_bolts,
    build_shaft_support_pin,
)

cfg = DEFAULT_CONFIG
stack = cfg.stack_up
e = cfg.gear.eccentricity  # 1.5mm

# At input angle φ=0, disc 1 center is at (+e, 0) and disc 2 at (-e, 0).
# Each disc + bearing is concentric with its shaft lobe.

# ── Disc 1 + its 6003 bearing ──────────────────────────────────
disc1 = build_cycloidal_disc()
disc1 = disc1.translate((e, 0, stack.z_disc1))

bearing_6003_1 = build_bearing_6003()
bearing_6003_1 = bearing_6003_1.translate((e, 0, stack.z_disc1))

show_object(disc1, name="disc_1", options={"color": "steelblue", "alpha": 0.6})
show_object(bearing_6003_1, name="bearing_6003_1", options={"color": "orange"})

# ── Disc 2 + its 6003 bearing (kinematic phase offset, no assembly rotation) ──
disc2 = build_cycloidal_disc(phase_offset_deg=cfg.gear.disc2_phase_deg)
disc2 = disc2.translate((-e, 0, stack.z_disc2))

bearing_6003_2 = build_bearing_6003()
bearing_6003_2 = bearing_6003_2.translate((-e, 0, stack.z_disc2))

show_object(disc2, name="disc_2", options={"color": "lightblue", "alpha": 0.6})
show_object(bearing_6003_2, name="bearing_6003_2", options={"color": "orange"})

# ── 6814 output bearings ─────────────────────────────────────
bearing_6814_1 = build_bearing_6814()
bearing_6814_1 = bearing_6814_1.translate((0, 0, stack.z_output_bearings))
show_object(bearing_6814_1, name="bearing_6814_1", options={"color": "coral"})

bearing_6814_2 = build_bearing_6814()
bearing_6814_2 = bearing_6814_2.translate((0, 0, stack.z_output_bearings + cfg.bearings.out_width))
show_object(bearing_6814_2, name="bearing_6814_2", options={"color": "coral"})

# ── Eccentric shaft ────────────────────────────────────────────
shaft = build_eccentric_shaft()
show_object(shaft, name="eccentric_shaft", options={"color": "silver"})

# ── Ring pins ──────────────────────────────────────────────────
# Symmetric engagement: 3.5mm in motor plate inner end + 28mm open
# bore zone (disc zone 26mm + output clearance 2mm) + 3.5mm in ring
# gear body bearing-zone wall = 35mm.  Outer 5.5mm of motor plate
# through-hole sits empty above the pin.
bore_zone = stack.disc_zone + stack.output_clearance  # 28mm
pin_engagement = (cfg.gear.ring_pin_length - bore_zone) / 2.0  # 3.5mm
z_pins = stack.z_motor_plate_inner - pin_engagement  # 5.5mm
pins = build_ring_pins()
pins = pins.translate((0, 0, z_pins))
show_object(pins, name="ring_pins", options={"color": "gray"})

# ── Output pins (4mm × 45mm dowels, captured between hub ceiling and motor plate) ──
output = build_output_pins()
output = output.translate((
    0,
    0,
    stack.z_bearing_top - cfg.output_hub.output_hub_pin_ceiling - cfg.disc.output_pin_length,
))
show_object(output, name="output_pins", options={"color": "darkgray"})

# ── Motor ──────────────────────────────────────────────────────
motor = build_nema17_motor()
show_object(motor, name="nema17_motor", options={"color": "dimgray"})

# ── Motor mounting bolts (M3 × 10mm SHCS) ─────────────────────
# Heads recessed inside motor plate; tip 4mm into motor body.
motor_bolts = build_motor_bolts()
motor_bolts = motor_bolts.translate((0, 0, stack.z_motor_plate_inner - cfg.motor.motor_bolt_total_length - 1.0))
show_object(motor_bolts, name="motor_bolts", options={"color": "dimgray"})

# ── Motor plate ──────────────────────────────────────────────────
motor_plate = build_motor_plate()
show_object(motor_plate, name="motor_plate", options={"color": "slategray", "alpha": 0.4})

# ── Ring gear body ─────────────────────────────────────────────
ring_body = build_ring_gear_body()
ring_body = ring_body.translate((0, 0, stack.z_motor_plate_inner))
show_object(ring_body, name="ring_gear_body", options={"color": "slategray", "alpha": 0.3})

# ── Output hub ────────────────────────────────────────────────
output_hub = build_output_hub()
output_hub = output_hub.translate((0, 0, stack.z_output_bearings))
show_object(output_hub, name="output_hub", options={"color": "goldenrod", "alpha": 0.6})

# ── Output shaft pin (steel dowel) ───────────────────────────
z_pin_base = stack.z_disc2 + cfg.disc.thickness - cfg.shaft.support_pin_hole_depth
shaft_pin = build_shaft_support_pin()
shaft_pin = shaft_pin.translate((0, 0, z_pin_base))
show_object(shaft_pin, name="shaft_support_pin", options={"color": "red"})

# ── 625 bearing (output-side shaft support) ───────────────────
# Sits in the output hub's inner-face pocket at z_output_bearings
bearing_625 = build_bearing_625()
bearing_625 = bearing_625.translate((0, 0, stack.z_output_bearings))
show_object(bearing_625, name="bearing_625", options={"color": "orange"})

# ── Housing bolts (M4 × 55mm SHCS) ──────────────────────────
# Head top at Z=0 (motor plate outer face), recessed in counterbore.
# Translate so head bottom aligns with counterbore depth.
h = cfg.housing
cb_recess = h.bolt_counterbore_depth - h.bolt_head_height  # 0.5mm recess
housing_bolts = build_housing_bolts()
housing_bolts = housing_bolts.translate((0, 0, cb_recess))
show_object(housing_bolts, name="housing_bolts", options={"color": "dimgray"})

# ── Housing nuts (M4 hex) ────────────────────────────────────
# Seated in the ring gear body output-face pockets (floor at z_bearing_top=57).
z_nut = stack.total_housing_depth - h.bolt_nut_depth
housing_nuts = build_housing_nuts()
housing_nuts = housing_nuts.translate((0, 0, z_nut))
show_object(housing_nuts, name="housing_nuts", options={"color": "dimgray"})
