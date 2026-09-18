"""Single source of truth for the shared dimensions of the robotic-arm CAD.

Every constant carries a provenance tag:
  [MEASURE]   verify with calipers on real hardware
  [DATASHEET] manufacturer spec sheet
  [DESIGN]    our chosen value (free to tune)
  [REFERENCE] read off the SolidWorks reference geometry (reference/*.step, see
              reference/manifest.json) - replace with a designed value when the
              part is converted to parametric build123d
  [ESTIMATE]  placeholder until measured / looked up

Units: millimetres and grams. Datum: the SolidWorks capture frame of reference/placements.json
- **+Y is up** (the J1 axis), the arm extends toward -X, Z is the pitch-axis direction. Each part
keeps its SolidWorks part-file frame until it is converted (see CLAUDE.md). The URDF's REP-103
base frame (Z up, X forward) is defined in robot/frames.py; assemblies/arm.py EMITS the arm in that
frame (ARM_FROM_W - cadgen's viewer and snapshots are Z-up), the placements themselves stay in the
capture frame. `lib/` never imports `parts/`.
"""
import math

# --- Units / print globals ----------------------------------------------------
IN = 25.4               # mm per inch (some SolidWorks exports are inch-unit; OCCT converts on import)
NUDGE = 0.01            # tiny overshoot so boolean cuts punch fully through a face
PETG_DENSITY = 1.27e-3  # [DESIGN] g/mm^3 - printed-part mass estimates

# --- Fasteners ----------------------------------------------------------------
M3_CLEAR = 3.4          # [DESIGN] close clearance hole for an M3 screw
M4_CLEAR = 4.5          # [DESIGN] close clearance hole for an M4 screw
M5_CLEAR = 5.5          # [DESIGN] close clearance hole for an M5 screw

# --- Belt drive (GT2) ---------------------------------------------------------
GT2_PITCH = 2.0                                     # [DATASHEET] GT2 tooth pitch
GT2_PULLEY_90T_TEETH = 90                           # [REFERENCE] printed 90T pulley (parts/gt2_pulley_90t), used x2
GT2_PULLEY_20T_TEETH = 20                           # [REFERENCE] purchased 20T pulley (parts/gt2_pulley_20t)
GT2_PULLEY_90T_PITCH_DIA = GT2_PULLEY_90T_TEETH * GT2_PITCH / math.pi   # 57.30 mm pitch diameter
GT2_PULLEY_20T_PITCH_DIA = GT2_PULLEY_20T_TEETH * GT2_PITCH / math.pi   # 12.73 mm
GT2_RATIO = GT2_PULLEY_90T_TEETH / GT2_PULLEY_20T_TEETH                 # 4.5:1 [REFERENCE] candidate gear_ratio for src/config.py JOINTS

# --- Gripper hardware ---------------------------------------------------------
RAIL_DIA = 6.0          # [REFERENCE] round linear rail (parts/gripper_rail_6mm), used x2
RAIL_LEN = 125.0        # [REFERENCE] reference geometry length

# MG996R standard servo (parts/mg996r_servo). [DATASHEET] TowerPro MG996R; verify on the unit in hand.
MG996R_BODY_L = 40.7            # [DATASHEET] body length (along the mounting tabs)
MG996R_BODY_W = 19.7            # [DATASHEET] body width
MG996R_BODY_H = 42.9            # [DATASHEET] body height incl. output boss, excl. horn
MG996R_TAB_L = 54.5             # [DATASHEET] overall length over the mounting tabs
MG996R_MOUNT_HOLE_SP = 49.5     # [DATASHEET] mounting-hole spacing along the tabs
MG996R_MOUNT_HOLE_SP_W = 10.0   # [DATASHEET] mounting-hole spacing across the tabs
MG996R_MASS_G = 55.0            # [DATASHEET]

# --- NEMA 17 pancake stepper (parts/nema17_pancake) ---------------------------------
# The SolidWorks model is a 7-part sub-assembly of the motor's internals, flattened into
# one vendor STEP (vendor/nema17_pancake.step). Envelope from its reference bounding box.
NEMA17_FACE = 42.3              # [DATASHEET] NEMA 17 square face
NEMA17_BOLT_SP = 31.0           # [DATASHEET] mounting-hole square pattern
NEMA17_PILOT_DIA = 22.0         # [DATASHEET] locating boss
NEMA17_SHAFT_DIA = 5.0          # [DATASHEET]
PANCAKE_BODY_W = 41.5           # [REFERENCE] body width  (reference bbox 41.5 x 47.0 x 43.0 incl. connector + shaft)
PANCAKE_BODY_D = 47.0           # [REFERENCE] body depth incl. connector
PANCAKE_BODY_H = 43.0           # [REFERENCE] height incl. shaft
PANCAKE_MASS_G = 180.0          # [ESTIMATE] typical 17HS08-type pancake 150-200 g - replace with the datasheet value

# --- Cycloidal drive (lib/cycloidal/, assemblies/cycloidal_drive.py, docs/cycloidal_drive.md) -----
# The drive's own dimensions live in lib/cycloidal/params.py (DriveConfig, ported from the
# cycloidal_drive repo). These are the interface values the rest of the arm needs, re-exported
# from that config so every number exists exactly once. Drive frame: Z = motor axis, z=0 = the
# motor-plate outer face, +Z toward the output hub; it sits in the arm at placements.json
# "cycloidal_drive#1" (housing in the j1_coupler yoke, hub output face bolted to j1_link) and IS
# the robot's shoulder_pitch joint (robot/frames.py: stator in shoulder_link, rotor in upper_arm_link).
from lib.cycloidal.params import DEFAULT_CONFIG as _DRIVE  # noqa: E402

CYCLOIDAL_RATIO = _DRIVE.gear.gear_ratio                          # 20:1 [DESIGN] 20 lobes / 21 ring pins.
#   NOTE: src/config.py JOINTS still carries gear_ratio 1.0 on J1..J3 - which MKS motor drives the
#   shoulder_pitch joint (if any of them) is unconfirmed.
CYCLOIDAL_HOUSING_OD = _DRIVE.housing.od                          # 140 [DESIGN]
CYCLOIDAL_STACK_DEPTH = _DRIVE.stack_up.total_housing_depth       # 60 [DESIGN] motor-plate outer face -> housing output face
CYCLOIDAL_MOTOR_BODY_LEN = _DRIVE.motor.body_length               # 48 [DATASHEET] NEMA 17 body behind the plate (-Z)
CYCLOIDAL_HUB_OD = _DRIVE.output_hub.od                           # 70.3 [DESIGN]
CYCLOIDAL_HUB_PROUD = _DRIVE.output_hub.proud_above_housing       # 5 [DESIGN] hub face past the housing output face
CYCLOIDAL_OUTPUT_FACE_Z = CYCLOIDAL_STACK_DEPTH + CYCLOIDAL_HUB_PROUD   # 65 [DESIGN] the j1_link mounting face
CYCLOIDAL_ARM_MOUNT_BOLT_CIRCLE = _DRIVE.output_hub.arm_mount_bolt_circle_dia       # 50 [DESIGN]
CYCLOIDAL_ARM_MOUNT_BOLT_COUNT = _DRIVE.output_hub.arm_mount_bolt_count             # 4x M4 into captive nuts
CYCLOIDAL_ARM_MOUNT_ANGLE_OFFSET_DEG = _DRIVE.output_hub.arm_mount_angle_offset_deg # 45 (between the output pins)
CYCLOIDAL_ARM_MOUNT_BOLT_DIA = _DRIVE.housing.bolt_dia            # 4 (M4)

# Purchased parts of the drive (parts/bearing_*.py, nema17_48mm, cycloidal_*_pins/bolts/nuts).
STEEL_DENSITY = 7.85e-3          # [DATASHEET] g/mm^3 - dowel pins, bolts, nuts
CYCLOIDAL_MOTOR_MASS_G = 400.0   # [DATASHEET] 48 mm-body NEMA 17 (17HS19-2004S1 class); verify on the unit in hand
BEARING_6003_MASS_G = 39.0       # [DATASHEET] 6003-2RS 17x35x10
BEARING_6814_MASS_G = 110.0      # [DATASHEET] 6814-2RS (61814) 70x90x10; verify
BEARING_625_MASS_G = 5.0         # [DATASHEET] 625-2RS 5x16x5


def _cyl_vol(radius, height):
    return math.pi * radius * radius * height


def _hex_vol(across_flats, height):
    r = across_flats / math.cos(math.radians(30)) / 2.0      # circumradius
    return 1.5 * math.sqrt(3) * r * r * height


# [ESTIMATE] simplified-geometry volumes x steel density (the parts are plain cylinders / hex prisms)
CYCLOIDAL_RING_PINS_MASS_G = STEEL_DENSITY * _DRIVE.gear.num_ring_pins * _cyl_vol(_DRIVE.gear.ring_pin_radius, _DRIVE.gear.ring_pin_length)            # 72.5, 21x
CYCLOIDAL_OUTPUT_PINS_MASS_G = STEEL_DENSITY * _DRIVE.disc.output_pin_count * _cyl_vol(_DRIVE.disc.output_pin_dia / 2, _DRIVE.disc.output_pin_length)   # 17.8, 4x
CYCLOIDAL_SUPPORT_PIN_MASS_G = STEEL_DENSITY * _cyl_vol(_DRIVE.shaft.support_pin_dia / 2, _DRIVE.shaft.support_pin_length)                             # 3.1
CYCLOIDAL_HOUSING_BOLTS_MASS_G = STEEL_DENSITY * _DRIVE.housing.bolt_count * (
    _cyl_vol(_DRIVE.housing.bolt_head_dia / 2, _DRIVE.housing.bolt_head_height) + _cyl_vol(_DRIVE.housing.bolt_dia / 2, _DRIVE.housing.bolt_length))    # 53.1, 8x M4x55
CYCLOIDAL_HOUSING_NUTS_MASS_G = STEEL_DENSITY * _DRIVE.housing.bolt_count * _hex_vol(_DRIVE.housing.bolt_nut_af, _DRIVE.housing.bolt_nut_thickness)     # 8.5, 8x M4
CYCLOIDAL_MOTOR_BOLTS_MASS_G = STEEL_DENSITY * 4 * (
    _cyl_vol(_DRIVE.motor.bolt_dia / 2, _DRIVE.motor.motor_bolt_thread_length) + _cyl_vol(_DRIVE.motor.motor_bolt_head_dia / 2, _DRIVE.motor.motor_bolt_head_height))   # 4.3, 4x M3x10

# --- Robot description (robot/frames.py, robot/arm.urdf) --------------------------------------
# Joint limits and actuator ratings are PLACEHOLDERS until measured on the hardware; the URDF
# and SDF are checked against these by tools/robot/frames.py --check.
BASE_YAW_LIMIT_DEG = 175.0        # [ESTIMATE] symmetric +/- range; j1_coupler (carrying the drive's stator) turns on the base
SHOULDER_PITCH_LIMIT_DEG = 120.0  # [ESTIMATE] the 20:1 cycloidal drive (CYCLOIDAL_RATIO) between j1_coupler and j1_link
ELBOW_PITCH_LIMIT_DEG = 120.0     # [ESTIMATE] GT2 belt at j2_link
WRIST_PITCH_LIMIT_DEG = 120.0     # [ESTIMATE] GT2 belt at wrist_link
WRIST_ROLL_LIMIT_DEG = 180.0      # [ESTIMATE] NEMA17 pancake wrist roll (not CAN-driven yet)
JAW_TRAVEL_MM = 10.0            # [ESTIMATE] symmetric +/- jaw travel about the capture pose
ARM_JOINT_EFFORT_NM = 5.0       # [ESTIMATE] MKS SERVO42D through the reductions
ARM_JOINT_VELOCITY_RAD_S = 1.0  # [ESTIMATE]
WRIST_EFFORT_NM = 1.0           # [ESTIMATE]
WRIST_VELOCITY_RAD_S = 2.0      # [ESTIMATE]
JAW_EFFORT_N = 20.0             # [ESTIMATE] MG996R through the crank linkage
JAW_VELOCITY_M_S = 0.05         # [ESTIMATE]

# --- Joint stack -----------------------------------------------------------------------
# TODO: add J1/J2/J3 stack dimensions (bearing seats, link lengths, bolt patterns) as the
# joint parts are converted; measure them with
#   ./cadtool inspect refs reference/solidworks/<name>.step --facts --planes --positioning
