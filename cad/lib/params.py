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
base frame (Z up, X forward) is defined in robot/frames.py. `lib/` never imports `parts/`.
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

# --- Cycloidal drive (NOT modelled here; source lives in the cycloidal_drive repo) ----------
# Its world pose in the SolidWorks arm assembly is recorded under "skipped" in
# reference/placements.json for re-attaching it later.

# --- Joint stack -----------------------------------------------------------------------
# TODO: add J1/J2/J3 stack dimensions (bearing seats, link lengths, bolt patterns) as the
# joint parts are converted; measure them with
#   ./cadtool inspect refs reference/<name>.step --facts --planes --positioning
