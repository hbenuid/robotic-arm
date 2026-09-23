"""The arm's motors and their driver boards - a leaf module (imports nothing above lib/units.py and
lib/cycloidal/params.py).

lib/params.py re-exports every name here (`from lib.params import NEMA17_40_BODY_LEN` keeps working);
lib/forearm/ imports them from HERE, because lib/params.py imports lib/forearm/params.py for its
interface values and a module below lib/params.py must never import back up into it.

Tags as in lib/params.py: [MEASURE] [DATASHEET] [DESIGN] [REFERENCE] [ESTIMATE]. Units mm / g.
"""
from dataclasses import replace

from lib.cycloidal.params import DEFAULT_CONFIG as _DRIVE

# --- NEMA 17 (datasheet interface, shared by every motor of the arm) ----------------------------
NEMA17_FACE = 42.3              # [DATASHEET] NEMA 17 square face
NEMA17_BOLT_SP = 31.0           # [DATASHEET] mounting-hole square pattern
NEMA17_PILOT_DIA = 22.0         # [DATASHEET] locating boss
NEMA17_SHAFT_DIA = 5.0          # [DATASHEET]

# --- NEMA 17 pancake stepper (parts/wrist/nema17_pancake) - the wrist_roll motor ---------------------
# The SolidWorks model is a 7-part sub-assembly of the motor's internals, flattened into
# one vendor STEP (vendor/nema17_pancake.step). Envelope from its reference bounding box.
PANCAKE_BODY_W = 41.5           # [REFERENCE] body width  (reference bbox 41.5 x 47.0 x 43.0 incl. connector + shaft)
PANCAKE_BODY_D = 47.0           # [REFERENCE] body depth incl. connector
PANCAKE_BODY_H = 43.0           # [REFERENCE] height incl. shaft
PANCAKE_MASS_G = 180.0          # [ESTIMATE] typical 17HS08-type pancake 150-200 g - replace with the datasheet value

# --- Belt-drive motors: NEMA 17 x 40 mm + MKS SERVO42D (parts/joints/nema17_40mm, mks_servo42d) ----
# The elbow_pitch / wrist_pitch (and, once the forearm roll exists, forearm_roll) motors of the arm: the
# "nema17x40_with_mks" SolidWorks export (a kit: motor + driver board, split by
# tools/reference/split_mks_motor.py into vendor/nema17_40mm.step and vendor/mks_servo42d.step). Part frame
# like the drive motor's (mounting face z=0, body -Z, shaft +Z, D-flat +Y); the board's frame has z=0 at
# the motor's REAR face, its stack in -Z. base_yaw takes the 48 mm kit motor (parts/cycloidal/nema17_48mm).
NEMA17_40_BODY_W = 42.0         # [REFERENCE] the export's body square (datasheet NEMA17_FACE 42.3)
NEMA17_40_BODY_LEN = 39.5       # [REFERENCE] mounting face -> rear face ("40 mm" class)
NEMA17_40_REAR_STUB_DIA = 8.0   # [REFERENCE] rear bearing boss proud of the rear face
NEMA17_40_REAR_STUB_LEN = 0.9   # [REFERENCE]
NEMA17_40_CONNECTOR_W = 16.0    # [REFERENCE] cable-connector boss on the -Y side, 7 mm proud of the body ...
NEMA17_40_CONNECTOR_D = 7.0     # [REFERENCE]
NEMA17_40_CONNECTOR_Z0 = -39.5  # [REFERENCE] ... spanning z -39.5..-29.9 (the rear 9.6 mm of the body)
NEMA17_40_CONNECTOR_Z1 = -29.9  # [REFERENCE]
NEMA17_40_MASS_G = 280.0        # [ESTIMATE] 17HS4401 / 17HS15 class 40 mm NEMA 17 (bare motor); verify on the unit
MKS_SERVO42D_STANDOFF = 3.0     # [REFERENCE] 4x 7x3 standoffs between the rear face and the board
MKS_SERVO42D_BOARD_STACK = 11.1 # [REFERENCE] PCB + cover
MKS_SERVO42D_STACK = MKS_SERVO42D_STANDOFF + MKS_SERVO42D_BOARD_STACK   # 14.1 rear face -> cover face
MKS_SERVO42D_W = 43.0           # [REFERENCE] cover square
MKS_SERVO42D_SCREW_REACH = 19.6 # [REFERENCE] the 4x M3x30 reach this far into the motor's through-holes (+Z of the rear face)
MKS_SERVO42D_MASS_G = 35.0      # [ESTIMATE] PCB + cover + 4 screws + 4 standoffs; verify on the unit

# The drive motor's parameters (lib/cycloidal/params.py MotorParams: 31 mm bolt square, Ø22 x 2 pilot,
# Ø5 x 22 D-shaft - every motor in the arm carries that interface) with the 40 mm body.
MOTOR_40 = replace(_DRIVE.motor, body_width=NEMA17_40_BODY_W, body_length=NEMA17_40_BODY_LEN)
