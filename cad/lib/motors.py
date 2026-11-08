"""The arm's motors, their driver boards and the elbow motor's gearbox - a leaf module (imports nothing above
lib/units.py and lib/cycloidal/params.py).

lib/params.py re-exports every name here (`from lib.params import NEMA17_40_BODY_LEN` keeps working);
lib/forearm/ imports them from HERE, because lib/params.py imports lib/forearm/params.py for its
interface values and a module below lib/params.py must never import back up into it.

Tags as in lib/params.py: [MEASURE] [DATASHEET] [DESIGN] [REFERENCE] [ESTIMATE]. Units mm / g.
"""
from dataclasses import dataclass, replace

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
# The board's three plug headers (vendor/mks_servo42d.step's three connector bodies), in the board's frame: per header
# its box (x0, y0, z0, x1, y1, z1) and the way its plug goes in, (x, y) - the two on the board's sides and the motor
# cable's on -Y, the motor's own connector side. The plugs stay in as the arm moves: each needs MKS_PLUG_ROOM of
# straight room out from its header's face, its outline wide, at every pose (tests/test_sweeps.py).
MKS_SERVO42D_HEADERS = {        # [REFERENCE]
    "side_neg_x": ((-20.5, -8.0, -12.9, -14.5, 5.0, -4.6), (-1, 0)),
    "side_pos_x": ((14.5, -8.0, -12.9, 20.5, 8.0, -4.6), (1, 0)),
    "motor": ((-5.75, -20.5, -12.9, 5.75, -14.5, -4.6), (0, -1)),
}
MKS_PLUG_ROOM = {"side_neg_x": 10.0, "side_pos_x": 10.0, "motor": 5.0}   # [MEASURE] each plug and its cable's bend

# The drive motor's parameters (lib/cycloidal/params.py MotorParams: 31 mm bolt square, Ø22 x 2 pilot,
# Ø5 x 22 D-shaft - every kit motor in the arm carries that interface) with the 40 mm body.
MOTOR_40 = replace(_DRIVE.motor, body_width=NEMA17_40_BODY_W, body_length=NEMA17_40_BODY_LEN)

# --- The elbow motor's planetary gearbox: StepperOnline EG17-G5 (parts/joints/eg17_g5_gearbox) --------------------
# A 5:1 planetary for a NEMA 17, bought for the elbow_pitch motor and bolted between it and j1_link's web
# (lib/mounts.py). From its datasheet drawing: [DATASHEET] the dimensioned values, [ESTIMATE] the few read off it at
# scale. Part frame = the motor's: the output face z=0 (where the motor's face would be), the body -Z, the output
# shaft +Z, the 4x M4 on the X and Y axes, the keyway on +X, the clamp's access holes in the X faces.
@dataclass(frozen=True)
class GearboxParams:
    ratio: float = 5.0               # [DATASHEET] motor turns per output turn
    length: float = 56.0             # [DATASHEET] output face -> input face
    square: float = 42.0             # [DATASHEET] the input end's square; the round output flange and band as wide
    flange_t: float = 9.8            # [ESTIMATE] the round output flange ...
    band_t: float = 17.9             # [ESTIMATE] ... and the round band behind it; the square input end the rest
    corner_ch: float = 4.0           # [ESTIMATE] the input end's clipped corners (45 degrees)
    channel: float = 12.5            # [ESTIMATE] the corner channels down to the input plate, the corners cut back to this
    #                                  off the axis both ways: the drawing's 12.8 less what clears the motor screws' heads
    #                                  (M3, Ø5.5 on the 31 square: 12.75) - drawn there as a channel ending in a ramp
    plate_t: float = 4.7             # [DATASHEET] the input plate the channels stop at ...
    plate_sp: float = NEMA17_BOLT_SP # [DATASHEET] ... its 4 holes on the NEMA 17 square ...
    plate_hole: float = 3.5          # [DATASHEET] ... Ø3.5 (the motor's M3 screws, from the front)
    bore_dia: float = 22.0           # [DATASHEET] the input bore (Ø22H7: the motor's pilot) ...
    bore_depth: float = 25.5         # [DATASHEET] ... this deep from the input face
    collar_dia: float = 18.5         # [ESTIMATE] the shaft clamp's collar in it ...
    collar_from: float = 7.7         # [ESTIMATE] ... from this deep in, ...
    input_dia: float = NEMA17_SHAFT_DIA   # [DATASHEET] ... bored for the motor's Ø5 shaft (24 long at most)
    access_dia: float = 5.8          # [ESTIMATE] the clamp's two access holes, one in each X face, ...
    access_cb_dia: float = 7.0       # [ESTIMATE] ... counterbored at the face (the plugs' seats) ...
    access_cb_depth: float = 2.6     # [ESTIMATE]
    access_from_input: float = 21.4  # [ESTIMATE] ... this far from the input face, ...
    access_off: float = 4.5          # [ESTIMATE] ... this far off the middle: +Y in the +X face, -Y in the -X face
    boss_dia: float = 25.0           # [DATASHEET] the output face's boss (Ø25h7) ...
    boss_t: float = 2.0              # [DATASHEET]
    shoulder_dia: float = 10.0       # [DATASHEET] the shaft's shoulder on it ...
    shoulder_t: float = 1.5          # [ESTIMATE] ... (21.5 less the shaft's 18 and the boss)
    shaft_dia: float = 8.0           # [DATASHEET] the output shaft (Ø8h7), ...
    shaft_end: float = 21.5          # [DATASHEET] ... its end this far from the output face (+/- 0.5)
    key_w: float = 3.0               # [DATASHEET] its key: 3 wide, ...
    key_l: float = 14.0              # [DATASHEET] ... 14 long, ...
    key_from_end: float = 2.0        # [DATASHEET] ... from 2 short of the shaft's end, ...
    key_over: float = 9.2            # [DATASHEET] ... its top this far over the shaft's far side, ...
    key_h: float = 3.0               # [ESTIMATE] ... 3 high (DIN 6885's 3 x 3 for an 8 shaft): the keyway is key_h less
    #                                  what the key stands proud (key_over - shaft_dia) deep - the key is left out, a set
    #                                  screw of the 20T in its keyway (parts/joints/gt2_pulley_20t_8mm)
    end_tap_dia: float = 2.5         # [DATASHEET] M3 x 10 in the shaft's end (the tap drill)
    end_tap_depth: float = 10.0      # [DATASHEET]
    bolt_pcd: float = 35.0           # [DATASHEET] the output face's 4x M4 x 8, on Ø35, on the axes (the drawing's front view)
    bolt_hole_dia: float = 4.0       # [DESIGN] modelled at the M4's major diameter (the 40 mm motor's tapped holes too): the
    #                                  screws' plain shanks in them without overlap - the real holes are tapped
    bolt_depth: float = 8.0          # [DATASHEET]

    def keyway_depth(self) -> float:
        return self.key_h - (self.key_over - self.shaft_dia)   # 1.8


EG17_G5 = GearboxParams()
EG17_G5_MASS_G = 422.0          # [DATASHEET] the listing's 14.9 oz
