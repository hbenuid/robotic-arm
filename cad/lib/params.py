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
keeps its SolidWorks part-file frame until it is converted (see AGENTS.md). The URDF's REP-103
base frame (Z up, X forward) is lib/datum.py base_frame() (robot/frames.py builds the kinematics on
it); assemblies/arm.py EMITS the arm in that frame (arm_from_w() - cadgen's viewer and snapshots are
Z-up), the placements themselves stay in the capture frame. `lib/` never imports `parts/`.
"""
import math

# --- Units / print globals ----------------------------------------------------
# IN and NUDGE are defined in lib/units.py (a leaf module: lib/cycloidal/ needs NUDGE and this
# module imports lib/cycloidal/params.py below - it must not import back up) and re-exported here.
from lib.units import IN, NUDGE  # noqa: F401

PETG_DENSITY = 1.27e-3  # [DESIGN] g/mm^3 - printed-part mass estimates

# --- Fasteners - lib/fasteners.py (a leaf: the sizes, the clearance holes, the plain screw / nut builders) ----
from lib.fasteners import (  # noqa: E402, F401
    M3_CLEAR, M3_CSK, M3_NUT, M3_PITCH, M3_SHCS, M4_CLEAR, M4_NUT, M4_PITCH, M4_SHCS, M5_CLEAR, csk_volume, nut_volume, shcs_volume,
)

# --- Belt drive (GT2) - lib/belts.py (a leaf: lib/forearm/ imports it directly) ------------------
from lib.belts import (  # noqa: E402, F401
    GT2_BELT_W, GT2_BLEND_R, GT2_FLANK_ANGLE, GT2_FLANK_R, GT2_GROOVE_R, GT2_IDLER_BORE, GT2_IDLER_CHANNEL_W,
    GT2_IDLER_FLANGE_DIA, GT2_IDLER_SEAT_DIA, GT2_IDLER_WIDTH, GT2_PITCH, GT2_PLD, GT2_PULLEY_20T_PITCH_DIA,
    GT2_PULLEY_20_60T_TEETH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_BOLT_R, GT2_PULLEY_90T_FACE_Y, GT2_PULLEY_90T_HEAD_SEAT, GT2_PULLEY_90T_PITCH_DIA,
    GT2_PULLEY_90T_TEETH, GT2_PULLEY_120T_TEETH, GT2_RATIO, GT2_TIP_R, GT2_TOOTH_DEPTH, STANDARD_2GT_LENGTHS, flank_offset, pulley_90t_bolt_points,
    pulley_od,
)
# --- The belt joints' bearings + the base_yaw thrust bearing - lib/bearings.py (a leaf: lib/base/, lib/coupler/, lib/forearm/, lib/yaw_coupler/ import it)
from lib.bearings import (  # noqa: E402, F401
    BEARING_6806_BORE, BEARING_6806_MASS_G, BEARING_6806_OD, BEARING_6806_SHOULDER_DIA, BEARING_6806_WIDTH,
    PULLEY_SEAT_SHIFT, THRUST_BORE, THRUST_CAGE_MASS_G, THRUST_CAGE_WIDTH, THRUST_OD, THRUST_STACK,
    THRUST_WASHER_MASS_G, THRUST_WASHER_WIDTH,
)

# --- Gripper hardware ---------------------------------------------------------
RAIL_DIA = 6.0          # [REFERENCE] round linear rail (parts/gripper/gripper_rail_6mm), used x2
RAIL_LEN = 125.0        # [REFERENCE] reference geometry length

# MG996R standard servo (parts/gripper/mg996r_servo). [DATASHEET] TowerPro MG996R; verify on the unit in hand.
MG996R_BODY_L = 40.7            # [DATASHEET] body length (along the mounting tabs)
MG996R_BODY_W = 19.7            # [DATASHEET] body width
MG996R_BODY_H = 42.9            # [DATASHEET] body height incl. output boss, excl. horn
MG996R_TAB_L = 54.5             # [DATASHEET] overall length over the mounting tabs
MG996R_MOUNT_HOLE_SP = 49.5     # [DATASHEET] mounting-hole spacing along the tabs
MG996R_MOUNT_HOLE_SP_W = 10.0   # [DATASHEET] mounting-hole spacing across the tabs
MG996R_MASS_G = 55.0            # [DATASHEET]

# --- Motors - lib/motors.py (a leaf: the NEMA 17 interface, the pancake, the 40 mm kit motor + its MKS
# SERVO42D board; lib/forearm/ imports it directly) -----------------------------------------------------
from lib.motors import (  # noqa: E402, F401
    MKS_SERVO42D_BOARD_STACK, MKS_SERVO42D_MASS_G, MKS_SERVO42D_SCREW_REACH, MKS_SERVO42D_STACK, MKS_SERVO42D_STANDOFF,
    MKS_SERVO42D_W, MOTOR_40, NEMA17_40_BODY_LEN, NEMA17_40_BODY_W, NEMA17_40_CONNECTOR_D, NEMA17_40_CONNECTOR_W,
    NEMA17_40_CONNECTOR_Z0, NEMA17_40_CONNECTOR_Z1, NEMA17_40_MASS_G, NEMA17_40_REAR_STUB_DIA, NEMA17_40_REAR_STUB_LEN,
    NEMA17_BOLT_SP, NEMA17_FACE, NEMA17_PILOT_DIA, NEMA17_SHAFT_DIA, PANCAKE_BODY_D, PANCAKE_BODY_H, PANCAKE_BODY_W,
    PANCAKE_MASS_G,
)

# --- Sensors - lib/sensors.py (a leaf: the KY-003 hall module, the joints' endstop / home sensor) -----------------
from lib.sensors import (  # noqa: E402, F401
    A3144_BODY_H, A3144_BODY_T, A3144_BODY_W, A3144_HALL_DEPTH, A3144_LEAD_PITCH, A3144_LEAD_T, A3144_LEAD_W,
    KY003_BOARD_L, KY003_BOARD_T, KY003_BOARD_W, KY003_HEADER_H, KY003_HOLE_DIA, KY003_HOLE_INSET, KY003_LEAD_BEND_X,
    KY003_LEAD_OUT, KY003_LEAD_TAIL, KY003_MASS_G, KY003_PIN_OUT, KY003_PIN_PITCH, KY003_PIN_TAIL, KY003_PIN_W,
    KY003_PINS, ky003_hall_point,
)

# The mounts (host-part frames, mm): where the pads sit in the SolidWorks links. base_yaw takes the 48 mm motor
# (parts/cycloidal/nema17_48mm, the drive's), elbow_pitch / wrist_pitch the 40 mm one. The base is parametric
# (lib/base/params.py BaseConfig - a leaf like the links'): its motor seat - on the base's bolt-on motor mount
# (parts/base/base_motor_mount), in the base's part frame - comes from its DEFAULT configuration.
from lib.base.params import BOARD_CLEAR as _BOARD_CLEAR  # noqa: E402
from lib.base.params import DEFAULT as _BASE  # noqa: E402

BASE_MOTOR_PATTERN_CENTRE = (_BASE.motor.centre[0], _BASE.plate.y[0], _BASE.motor.centre[1])   # (81.972, -44.9, 0.084) [DESIGN] the motor mount: 4x M3 on 31 x 31 through the 5 mm plate (slots of +/- travel along X), on its -Y face; x where the 280-2GT belt puts it
BASE_MOTOR_TABLE_CLEAR = _BOARD_CLEAR   # [DESIGN] 5.0 - the 48 mm motor + board (48 + 14.1 under the plate) end this far
#                                          ABOVE the base's bottom face (lib/base/params.py: the face sits under them)
BASE_YAW_RATIO = GT2_PULLEY_120T_TEETH / GT2_PULLEY_20T_TEETH   # 6:1 [DESIGN] the base_yaw belt: the 120T (parts/base/gt2_pulley_120t) on the 48 mm motor's 20T
# j1_link and j2_link are parametric (lib/upper_arm/params.py UpperArmConfig, lib/forearm/params.py ForearmConfig -
# leaves like lib/cycloidal/params.py): the motors' pad faces and j2_link's slide come from their DEFAULT configurations.
from lib.upper_arm.layout import arm_slide as _arm_slide  # noqa: E402
from lib.upper_arm.params import DEFAULT as _UPPER_ARM  # noqa: E402
from lib.upper_arm.params import ELBOW_BELT as _ELBOW_BELT  # noqa: E402

J1_ARM_SLIDE = _arm_slide(_UPPER_ARM)              # 39.27 [DESIGN] j1_link: the plate - its elbow end, the arm's slab - slid along +Y (N) (ArmParams)
J1_MOTOR_PAD_FACE_Y = _UPPER_ARM.arm.y_outer       # 27.5 [DESIGN] j1_link: the elbow motor's face, on the web across its hole through the slab (the
#                                                    -N side; ArmParams), the motor on +N down the hole, its shaft -N; the holes about the pad's axis
J1_MOTOR_SCREW_THREAD = 4.0                        # [DESIGN] the elbow motor's 4x M3 (parts/joints/elbow_motor_screws) this far into its
#                                                    tapped holes - the drive's motor bolts' (bolt_hole_depth - motor_bolt_thread_margin) ...
J1_MOTOR_SCREW_LEN = _UPPER_ARM.arm.motor_plate_t + J1_MOTOR_SCREW_THREAD   # 10 ... under the head, through the web (ArmParams.motor_plate_t)
J1_MOTOR_20T_HUB_Z = 8.978                         # [DESIGN] the elbow motor's 20T (lib/mounts.py gt2_pulley_20t#2): its hub face this far out
#                                                    along the shaft from the motor's face - through the web's pilot hole, level with its
#                                                    underside -, its tooth band
#                                                    (RollDriveParams.t20_hub on) level with the elbow 90T's (tests/test_mounts.py)
ELBOW_BELT_LENGTH = _ELBOW_BELT                    # 280-2GT [DESIGN] the elbow belt (20T on j1_link's motor, 90T at the elbow); it sets the pad's x
from lib.forearm.params import DEFAULT as _FOREARM  # noqa: E402

J2_MOTOR_WEB_FACE_Z = _FOREARM.web.z1              # 19 [REFERENCE] j2_link: the web's +Z face; the motor bolts through its slots
J2_MOTOR_SLIDE_RANGE = _FOREARM.slide_range        # (-103.5, -94) [DESIGN] motor-axis x range the (shortened) slots allow
J2_MOTOR_SLIDE_X = _FOREARM.motor_x                # -99.52: set by the stock WRIST_BELT_LENGTH (lib/belts.py centre_distance) [ESTIMATE]
WRIST_BELT_LENGTH = _FOREARM.roll_end.wrist_belt   # 258-2GT [ESTIMATE] the wrist-pitch belt (90T at the wrist, 20T on the motor)
# The forearm roll (lib/forearm/params.py RollEndParams; the rotor's wall on j2_link, the elbow block + shaft in M3):
FOREARM_ROLL_AXIS_Z = _FOREARM.roll_end.axis_z     # 25 [REFERENCE] the roll axis' N-station in j2_link's frame = the wrist centre's (42 - 17)
FOREARM_WALL_X = _FOREARM.roll_end.wall_x          # (-56, -48) [DESIGN] the flange wall: wrist face .. elbow face (48 from the elbow axis)
FOREARM_WALL_OD = _FOREARM.roll_end.wall_od        # 60 [DESIGN] the flange wall's round outline about the roll axis (on a foot as wide as the web's neck)
FOREARM_PLUG_CLEARANCE = _FOREARM.roll_end.plug_clearance   # 10 [DESIGN] the wrist motor's connector plug to the wall
FOREARM_FLANGE_DIA = _FOREARM.roll_end.flange_dia  # 39.7 [DESIGN] the roll shaft's end spigot the forearm wall bolts onto
FOREARM_ROLL_BELT_LENGTH = _FOREARM.drive.roll_belt          # 240-2GT [ESTIMATE] the roll belt (90T ring on the shaft, 20T on the motor)
FOREARM_ROLL_MOTOR_XY = (0.0, _FOREARM.drive.motor_y)        # (0, -60.9) [DESIGN] the roll motor's axis in the module frame: on the elbow axis, under the housing by what the belt sets
ELBOW_ROLL_OFFSET = _FOREARM.elbow_offset                    # 60.9 [DESIGN] the roll axis' distance from the elbow axis (along j2_link's +Y): the roll belt's centre
#                                                              distance, the roll motor on the elbow axis - the arm's elbow offset (lib/placements.py SHIFTS)
FOREARM_ROLL_RATIO = _FOREARM.drive.ring_teeth / GT2_PULLEY_20T_TEETH   # 4.5:1 [DESIGN] like the other belt joints
FOREARM_ROLL_BLOCK_X = _FOREARM.drive.block_x       # (-33, 33) [DESIGN] the roll frame's extent along N: its underside 3.0 mm above the upper arm's flat top (host z -11)

# --- Cycloidal drive (lib/cycloidal/, assemblies/cycloidal_drive.py, docs/cycloidal_drive.md) -----
# The drive's own dimensions live in lib/cycloidal/params.py (DriveConfig, ported from the
# cycloidal_drive repo). These are the interface values the rest of the arm needs, re-exported
# from that config so every number exists exactly once. Drive frame: Z = motor axis, z=0 = the
# motor-plate outer face, +Z toward the output hub; it sits in the arm at placements.json
# "cycloidal_drive#1" and IS the robot's shoulder_pitch joint. Its shell turns (ShellParams): the j1_coupler yoke holds
# the hub and the motor plate's sleeve (robot/frames.py: shoulder_link), j1_link rises off the shell's middle, printed
# with its body (upper_arm_link).
from lib.cycloidal.layout import arm_zone as _arm_zone  # noqa: E402
from lib.cycloidal.layout import shell_ends as _shell_ends  # noqa: E402
from lib.cycloidal.layout import sleeve_end as _sleeve_end  # noqa: E402
from lib.cycloidal.params import DEFAULT_CONFIG as _DRIVE  # noqa: E402

CYCLOIDAL_RATIO = _DRIVE.ratio                                    # 21:1 [DESIGN] the 21 ring pins, the carrier held; the output (the shell) turns with the motor.
#   NOTE: software/control/src/config.py JOINTS still carries gear_ratio 1.0 on J1..J3 - which MKS motor drives the
#   shoulder_pitch joint (if any of them) is unconfirmed.
CYCLOIDAL_HOUSING_OD = _DRIVE.housing.od                          # 129.2 [DESIGN] the pillar tips; lib/cycloidal/params.py RING_INSET, LUG_WALL
CYCLOIDAL_STACK_DEPTH = _DRIVE.stack_up.total_housing_depth       # 61 [DESIGN] motor-plate outer face -> the shell's hub end
CYCLOIDAL_SHELL_ENDS_Z = _shell_ends(_DRIVE)                      # (-13, 61) [DESIGN] the turning shell's two ends: a lip past each 6814
CYCLOIDAL_ARM_ZONE_Z = _arm_zone(_DRIVE)                          # (9, 39) [DESIGN] where j1_link rises off the shell: between the two plates
CYCLOIDAL_MOTOR_BODY_LEN = _DRIVE.motor.body_length               # 48 [DATASHEET] NEMA 17 body behind the plate (-Z)
CYCLOIDAL_HUB_OD = _DRIVE.output_hub.od                           # 70.3 [DESIGN]
CYCLOIDAL_HUB_PROUD = _DRIVE.output_hub.proud_above_housing       # 1 [DESIGN] hub face past the shell's hub end: on the yoke's leg
CYCLOIDAL_HUB_FACE_Z = CYCLOIDAL_STACK_DEPTH + CYCLOIDAL_HUB_PROUD   # 62 [DESIGN] the hub's face on the yoke's hub-side leg
CYCLOIDAL_SLEEVE_END_Z = _sleeve_end(_DRIVE)                      # -22 [DESIGN] the motor plate's sleeve's end on the yoke's motor-side leg
CYCLOIDAL_HUB_BOLT_CIRCLE = _DRIVE.output_hub.arm_mount_bolt_circle_dia       # 50 [DESIGN] the hub's bolts into the yoke's leg ...
CYCLOIDAL_HUB_BOLT_COUNT = _DRIVE.output_hub.arm_mount_bolt_count             # ... 4x M4 into captive nuts in the hub
CYCLOIDAL_HUB_BOLT_ANGLE_OFFSET_DEG = _DRIVE.output_hub.arm_mount_angle_offset_deg # 45 (between the output pins)
CYCLOIDAL_HUB_BOLT_DIA = _DRIVE.housing.bolt_dia                  # 4 (M4)
CYCLOIDAL_SHELL_BOLT_CIRCLE = _DRIVE.housing.bolt_circle_dia      # 117 [DESIGN] the housing bolts, through the shell end to end ...
CYCLOIDAL_SHELL_BOLT_COUNT = _DRIVE.housing.bolt_count            # ... 6x M4, their nuts in its body (j1_link's)

# Purchased parts of the drive (parts/cycloidal/bearing_*.py, nema17_48mm, cycloidal_*_pins/bolts/nuts).
STEEL_DENSITY = 7.85e-3          # [DATASHEET] g/mm^3 - dowel pins, bolts, nuts
CYCLOIDAL_MOTOR_MASS_G = 400.0   # [DATASHEET] 48 mm-body NEMA 17 (17HS19-2004S1 class); verify on the unit in hand
BEARING_6003_MASS_G = 39.0       # [DATASHEET] 6003-2RS 17x35x10
BEARING_6814_MASS_G = 110.0      # [DATASHEET] 6814-2RS (61814) 70x90x10; verify
BEARING_625_MASS_G = 5.0         # [DATASHEET] 625-2RS 5x16x5
BEARING_6808_MASS_G = 33.0       # [DATASHEET] 6808-2RS (61808) 40x52x7 - the forearm roll's ring bearings (parts/joints/bearing_6808); verify


def _cyl_vol(radius, height):
    return math.pi * radius * radius * height


def _hex_vol(across_flats, height):
    r = across_flats / math.cos(math.radians(30)) / 2.0      # circumradius
    return 1.5 * math.sqrt(3) * r * r * height


# [ESTIMATE] simplified-geometry volumes x steel density (the parts are plain cylinders / hex prisms)
CYCLOIDAL_RING_PINS_MASS_G = STEEL_DENSITY * _DRIVE.gear.num_ring_pins * _cyl_vol(_DRIVE.gear.ring_pin_radius, _DRIVE.gear.ring_pin_length)            # 82.9, 21x 4 x 40
CYCLOIDAL_OUTPUT_PINS_MASS_G = STEEL_DENSITY * _DRIVE.disc.output_pin_count * _cyl_vol(_DRIVE.disc.output_pin_dia / 2, _DRIVE.disc.output_pin_length)   # 17.8, 4x
CYCLOIDAL_SUPPORT_PIN_MASS_G = STEEL_DENSITY * _cyl_vol(_DRIVE.shaft.support_pin_dia / 2, _DRIVE.shaft.support_pin_length)                             # 3.1
CYCLOIDAL_HOUSING_BOLTS_MASS_G = STEEL_DENSITY * _DRIVE.housing.bolt_count * (
    _cyl_vol(_DRIVE.housing.bolt_head_dia / 2, _DRIVE.housing.bolt_head_height) + _cyl_vol(_DRIVE.housing.bolt_dia / 2, _DRIVE.housing.bolt_length))    # 45.7, 6x M4x65
CYCLOIDAL_HOUSING_NUTS_MASS_G = STEEL_DENSITY * _DRIVE.housing.bolt_count * _hex_vol(_DRIVE.housing.bolt_nut_af, _DRIVE.housing.bolt_nut_thickness)     # 6.4, 6x M4
CYCLOIDAL_MOTOR_BOLTS_MASS_G = STEEL_DENSITY * 4 * (
    _cyl_vol(_DRIVE.motor.bolt_dia / 2, _DRIVE.motor.motor_bolt_thread_length) + _cyl_vol(_DRIVE.motor.motor_bolt_head_dia / 2, _DRIVE.motor.motor_bolt_head_height))   # 4.3, 4x M3x10

# The pulley bolts of the elbow's and the wrist's 90T and the base_yaw 120T (parts/joints/{elbow,wrist}_pulley_{screws,nuts},
# parts/base/yaw_pulley_{screws,nuts}): the same estimate over their modelled geometry (lib/fasteners.py: the socket and
# the nut's bore taken out)
from lib.coupler.params import DEFAULT as _COUPLER  # noqa: E402
from lib.yaw_coupler.params import DEFAULT as _YAW_COUPLER  # noqa: E402

_PULLEY_BOLTS = len(pulley_90t_bolt_points())
ELBOW_PULLEY_SCREWS_MASS_G = STEEL_DENSITY * _PULLEY_BOLTS * shcs_volume(M4_SHCS, _FOREARM.drive.pulley_screw_len)   # 18.1, 4x M4x35
WRIST_PULLEY_SCREWS_MASS_G = STEEL_DENSITY * _PULLEY_BOLTS * shcs_volume(M4_SHCS, _COUPLER.pulley_screw_len)         # 22.1, 4x M4x45
YAW_PULLEY_SCREWS_MASS_G = STEEL_DENSITY * _PULLEY_BOLTS * shcs_volume(M4_SHCS, _YAW_COUPLER.hub.pulley_screw_len)   # 20.1, 4x M4x40
PULLEY_NUTS_MASS_G = STEEL_DENSITY * _PULLEY_BOLTS * nut_volume(M4_NUT)                                              # 3.0, 4x M4 (each joint)

# The base motor mount's screws and nuts (parts/base/base_motor_mount_{screws,nuts}): the same estimate
from lib.base.layout import joint_bolt_points as _joint_bolt_points  # noqa: E402

_JOINT_BOLTS = len(_joint_bolt_points(_BASE))
BASE_MOUNT_SCREWS_MASS_G = STEEL_DENSITY * _JOINT_BOLTS * shcs_volume(_BASE.joint.screw, _BASE.joint.screw_len)   # 12.2, 4x M4x20
BASE_MOUNT_NUTS_MASS_G = STEEL_DENSITY * _JOINT_BOLTS * nut_volume(_BASE.joint.nut)                              # 3.0, 4x M4

# The elbow motor's screws (parts/joints/elbow_motor_screws): the same estimate
from lib.upper_arm.layout import pad_holes as _pad_holes  # noqa: E402

ELBOW_MOTOR_SCREWS_MASS_G = STEEL_DENSITY * len(_pad_holes(_UPPER_ARM)) * shcs_volume(_UPPER_ARM.arm.motor_screw, J1_MOTOR_SCREW_LEN)   # 4.2, 4x M3x10

# --- Robot description (robot/frames.py, robot/arm.urdf) --------------------------------------
# Joint limits and actuator ratings are PLACEHOLDERS until measured on the hardware; the URDF
# and SDF are checked against these by tools/robot/derive.py --check.
BASE_YAW_LIMIT_DEG = 175.0        # [ESTIMATE] symmetric +/- range; j1_coupler (carrying the drive's stator) turns on the base
# the 21:1 cycloidal drive (CYCLOIDAL_RATIO) between j1_coupler and j1_link: (lower, upper); the upper - the arm pitched
# down in front - stops 3 deg before the upper arm, rising off the drive's middle, reaches j1_coupler's disc (70.5 deg;
# its 1 mm running gap held to 69.5 - tests/test_sweeps.py), the lower is an [ESTIMATE] (the arm clears to -150)
SHOULDER_PITCH_LIMITS_DEG = (-120.0, 67.0)   # [DESIGN]
# the 4.5:1 elbow belt: (lower, upper); the lower - the forearm lifted back - stops 3 deg before the forearm roll drive's
# housing (its rear end, ELBOW_ROLL_OFFSET over the elbow axis) comes within 1 mm of the elbow motor, which stands on the
# same side of the upper arm (-50.0 deg - tests/forearm/test_roll_drive.py; the roll motor, on the elbow axis, never
# reaches it); the upper kept at 90 (docs/open_issues.md): SHOULDER_PITCH_LIMITS_DEG was set over this range
ELBOW_PITCH_LIMITS_DEG = (-47.0, 90.0)   # [DESIGN]
# the GT2 belt at wrist_link: (lower, upper), the clear range (the wrist body's back corners reach j2_link's web at
# -109 / +74.5 at any roll, tests/test_sweeps.py) less 4 deg
WRIST_PITCH_LIMITS_DEG = (-105.0, 70.0)   # [DESIGN]
FOREARM_ROLL_LIMIT_DEG = 170.0    # [ESTIMATE] the forearm roll (GT2 belt in the elbow block): a hard stop keeps the cables from winding
WRIST_ROLL_LIMIT_DEG = 180.0      # [ESTIMATE] NEMA17 pancake wrist roll (not CAN-driven yet)
JAW_TRAVEL_MM = 10.0            # [ESTIMATE] symmetric +/- jaw travel about the capture pose
ARM_JOINT_EFFORT_NM = 5.0       # [ESTIMATE] MKS SERVO42D through the reductions
ARM_JOINT_VELOCITY_RAD_S = 1.0  # [ESTIMATE]
WRIST_EFFORT_NM = 1.0           # [ESTIMATE]
WRIST_VELOCITY_RAD_S = 2.0      # [ESTIMATE]
JAW_EFFORT_N = 20.0             # [ESTIMATE] MG996R through the crank linkage
JAW_VELOCITY_M_S = 0.05         # [ESTIMATE]
