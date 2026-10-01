"""Kinematic decomposition of the arm: rigid links and joint frames, derived from the
SolidWorks capture (reference/placements.json) with the design's link-length shifts (lib/placements.py SHIFTS)
applied. Single source for robot/links/*.py, tools/robot/derive.py, tools/robot/export_link_meshes.py and
tests/test_robot.py.

LINKS holds placement keys: part occurrences and designed-module keys - "cycloidal_drive#1", or
one rigid body of it, "cycloidal_drive#1:stator" / ":rotor" (assemblies/cycloidal_drive.py
BODIES) - which assemblies/_occurrences.world_rows expands into world-placed parts.

Chain: base_link -base_yaw-> shoulder_link -shoulder_pitch-> upper_arm_link -elbow_pitch-> elbow_link
-forearm_roll-> forearm_link -wrist_pitch-> wrist_pitch_link -wrist_roll-> wrist_roll_link -jaw_a/jaw_b->
jaw_*_link, + tool0 (frame-only). The cycloidal drive IS the shoulder_pitch joint: its stator
(the held carrier + motor, in the j1_coupler fork) rides in shoulder_link, its rotor (the turning
shell, bolted to j1_link) in upper_arm_link (docs/cycloidal_drive.md "Attachment").

Frames (all in the SolidWorks WORLD frame W, millimetres; W is +Y up, the arm extends toward
-X, see reference/README.md):
  * base_link frame B: REP-103 (Z up, X forward) at the base_yaw axis foot on the base's bottom
    face: origin (0, BASE_BOTTOM_Y, 0), X_B = -X_W, Y_B = +Z_W, Z_B = +Y_W. base_frame(), frame()
    and the U / BASE_FORWARD vectors are defined in lib/datum.py (assemblies/arm.py needs them too
    and assemblies/ never imports robot/) and re-exported here.
  * Every joint frame has Z along the joint axis and X along the child link's long direction
    at the capture pose (projected perpendicular to Z); Y = Z x X.
  * A child link's frame IS its joint frame at the capture pose, so ALL JOINT VALUES ARE 0 AT
    THE CAPTURE POSE and every link mesh (exported in the link frame) has an identity origin.
    Note: in that pose shoulder_link is yawed +3.694 deg about base_yaw relative to the base's
    forward axis (the captured pitch axes are not exactly parallel to Y_B) - this is a POSE,
    not a design offset.
Axis signs (positive-motion direction) and all limits are placeholders [ESTIMATE] until
confirmed by viewer sweeps / on hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from cadgen import build123d as bd

from lib import params as PARAMS
from lib import placements as P
from lib.datum import (  # noqa: F401  (the datum frames live below assemblies/)
    BASE_BOTTOM_Y,
    BASE_FORWARD,
    U,
    base_frame,
    frame,
)

ROBOT_NAME = "arm"

# --- unit vectors of the capture pose (W frame); U (world up) and BASE_FORWARD are lib/datum.py's ---
N = (0.064439, 0.0, 0.997922)                # the three pitch axes (parallel): 90T pulley / J3-coupler bore direction; the cycloidal drive's +Z is -N
F = (-0.865419, 0.497923, 0.055880)          # wrist-roll axis: NEMA17 pancake shaft / 20T pulley bore (toward the tool)
PJ = (0.499699, 0.865884, 0.023354)          # jaw travel: the two Ø6 gripper rails (slider#1 -> slider#2)

BASE_YAW_ORIGIN = (0.0, 85.010435, 0.0)      # [REFERENCE] on the base_yaw axis at the cycloidal drive's axis height (its node's Y)
SHOULDER_ORIGIN = (-2.440595, 85.010435, -34.915297)   # [REFERENCE] j1_link#1 origin: on the cycloidal drive's axis, 66.5 mm along it from the
#                                                        motor-plate face (1.5 past the shell's output face, CYCLOIDAL_OUTPUT_FACE_Z)
# The origins beyond a link move with it: the capture point [REFERENCE] + the SHIFTS of the record it sits on.
ELBOW_ORIGIN = P.shifted("j2_link#1", (-143.15, 240.05, -15.81))   # j2_link#1's origin (on the elbow_pitch axis; the retired j3_coupler#1 shared it)
WRIST_PITCH_ORIGIN = P.shifted("j3_coupler#2", (-283.37, 393.63, 35.33))   # j3_coupler#2 origin (on the wrist_pitch axis)
WRIST_ROLL_ORIGIN = P.shifted("gt2_pulley_20t#1", (-379.355, 448.22, 24.495))   # 20T pulley origin, on the pancake shaft axis
JAW_A_ORIGIN = P.shifted("gripper#1", (-453.626, 429.287, 26.317))   # gripper_slider#1 world bbox centre
JAW_B_ORIGIN = P.shifted("gripper#1", (-400.194, 521.876, 28.815))   # gripper_slider#2 world bbox centre
TOOL0_ORIGIN = P.shifted("gripper#1", (-480.567, 506.452, 31.03))    # midpoint between the two finger ends
# The link directions (only their direction is used - lib/datum.py frame() projects them; the SHIFTS are parallel):
SHOULDER_TO_ELBOW_INPLANE = (-141.353693, 155.039565, 9.127651)   # [REFERENCE] upper_arm_link long direction: the capture's
#                                                                   ELBOW_ORIGIN - SHOULDER_ORIGIN minus its 10.0 mm component along N
ELBOW_TO_WRIST_INPLANE = (-142.926, 153.58, 9.229)   # [REFERENCE] forearm_link long direction (elbow -> wrist_pitch, perpendicular to N)

# --- the forearm roll (the 6th joint, assemblies/forearm_roll_drive.py): its axis runs along the forearm through the
# WRIST CENTRE - the point where the wrist_pitch and wrist_roll axes meet (the wrist_roll origin sits
# WRIST_CENTRE_ALONG_N back along N from WRIST_PITCH_ORIGIN: the URDF's wrist_roll z = -0.016998) - so the last three
# axes stay concurrent (a spherical wrist). It crosses the elbow axis FOREARM_ROLL_AXIS_Z along N from the elbow
# origin (42 - 17 = 25, lib/params.py); tests/forearm/test_roll_drive.py checks WRIST_CENTRE lies on it.
WRIST_CENTRE_ALONG_N = -17.0                 # [REFERENCE] see above
_len = math.sqrt(sum(v * v for v in ELBOW_TO_WRIST_INPLANE))
FOREARM_ROLL_AXIS = tuple(v / _len for v in ELBOW_TO_WRIST_INPLANE)                    # unit, elbow -> wrist
FOREARM_ROLL_ORIGIN = tuple(o + PARAMS.FOREARM_ROLL_AXIS_Z * n for o, n in zip(ELBOW_ORIGIN, N, strict=True))   # on the elbow axis
WRIST_CENTRE = tuple(o + WRIST_CENTRE_ALONG_N * n for o, n in zip(WRIST_PITCH_ORIGIN, N, strict=True))

# --- rigid links: placement keys that move together -----------------------------------------
LINK_ORDER = ["base_link", "shoulder_link", "upper_arm_link", "elbow_link", "forearm_link", "wrist_pitch_link",
              "wrist_roll_link", "jaw_a_link", "jaw_b_link", "tool0"]
LINKS: dict[str, list[str]] = {
    # the base's motor mount (its +X lobe) is bolted to it with its M4 screws + nuts; the base_yaw motor + its MKS
    # board hang under the mount's plate (lib/mounts.py) - they turn nothing themselves; the base_yaw bearing pair sits
    # in the base's bore (a bearing rides with its housing), the thrust bearing's lower washer and its cage in the
    # base's groove
    "base_link": ["base#1", "bearing_6806#1", "bearing_6806#2", "washer_as6590#1", "bearing_axk6590#1",
                  "base_motor_mount#1", "base_motor_mount_screws#1", "base_motor_mount_nuts#1", "nema17_48mm#1",
                  "mks_servo42d#1"],
    # j1_coupler (the holder) turns on the base, its clamp cap bolted on; the cycloidal drive's stator - the held carrier
    # (motor plate, output hub, output pins), the motor (+ its MKS board) and the gear train - sits in its fork
    # (assemblies/cycloidal_drive.py BODIES); the thrust bearing's upper washer turns with it, under its seat, and so does
    # the base_yaw 120T bolted to its stub's end inside the base (the base_yaw output, the driven side of its belt), with
    # the M4 screws + nuts that clamp it.
    "shoulder_link": ["j1_coupler#1", "j1_coupler_cap#1", "washer_as6590#2", "gt2_pulley_120t#1", "yaw_pulley_screws#1",
                      "yaw_pulley_nuts#1", "cycloidal_drive#1:stator"],
    # the drive's rotor (its turning shell, the ring pins and both 6814s) is bolted to j1_link: the shoulder_pitch output;
    # the elbow_pitch motor + board bolt to j1_link's pad, the elbow bearing pair sits in its elbow bore (lib/mounts.py)
    "upper_arm_link": ["cycloidal_drive#1:rotor", "j1_link#1", "bearing_6806#3", "bearing_6806#4", "nema17_40mm#2", "mks_servo42d#2"],
    # the elbow 90T pulley (the elbow_pitch output, assumed the driven side  [ASSUMPTION]) and the M4 screws + nuts
    # that clamp it carry the forearm roll drive's STATOR - the elbow block that IS the elbow coupler now (j3_coupler#1
    # is retired, lib/placements.py), both bearings, the end cap, the roll motor + board and its 20T
    # (assemblies/forearm_roll_drive.py BODIES)
    "elbow_link": ["gt2_pulley_90t#3", "elbow_pulley_screws#1", "elbow_pulley_nuts#1", "forearm_roll_drive#1:stator"],
    # the drive's ROTOR - the hollow roll shaft - IS the forearm's elbow end (its flange bolts to j2_link's wall);
    # the wrist_pitch motor + board bolt to j2_link's web, the wrist bearing pair sits in its wrist boss (lib/mounts.py)
    "forearm_link": ["forearm_roll_drive#1:rotor", "j2_link#1", "bearing_6806#5", "bearing_6806#6", "nema17_40mm#3", "mks_servo42d#3"],
    # likewise the wrist 90T pulley, its M4 screws + nuts and the J3-coupler ride with the wrist-pitch body  [ASSUMPTION]
    "wrist_pitch_link": ["wrist_link#1", "gripper_clamp_bracket#1", "nema17_pancake#1", "gt2_pulley_90t#4",
                         "wrist_pulley_screws#1", "wrist_pulley_nuts#1", "j3_coupler#2"],
    # the gripper base rolls with the 20T pulley; the servo crank linkage is merged in  [ASSUMPTION]
    "wrist_roll_link": [
        "gt2_pulley_20t#1", "gripper_j3_connector#1", "servo_holder#1", "mg996r_servo#1", "mg996r_horn#1",
        "gripper_cover#1", "gripper_rail_6mm#1", "gripper_rail_6mm#2",
        "gripper_link_1#1", "gripper_link_2#1", "gripper_link_1#2", "gripper_link_2#2",
    ],
    "jaw_a_link": ["gripper_slider#1", "gripper_finger_left#1", "gripper_finger_left#2", "gripper_end#1"],
    "jaw_b_link": ["gripper_slider#2", "gripper_finger_right#1", "gripper_finger_right#2", "gripper_end#2"],
    "tool0": [],   # frame-only link at the fingertip midpoint
}


@dataclass(frozen=True)
class Joint:
    name: str
    type: str                     # revolute | prismatic | fixed
    parent: str
    child: str
    origin_w: tuple               # point on the axis, W frame, mm
    axis_w: tuple                 # joint axis direction, W frame (unit)
    x_hint_w: tuple               # child link's long direction at capture (defines the frame's X)
    lower: float = 0.0            # rad or m  [ESTIMATE]
    upper: float = 0.0
    effort: float = 0.0           # N.m or N  [ESTIMATE]
    velocity: float = 0.0         # rad/s or m/s  [ESTIMATE]
    mimic: tuple | None = None    # (joint, multiplier, offset)
    notes: str = ""


DEG = math.pi / 180.0
JOINTS: list[Joint] = [
    Joint("base_yaw", "revolute", "base_link", "shoulder_link", BASE_YAW_ORIGIN, U, BASE_FORWARD,
          -PARAMS.BASE_YAW_LIMIT_DEG * DEG, PARAMS.BASE_YAW_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="j1_coupler (carrying the cycloidal drive's stator) turns on the base in the bearing_6806#1 / #2 pair and stands "
                "on the thrust bearing in its groove (washer_as6590#1, bearing_axk6590#1, washer_as6590#2), held down by "
                "the GT2 120T bolted to its stub's end under the lower bearing (gt2_pulley_120t#1); belt-driven (BASE_YAW_RATIO) by "
                "nema17_48mm#1 (the 48 mm motor) + mks_servo42d#1 under the base plate (lib/mounts.py) "
                "[which CAN id (software/control/src/config.py J1..J3) it is: unconfirmed]"),
    Joint("shoulder_pitch", "revolute", "shoulder_link", "upper_arm_link", SHOULDER_ORIGIN, N, SHOULDER_TO_ELBOW_INPLANE,
          PARAMS.SHOULDER_PITCH_LIMITS_DEG[0] * DEG, PARAMS.SHOULDER_PITCH_LIMITS_DEG[1] * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="the 21:1 cycloidal drive (CYCLOIDAL_RATIO, its own NEMA 17 x 48 + MKS board): the j1_coupler fork holds its "
                "hub and motor, its shell - the output, turning with the motor - is bolted to j1_link [which CAN id: unconfirmed]"),
    Joint("elbow_pitch", "revolute", "upper_arm_link", "elbow_link", ELBOW_ORIGIN, N, ELBOW_TO_WRIST_INPLANE,
          -PARAMS.ELBOW_PITCH_LIMIT_DEG * DEG, PARAMS.ELBOW_PITCH_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + the roll drive's block (its stator) at the elbow, turning in the bearing_6806#3 / #4 pair; "
                "belt-driven (GT2_RATIO, one stage) by nema17_40mm#2 + mks_servo42d#2 on j1_link's pad (lib/mounts.py) "
                "[which CAN id: unconfirmed]"),
    # the roll: Z along the forearm (its child link's long direction IS the axis), so X = N, the pitch-axis direction
    Joint("forearm_roll", "revolute", "elbow_link", "forearm_link", FOREARM_ROLL_ORIGIN, FOREARM_ROLL_AXIS, N,
          -PARAMS.FOREARM_ROLL_LIMIT_DEG * DEG, PARAMS.FOREARM_ROLL_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="the forearm roll drive (assemblies/forearm_roll_drive.py, FOREARM_ROLL_RATIO 4.5): the hollow roll shaft's "
                "flange bolts to j2_link's wall; belt-driven by the drive's own nema17_40mm + mks_servo42d on the elbow "
                "block's pad [a 4th CAN id - software/control/src/config.py has three: unconfirmed]; hard stop +/- FOREARM_ROLL_LIMIT_DEG"),
    Joint("wrist_pitch", "revolute", "forearm_link", "wrist_pitch_link", WRIST_PITCH_ORIGIN, N, F,
          PARAMS.WRIST_PITCH_LIMITS_DEG[0] * DEG, PARAMS.WRIST_PITCH_LIMITS_DEG[1] * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + J3-coupler at the wrist, turning in the bearing_6806#5 / #6 pair; belt-driven by nema17_40mm#3 + mks_servo42d#3 on "
                "j2_link's web (lib/mounts.py) [which CAN id: unconfirmed]"),
    Joint("wrist_roll", "revolute", "wrist_pitch_link", "wrist_roll_link", WRIST_ROLL_ORIGIN, F, PJ,
          -PARAMS.WRIST_ROLL_LIMIT_DEG * DEG, PARAMS.WRIST_ROLL_LIMIT_DEG * DEG, PARAMS.WRIST_EFFORT_NM, PARAMS.WRIST_VELOCITY_RAD_S,
          notes="NEMA17 pancake + 20T pulley; NOT driven by software/control/src/config.py yet"),
    Joint("jaw_a", "prismatic", "wrist_roll_link", "jaw_a_link", JAW_A_ORIGIN, tuple(-v for v in PJ), F,
          -PARAMS.JAW_TRAVEL_MM * 1e-3, PARAMS.JAW_TRAVEL_MM * 1e-3, PARAMS.JAW_EFFORT_N, PARAMS.JAW_VELOCITY_M_S,
          notes="positive = opening (jaw A moves away from jaw B); MG996R crank"),
    Joint("jaw_b", "prismatic", "wrist_roll_link", "jaw_b_link", JAW_B_ORIGIN, tuple(-v for v in PJ), F,
          -PARAMS.JAW_TRAVEL_MM * 1e-3, PARAMS.JAW_TRAVEL_MM * 1e-3, PARAMS.JAW_EFFORT_N, PARAMS.JAW_VELOCITY_M_S,
          mimic=("jaw_a", -1.0, 0.0), notes="mirrors jaw_a"),
    Joint("tool0_joint", "fixed", "wrist_roll_link", "tool0", TOOL0_ORIGIN, F, PJ,
          notes="tool frame: Z = approach (along the forearm), X = jaw travel"),
]
JOINT_BY_NAME = {j.name: j for j in JOINTS}
JOINT_OF_CHILD = {j.child: j for j in JOINTS}


def joint_frame_world(name: str) -> bd.Location:
    j = JOINT_BY_NAME[name]
    return frame(j.origin_w, j.axis_w, j.x_hint_w)


def link_frame_world(link: str) -> bd.Location:
    """The link's frame in W: base_link -> base_frame(); any other link -> its parent joint's frame."""
    if link == "base_link":
        return base_frame()
    return joint_frame_world(JOINT_OF_CHILD[link].name)


def all_keys() -> list[str]:
    return [k for link in LINK_ORDER for k in LINKS[link]]
