"""Kinematic decomposition of the arm: rigid links and joint frames, derived from the
SolidWorks capture (reference/placements.json). Single source for robot/links/*.py,
tools/robot/derive.py, tools/robot/export_link_meshes.py and tests/test_robot.py.

LINKS holds placement keys: part occurrences and designed-module keys - "cycloidal_drive#1", or
one rigid body of it, "cycloidal_drive#1:stator" / ":rotor" (assemblies/cycloidal_drive.py
BODIES) - which assemblies/_occurrences.world_rows expands into world-placed parts.

Chain: base_link -base_yaw-> shoulder_link -shoulder_pitch-> upper_arm_link -elbow_pitch->
forearm_link -wrist_pitch-> wrist_pitch_link -wrist_roll-> wrist_roll_link -jaw_a/jaw_b->
jaw_*_link, + tool0 (frame-only). The cycloidal drive IS the shoulder_pitch joint: its stator
(housing + motor, bolted into the j1_coupler yoke) rides in shoulder_link, its rotor (output
hub + pins, bolted to j1_link) in upper_arm_link (docs/cycloidal_drive.md "Attachment").

Frames (all in the SolidWorks WORLD frame W, millimetres; W is +Y up, the arm extends toward
-X, see reference/README.md):
  * base_link frame B: REP-103 (Z up, X forward) at the base_yaw axis foot on the base's bottom
    face: origin (0, BASE_BOTTOM_Y, 0), X_B = -X_W, Y_B = +Z_W, Z_B = +Y_W. BASE_FRAME, frame()
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
from dataclasses import dataclass, field

from build123d import Location

from lib import params as PARAMS
from lib import placements as P
from lib.datum import BASE_BOTTOM_Y, BASE_FORWARD, BASE_FRAME, U, frame  # noqa: F401  (the datum frames live below assemblies/)

ROBOT_NAME = "arm"

# --- unit vectors of the capture pose (W frame); U (world up) and BASE_FORWARD are lib/datum.py's ---
N = (0.064439, 0.0, 0.997922)                # the three pitch axes (parallel): 90T pulley / J3-coupler bore direction; the cycloidal drive's +Z is -N
F = (-0.865419, 0.497923, 0.055880)          # wrist-roll axis: NEMA17 pancake shaft / 20T pulley bore (toward the tool)
PJ = (0.499699, 0.865884, 0.023354)          # jaw travel: the two Ø6 gripper rails (slider#1 -> slider#2)

BASE_YAW_ORIGIN = (0.0, 85.010435, 0.0)      # [REFERENCE] on the base_yaw axis at the cycloidal drive's axis height (its node's Y)
SHOULDER_ORIGIN = (-2.440595, 85.010435, -34.915297)   # [REFERENCE] j1_link#1 origin: on the cycloidal drive's axis, 66.5 mm along it from the
#                                                        motor-plate face (1.5 past the hub's arm-mount face, CYCLOIDAL_OUTPUT_FACE_Z)
ELBOW_ORIGIN = (-143.15, 240.05, -15.81)     # [REFERENCE] j2_link#1 / j3_coupler#1 origin (on the elbow_pitch axis)
WRIST_PITCH_ORIGIN = (-283.37, 393.63, 35.33)   # [REFERENCE] j3_coupler#2 origin (on the wrist_pitch axis)
WRIST_ROLL_ORIGIN = (-379.355, 448.22, 24.495)  # [REFERENCE] 20T pulley origin, on the pancake shaft axis
JAW_A_ORIGIN = (-453.626, 429.287, 26.317)   # [REFERENCE] gripper_slider#1 world bbox centre
JAW_B_ORIGIN = (-400.194, 521.876, 28.815)   # [REFERENCE] gripper_slider#2 world bbox centre
TOOL0_ORIGIN = (-480.567, 506.452, 31.03)    # [REFERENCE] midpoint between the two finger ends
SHOULDER_TO_ELBOW_INPLANE = (-141.353693, 155.039565, 9.127651)   # [REFERENCE] upper_arm_link long direction: ELBOW_ORIGIN - SHOULDER_ORIGIN
#                                                                   minus its 10.0 mm component along N (210.0 mm in the pitch plane)
ELBOW_TO_WRIST_INPLANE = (-142.926, 153.58, 9.229)   # [REFERENCE] forearm_link long direction (elbow -> wrist_pitch, perpendicular to N)

# --- rigid links: placement keys that move together -----------------------------------------
LINK_ORDER = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_pitch_link",
              "wrist_roll_link", "jaw_a_link", "jaw_b_link", "tool0"]
LINKS: dict[str, list[str]] = {
    "base_link": ["base#1"],
    # j1_coupler (the holder) turns on the base; the cycloidal drive's stator - housing, motor
    # and the gear train - is bolted into its yoke (assemblies/cycloidal_drive.py BODIES).
    "shoulder_link": ["j1_coupler#1", "cycloidal_drive#1:stator"],
    # the drive's rotor (output hub + output pins) is bolted to j1_link: the shoulder_pitch output.
    "upper_arm_link": ["cycloidal_drive#1:rotor", "j1_link#1", "j1_cap#1"],
    # the elbow 90T pulley + J3-coupler assumed bolted to the forearm (the driven side)  [ASSUMPTION]
    "forearm_link": ["j2_link#1", "j2_cap_1#1", "j2_cap_2#1", "gt2_pulley_90t#1", "j3_coupler#1"],
    # likewise the wrist 90T pulley + J3-coupler ride with the wrist-pitch body  [ASSUMPTION]
    "wrist_pitch_link": ["wrist_link#1", "gripper_clamp_bracket#1", "nema17_pancake#1", "gt2_pulley_90t#2", "j3_coupler#2"],
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
          notes="j1_coupler (carrying the cycloidal drive's stator) turns on the base "
                "[which MKS motor (src/config.py J1..J3) drives it: unconfirmed]"),
    Joint("shoulder_pitch", "revolute", "shoulder_link", "upper_arm_link", SHOULDER_ORIGIN, N, SHOULDER_TO_ELBOW_INPLANE,
          -PARAMS.SHOULDER_PITCH_LIMIT_DEG * DEG, PARAMS.SHOULDER_PITCH_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="the 20:1 cycloidal drive (CYCLOIDAL_RATIO, its own NEMA 17): stator in the j1_coupler yoke, "
                "output hub bolted to j1_link [which MKS motor: unconfirmed]"),
    Joint("elbow_pitch", "revolute", "upper_arm_link", "forearm_link", ELBOW_ORIGIN, N, ELBOW_TO_WRIST_INPLANE,
          -PARAMS.ELBOW_PITCH_LIMIT_DEG * DEG, PARAMS.ELBOW_PITCH_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + J3-coupler at the elbow; belt-driven [which MKS motor: unconfirmed]"),
    Joint("wrist_pitch", "revolute", "forearm_link", "wrist_pitch_link", WRIST_PITCH_ORIGIN, N, F,
          -PARAMS.WRIST_PITCH_LIMIT_DEG * DEG, PARAMS.WRIST_PITCH_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + J3-coupler at the wrist; belt-driven [which MKS motor: unconfirmed]"),
    Joint("wrist_roll", "revolute", "wrist_pitch_link", "wrist_roll_link", WRIST_ROLL_ORIGIN, F, PJ,
          -PARAMS.WRIST_ROLL_LIMIT_DEG * DEG, PARAMS.WRIST_ROLL_LIMIT_DEG * DEG, PARAMS.WRIST_EFFORT_NM, PARAMS.WRIST_VELOCITY_RAD_S,
          notes="NEMA17 pancake + 20T pulley; NOT driven by src/config.py yet"),
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


def joint_frame_world(name: str) -> Location:
    j = JOINT_BY_NAME[name]
    return frame(j.origin_w, j.axis_w, j.x_hint_w)


def link_frame_world(link: str) -> Location:
    """The link's frame in W: base_link -> BASE_FRAME; any other link -> its parent joint's frame."""
    if link == "base_link":
        return BASE_FRAME
    return joint_frame_world(JOINT_OF_CHILD[link].name)


def all_keys() -> list[str]:
    return [k for link in LINK_ORDER for k in LINKS[link]]
