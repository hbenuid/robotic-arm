"""Kinematic decomposition of the arm: rigid links and joint frames, derived from the
SolidWorks capture (reference/placements.json). Single source for robot/links/*.py,
tools/robot_frames.py, tools/export_link_meshes.py and tests/test_robot.py.

Frames (all in the SolidWorks WORLD frame W, millimetres; W is +Y up, the arm extends toward
-X, see reference/README.md):
  * base_link frame B: REP-103 (Z up, X forward) at the J1 axis foot on the base's bottom
    face: origin (0, BASE_BOTTOM_Y, 0), X_B = -X_W, Y_B = +Z_W, Z_B = +Y_W.
  * Every joint frame has Z along the joint axis and X along the child link's long direction
    at the capture pose (projected perpendicular to Z); Y = Z x X.
  * A child link's frame IS its joint frame at the capture pose, so ALL JOINT VALUES ARE 0 AT
    THE CAPTURE POSE and every link mesh (exported in the link frame) has an identity origin.
    Note: in that pose link1 is yawed +3.694 deg about J1 relative to the base's forward axis
    (the captured J2/J3 axes are not exactly parallel to Y_B) - this is a POSE, not a design
    offset.
Axis signs (positive-motion direction) and all limits are placeholders [ESTIMATE] until
confirmed by viewer sweeps / on hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from build123d import Location, Plane, Vector

from lib import params as PARAMS
from lib import placements as P

ROBOT_NAME = "arm"

# --- unit vectors of the capture pose (W frame) -----------------------------------------------
U = (0.0, 1.0, 0.0)                          # J1 axis: world up
N = (0.064439, 0.0, 0.997922)                # J2 and J3 axes (parallel): 90T pulley / J3-coupler bore direction
F = (-0.865419, 0.497923, 0.055880)          # wrist-roll axis: NEMA17 pancake shaft / 20T pulley bore (toward the tool)
PJ = (0.499699, 0.865884, 0.023354)          # jaw travel: the two Ø6 gripper rails (slider#1 -> slider#2)
BASE_FORWARD = (-1.0, 0.0, 0.0)              # the arm extends toward -X_W

BASE_BOTTOM_Y = -100.9                       # [REFERENCE] base world bbox min Y (mounting face)
J1_ORIGIN = (0.0, 85.010435, 0.0)            # [REFERENCE] on the J1 axis at the link1 / cycloidal-output height
J2_ORIGIN = (-143.15, 240.05, -15.81)        # [REFERENCE] j2_link#1 / j3_coupler#1 origin (on the J2 axis)
J3_ORIGIN = (-283.37, 393.63, 35.33)         # [REFERENCE] j3_coupler#2 origin (on the J3 axis)
WRIST_ORIGIN = (-379.355, 448.22, 24.495)    # [REFERENCE] 20T pulley origin, on the pancake shaft axis
JAW_A_ORIGIN = (-453.626, 429.287, 26.317)   # [REFERENCE] gripper_slider#1 world bbox centre
JAW_B_ORIGIN = (-400.194, 521.876, 28.815)   # [REFERENCE] gripper_slider#2 world bbox centre
TOOL0_ORIGIN = (-480.567, 506.452, 31.03)    # [REFERENCE] midpoint between the two finger ends
J2_TO_J3_INPLANE = (-142.926, 153.58, 9.229) # [REFERENCE] link2 long direction (J2 -> J3, perpendicular to N)

# --- rigid links: placement keys that move together -----------------------------------------
LINK_ORDER = ["base_link", "link1", "link2", "link3", "wrist_roll_link", "jaw_a_link", "jaw_b_link", "tool0"]
LINKS: dict[str, list[str]] = {
    "base_link": ["base#1"],
    # j1_coupler assumed to rotate with J1 (couples the cycloidal output to link1)  [ASSUMPTION]
    "link1": ["j1_coupler#1", "j1_link#1", "j1_cap#1"],
    # the J2 90T pulley + J3-coupler assumed bolted to link2 (the driven side)  [ASSUMPTION]
    "link2": ["j2_link#1", "j2_cap_1#1", "j2_cap_2#1", "gt2_pulley_90t#1", "j3_coupler#1"],
    "link3": ["wrist_link#1", "gripper_clamp_bracket#1", "nema17_pancake#1", "gt2_pulley_90t#2", "j3_coupler#2"],
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
    Joint("j1", "revolute", "base_link", "link1", J1_ORIGIN, U, BASE_FORWARD,
          -PARAMS.J1_LIMIT_DEG * DEG, PARAMS.J1_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="cycloidal-drive output; MKS SERVO42D J1"),
    Joint("j2", "revolute", "link1", "link2", J2_ORIGIN, N, J2_TO_J3_INPLANE,
          -PARAMS.J2_LIMIT_DEG * DEG, PARAMS.J2_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + J3-coupler at the shoulder; MKS J2"),
    Joint("j3", "revolute", "link2", "link3", J3_ORIGIN, N, F,
          -PARAMS.J3_LIMIT_DEG * DEG, PARAMS.J3_LIMIT_DEG * DEG, PARAMS.ARM_JOINT_EFFORT_NM, PARAMS.ARM_JOINT_VELOCITY_RAD_S,
          notes="GT2 90T pulley + J3-coupler at the elbow; MKS J3"),
    Joint("wrist_roll", "revolute", "link3", "wrist_roll_link", WRIST_ORIGIN, F, PJ,
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


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def frame(origin_w, z_w, x_hint_w) -> Location:
    """World Location of a right-handed frame: Z along z_w, X along x_hint_w projected
    perpendicular to Z, origin at origin_w (mm)."""
    z = _unit(z_w)
    d = sum(a * b for a, b in zip(x_hint_w, z))
    x = _unit(tuple(a - d * b for a, b in zip(x_hint_w, z)))
    return Location(Plane(origin=Vector(*origin_w), x_dir=Vector(*x), z_dir=Vector(*z)))


BASE_FRAME: Location = frame((0.0, BASE_BOTTOM_Y, 0.0), U, BASE_FORWARD)


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
