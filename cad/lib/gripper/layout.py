"""Where the gripper module (assemblies/gripper.py) puts its parts - frames as data, pure Python.

Two frames:
- the base frame B (lib/gripper/params.py BaseParams): the plate's bottom on z 0, x along the jaws' travel, the servo
  toward +z, the fingers toward -z - every placement is worked out in it;
- the module frame M (the arm places it: lib/mounts.py MODULE_MOUNTS "gripper#1", lib/wrist/layout.py
  gripper_frame_in_module): its origin on the wrist roll's axis R' on gripper_j3_connector's plate's gripper face, +Z
  along R' toward the fingertips, +Y along the jaws' travel (the connector plate's long axis), +X = Y x Z - so
  M = R_MB (B - (x0, 0, seat)): X = B's y, Y = B's x, Z = -B's z, the pinion's axis on R'.

At zero the racks are centred (the jaws 23 apart, their faces at x0 -/+ 11.5): rack a on +y and jaw a's finger on -x,
rack b and jaw b turned half round the axis; the pinion meshes both at 0 deg (90 deg is 3.5 of its 14 teeth: a gap faces
each rack's middle tooth). Jaw a opens toward -x (M -Y), jaw b toward +x.
"""
from __future__ import annotations

from lib.datum import angles
from lib.gripper.params import DEFAULT, GripperConfig

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_TURN_Y = ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))   # half a turn about y: rack a / jaw a, turned over
_TURN_X = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))   # half a turn about x: rack b / jaw b
_SERVO = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))     # the servo's frame upside down: its +Y (to the spline) -> -z
_M_FROM_B = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_CONNECTOR = ((-1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, -1.0, 0.0))   # gripper_j3_connector's frame in M: its plate's +Y
#                                                                       face toward the pulley (-Z), its long axis z on -Y


def x0(cfg: GripperConfig = DEFAULT) -> float:
    """The pinion's axis on the base's x: the servo's case centred between the posts' inner faces, its spline
    servo.spline_x off the case's middle (0.75)."""
    b, s = cfg.base, cfg.servo
    return (b.posts_x[0][1] + b.posts_x[1][0]) / 2.0 - ((s.case_x[0] + s.case_x[1]) / 2.0 - s.spline_x)


def stations(cfg: GripperConfig = DEFAULT) -> dict[str, float]:
    """B-frame z of everything stacked along the axis (seat: the back plate's outer face on the connector's plate, M's
    z 0)."""
    b, s, c, r = cfg.base, cfg.servo, cfg.cage, cfg.rack
    tab_face = b.post_top_z                                   # 22: the tabs on the posts' tops
    servo_back = tab_face + s.tab_face                        # 53.2: the servo's back (its frame's y 0)
    back_in = servo_back + c.back_gap                         # 53.4: the cage's back plate, from here ...
    rack = b.plate_t + r.t                                    # 10: each rack turned over, its bar on the plate ...
    jaw = rack - r.post_top                                   # -0.3: ... its posts' tips, where its jaw's arm (turned over) sits
    return {
        "tab_face": tab_face, "tab_back": tab_face + s.tab_t, "servo_back": servo_back, "back_in": back_in,
        "seat": back_in + c.back_t,                           # 58.4: ... to the connector's plate
        "spline_end": servo_back - s.spline_top,              # 8.0
        "pinion": b.plate_t + cfg.layout.pinion_gap,          # 4.25
        "rack": rack, "jaw": jaw,
        "tips": jaw - cfg.jaw.finger_top,                     # -45.3: the fingertips
    }


def _mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def to_module(point, cfg: GripperConfig = DEFAULT) -> tuple:
    """A point of the base frame in the module frame."""
    seat = stations(cfg)["seat"]
    return tuple(round(v, 9) + 0.0 for v in _vec(_M_FROM_B, (point[0] - x0(cfg), point[1], point[2] - seat)))


def _row(rot_b, at_b, cfg: GripperConfig):
    """A part placed in the base frame (rotation, its origin) as a module-frame row placement."""
    return to_module(at_b, cfg), angles(_mul(_M_FROM_B, rot_b))


def rows(cfg: GripperConfig = DEFAULT) -> list:
    """The module's (part, role | None, placement in M) rows, at zero (the racks centred)."""
    s, z, x = cfg.servo, stations(cfg), x0(cfg)
    return [
        ("gripper_j3_connector", None, ((0.0, 0.0, 0.0), angles(_CONNECTOR))),
        ("gripper_cage", None, _row(_I, (0.0, 0.0, 0.0), cfg)),
        ("mg996r_servo", "gripper", _row(_SERVO, (x - s.spline_x, 0.0, z["servo_back"]), cfg)),
        ("gripper_base", None, _row(_I, (0.0, 0.0, 0.0), cfg)),
        ("gripper_pinion", None, _row(_I, (x, 0.0, z["pinion"]), cfg)),
        ("gripper_rack", "a", _row(_TURN_Y, (x, 0.0, z["rack"]), cfg)),
        ("gripper_rack", "b", _row(_TURN_X, (x, 0.0, z["rack"]), cfg)),
        ("gripper_jaw", "a", _row(_TURN_Y, (x, 0.0, z["jaw"]), cfg)),
        ("gripper_jaw", "b", _row(_TURN_X, (x, 0.0, z["jaw"]), cfg)),
    ]


def tool0(cfg: GripperConfig = DEFAULT) -> tuple:
    """The fingertips' midpoint in the module frame (robot/frames.py TOOL0_ORIGIN): on R'."""
    return to_module((x0(cfg), 0.0, stations(cfg)["tips"]), cfg)


def jaw_origin(cfg: GripperConfig = DEFAULT) -> tuple:
    """Where both jaw joints sit (robot/frames.py): on R', level with the racks' bars' outer faces."""
    return to_module((x0(cfg), 0.0, stations(cfg)["rack"]), cfg)


# jaw a's opening direction in the module frame (B's -x); jaw b mimics it, multiplier -1
JAW_A_OPENS = (0.0, -1.0, 0.0)
