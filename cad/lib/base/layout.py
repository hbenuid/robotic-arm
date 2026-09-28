"""The base's derived points and stations (pure math, no kernel): what lib/base/body.py and the tests share."""
from __future__ import annotations

import math

from lib.base.params import DEFAULT, BaseConfig
from lib.motors import NEMA17_BOLT_SP


def motor_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the base_yaw motor's 4 holes: the NEMA 17 square about the pattern centre."""
    (cx, cz), h = cfg.motor.centre, NEMA17_BOLT_SP / 2.0
    return [(cx + sx * h, cz + sz * h) for sx in (-1, 1) for sz in (-1, 1)]


def side_stub_x(cfg: BaseConfig = DEFAULT) -> float:
    """Where the straight sides stop above the plate: the inside of a side (z = +/- (r - wall)) meets the outer round."""
    s = cfg.shell
    return math.sqrt(s.r ** 2 - (s.r - s.wall) ** 2)


def chamfer_inset(cfg: BaseConfig = DEFAULT) -> float:
    """How far the 45 degree chamfer under the cap runs in from the wall's inside: its height."""
    return cfg.cap.underside_y - cfg.cap.chamfer_y0


def joint_bolt_points(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(z, y) of the 4 M4 through the joint's ribs (JointParams): each rib's centreline, bolt_inset under the plate's
    underside and above the bottom face - also the (x, y) of the screw / nut patterns (base_motor_mount_screws), whose
    +Z the mounts turn down the base's -X."""
    s, j = cfg.shell, cfg.joint
    z = s.r - s.wall - j.rib_w / 2.0
    return [(sz * z, y) for sz in (1, -1) for y in (cfg.plate.y[0] - j.bolt_inset, s.y0 + j.bolt_inset)]


def joint_stations(cfg: BaseConfig = DEFAULT) -> dict[str, float]:
    """The joint's stations along X: the base rib's back face (the nuts' outer faces flush with it), the joint face,
    the mount rib's inside face (under the screws' heads), the nuts' bearing faces and the screws' tips."""
    j = cfg.joint
    back, head = j.split_x - j.rib_t, j.split_x + j.rib_t
    return {"x_base_rib": back, "x_split": j.split_x, "x_head": head, "x_nut_face": back + j.nut.h,
            "x_tip": head - j.screw_len}
