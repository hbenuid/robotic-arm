"""The base's derived points and stations (pure math, no kernel): what lib/base/body.py and the tests share."""
from __future__ import annotations

import math

from lib.base.params import DEFAULT, BaseConfig
from lib.motors import MKS_SERVO42D_W, NEMA17_BOLT_SP


def motor_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the base_yaw motor's 4 holes: the NEMA 17 square about the pattern centre."""
    (cx, cz), h = cfg.motor.centre, NEMA17_BOLT_SP / 2.0
    return [(cx + sx * h, cz + sz * h) for sx in (-1, 1) for sz in (-1, 1)]


def outer_r(cfg: BaseConfig, y: float) -> float:
    """The walls' outer radius (the round half's, the sides' |z|) at height y: r at the cap's top face, `draft` more per
    mm down."""
    s = cfg.shell
    return s.r + s.draft * (cfg.cap.top_y - y)


def inner_r(cfg: BaseConfig, y: float) -> float:
    """The walls' inner radius at height y: `wall` in from the outside."""
    return outer_r(cfg, y) - cfg.shell.wall


def side_stub_x(cfg: BaseConfig = DEFAULT) -> float:
    """Where the straight sides stop above the plate: the inside of a side (z = +/- (r - wall)) meets the outer round -
    at the plate's top face, where the leaning walls are widest above it."""
    r = outer_r(cfg, cfg.plate.y[1])
    return math.sqrt(r ** 2 - (r - cfg.shell.wall) ** 2)


def flare_points(cfg: BaseConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The foot's chamfer as (radius, y) ends: its foot on the flange's top, its top on the wall's outside."""
    s, f = cfg.shell, cfg.foot
    top_y = s.y0 + f.flare_h
    return (f.flare_r, s.y0 + f.t), (outer_r(cfg, top_y), top_y)


def foot_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the foot's screw holes: on the circle hole_r at hole_deg on the round half, at side_hole_x on the
    sides."""
    f = cfg.foot
    pts = [(f.hole_r * math.cos(math.radians(a)), f.hole_r * math.sin(math.radians(a))) for a in f.hole_deg]
    return pts + [(f.side_hole_x, sz * f.hole_r) for sz in (1, -1)]


def chamfer_inset(cfg: BaseConfig = DEFAULT) -> float:
    """How far the 45 degree chamfer under the cap runs in from the wall's inside: its height."""
    return cfg.cap.underside_y - cfg.cap.chamfer_y0


def mount_inner_half(cfg: BaseConfig = DEFAULT) -> float:
    """The motor mount's inside half-width (|z| of its side walls' inner faces, about the axis): the MKS board's square
    about the motor's centre, and MountParams.room clear of it."""
    return MKS_SERVO42D_W / 2.0 + abs(cfg.motor.centre[1]) + cfg.mount.room


def mount_x1(cfg: BaseConfig = DEFAULT) -> float:
    """The outer face of the motor mount's end wall: room past the board's square at the slots' middle, and the wall."""
    return cfg.motor.centre[0] + MKS_SERVO42D_W / 2.0 + cfg.mount.room + cfg.mount.wall


def truss_panels(cfg: BaseConfig = DEFAULT) -> list[dict]:
    """The motor mount's three trussed walls (MountParams): per wall the `axis` across it ("z" the two side walls,
    "x" the end wall) and the `span` of its thickness on that axis, `u` - the part axis along it ("x" / "z") -, the
    frame's inside `window` ((u0, u1), (v0, v1)), v the part's y (a `strut` wide rail under the plate and on the table,
    a post at each end from the outer corner - the side walls' at the joint as deep as the ears, which it backs), and
    the V's two `struts`: centrelines from the window's top corners down to the middle of its bottom edge."""
    s, p, j, m = cfg.shell, cfg.plate, cfg.joint, cfg.mount
    w_in, x1 = mount_inner_half(cfg), mount_x1(cfg)
    w_out = w_in + m.wall
    v = (s.y0 + m.strut, p.y[0] - m.strut)

    def panel(axis: str, span: tuple, u_axis: str, u: tuple) -> dict:
        um = (u[0] + u[1]) / 2.0
        return {"axis": axis, "span": span, "u": u_axis, "window": (u, v),
                "struts": [((u[0], v[1]), (um, v[0])), ((u[1], v[1]), (um, v[0]))]}

    side_u = (j.split_x + max(m.ear_t, m.strut), x1 - m.strut)
    end_u = (-(w_out - m.strut), w_out - m.strut)
    return [panel("z", (w_in, w_out), "x", side_u), panel("z", (-w_out, -w_in), "x", side_u),
            panel("x", (x1 - m.wall, x1), "z", end_u)]


def joint_bolt_points(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(z, y) of the 4 M4 through the mount's ears and the base's posts (JointParams, MountParams): each ear's
    centreline, bolt_inset under the plate's underside and above the bottom face - also the (x, y) of the screw / nut
    patterns (base_motor_mount_screws), whose +Z the mounts turn down the base's -X."""
    s, j, m = cfg.shell, cfg.joint, cfg.mount
    z = mount_inner_half(cfg) + m.wall + m.ear_w / 2.0
    return [(sz * z, y) for sz in (1, -1) for y in (cfg.plate.y[0] - j.bolt_inset, s.y0 + j.bolt_inset)]


def joint_stations(cfg: BaseConfig = DEFAULT) -> dict[str, float]:
    """The joint's stations along X: the base posts' back face (the nuts' outer faces flush with it), the joint face,
    the ears' outer face (under the screws' heads), the nuts' bearing faces and the screws' tips."""
    j = cfg.joint
    back, head = j.split_x - j.post_t, j.split_x + cfg.mount.ear_t
    return {"x_post": back, "x_split": j.split_x, "x_head": head, "x_nut_face": back + j.nut.h,
            "x_tip": head - j.screw_len}
