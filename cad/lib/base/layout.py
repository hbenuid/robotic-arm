"""The base's derived points and stations (pure math, no kernel): what lib/base/body.py and the tests share."""
from __future__ import annotations

import math

from lib.base.params import DEFAULT, YAW_BELT, YAW_BELT_Y, BaseConfig
from lib.belts import GT2_IDLER_WIDTH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_120T_TEETH, idler_on_run, pulley_circle
from lib.motors import MKS_SERVO42D_W, NEMA17_BOLT_SP


def yaw_belt_loop(cfg: BaseConfig = DEFAULT, *, idler: bool = True) -> list[tuple[float, float, float]]:
    """The base_yaw belt's loop (lib/belts.py) in the belt's own frame (yaw_belt_frame()): the motor's 20T on the
    origin, the 120T on the base_yaw axis on +X, the motor's centre distance from it, and - with cfg.idler - the
    tensioner on the run YawIdlerParams.run, `x` out from the axis, pushed in till the loop is YAW_BELT long (the stock
    belt is longer than the open loop)."""
    cx, cz = cfg.motor.centre
    c = math.hypot(cx, cz)
    loop = [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(c, 0.0, GT2_PULLEY_120T_TEETH)]
    if idler and cfg.idler is not None:
        loop.insert(cfg.idler.run + 1, idler_on_run(loop, cfg.idler.run, c - cfg.idler.x, YAW_BELT))
    return loop


def yaw_belt_frame(cfg: BaseConfig = DEFAULT) -> tuple:
    """The base_yaw belt's frame as data in the base's: on the motor's axis at YAW_BELT_Y, its +Z up the base_yaw axis
    (the base's +Y), its +X toward the axis."""
    cx, cz = cfg.motor.centre
    return ((cx, YAW_BELT_Y, cz), (-90.0, 0.0, round(180.0 - math.degrees(math.atan2(cz, cx)), 6)))


def yaw_belt_point(bx: float, by: float, cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """A point of the belt's frame (yaw_belt_frame(): x from the motor's axis toward the base_yaw axis, y across) as the
    base's (x, z): the frame's +X is -(cx, cz) / c, its +Y the base's +Z turned with it (the motor's small z)."""
    cx, cz = cfg.motor.centre
    c = math.hypot(cx, cz)
    ux, uz = -cx / c, -cz / c          # the frame's +X; its +Y is (uz, -ux)
    return (cx + bx * ux + by * uz, cz + bx * uz - by * ux)


def yaw_idler_xz(cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """The base_yaw idler's axis (the base's x, z) with the stock belt: yaw_belt_loop()'s idler."""
    bx, by, _ = yaw_belt_loop(cfg)[cfg.idler.run + 1]
    return yaw_belt_point(bx, by, cfg)


def yaw_idler_slot(cfg: BaseConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The idler's slots' end centres (the base's x, z): `x` out along the belt's centre line, slot_y across it."""
    cx, cz = cfg.motor.centre
    bx = math.hypot(cx, cz) - cfg.idler.x
    return yaw_belt_point(bx, cfg.idler.slot_y[0], cfg), yaw_belt_point(bx, cfg.idler.slot_y[1], cfg)


def yaw_idler_y(cfg: BaseConfig = DEFAULT) -> float:
    """The idler's lower face along Y: level with the belt (its band centred on YAW_BELT_Y); the spacer's land on it."""
    return YAW_BELT_Y - GT2_IDLER_WIDTH / 2.0


def yaw_idler_spacer_h(cfg: BaseConfig = DEFAULT) -> float:
    """The idler's spacer: the plate's top to the idler's lower face."""
    return yaw_idler_y(cfg) - cfg.plate.y[1]


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
    the V's two `struts`: centrelines from the window's top corners down to the middle of its bottom edge. A wall wider
    than MountParams.bay between its end posts splits into equal bays, a `strut` wide post between: each bay a panel of
    its own, with its V."""
    s, p, j, m = cfg.shell, cfg.plate, cfg.joint, cfg.mount
    w_in, x1 = mount_inner_half(cfg), mount_x1(cfg)
    w_out = w_in + m.wall
    v = (s.y0 + m.strut, p.y[0] - m.strut)

    def panel(axis: str, span: tuple, u_axis: str, u: tuple) -> dict:
        um = (u[0] + u[1]) / 2.0
        return {"axis": axis, "span": span, "u": u_axis, "window": (u, v),
                "struts": [((u[0], v[1]), (um, v[0])), ((u[1], v[1]), (um, v[0]))]}

    def bays(u: tuple) -> list[tuple[float, float]]:
        n = max(1, math.ceil((u[1] - u[0]) / m.bay))
        w = (u[1] - u[0] - (n - 1) * m.strut) / n
        return [(u[0] + k * (w + m.strut), u[0] + k * (w + m.strut) + w) for k in range(n)]

    side_u = (j.split_x + max(m.ear_t, m.strut), x1 - m.strut)
    end_u = (-(w_out - m.strut), w_out - m.strut)
    return ([panel("z", (w_in, w_out), "x", u) for u in bays(side_u)]
            + [panel("z", (-w_out, -w_in), "x", u) for u in bays(side_u)]
            + [panel("x", (x1 - m.wall, x1), "z", u) for u in bays(end_u)])


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
